#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <sys/time.h>
#include <time.h>
#include "pool.h"
#include "mpi.h"

#define VERBOSE_ROUTE_PLANNER 0
#define LARGE_NUM 99999999.0
#define BUFFER_SIZE 1024*1024*10

#define MAX_ROAD_LEN 100
#define MAX_VEHICLES 200
#define MAX_MINS 5
#define MIN_LENGTH_SECONDS 2
#define MAX_NUM_ROADS_PER_JUNCTION 50
#define SUMMARY_FREQUENCY 5
#define INITIAL_VEHICLES 100

#define BUS_PASSENGERS 80
#define BUS_MAX_SPEED 50
#define BUS_MIN_FUEL 10
#define BUS_MAX_FUEL 100
#define CAR_PASSENGERS 4
#define CAR_MAX_SPEED 100
#define CAR_MIN_FUEL 1
#define CAR_MAX_FUEL 40
#define MINI_BUS_MAX_SPEED 80
#define MINI_BUS_PASSENGERS 15
#define MINI_BUS_MIN_FUEL 2
#define MINI_BUS_MAX_FUEL 75
#define COACH_MAX_SPEED 60
#define COACH_PASSENGERS 40
#define COACH_MIN_FUEL 20
#define COACH_MAX_FUEL 200
#define MOTOR_BIKE_MAX_SPEED 120
#define MOTOR_BIKE_PASSENGERS 2
#define MOTOR_BIKE_MIN_FUEL 1
#define MOTOR_BIKE_MAX_FUEL 20
#define BIKE_MAX_SPEED 10
#define BIKE_PASSENGERS 1
#define BIKE_MIN_FUEL 2
#define BIKE_MAX_FUEL 10

#define VEHICLE 0
#define MAP 1


enum MPI_TAG {
  CONTROL_TAG,
  VEHICLE_TAG,
  DATAPACK_TAG,
  SPEED_TAG
};

enum ReadMode {
  NONE,
  ROADMAP,
  TRAFFICLIGHTS
};

enum VehicleType {
  CAR,
  BUS,
  MINI_BUS,
  COACH,
  MOTORBIKE,
  BIKE
};

struct JunctionStruct {
  int id, num_roads, num_vehicles;
  char hasTrafficLights;
  int trafficLightsRoadEnabled;
  int total_number_crashes, total_number_vehicles;
  struct RoadStruct * roads;
};


struct RoadStruct {
  struct JunctionStruct *from, *to;
  int roadLength, maxSpeed, numVehiclesOnRoad, currentSpeed;
  int total_number_vehicles, max_concurrent_vehicles;
};

struct VehicleStruct {
  int passengers, source, dest, maxSpeed;
  // Distance is in meters
  int speed, arrived_road_time, fuel;
  time_t last_distance_check_secs, start_t;
  double remaining_distance;
  char active;
  struct JunctionStruct * currentJunction;
  struct RoadStruct * roadOn;
};

struct VehicleforMPI {
  int passengers, source, dest, maxSpeed;
  int speed, arrived_road_time, fuel;
  time_t last_distance_check_secs, start_t;
  double remaining_distance;
  char active;
  int currentJunction;
  int road_from;
  int road_to;
};

struct DataPacket{
  int passengers_delivered;
  int vehicles_exhausted_fuel;
  int passengers_stranded;
  int vehicles_crashed;
};

struct VehicleandData{
  struct VehicleforMPI vehicle;
  struct DataPacket dataPacket;
};

static void handleVehicleUpdate(struct VehicleStruct* , struct DataPacket*);
static int findAppropriateRoad(int, struct JunctionStruct*);
static int planRoute(int, int);
static int findIndexOfMinimum(double*, char*);
static void initVehicles(int);
static int activateRandomVehicle();
static int activateVehicle(enum VehicleType);
static int findFreeVehicle();
static void loadRoadMap(char*);
static int getRandomInteger(int, int);
static void writeDetailedInfo();
static time_t getCurrentSeconds();
static void MPItoVehicle(struct VehicleforMPI*, struct VehicleStruct*);
static void VehicletoMPI(struct VehicleStruct*, struct VehicleforMPI*);
static void Vehicle();
static void Map();
static void ActorCode();
static void createInitialActor(int);
static void FindRoad(int, int, struct RoadStruct**);
static void cleanindices(int*);
void printVehicle(struct VehicleStruct*);




struct JunctionStruct * roadMap;
struct VehicleStruct * vehicles;
int num_junctions, num_roads=0;
int elapsed_mins=0;
int total_vehicles=0, passengers_delivered=0, vehicles_exhausted_fuel=0, passengers_stranded=0, vehicles_crashed=0;
int maprank = -1;

int size = 0;
int debug = 0;
int main(int argc, char *argv[]) {

  MPI_Init(&argc, &argv);
  
  int rank, detachsize;
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  MPI_Comm_size(MPI_COMM_WORLD, &size);

  if (argc < 2) {
      if (rank == 0) {
          printf("Usage: %s <roadmap_file>\n", argv[0]);
      }
      MPI_Finalize();
      return 1;
  }

  srand(time(0));
  char * buffer = (char*) malloc(BUFFER_SIZE);
	MPI_Buffer_attach(buffer, BUFFER_SIZE);

  loadRoadMap(argv[1]);

  int statusCode = processPoolInit();
  if (statusCode == 1) {        
    ActorCode();
  } else if (statusCode == 2) {
    createInitialActor(MAP);
    // for (int i = 0; i < INITIAL_VEHICLES; i++) {
    //   createInitialActor(VEHICLE);  //可能无法支持所有的初始汽车
    // }
    int masterStatus = masterPoll();
    while (masterStatus) {
      masterStatus=masterPoll();
    }
  }

  if (rank == 0) {
    free(roadMap);
  }
  processPoolFinalise();
  MPI_Buffer_detach(buffer, &detachsize);
  MPI_Finalize();
  return 0;
}

static void ActorCode() {
  int workerStatus = 1, data[1];
  while (workerStatus) {
    int parentId = getCommandData();
    MPI_Recv(data, 1, MPI_INT, parentId, CONTROL_TAG, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
    if (data[0] == VEHICLE) {
      // int rank;
      // MPI_Comm_rank(MPI_COMM_WORLD, &rank);
      // printf("my vehicle rank is %d\n", rank);
      Vehicle();
    } else if (data[0] == MAP) {
      // int rank;
      // MPI_Comm_rank(MPI_COMM_WORLD, &rank);
      // printf("my rank is %d\n", rank);
      Map();
    }
    workerStatus=workerSleep();
  }
}


static void Vehicle() {
  MPI_Status status;
  
  struct DataPacket dataPacket;
  dataPacket.passengers_delivered = 0;
  dataPacket.vehicles_exhausted_fuel = 0;
  dataPacket.passengers_stranded = 0;
  dataPacket.vehicles_crashed = 0;
  struct VehicleforMPI vehicleMPI;
  MPI_Recv(&vehicleMPI, sizeof(vehicleMPI), MPI_BYTE, MPI_ANY_SOURCE, VEHICLE_TAG, MPI_COMM_WORLD, &status);
  maprank = status.MPI_SOURCE;
  //print vehicleMPI

  struct VehicleStruct vehicle;
  MPItoVehicle(&vehicleMPI, &vehicle);

  struct JunctionStruct* junction;
  if(vehicle.roadOn != NULL){
    junction = vehicle.roadOn->to;
  }else if(vehicle.currentJunction != NULL){
    junction = vehicle.currentJunction;
  }else{
    printf("vehicle is not on road or junction\n");
  }
  //int* speeds = (int*)malloc(junction->num_roads * sizeof(int));
  //printf("num_roads: %d\n", junction->num_roads);
  // MPI_Recv(speeds, junction->num_roads, MPI_INT, maprank, SPEED_TAG, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
  // for(int j = 0; j < junction->num_roads; j++) {
  //   junction->roads[j].currentSpeed = speeds[j];
  // }
  // free(speeds);

  // handleVehicleUpdate(&vehicle, &dataPacket);
  // VehicletoMPI(&vehicle, &vehicleMPI);

  struct VehicleandData vehicleanddata;
  vehicleanddata.vehicle = vehicleMPI;
  vehicleanddata.dataPacket = dataPacket;
  MPI_Bsend(&vehicleanddata, sizeof(struct VehicleandData), MPI_BYTE, maprank, DATAPACK_TAG, MPI_COMM_WORLD);

}

static void VehicletoMPI(struct VehicleStruct * vehicle, struct VehicleforMPI * vehicleMPI) {
  vehicleMPI->passengers = vehicle->passengers;
  vehicleMPI->source = vehicle->source;
  vehicleMPI->dest = vehicle->dest;
  vehicleMPI->maxSpeed = vehicle->maxSpeed;
  vehicleMPI->speed = vehicle->speed;
  vehicleMPI->arrived_road_time = vehicle->arrived_road_time;
  vehicleMPI->fuel = vehicle->fuel;
  vehicleMPI->last_distance_check_secs = vehicle->last_distance_check_secs;
  vehicleMPI->start_t = vehicle->start_t;
  vehicleMPI->remaining_distance = vehicle->remaining_distance;
  vehicleMPI->active = vehicle->active;
  if (vehicle->currentJunction == NULL) {
    vehicleMPI->currentJunction = -1;
  } else {
    vehicleMPI->currentJunction = vehicle->currentJunction->id;
  }
  if (vehicle->roadOn == NULL) {
    vehicleMPI->road_from = -1;
    vehicleMPI->road_to = -1;
  } else {
    vehicleMPI->road_from = vehicle->roadOn->from->id;
    vehicleMPI->road_to = vehicle->roadOn->to->id;
  }
}

static void MPItoVehicle(struct VehicleforMPI * vehicleMPI, struct VehicleStruct * vehicle) {
  vehicle->passengers = vehicleMPI->passengers;
  vehicle->source = vehicleMPI->source;
  vehicle->dest = vehicleMPI->dest;
  vehicle->maxSpeed = vehicleMPI->maxSpeed;
  vehicle->speed = vehicleMPI->speed;
  vehicle->arrived_road_time = vehicleMPI->arrived_road_time;
  vehicle->fuel = vehicleMPI->fuel;
  vehicle->last_distance_check_secs = vehicleMPI->last_distance_check_secs;
  vehicle->start_t = vehicleMPI->start_t;
  vehicle->remaining_distance = vehicleMPI->remaining_distance;
  vehicle->active = vehicleMPI->active;
  if (vehicleMPI->currentJunction == -1) {
    vehicle->currentJunction = NULL;
  } else {
    vehicle->currentJunction = &roadMap[vehicleMPI->currentJunction];
  }
  if (vehicleMPI->road_from == -1) {
    vehicle->roadOn = NULL;
  } else {
    FindRoad(vehicleMPI->road_from, vehicleMPI->road_to, &vehicle->roadOn);
  }
}

static void FindRoad(int road_from, int road_to, struct RoadStruct ** road) {
  for (int i = 0; i < roadMap[road_from].num_roads; i++) {
      if (roadMap[road_from].roads[i].to->id == road_to) {
          *road = &roadMap[road_from].roads[i];
          return;
      }
  }
}

static void handleVehicleUpdate(struct VehicleStruct* vehicle, struct DataPacket* dataPacket) {
  if (getCurrentSeconds() - vehicle->start_t > vehicle->fuel) {
    dataPacket->vehicles_exhausted_fuel++;
    dataPacket->passengers_stranded += vehicle->passengers;
    vehicle->active = 0;
    return;
  }
  if (vehicle->roadOn != NULL && vehicle->currentJunction == NULL) {
    // Means that the vehicle is currently on a road
    time_t sec = getCurrentSeconds();
    int latest_time = sec - vehicle->last_distance_check_secs;
    if (latest_time < 1) return;
    vehicle->last_distance_check_secs = sec;
    double travelled_length = latest_time * vehicle->speed;
    vehicle->remaining_distance -= travelled_length;
    if (vehicle->remaining_distance <= 0) {
      // Left the road and arrived at the target junction
      vehicle->arrived_road_time = 0;
      vehicle->last_distance_check_secs = 0;
      vehicle->remaining_distance = 0;
      vehicle->speed = 0;
      vehicle->currentJunction = vehicle->roadOn->to;
      vehicle->currentJunction->num_vehicles++;
      vehicle->currentJunction->total_number_vehicles++;
      vehicle->roadOn->numVehiclesOnRoad--;
      vehicle->roadOn = NULL;
    }
  }
  if (vehicle->currentJunction != NULL) {
    if (vehicle->roadOn == NULL) {
      // If the road is NULL then the vehicle is on a junction and not on a road
      if (vehicle->currentJunction->id == vehicle->dest) {
        // Arrived! Job done!
        dataPacket->passengers_delivered += vehicle->passengers;
        vehicle->active = 0;
      } else {
        // Plan route from here
        int next_junction_target = planRoute(vehicle->currentJunction->id, vehicle->dest);
        if (next_junction_target != -1) {
          int road_to_take = findAppropriateRoad(next_junction_target, vehicle->currentJunction);
          assert(vehicle->currentJunction->roads[road_to_take].to->id == next_junction_target);
          vehicle->roadOn = &(vehicle->currentJunction->roads[road_to_take]);
          vehicle->roadOn->numVehiclesOnRoad++;
          vehicle->roadOn->total_number_vehicles++;
          if (vehicle->roadOn->max_concurrent_vehicles < vehicle->roadOn->numVehiclesOnRoad) {
            vehicle->roadOn->max_concurrent_vehicles = vehicle->roadOn->numVehiclesOnRoad;
          }
          vehicle->remaining_distance = vehicle->roadOn->roadLength;
          vehicle->speed = vehicle->roadOn->currentSpeed;
          if (vehicle->speed > vehicle->maxSpeed) vehicle->speed = vehicle->maxSpeed;
        } else {
          // Report error (this should never happen)
          fprintf(stderr, "No longer a viable route\n");
          exit(-1);
        }
      }
    }
    // Here we have selected a junction, now it's time to determine if the vehicle can be released from the junction
    char take_road = 0;
    if (vehicle->currentJunction->hasTrafficLights) {
      // Need to check that we can go, otherwise need to wait until road enabled by traffic light
      take_road = vehicle->roadOn == &vehicle->currentJunction->roads[vehicle->currentJunction->trafficLightsRoadEnabled];
    } else {
      // If not traffic light then there is a chance of collision
      int collision = getRandomInteger(0, 8) * vehicle->currentJunction->num_vehicles;
      if (collision > 40) {
        // Vehicle has crashed!
        dataPacket->passengers_stranded += vehicle->passengers;
        dataPacket->vehicles_crashed++;
        vehicle->active = 0;
        vehicle->currentJunction->total_number_crashes++; //!!!!!!!!!路口变量需要更新
      }
      take_road = 1;
    }
    // If take the road then clear the junction
    if (take_road) {
      vehicle->last_distance_check_secs = getCurrentSeconds();
      vehicle->currentJunction->num_vehicles--;
      vehicle->currentJunction = NULL;
    }
  }
}

static void cleanindices(int* indices) {
  for (int i = 0; i < MAX_VEHICLES; i++) {
    indices[i] = -1;
  }
}

static void Map() {
  initVehicles(INITIAL_VEHICLES);
  struct VehicleandData* vehicleanddata = (struct VehicleandData*)malloc(sizeof(struct VehicleandData) * MAX_VEHICLES);
  int indices[MAX_VEHICLES];
  cleanindices(indices);
  MPI_Request* requests = (MPI_Request*) malloc(sizeof(MPI_Request) * size);
  int requestCount = 0;
  time_t seconds=0;
  time_t start_seconds=getCurrentSeconds();
  
  printf("Starting simulation with %d junctions, %d roads and %d vehicles\n", num_junctions, num_roads, INITIAL_VEHICLES);
  while (elapsed_mins < MAX_MINS) {
      time_t current_seconds=getCurrentSeconds();
      if (current_seconds != seconds) {
          seconds=current_seconds;
          if (seconds-start_seconds > 0) {
              if ((seconds-start_seconds) % MIN_LENGTH_SECONDS == 0) {
                  elapsed_mins++;
                  debug = 1;
                  // Add a random number of new vehicles to the simulation
                  int num_new_vehicles=getRandomInteger(100, 200);
                  //print num_new_vehicles
                  printf("num_new_vehicles: %d\n", num_new_vehicles);
                  for (int i=0;i<num_new_vehicles;i++) {
                      activateRandomVehicle();
                  }
                  if (elapsed_mins % SUMMARY_FREQUENCY == 0) {
                      printf("[Time: %d mins] %d vehicles, %d passengers delivered, %d stranded passengers, %d crashed vehicles, %d vehicles exhausted fuel\n",
                      elapsed_mins, total_vehicles, passengers_delivered, passengers_stranded, vehicles_crashed, vehicles_exhausted_fuel);
                  }
                  
              }
          }
      }

      // State update for junctions
      for (int i=0;i<num_junctions;i++) {
          if (roadMap[i].hasTrafficLights && roadMap[i].num_roads > 0) {
              // Each minute of simulation time we enable a new road to be taken
              roadMap[i].trafficLightsRoadEnabled=elapsed_mins % roadMap[i].num_roads;
          }
          for (int j=0;j<roadMap[i].num_roads;j++) {
              struct RoadStruct * road=&roadMap[i].roads[j];
              int num_vehicles_on_road=0;
              for (int k=0;k<MAX_VEHICLES;k++) {
                  if (vehicles[k].active && vehicles[k].roadOn == road) num_vehicles_on_road++;
              }
              // Recalculate current speed based on congestion and length (minus from max speed)
              road->currentSpeed=road->maxSpeed - road->numVehiclesOnRoad;
              if (road->currentSpeed < 10) road->currentSpeed=10;
          }
      }

      // State update for vehicles
      for (int i=0;i<MAX_VEHICLES;i++) {
          while (vehicles[i].active) {
              int rank = startWorkerProcess();
              //printf("rank: %d started\n", rank);
              if(rank != -1){
                  indices[i] = 1;
                  int data[1];
                  data[0]=VEHICLE;
                  MPI_Bsend(data, 1, MPI_INT, rank, CONTROL_TAG, MPI_COMM_WORLD);

                  struct VehicleforMPI vehicleMPI;
                  VehicletoMPI(&vehicles[i], &vehicleMPI);
                  //printVehicle(&vehicles[i]);

                  MPI_Bsend(&vehicleMPI, sizeof(vehicleMPI), MPI_BYTE, rank, VEHICLE_TAG, MPI_COMM_WORLD);

                  //send current speed to vehicle
                  struct JunctionStruct *junction;
                  if(vehicles[i].roadOn != NULL){
                    junction = vehicles[i].roadOn->to;
                  }else if(vehicles[i].currentJunction != NULL){
                    junction = vehicles[i].currentJunction;
                  }
                  // int* speeds = (int*)malloc(junction->num_roads * sizeof(int));
                  // for(int j = 0; j < junction->num_roads; j++) {
                  //   speeds[j] = junction->roads[j].currentSpeed;
                  // }
                  // MPI_Bsend(speeds, junction->num_roads, MPI_INT, rank, SPEED_TAG, MPI_COMM_WORLD);
                  // free(speeds);
                  MPI_Irecv(&vehicleanddata[i], sizeof(struct VehicleandData), MPI_BYTE, rank, DATAPACK_TAG, MPI_COMM_WORLD, &requests[requestCount++]);
                  MPI_Wait(&requests[requestCount-1], MPI_STATUS_IGNORE);
                  //打印已经处理了多少辆车
                  //printf("vehicle %d has been processed\n", i);
                  break;
              }
          }
          //printf("vehicle %d's active has been checked\n", i);
      }
      //打印已经处理了一次循环
      //printf("one loop has been processed--------------------------------------\n");
      //MPI_Waitall(requestCount, requests, MPI_STATUSES_IGNORE);
      requestCount = 0;
      for(int i = 0; i < MAX_VEHICLES; i++) {
        if(indices[i] != -1) {
          //printf("vehicle %d has been copied\n", i);
          struct VehicleforMPI vehicleMPI = vehicleanddata[i].vehicle;
          MPItoVehicle(&vehicleMPI, &vehicles[i]);
          passengers_delivered += vehicleanddata[i].dataPacket.passengers_delivered;
          passengers_stranded += vehicleanddata[i].dataPacket.passengers_stranded;
          vehicles_exhausted_fuel += vehicleanddata[i].dataPacket.vehicles_exhausted_fuel;
          vehicles_crashed += vehicleanddata[i].dataPacket.vehicles_crashed;
        }
      }
      cleanindices(indices);
  }
  // On termination display a final summary and write detailed information to file
  printf("Finished after %d mins: %d vehicles, %d passengers delivered, %d passengers stranded, %d crashed vehicles, %d vehicles exhausted fuel\n",
          elapsed_mins, total_vehicles, passengers_delivered, passengers_stranded, vehicles_crashed, vehicles_exhausted_fuel);
  free(requests);
  writeDetailedInfo();
}


static void createInitialActor(int type) {
  int workerPid = startWorkerProcess();
  int data[1];
  data[0] = type;
  MPI_Bsend(data, 1, MPI_INT, workerPid, CONTROL_TAG, MPI_COMM_WORLD);
}




static void initVehicles(int num_initial) {
  vehicles=(struct VehicleStruct*) malloc(sizeof(struct VehicleStruct) * MAX_VEHICLES);
  for (int i=0;i<MAX_VEHICLES;i++) {
    vehicles[i].active=0;
    vehicles[i].roadOn=NULL;
    vehicles[i].currentJunction=NULL;
    vehicles[i].maxSpeed = 0;
  }
  for (int i=0;i<num_initial;i++) {
    activateRandomVehicle();
  }
}

static int activateRandomVehicle() {
  int random_vehicle_type=getRandomInteger(0, 5);
  enum VehicleType vehicleType;
  if (random_vehicle_type == 0) {
    vehicleType=BUS;
  } else if (random_vehicle_type == 1) {
    vehicleType=CAR;
  } else if (random_vehicle_type == 2) {
    vehicleType=MINI_BUS;
  } else if (random_vehicle_type == 3) {
    vehicleType=COACH;
  } else if (random_vehicle_type == 4) {
    vehicleType=MOTORBIKE;
  } else if (random_vehicle_type == 5) {
    vehicleType=BIKE;
  }
  return activateVehicle(vehicleType);
}

static int activateVehicle(enum VehicleType vehicleType) {
  int id=findFreeVehicle();
  if (id >= 0) {
    total_vehicles++;
    vehicles[id].active=1;
    vehicles[id].start_t=getCurrentSeconds();
    vehicles[id].last_distance_check_secs=0;
    vehicles[id].speed=0;
    vehicles[id].remaining_distance=0;
    vehicles[id].arrived_road_time=0;
    vehicles[id].source=vehicles[id].dest=getRandomInteger(0, num_junctions);
    while (vehicles[id].dest == vehicles[id].source) {
      // Ensure that the source and destination are different
      vehicles[id].dest=getRandomInteger(0, num_junctions);
      if (vehicles[id].dest != vehicles[id].source) {
        // See if there is a viable route between the source and destination
        int next_jnct=planRoute(vehicles[id].source, vehicles[id].dest);
        
        if (next_jnct == -1) {
          // Regenerate source and dest
          vehicles[id].source=vehicles[id].dest=getRandomInteger(0, num_junctions);
        }
      }
    }
    vehicles[id].currentJunction=&roadMap[vehicles[id].source];
    vehicles[id].currentJunction->num_vehicles++;
    vehicles[id].currentJunction->total_number_vehicles++;
    vehicles[id].roadOn=NULL;
    if (vehicleType == CAR) {
      vehicles[id].maxSpeed=CAR_MAX_SPEED;
      vehicles[id].passengers=getRandomInteger(1, CAR_PASSENGERS);
      vehicles[id].fuel=getRandomInteger(CAR_MIN_FUEL, CAR_MAX_FUEL);
    } else if (vehicleType == BUS) {
      vehicles[id].maxSpeed=BUS_MAX_SPEED;
      vehicles[id].passengers=getRandomInteger(1, BUS_PASSENGERS);
      vehicles[id].fuel=getRandomInteger(BUS_MIN_FUEL, BUS_MAX_FUEL);
    } else if (vehicleType == MINI_BUS) {
      vehicles[id].maxSpeed=MINI_BUS_MAX_SPEED;
      vehicles[id].passengers=getRandomInteger(1, MINI_BUS_PASSENGERS);
      vehicles[id].fuel=getRandomInteger(MINI_BUS_MIN_FUEL, MINI_BUS_MAX_FUEL);
    } else if (vehicleType == COACH) {
      vehicles[id].maxSpeed=COACH_MAX_SPEED;
      vehicles[id].passengers=getRandomInteger(1, COACH_PASSENGERS);
      vehicles[id].fuel=getRandomInteger(COACH_MIN_FUEL, COACH_MAX_FUEL);
    } else if (vehicleType == MOTORBIKE) {
      vehicles[id].maxSpeed=MOTOR_BIKE_MAX_SPEED;
      vehicles[id].passengers=getRandomInteger(1, MOTOR_BIKE_PASSENGERS);
      vehicles[id].fuel=getRandomInteger(MOTOR_BIKE_MIN_FUEL, MOTOR_BIKE_MAX_FUEL);
    } else if (vehicleType == BIKE) {
      vehicles[id].maxSpeed=BIKE_MAX_SPEED;
      vehicles[id].passengers=getRandomInteger(1, BIKE_PASSENGERS);
      vehicles[id].fuel=getRandomInteger(BIKE_MIN_FUEL, BIKE_MAX_FUEL);
    } else {
      fprintf(stderr, "Unknown vehicle type\n");
    }
    return id;
  }
  return -1;
}

static int findFreeVehicle() {
  for (int i=0;i<MAX_VEHICLES;i++) {
    if (!vehicles[i].active) return i;
  }
  return -1;
}

static void loadRoadMap(char * filename) {
  enum ReadMode currentMode=NONE;
  char buffer[MAX_ROAD_LEN];
  FILE * f=fopen(filename, "r");
  if (f == NULL) {
    fprintf(stderr, "Error opening roadmap file '%s'\n", filename);
    exit(-1);
  }

  while(fgets(buffer, MAX_ROAD_LEN, f)) {
    if (buffer[0]=='%') continue;
    if (buffer[0]=='#') {
      if (strncmp("# Road layout:", buffer, 14)==0) {
        char * s=strstr(buffer, ":");
        num_junctions=atoi(&s[1]);
        roadMap=(struct JunctionStruct*) malloc(sizeof(struct JunctionStruct) * num_junctions);
        for (int i=0;i<num_junctions;i++) {
          roadMap[i].id=i;
          roadMap[i].num_roads=0;
          roadMap[i].num_vehicles=0;
          roadMap[i].hasTrafficLights=0;
          roadMap[i].total_number_crashes=0;
          roadMap[i].total_number_vehicles=0;
          // Not ideal to allocate all roads size here
          roadMap[i].roads=(struct RoadStruct*) malloc(sizeof(struct RoadStruct) * MAX_NUM_ROADS_PER_JUNCTION);
        }
        currentMode=ROADMAP;
      }
      if (strncmp("# Traffic lights:", buffer, 17)==0) {
        currentMode=TRAFFICLIGHTS;
      }
    } else {
      if (currentMode == ROADMAP) {
        char * space=strstr(buffer, " ");
        *space='\0';
        int from_id=atoi(buffer);
        char * nextspace=strstr(&space[1], " ");
        *nextspace='\0';
        int to_id=atoi(&space[1]);
        char * nextspace2=strstr(&nextspace[1], " ");
        *nextspace='\0';
        int roadlength=atoi(&nextspace[1]);
        int speed=atoi(&nextspace2[1]);
        if (roadMap[from_id].num_roads >= MAX_NUM_ROADS_PER_JUNCTION) {
          fprintf(stderr, "Error: Tried to create road %d at junction %d, but maximum number of roads is %d, increase 'MAX_NUM_ROADS_PER_JUNCTION'",
            roadMap[from_id].num_roads, from_id, MAX_NUM_ROADS_PER_JUNCTION);
          exit(-1);
        }
        roadMap[from_id].roads[roadMap[from_id].num_roads].from=&roadMap[from_id];
        roadMap[from_id].roads[roadMap[from_id].num_roads].to=&roadMap[to_id];
        roadMap[from_id].roads[roadMap[from_id].num_roads].roadLength=roadlength;
        roadMap[from_id].roads[roadMap[from_id].num_roads].maxSpeed=speed;
        roadMap[from_id].roads[roadMap[from_id].num_roads].numVehiclesOnRoad=0;
        roadMap[from_id].roads[roadMap[from_id].num_roads].currentSpeed=speed;
        roadMap[from_id].roads[roadMap[from_id].num_roads].total_number_vehicles=0;
        roadMap[from_id].roads[roadMap[from_id].num_roads].max_concurrent_vehicles=0;
        roadMap[from_id].num_roads++;
        num_roads++;
      } else if (currentMode == TRAFFICLIGHTS) {
        int id=atoi(buffer);
        if (roadMap[id].num_roads > 0) roadMap[id].hasTrafficLights=1;
      }
    }
  }
  fclose(f);
}




static int findAppropriateRoad(int dest_junction, struct JunctionStruct * junction) {
  for (int j=0;j<junction->num_roads;j++) {
    if (junction->roads[j].to->id == dest_junction) return j;
  }
  return -1;
}

static int planRoute(int source_id, int dest_id) {
  //print num_junctions
  if(debug == 1){
    printf("num_junctions: %d\n", num_junctions);
  }
  if (VERBOSE_ROUTE_PLANNER) printf("Search for route from %d to %d\n", source_id, dest_id);
  double * dist=(double*) malloc(sizeof(double) * num_junctions);
  char * active=(char*) malloc(sizeof(char) * num_junctions);
  if(debug == 1){
    printf("Search for route from %d to %d\n", source_id, dest_id);
  }
  struct JunctionStruct ** prev=(struct JunctionStruct **) malloc(sizeof( struct JunctionStruct *) * num_junctions);
  
  int activeJunctions=num_junctions;
  for (int i=0;i<num_junctions;i++) {
    active[i]=1;
    prev[i]=NULL;
    if (i != source_id) {
      dist[i]=LARGE_NUM;
    }
  }
  dist[source_id]=0;

  while (activeJunctions > 0) {
    int v_idx=findIndexOfMinimum(dist, active);
    if (v_idx == dest_id) break;
    struct JunctionStruct * v=&roadMap[v_idx];
    active[v_idx]=0;
    activeJunctions--;

    for (int i=0;i<v->num_roads;i++) {
      if (active[v->roads[i].to->id] && dist[v_idx] != LARGE_NUM) {
        double alt=dist[v_idx] + v->roads[i].roadLength / (v->id == source_id ? v->roads[i].currentSpeed : v->roads[i].maxSpeed);
        if (alt < dist[v->roads[i].to->id]) {
          dist[v->roads[i].to->id]=alt;
          prev[v->roads[i].to->id]=v;
        }
      }
    }
  }
  free(dist);
  free(active);
  int u_idx=dest_id;
  int * route=(int*) malloc(sizeof(int) * num_junctions);
  int route_len=0;
  if (prev[u_idx] != NULL || u_idx == source_id) {
    if (VERBOSE_ROUTE_PLANNER) printf("Start at %d\n", u_idx);
    while (prev[u_idx] != NULL) {
      route[route_len]=u_idx;
      u_idx=prev[u_idx]->id;
      if (VERBOSE_ROUTE_PLANNER) printf("Route %d\n", u_idx);
      route_len++;
    }
  }
  free(prev);
  if (route_len > 0) {
    int next_jnct=route[route_len-1];
    if (VERBOSE_ROUTE_PLANNER) printf("Found next junction is %d\n", next_jnct);
    free(route);
    return next_jnct;
  }
  if (VERBOSE_ROUTE_PLANNER) printf("Failed to find route between %d and %d\n", source_id, dest_id);
  free(route);
  return -1;
}

static int findIndexOfMinimum(double * dist, char * active) {
  double min_dist=LARGE_NUM+1;
  int current_min=-1;
  for (int i=0;i<num_junctions;i++) {
    if (active[i] && dist[i] < min_dist) {
      min_dist=dist[i];
      current_min=i;
    }
  }
  return current_min;
}

static int getRandomInteger(int from, int to) {
  return (rand() % (to-from)) + from;
}

static void writeDetailedInfo() {
  FILE * f=fopen("results", "w");
  for (int i=0;i<num_junctions;i++) {
    fprintf(f, "Junction %d: %d total vehicles and %d crashes\n", i, roadMap[i].total_number_vehicles, roadMap[i].total_number_crashes);
    for (int j=0;j<roadMap[i].num_roads;j++) {
      fprintf(f, "--> Road from %d to %d: Total vehicles %d and %d maximum concurrently\n", roadMap[i].roads[j].from->id,
        roadMap[i].roads[j].to->id, roadMap[i].roads[j].total_number_vehicles, roadMap[i].roads[j].max_concurrent_vehicles);
    }
  }
  fclose(f);
}

static time_t getCurrentSeconds() {
  struct timeval curr_time;
  gettimeofday(&curr_time, NULL);
  time_t current_seconds=curr_time.tv_sec;
  return current_seconds;
}


void printVehicle(struct VehicleStruct * vehicle) {
  printf("----------------------------");
  printf("passengers: %d\n", vehicle->passengers);
  printf("source: %d\n", vehicle->source);
  printf("dest: %d\n", vehicle->dest);
  printf("maxSpeed: %d\n", vehicle->maxSpeed);
  printf("speed: %d\n", vehicle->speed);
  printf("arrived_road_time: %d\n", vehicle->arrived_road_time);
  printf("fuel: %d\n", vehicle->fuel);
  printf("last_distance_check_secs: %d\n", vehicle->last_distance_check_secs);
  printf("start_t: %d\n", vehicle->start_t);
  printf("remaining_distance: %f\n", vehicle->remaining_distance);
  printf("active: %d\n", vehicle->active);
  if (vehicle->currentJunction == NULL) {
    printf("currentJunction: NULL\n");
  } else {
    printf("currentJunction: %d\n", vehicle->currentJunction->id);
  }
  if (vehicle->roadOn == NULL) {
    printf("roadOn: NULL\n");
  } else {
    printf("roadOn: %d -> %d\n", vehicle->roadOn->from->id, vehicle->roadOn->to->id);
  }
  printf("----------------------------");
}
