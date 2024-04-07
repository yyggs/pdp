#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <sys/time.h>
#include <time.h>

#define VERBOSE_ROUTE_PLANNER 0
#define LARGE_NUM 99999999.0

#define MAX_ROAD_LEN 100
#define MAX_VEHICLES 1000
#define MAX_MINS 100
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

struct JunctionStruct * roadMap;
struct VehicleStruct * vehicles;
int num_junctions, num_roads=0;
int elapsed_mins=0;

int total_vehicles=0, passengers_delivered=0, vehicles_exhausted_fuel=0, passengers_stranded=0, vehicles_crashed=0;

static void handleVehicleUpdate(int);
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

/**
 * Program entry point and main loop
 **/
int main(int argc, char * argv[]) {
  if (argc != 2) {
    fprintf(stderr, "Error: You need to provide the roadmap file as the only argument\n");
    exit(-1);
  }
  srand(time(0));
  // Load road map graph (and traffic lights) from file
  loadRoadMap(argv[1]);
  printf("Loaded road map from file\n");
  // Create initial vehicles
  initVehicles(INITIAL_VEHICLES);
  // Initialise time
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
          // Add a random number of new vehicles to the simulation
          int num_new_vehicles=getRandomInteger(100, 200);
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
      if (vehicles[i].active) {
        handleVehicleUpdate(i);
      }
    }
  }
  // On termination display a final summary and write detailed information to file
  printf("Finished after %d mins: %d vehicles, %d passengers delivered, %d passengers stranded, %d crashed vehicles, %d vehicles exhausted fuel\n",
          elapsed_mins, total_vehicles, passengers_delivered, passengers_stranded, vehicles_crashed, vehicles_exhausted_fuel);
  writeDetailedInfo();
  return 0;
}

/**
 * Handles an update for a specific vehicle that is on a road or waiting at a junction
 **/
static void handleVehicleUpdate(int i) {
  if (getCurrentSeconds() - vehicles[i].start_t > vehicles[i].fuel) {
    vehicles_exhausted_fuel++;
    passengers_stranded+=vehicles[i].passengers;
    vehicles[i].active=0;
    return;
  }
  if (vehicles[i].roadOn != NULL && vehicles[i].currentJunction == NULL) {
    // Means that the vehicle is currently on a road
    time_t sec=getCurrentSeconds();
    int latest_time=sec - vehicles[i].last_distance_check_secs;
    if (latest_time < 1) return;
    vehicles[i].last_distance_check_secs=sec;
    double travelled_length=latest_time * vehicles[i].speed;
    vehicles[i].remaining_distance-=travelled_length;
    if (vehicles[i].remaining_distance <= 0) {
      // Left the road and arrived at the target junction
      vehicles[i].arrived_road_time=0;
      vehicles[i].last_distance_check_secs=0;
      vehicles[i].remaining_distance=0;
      vehicles[i].speed=0;
      vehicles[i].currentJunction=vehicles[i].roadOn->to;
      vehicles[i].currentJunction->num_vehicles++;
      vehicles[i].currentJunction->total_number_vehicles++;
      vehicles[i].roadOn->numVehiclesOnRoad--;
      vehicles[i].roadOn=NULL;
    }
  }
  if (vehicles[i].currentJunction != NULL) {
    if (vehicles[i].roadOn == NULL) {
      // If the road is NULL then the vehicle is on a junction and not on a road
      if (vehicles[i].currentJunction->id == vehicles[i].dest) {
        // Arrived! Job done!
        passengers_delivered+=vehicles[i].passengers;
        vehicles[i].active=0;
      } else {
        // Plan route from here
        int next_junction_target=planRoute(vehicles[i].currentJunction->id, vehicles[i].dest);
        if (next_junction_target != -1) {
          int road_to_take=findAppropriateRoad(next_junction_target, vehicles[i].currentJunction);
          assert(vehicles[i].currentJunction->roads[road_to_take].to->id == next_junction_target);
          vehicles[i].roadOn=&(vehicles[i].currentJunction->roads[road_to_take]);
          vehicles[i].roadOn->numVehiclesOnRoad++;
          vehicles[i].roadOn->total_number_vehicles++;
          if (vehicles[i].roadOn->max_concurrent_vehicles < vehicles[i].roadOn->numVehiclesOnRoad) {
            vehicles[i].roadOn->max_concurrent_vehicles=vehicles[i].roadOn->numVehiclesOnRoad;
          }
          vehicles[i].remaining_distance=vehicles[i].roadOn->roadLength;
          vehicles[i].speed=vehicles[i].roadOn->currentSpeed;
          if (vehicles[i].speed > vehicles[i].maxSpeed) vehicles[i].speed=vehicles[i].maxSpeed;
        } else {
          // Report error (this should never happen)
          fprintf(stderr, "No longer a viable route\n");
          exit(-1);
        }
      }
    }
    // Here we have selected a junction, now it's time to determine if the vehicle can be released from the junction
    char take_road=0;
    if (vehicles[i].currentJunction->hasTrafficLights) {
      // Need to check that we can go, otherwise need to wait until road enabled by traffic light
      take_road=vehicles[i].roadOn == &vehicles[i].currentJunction->roads[vehicles[i].currentJunction->trafficLightsRoadEnabled];
    } else {
      //printf("num_vehicles: %d\n", vehicles[i].currentJunction->num_vehicles);
      // If not traffic light then there is a chance of collision
      int collision=getRandomInteger(0,8)*vehicles[i].currentJunction->num_vehicles;
      if (collision > 40) {
        // Vehicle has crashed!
        passengers_stranded+=vehicles[i].passengers;
        vehicles_crashed++;
        vehicles[i].active=0;
        vehicles[i].currentJunction->total_number_crashes++;
      }
      take_road=1;
    }
    // If take the road then clear the junction
    if (take_road) {
      vehicles[i].last_distance_check_secs=getCurrentSeconds();
      vehicles[i].currentJunction->num_vehicles--;
      vehicles[i].currentJunction = NULL;
    }
  }
}

/**
 * Finds the road index out of the junction's roads that leads to a specific
 * destination junction
 **/
static int findAppropriateRoad(int dest_junction, struct JunctionStruct * junction) {
  for (int j=0;j<junction->num_roads;j++) {
    if (junction->roads[j].to->id == dest_junction) return j;
  }
  return -1;
}

/**
 * Plans a route from the source to destination junction, returning the junction after
 * the source junction. This will be called to plan a route from A (where the vehicle
 * is currently) to B (the destination), so will return the junction that most be travelled
 * to next. -1 is returned if no route is found
 **/
static int planRoute(int source_id, int dest_id) {
  if (VERBOSE_ROUTE_PLANNER) printf("Search for route from %d to %d\n", source_id, dest_id);
  double * dist=(double*) malloc(sizeof(double) * num_junctions);
  char * active=(char*) malloc(sizeof(char) * num_junctions);
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

/**
 * Finds the index of the input array that is active and has the smallest number
 **/
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

/**
 * Initialises vehicles at the start of the simulation and will activate a
 * specified number
 **/
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

/**
 * Activates a vehicle and sets its type and route randomly
 **/
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

/**
 * Activates a vehicle with a specific type, will find an idle vehicle data
 * element and then initialise this with a random (but valid) route between
 * two junction. The new vehicle's index is returned,
 * or -1 if there are no free slots.
 **/
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

/**
 * Iterates through the vehicles array and finds the first free entry, returns
 * the index of this or -1 if none is found
 **/
static int findFreeVehicle() {
  for (int i=0;i<MAX_VEHICLES;i++) {
    if (!vehicles[i].active) return i;
  }
  return -1;
}

/**
 * Parses the provided roadmap file and uses this to build the graph of
 * junctions and roads, as well as reading traffic light information
 **/
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

/**
 * Generates a random integer between two values, including the from value up to the to value minus
 * one, i.e. from=0, to=100 will generate a random integer between 0 and 99 inclusive
 **/
static int getRandomInteger(int from, int to) {
  return (rand() % (to-from)) + from;
}

/**
 * Writes out to file detailed information (at termination) around junction and road metrics
 **/
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

/**
 * Retrieves the current time in seconds
 **/
static time_t getCurrentSeconds() {
  struct timeval curr_time;
  gettimeofday(&curr_time, NULL);
  time_t current_seconds=curr_time.tv_sec;
  return current_seconds;
}
