SRC = new.c pool.c
CC=mpicc
CFLAGS=-O3

all: 
	$(CC) -o new $(SRC) $(CFLAGS)

clean:
	rm -f new
