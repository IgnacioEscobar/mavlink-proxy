CC = gcc
CFLAGS = -I -g -ggdb

build: main.c tcp.c droneServer.c behavior.c
	$(CC) -o App main.c tcp.c droneServer.c behavior.c $(CFLAGS)

.PHONY: clean

clean:
	rm -f *.o App
