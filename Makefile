CC=gcc
CFLAGS=-Imavlink/common -g -ggdb
LDFLAGS=-lm

mavlink_udp: mavlink_udp.o
	$(CC) -o mavlink_udp mavlink_udp.o $(CFLAGS) $(LDFLAGS)

.PHONY: clean

clean:
	rm -f mavlink_udp mavlink_udp.o
