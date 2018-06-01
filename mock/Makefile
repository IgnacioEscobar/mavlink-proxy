CC = gcc
CFLAGS = -I../mavlink/common -g -ggdb
LDFLAGS = -lm

tcp: mock_drone_tcp.o
	$(CC) -o $@Mock $^ $(CFLAGS) $(LDFLAGS)

udp: mock_drone_tcp.o
	$(CC) -o $@Mock $^ $(CFLAGS) $(LDFLAGS)

.PHONY: clean

clean:
	rm -f *.o *Mock
