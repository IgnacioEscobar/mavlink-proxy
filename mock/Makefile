CC = gcc
CFLAGS = -I../mavlink/common -g -ggdb
LDFLAGS = -lm

tcpClient: mock_drone_tcp_client.o tcp.o
	$(CC) -o $@Mock $^ $(CFLAGS) $(LDFLAGS)

tcpServer: mock_drone_tcp_server.o
	$(CC) -o $@Mock $^ $(CFLAGS) $(LDFLAGS)

udp: mock_drone_tcp.o
	$(CC) -o $@Mock $^ $(CFLAGS) $(LDFLAGS)

tcp.o: tcp.c
	$(CC) -c -o $@ $< $(CFLAGS)

.PHONY: clean

clean:
	rm -f *.o *Mock
