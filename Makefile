CC = gcc
CFLAGS = -Imavlink/common -g -ggdb
LDFLAGS = -lm

mock: mock_drone_tcp.o
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)

.PHONY: clean

clean:
	rm -f mock_drone_tcp.o mock
