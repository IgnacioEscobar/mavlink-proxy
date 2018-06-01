#include "tcp.h"
#include "droneServer.h"

#define BACKLOG 100

int main (int argc, char *argv[]) {
  // Link a ground control
  int groundControlListener = crear_listener("15600");
  int groundControlSocket = escuchar_socket(groundControlListener, BACKLOG);

  // Link a drones
  init_droneServer("15601");
}
