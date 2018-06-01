#include "tcp.h"
#include "droneServer.h"
#include "behavior.h"

#define BACKLOG 100
int groundControlSocket;

int main (int argc, char *argv[]) {
  // Link a ground control
  int groundControlListener = crear_listener("15600");
  groundControlSocket = escuchar_socket(groundControlListener, BACKLOG);

  // Link a drones
  init_droneServer("15601",pasajeBasico);
}
