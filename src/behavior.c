/*
 ============================================================================
 Nombre        : behavior.c
 Descripcion   :
 ============================================================================
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#define BUFFER_LENGTH 2041
extern int groundControlSocket;

void pasajeBasico(int socketDron){
  uint8_t buf[BUFFER_LENGTH];
  int bytesRecibidos;
  if((bytesRecibidos = recv(socketDron, (void *)buf, BUFFER_LENGTH, MSG_WAITALL)) == -1){
    perror("Error recibiendo mensaje del dron");
    exit(EXIT_FAILURE);
  }

  send(groundControlSocket, (void *)buf, bytesRecibidos, MSG_WAITALL);
}
