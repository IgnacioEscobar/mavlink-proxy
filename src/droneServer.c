#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define BACKLOG 100

void init_droneServer(const char* puertoEscucha){

  	// Recibir conexion
  	int listener = crear_listener(puertoEscucha);
  	int backlog  = BACKLOG;
  	// Multiplexar listener
  	fd_set master,auxfds;
		int i,fdmax,nuevoCliente;
    socklen_t addrlen;
    struct sockaddr_storage remoteaddr;
	  HEADER_T header;

		//Escuchar listener
		if(listen(listener,backlog)==-1){
			perror("Listen");
			exit(EXIT_FAILURE);
		}
		// Limpiar los sets de fd
		FD_ZERO(&master);
		FD_ZERO(&auxfds);
		//Agregar listener a la lista
		FD_SET(listener,&master);
		fdmax = listener;

		// Loop principal
		while(1){
			auxfds = master;
			if(select(fdmax+1, &auxfds, NULL, NULL, NULL)==-1){
				perror("Select");
				exit(EXIT_FAILURE);
			}

			for(i = 0; i<=fdmax;i++){
				if (FD_ISSET(i, &auxfds)) {
					if (i == listener) {
						// Nueva conexion
						addrlen = sizeof remoteaddr;
						nuevoCliente = accept(listener,(struct sockaddr *)&remoteaddr,&addrlen);
						if (nuevoCliente == -1) {
							perror("accept");
						}
						else {
							FD_SET(nuevoCliente, &master); // Agregar al set master
							if (nuevoCliente > fdmax) {
								fdmax = nuevoCliente;
								}
							log_trace(logYAMA, "Se conecto Master %d", idUltimoMasterCreado);
							agregarAListado(nuevoCliente);
						}
					} else { // Escuchar mensaje
            //////////////////////////////////////
            //////////////////////////////////////
            //////////////////////////////////////
            //////    Logica del proxy   /////////
            //////////////////////////////////////
            //////////////////////////////////////
            //////////////////////////////////////
					}
				}
			}
		}
};
