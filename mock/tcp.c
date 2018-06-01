/*
 ============================================================================
 Nombre        : tcp.c
 Descripcion   : Funciones basicas de sockets tcp
 ============================================================================
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <sys/types.h>
 #include <sys/socket.h>
 #include <netdb.h>
 #include <arpa/inet.h>
 #include <netinet/in.h>
 #include <errno.h>


int crear_conexion(const char* ip,const char* puerto){

	  int statusAddrInfo;
		struct addrinfo hints;
		memset(&hints, 0, sizeof hints);
		hints.ai_family   = AF_UNSPEC;
		hints.ai_socktype = SOCK_STREAM;
		int fail = 0;


		struct addrinfo *serverInfo;
		if ((statusAddrInfo = getaddrinfo(ip, puerto, &hints, &serverInfo)) != 0){
			fprintf(stderr, "Error en getaddrinfo() : %s", gai_strerror(statusAddrInfo));
			fail = 1;
		}
		/**/
		int socketFD;
		if ((socketFD = socket(serverInfo->ai_family, serverInfo->ai_socktype, serverInfo->ai_protocol)) == -1){
			perror("Error de Socket");
			fail = 1;
		}
		/**/
		// Evaluar mover la conexion a otra funcion
		if ((connect(socketFD, serverInfo->ai_addr, serverInfo->ai_addrlen)) == -1){
			perror("Error en la conexion");
			fail = 1;
		}
		freeaddrinfo(serverInfo);

		if (fail) return -1;
		return socketFD;
};
/*
 ============================================================================
 */
int crear_listener(const char* puerto){

	  int statusAddrInfo;
	  struct addrinfo hints;
		memset(&hints, 0, sizeof hints);
		hints.ai_family   = AF_UNSPEC;
		hints.ai_socktype = SOCK_STREAM;
		hints.ai_flags    = AI_PASSIVE;
		int fail = 0;

		struct addrinfo *serverInfo;
		if ((statusAddrInfo = getaddrinfo(NULL, puerto, &hints, &serverInfo)) != 0){
			fprintf(stderr, "Error en getaddrinfo() : %s", gai_strerror(statusAddrInfo));
			fail = 1;
		}
		/**/
		int listenerSocket;
		if ((listenerSocket = socket(serverInfo->ai_family, serverInfo->ai_socktype, serverInfo->ai_protocol)) == -1){
			perror("Error de Socket");
			fail = 1;
		}
		/**/
		int yes=1;
		if (setsockopt(listenerSocket,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof yes) == -1) {
		    perror("setsockopt");
		    exit(1);
		}
		if ((bind(listenerSocket, serverInfo->ai_addr, serverInfo->ai_addrlen)) == -1){
			perror("Error de bindeo");
			fail = 1;
		}

	freeaddrinfo(serverInfo);
	if (fail) return -1;
	return listenerSocket;
}
/*
 ============================================================================
 */
int escuchar_socket(const int listenerSocket, const int backlog){
	listen(listenerSocket, backlog);
	struct sockaddr_in addr;
	socklen_t addrlen = sizeof(addr);
	int socketCliente = accept(listenerSocket, (struct sockaddr *) &addr, &addrlen);
	return socketCliente;
};
