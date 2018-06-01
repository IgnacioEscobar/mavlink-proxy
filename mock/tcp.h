/*
 ============================================================================
 Nombre        : tcp.h
 Descripcion   : Funciones basicas de sockets tcp
 ============================================================================
 */
#ifndef TCP_H_
#define TCP_H_

/*
 ============================================================================
 Crea un socket y lo conecta a al servidor indicado
    Recicibe:
        ip      : ip del servidor en formato de cadena
        puerto  : puerto al cual conectarse en formato cadena
    Devuelve:
        retorno : file descriptor del socket creado en formato entero
                * (-1) en caso de error
 ============================================================================
*/
int crear_conexion(const char* ip, const char* puerto);

/*
 ============================================================================
 Crea un socket listener
    Recicibe:
        puerto  : puerto al cual se debe escuchar
    Devuelve:
        retorno : file descriptor del socket creado en formato entero
                * (-1) en caso de error
============================================================================
*/
int crear_listener(const char* puerto);

/*
 ============================================================================
 Escucha un socket listener
    Recicibe:
        listenerSocket : file descriptor del socket a escuchar
        backlog        : cantidad de conexiones simultaneas
   Devuelve:
        retorno : file descriptor del socket creado en formato entero
                * (-1) en caso de error
============================================================================
*/
int escuchar_socket(const int listenerSocket, const int backlog);

#endif /* TCP_H_ */
