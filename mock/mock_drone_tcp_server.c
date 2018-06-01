/*******************************************************************************
 Copyright (C) 2010  Bryan Godbolt godbolt ( a t ) ualberta.ca

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ****************************************************************************/
/*
 This program sends some data to qgroundcontrol using the mavlink protocol.  The sent packets
 cause qgroundcontrol to respond with heartbeats.  Any settings or custom commands sent from
 qgroundcontrol are printed by this program along with the heartbeats.


 I compiled this program sucessfully on Ubuntu 10.04 with the following command

 gcc -I ../../pixhawk/mavlink/include -o udp-server udp-server-test.c

 the rt library is needed for the clock_gettime on linux
 */
/* These headers are for QNX, but should all be standard on unix/linux */
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#endif

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include "mavlink.h"

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

#define IP_SERVER	"127.0.0.1"
#define IP_CLIENT	"127.0.0.1"

#define PORT_SERVER	10000
#define PORT_CLIENT	14501


uint64_t microsSinceEpoch();
void setearModo(const mavlink_message_t *msg);
void handleCommandLong(const mavlink_message_t *msg);
void respondWithAutopilotVersion(void);

uint8_t     _mavBaseMode = MAV_MODE_AUTO_DISARMED;
uint32_t    _mavCustomMode = 0;

struct sockaddr_in gcAddr;
int sock;

MAV_AUTOPILOT _firmwareType = MAV_AUTOPILOT_GENERIC;


/* mavlink_tcp_server.c
 * En tcp_server se hace una conexion mediante la llamada accept(), y cuando esta tiene
 * exito se guarda los datos del destino en la estructura client y en sockClient para
 * poder enviar datos
 * En el caso del server si o si hay que hacer la llamada a bind, sino el cliente no
 * no puede hacer un connect correctamente
 * El server no necesita ninguna ip del los clientes
 *
 * TCP: el server se bloquea hasta encontrar una conexion, luego envia datos al destino
 */


int main(int argc, char* argv[])
{
	int sockServer = socket(PF_INET, SOCK_STREAM, 0);
	int sockClient;

	struct sockaddr_in client;
	struct sockaddr_in server;

	uint8_t buf[BUFFER_LENGTH];
	ssize_t recsize;
	socklen_t fromlen;
	int bytes_sent;
	mavlink_message_t msg;
	uint16_t len;
	int i = 0;

	unsigned int temp = 0;
	socklen_t sin_size;

  /* Local address information */
	memset(&server, 0, sizeof(server));
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = inet_addr(IP_SERVER);
	server.sin_port = htons(PORT_SERVER);

  /* Lose the pesky "Address already in use" error message */
  int yes=1;
  if (setsockopt(sockServer,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof yes) == -1) {
    perror("<ERROR> setsockopt failed:");
    close(sockServer);
    exit(EXIT_FAILURE);
  }


	/* Bind  */
	if (bind(sockServer,(struct sockaddr *)&server, sizeof(struct sockaddr)) == -1) {
		perror("<ERROR> bind failed");
		close(sockServer);
		exit(EXIT_FAILURE);
  }

  /* Start listening for connections */
	if(listen(sockServer,2) == -1) {
    perror("<ERROR> listen failed:");
    close(sockServer);
    exit(EXIT_FAILURE);
  }

	/* Accept any incoming connections */
  sin_size=sizeof(struct sockaddr);
	if ((sockClient = accept(sockServer,(struct sockaddr *)&client, &sin_size))==-1) {
    	perror("<ERROR> accept failed:");
      close(sockServer);
      exit(EXIT_FAILURE);
	}

	printf("Se obtuvo una conexion desde %s\n",
     	inet_ntoa(client.sin_addr) ); /* mostrar la IP del cliente */

      unsigned int ticks = 60;
      float position[6] = {};
      float param = 0.0;

      for (;;)
      {

            if(ticks == 60)
            {
                ticks = 0;

                /*Send Heartbeat */
                mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, _mavBaseMode, 0, MAV_STATE_STANDBY);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                bytes_sent = send(sockClient, buf, len, 0);

                /* Send Status */
                mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                bytes_sent = send(sockClient, buf, len, 0);

                /* Send Local Position */
                mavlink_msg_local_position_ned_pack(1, 200, &msg, microsSinceEpoch(),
                                                                                position[0], position[1], position[2],
                                                                                position[3], position[4], position[5]);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                bytes_sent = send(sockClient, buf, len, 0);

                /* Send attitude */
                mavlink_msg_attitude_pack(1, 200, &msg, microsSinceEpoch(), 1.2, 1.7, 3.14, 0.01, 0.02, 0.03);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                bytes_sent = send(sockClient, buf, len, 0);

                memset(buf, 0, BUFFER_LENGTH);
                recsize = recv(sockClient, (void *)buf, BUFFER_LENGTH, 0);
                if (recsize > 0)
                {
                        // Something received - print out all bytes and parse packet
                        mavlink_message_t msg;
                        mavlink_status_t status;

                        printf("Bytes Received: %d\nDatagram: ", (int)recsize);
                        for (i = 0; i < recsize; ++i)
                        {
                                temp = buf[i];
                                printf("%02x ", (unsigned char)temp);
                                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
                                {
                                        // Packet received
                                        printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);

                                        switch(msg.msgid)
                                        {
                                            case MAVLINK_MSG_ID_SET_MODE:
                                                setearModo(&msg);
                                                break;

                                            case MAVLINK_MSG_ID_COMMAND_LONG:
                                                handleCommandLong(&msg);

                                            default:
                                                break;
                                        }
                                }
                        }
                        printf("\n");
                }
                memset(buf, 0, BUFFER_LENGTH);
            }

            /* Enviamos información del GPS 60 veces por segundo, solamente si está armed */
        		if(_mavBaseMode & MAV_MODE_FLAG_SAFETY_ARMED)
        		{
        		    mavlink_msg_gps_raw_int_pack(1,
        				    200,
        				    &msg,
        				    microsSinceEpoch(),                            // time since boot
        				    3,                                     // 3D fix
        				    (int32_t)((-34.5 + 0.001*sin(param))  * 1E7),
        				    (int32_t)((-58.5 + 0.001*cos(param)) * 1E7),
        				    (int32_t)(20  * 1000),
        				    UINT16_MAX, UINT16_MAX,                // HDOP/VDOP not known
        				    UINT16_MAX,                            // velocity not known
        				    UINT16_MAX,                            // course over ground not known
        				    8);                                    // satellite count
        		    len = mavlink_msg_to_send_buffer(buf, &msg);
        		    bytes_sent = send(sockClient, buf, len, 0);

        		    /* Send attitude */
        		    mavlink_msg_attitude_pack(1, 200, &msg, microsSinceEpoch(), 0.0, 0.0, -param, 0.01, 0.02, 0.03);
        		    len = mavlink_msg_to_send_buffer(buf, &msg);
        		    bytes_sent = send(sockClient, buf, len, 0);

        		    param += 2*M_PI/600;    // Trayectoria circular de período de 10 segundos

        		    if(param > 2*M_PI)
        			     param -= 2*M_PI;
        		}

        		//sleep(1); // Sleep one second
            ticks++;
            usleep((1.0/60)*1e6);   // Duerme 1/60 segundos
      }
}


void setearModo(const mavlink_message_t *msg)
{
    mavlink_set_mode_t request;
    mavlink_msg_set_mode_decode(msg, &request);

    _mavBaseMode = request.base_mode;
    _mavCustomMode = request.custom_mode;
}

void handleCommandLong(const mavlink_message_t *msg)
{
    mavlink_command_long_t request;
    uint8_t commandResult = MAV_RESULT_UNSUPPORTED;
    mavlink_message_t commandAck;
    uint16_t len;
    uint8_t buf[BUFFER_LENGTH];
    int bytes_sent;

    mavlink_msg_command_long_decode(msg, &request);

    switch (request.command) {
    case MAV_CMD_COMPONENT_ARM_DISARM:
        if (request.param1 == 0.0f) {
            _mavBaseMode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
        } else {
            _mavBaseMode |= MAV_MODE_FLAG_SAFETY_ARMED;
        }
        commandResult = MAV_RESULT_ACCEPTED;
        break;
    case MAV_CMD_PREFLIGHT_CALIBRATION:
        //_handlePreFlightCalibration(request);
        commandResult = MAV_RESULT_ACCEPTED;
        break;
    case MAV_CMD_PREFLIGHT_STORAGE:
        commandResult = MAV_RESULT_ACCEPTED;
        break;
    case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
        commandResult = MAV_RESULT_ACCEPTED;
        respondWithAutopilotVersion();
        break;
    }

    mavlink_msg_command_ack_pack(1,
                                 200,
                                 &commandAck,
                                 request.command,
                                 commandResult);

    len = mavlink_msg_to_send_buffer(buf, &commandAck);
    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));
}

void respondWithAutopilotVersion(void)
{
    mavlink_message_t msg;
    uint16_t len;
    uint8_t buf[BUFFER_LENGTH];
    int bytes_sent;

    uint8_t customVersion[8] = { };
    uint32_t flightVersion = 0;
    if (_firmwareType == MAV_AUTOPILOT_ARDUPILOTMEGA) {
        flightVersion |= 3 << (8*3);
        flightVersion |= 3 << (8*2);
        flightVersion |= 0 << (8*1);
        flightVersion |= FIRMWARE_VERSION_TYPE_DEV << (8*0);
    } else if (_firmwareType == MAV_AUTOPILOT_PX4) {
        flightVersion |= 1 << (8*3);
        flightVersion |= 4 << (8*2);
        flightVersion |= 1 << (8*1);
        flightVersion |= FIRMWARE_VERSION_TYPE_DEV << (8*0);
    }

    mavlink_msg_autopilot_version_pack(1,
                                       200,
                                       &msg,
                                       0,                               // capabilities,
                                       flightVersion,                   // flight_sw_version,
                                       0,                               // middleware_sw_version,
                                       0,                               // os_sw_version,
                                       0,                               // board_version,
                                       (uint8_t *)&customVersion,       // flight_custom_version,
                                       (uint8_t *)&customVersion,       // middleware_custom_version,
                                       (uint8_t *)&customVersion,       // os_custom_version,
                                       0,                               // vendor_id,
                                       0,                               // product_id,
                                       0);                              // uid

    len = mavlink_msg_to_send_buffer(buf, &msg);
    bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));
}

/* QNX timer version */
#if (defined __QNX__) | (defined __QNXNTO__)
uint64_t microsSinceEpoch()
{

	struct timespec time;

	uint64_t micros = 0;

	clock_gettime(CLOCK_REALTIME, &time);
	micros = (uint64_t)time.tv_sec * 1000000 + time.tv_nsec/1000;

	return micros;
}
#else
uint64_t microsSinceEpoch()
{

	struct timeval tv;

	uint64_t micros = 0;

	gettimeofday(&tv, NULL);
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

	return micros;
}
#endif
