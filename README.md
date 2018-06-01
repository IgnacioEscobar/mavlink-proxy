# Mavlink Proxy

## Mock
Aplicaciones que simulan a un dron, enviando mensajes mavlink de heartbeat y telemetria. Disponibles en protocolos TCP y UDP
### Compilacion
Compilacion simplemente con el comando make (en la carpeta mock)
``` bash
# Para compilar la version UDP
make TCP
# Para compilar la version TCP
make tcp
```
### Ejecucion
```TCP```: Al ejecutar se crea un servidor escuchando conexiones en el puerto ```10000```; al recibir una conexion enviara heartbeats y datos de telemetria en formato mavlink.

```UDP```: deprecated
