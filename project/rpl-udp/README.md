A simple RPL network with UDP communication. This is a self-contained example:
it includes a DAG root (`udp-server.c`) and DAG nodes (`udp-clients.c`).
This example runs without a border router -- this is a stand-alone RPL network.

The DAG root also acts as UDP server. The DAG nodes are UDP client. The clients
send a UDP request periodically, that simply includes a counter as payload.
When receiving a request, The server sends a response with the same counter
back to the originator.

The `.csc` files show example networks in the Cooja simulator, for sky motes and
for cooja motes.

For this example a "renode" make target is available, to run a two node
emulation in the Renode framework. For further instructions on installing and
using Renode please refer to [Contiki-NG wiki][1].

[1]: https://github.com/contiki-ng/contiki-ng/wiki/Tutorial:-Running-Contiki%E2%80%90NG-in-Renode

Command Example


1.SETTING COMMAND

send Broadcast packet to change sending rate to 44(command id=12)

na 02,FFFF,12,01,01,00,44

send unicast packet to A69D change sending rate to 44(command id=12)

na 02,A69D,12,01,01,00,44

send unicast packet to A69D change sending rate to 44(command id=12) and change temperature threshold to 40

na 02,A69D,12,02,01,00,44,02,01,40

2.ASK COMMAND

send Broadcast packet to ask mote's configuration (command id =12)

na 01,FFFF,12

send unicast packet to ask a69d's configuration (command id =12)

na 01,A69D,12


COMMAND Format:
|--Start Word--|--command Type--|--sending way--|--command id--|--sensor number--|--setting type--| setting title--|--value--|
     na             DATA 0x00	 unicast:Mac addr                                    RATE 0x01		 DEFAULT 0x00
					CONF 0x01	 broadcast:FFFF											.				.	
					SET  0x02															.				.
					ACK  0x03															.				.


|--Commnad Type--|
	DATA 0x00
	CONF 0x01
	SET  0x02
	ACK  0x03


|--- setting Type----|----Seting Title----|----setting Value----|
  RATE 		0x01		DEFAULT 	0x00
  THRESHOLD 0x02		TEMPERATURE 0x01
  STATE 	0x03		ELE_CURRENT 0x02
  MODBUS	0x04		ROTAT_SPEED 0x03
  PRIORITY	0x05		DISTANCE    0x04
  						TEMPERATURE_A 0x05
						TEMPERATURE_B 0x06
						BATTERY		0x07

						MODBUS_ADDR 0x11
						MODBUS_REG  0x12
						MODBUS_FREQ 0x13
						MODBUS_TYPE 0x14

						URGENT_VALUE 0x61
						URGENT_SOUND 0x62

