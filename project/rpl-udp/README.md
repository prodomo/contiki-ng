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
