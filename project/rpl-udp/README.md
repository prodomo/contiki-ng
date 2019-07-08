COMMAND Format: |--Start Word--|--command Type--|--sending way--|--command id--|--sensor number--|--setting type--| setting title--|--value--|


|--Commnad Type--| DATA 0x00, CONF 0x01, SET 0x02, ACK 0x03

|--- setting Type----| SET_TYPE_RATE 0x01, SET_TYPE_THRESHOLD  0x02, SET_TYPE_STATE 0x03, SET_TYPE_MODBUS 0x04, SET_TYPE_PRIORITY 0x05

|--- setting title----| SNR_TLE_DEFAULT 0x00, SNR_TLE_TEMPERATURE 0x01, SNR_TLE_TEMPERATURE_POINT 0x08, SNR_TLE_TEMPERATURE_ALARM 0x09, SNR_TLE_LED_FORECE 0x0A, SNR_TLE_BUZZ_FORCE	0x0B, SNR_TLE_S7S_FORCE 0x0C, SNR_TLE_S7S_VALUE	0x0D


Command Example

1.SETTING COMMAND

send Broadcast packet to change sending rate to 44(command id=12)

na 02,FFFF,12,01,01,00,44

send unicast packet to A69D change sending rate to 44(command id=12)

na 02,A69D,12,01,01,00,44

send unicast packet to A69D change sending rate to 44(command id=12) and change temperature threshold to 40

na 02,A69D,12,02,01,00,44,02,01,40

send unicast packet to A69D change temperature threshold to 40

na 02,A69D,12,01,02,01,40

na(startWd) 02(SET),A69D(MAC),12(command id),01(sensorNum),02(set threshold), 01(set temperature), 40(value)

2.ASK COMMAND

send Broadcast packet to ask mote's configuration (command id =12)

na 01,FFFF,12

send unicast packet to ask a69d's configuration (command id =12)

na 01,A69D,12
