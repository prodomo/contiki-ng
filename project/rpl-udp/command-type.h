#ifndef COMMAND_TYPE_H_
#define COMMAND_TYPE_H_

#define STARTWORD "SW"
#define ENDWORD  "EW"

//BroadCast
#define BROADCAST "FFFF"
//Command Type
#define CMD_TYPE_DATA 0x00
#define CMD_TYPE_CONF 0x01
#define CMD_TYPE_SET  0x02
#define CMD_TYPE_ACK  0x03

//Sensor Tittle 
#define SNR_TLE_DEFAULT 	0x00
#define SNR_TLE_TEMPERATURE 0x01
#define SNR_TLE_ELE_CURRENT 0x02
#define SNR_TLE_ROTAT_SPEED 0x03
#define SNR_TLE_DISTANCE    0x04
#define SNR_TLE_TEMPERATURE_A 0x05  //a of ax+b=y 
#define SNR_TLE_TEMPERATURE_B 0x06  //b of ax+b=y
#define SNR_TLE_BATTERY		0x07

#define SNR_TLE_MODBUS_ADDR 0x11
#define SNR_TLE_MODBUS_REG  0x12
#define SNR_TLE_MODBUS_FREQ 0x13
#define SNR_TLE_MODBUS_TYPE 0x14

#define SNR_TLE_URGENT_VALUE 61
#define SNR_TLE_URGENT_SOUND 62

//Setting Type
// #define SET_TYPE_ASK 0x00
#define SET_TYPE_RATE 		0x01
#define SET_TYPE_THRESHOLD  0x02
#define SET_TYPE_STATE 		0x03
#define SET_TYPE_MODBUS		0x04
#define SET_TYPE_PRIORITY	0x05


//GENERATATION STATE
#define DEFAULT_STATE 0x00
#define SOL_STATE 	  0x01  //Start of Life
#define PVT_STATE     0x02  //Production Validation Test 
#define MP_STATE      0x03  //Mass Production
#define EOMP_STATE    0x04  //End of Mass Production
#define EOL_STATE     0x05  //End of Life

#define START_CLOSE   0x06
#define CLOSE         0x07
#define START_OPEN	  0x08
#define OPEN          0x09
#define READY_OPEN    0x10

// extern struct simple_udp_connection udp_conn;

struct setting_msg {
	uint16_t setting_type;
	uint16_t sensor_tittle;
	uint16_t value;
};

struct  gpio_log
{
	uint16_t	gpio;
	uint16_t	state;
	
};

struct command{
	char *dst;
	const void *msg;
};

void serial_input(char *str);
int get_flag(void);
char* get_command(void);

#endif