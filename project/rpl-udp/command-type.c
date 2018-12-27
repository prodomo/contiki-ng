#include "contiki.h"
#include "command-type.h"
#include "sys/log.h"

#include <stdio.h>
#include <stdlib.h>

#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_NONE

int flag = 0;
char* command_data;
/*---------------------------------------------------------------------------*/
char* get_command(void)
{
	return command_data;
	flag=0;
}

/*---------------------------------------------------------------------------*/
void
serial_input(char *str)
{
	command_data = malloc(strlen(str) + 1);
    strcpy(command_data, str);

  	LOG_INFO("serial_input: %s\n", str);
  	flag = 1;

  // split = strtok (str," ,.-\\");
  // while (split != NULL)
  // {
  //   strcpy(temp[count], split);
  //   count++;
  //   split = strtok (NULL, " ,.-\\");
  // }

  // int sensor_num = atoi(temp[3]);
  // printf("senor_num %d\n", sensor_num);

  // struct msg
  // {
  //   uint16_t commandId;
  //   uint16_t commandType;
  //   uint16_t sensorNum;
  //   struct setting_msg setmsg[sensor_num];
  // };
  // struct msg msg;
  // memset(&msg, 0, sizeof(msg));

  // msg.commandType = CMD_TYPE_SET;
  // msg.commandId = (uint16_t)atoi(temp[2]);
  // msg.sensorNum = sensor_num;

  // printf("msg.commandType %d\n", msg.commandType);
  // printf("msg.commandId %d\n", msg.commandId);
  // printf("sensor_num%d\n", sensor_num);

  // for(int i=0; i<sensor_num; i++)
  // {
  //   msg.setmsg[i].setting_type = atoi(temp[4+i*3]);
  //   msg.setmsg[i].sensor_tittle = atoi(temp[5+i*3]);
  //   msg.setmsg[i].value = atoi(temp[6+i*3]);
  //   printf("setting_type %d\n", msg.setmsg[i].setting_type);
  //   printf("sensor_tittle %d\n", msg.setmsg[i].sensor_tittle);
  //   printf("value %d\n", msg.setmsg[i].value);
  // }
  
  // // buffer->msg = msg;
  // buffer->dst = temp[1];
  // memcpy(&command_data , msg, sizeof(msg));
}
/*---------------------------------------------------------------------------*/

int
get_flag()
{
  return flag;
}
/*---------------------------------------------------------------------------*/

