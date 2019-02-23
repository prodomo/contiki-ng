/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "net/routing/routing.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/ipv6/uip-sr.h"
#include "net/ipv6/uip.h"
#include "net/mac/tsch/tsch.h"
#include "dev/serial-line.h"
#include "command-type.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <string.h>

#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

// #define DEBUG DEBUG_PRINT
// #include "net/ipv6/uip-debug.h"

#define WITH_SERVER_REPLY  0
#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

#define COMMAND_PERIOD 1

struct simple_udp_connection udp_conn;
static uip_ipaddr_t temp_ipaddr;

PROCESS(udp_server_process, "UDP server");
AUTOSTART_PROCESSES(&udp_server_process);
/*---------------------------------------------------------------------------*/
void get_serial_input(char *str){
  LOG_INFO("get_serial_input: %s\n", str);
}
/*---------------------------------------------------------------------------*/
uint8_t
ascii_to_uint(char c)
{
  uint8_t result;
  uint8_t temp;
  temp = (uint8_t)c;
  if(temp<58)
    result = temp-'0';
  else if(temp<71 && temp>64)
    result = temp-'7';
  else if(temp<103  && temp>96)
    result = temp-'W';
  else
    result=temp;

  return result;
}
/*---------------------------------------------------------------------------*/
void command_send()
{
  char* command;
  char* split;
  char temp[20][20];
  int count=0;
  uint8_t  dst_u8[2];

  command = get_command();
  LOG_INFO("command_send: %s\n", command);

  split = strtok (command," ,.-\\");
  while (split != NULL)
  {
    strcpy(temp[count], split);
    count++;
    split = strtok (NULL, " ,.-\\");
  }

  int sensor_num = atoi(temp[3]);
  LOG_INFO("senor_num %d\n", sensor_num);

  struct msg
  {
    uint16_t commandId;
    uint16_t commandType;
    uint16_t sensorNum;
    struct setting_msg setmsg[sensor_num];
  };
  struct msg msg;
  memset(&msg, 0, sizeof(msg));

  msg.commandType = (uint16_t)atoi(temp[0]);
  msg.commandId = (uint16_t)atoi(temp[2]);
  msg.sensorNum = sensor_num;

  LOG_INFO("msg.commandType %d\n", msg.commandType);
  LOG_INFO("msg.commandId %d\n", msg.commandId);
  LOG_INFO("sensor_num%d\n", sensor_num);

  for(int i=0; i<sensor_num; i++)
  {
    msg.setmsg[i].setting_type = atoi(temp[4+i*3]);
    msg.setmsg[i].sensor_tittle = atoi(temp[5+i*3]);
    msg.setmsg[i].value = atoi(temp[6+i*3]);
    LOG_INFO("setting_type %d\n", msg.setmsg[i].setting_type);
    LOG_INFO("sensor_tittle %d\n", msg.setmsg[i].sensor_tittle);
    LOG_INFO("value %d\n", msg.setmsg[i].value);
  }

  LOG_INFO_6ADDR(&temp_ipaddr);

  if(strncmp(temp[1], BROADCAST, 4)==0)
  {
    LOG_INFO("broadcast\n");
    simple_udp_msend(&udp_conn, &msg, sizeof(msg));  //can't broadcast yet
  }
  else
  {

    dst_u8[0] =ascii_to_uint(temp[1][0])<<4;
    dst_u8[0] += ascii_to_uint(temp[1][1]);

    dst_u8[1] = ascii_to_uint(temp[1][2])<<4;
    dst_u8[1] += ascii_to_uint(temp[1][3]);
    
    temp_ipaddr.u8[14]=dst_u8[0];
    temp_ipaddr.u8[15]=dst_u8[1];

    LOG_INFO("mac 1: %x\n", dst_u8[0]);
    LOG_INFO("mac 2: %x\n", dst_u8[1]);


    LOG_INFO_6ADDR(&temp_ipaddr);
    simple_udp_sendto(&udp_conn, &msg, sizeof(msg), &temp_ipaddr);
  }

  // simple_udp_sendto(&udp_conn, &msg, sizeof(msg), &temp_ipaddr);


}
/*---------------------------------------------------------------------------*/
static void
udp_rx_callback(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
  uint16_t tempdata;

  LOG_INFO("Received request '%u' ", datalen);
  LOG_INFO("from ");
  LOG_INFO_6ADDR(sender_addr);
  LOG_INFO_("\n");

  printf("%u ", datalen);
  printf("%04x ", sender_addr->u8[14] + (sender_addr->u8[15] << 8));


  for(int i=0; i<(datalen/2); i++)
  {
    memcpy(&tempdata , data, sizeof(uint16_t));
    data+=sizeof(uint16_t);
    printf("%u ",tempdata);
  }
  printf("\n");

  uip_ipaddr_copy(&temp_ipaddr, sender_addr);


#if WITH_SERVER_REPLY
  /* send back the same string to the client as an echo reply */
  LOG_INFO("Sending response.\n");
  simple_udp_sendto(&udp_conn, data, datalen, sender_addr);

#endif /* WITH_SERVER_REPLY */
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
  int is_coordinator;
  static struct etimer command_timer;
  int flag;

  PROCESS_BEGIN();

  is_coordinator = 1;

  /* Initialize DAG root */
  if(is_coordinator) {
    NETSTACK_ROUTING.root_start();
  }

  NETSTACK_MAC.on();

  /* Initialize UDP connection */
  simple_udp_register(&udp_conn, UDP_SERVER_PORT, NULL,
                      UDP_CLIENT_PORT, udp_rx_callback);

  while(1) {
    PROCESS_YIELD();
    if(ev == serial_line_event_message) {
      LOG_INFO("serial_line_event_message.\n");
      etimer_set(&command_timer, CLOCK_SECOND * COMMAND_PERIOD);
    }
    else if(ev == PROCESS_EVENT_TIMER)
    {
      if(data == &command_timer){
        flag = get_flag();
        if(flag)
        {
          command_send();
        }
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
