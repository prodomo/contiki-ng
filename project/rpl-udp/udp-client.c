#include "contiki.h"
#include "net/routing/routing.h"
#include "net/routing/rpl-lite/rpl.h"
#include "net/routing/rpl-lite/rpl-neighbor.h"
#include "random.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/ipv6/uip-sr.h"
#include "net/mac/tsch/tsch.h"
#include "net/link-stats.h"
#include "dev/leds.h"
#include "sys/log.h"
#include "dev/modbus/modbus-api.h"
#include "dev/modbus/modbusDefines.h"

#include "command-type.h"
#include "net/link-stats.h"
#include "dev/cc2538-sensors.h"

#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

// #define DEBUG DEBUG_PRINT
// #include "net/ipv6/uip-debug.h"

#define WITH_SERVER_REPLY  1
#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

static struct simple_udp_connection udp_conn;

#define START_INTERVAL		(15 * CLOCK_SECOND)
#define POLLING_INTERVAL  (1 * CLOCK_SECOND)

typedef struct register_type {
  unsigned char HI;
  unsigned char LO;
} register_type_t;

static register_type_t tempReg[] = {
  {0x25, 0x80}, //0 baudrate
  {0x00, 0x08}, //1 bitLen
  {0x00, 0x00}, //2 parity
  {0x00, 0x01}, //3 stopBit
  {0x25, 0x80}, //4 baudrate
  {0x00, 0x80}, //5 bitLen
  {0x00, 0x00}, //6 parity
  {0x00, 0x01}, //7 stopBit
  {0x00, 0x14}, //8 param1
  {0x00, 0x1e}, //9 param2
  {0x00, 0x05}, //10 PacketRate
  {0x00, 0x05}, //11 SampleRate
  {0x00, 0x00}, //12 connected
  {0x00, 0x00}, //13 null
  {0x00, 0x00}, //14 null
  {0x02, 0x58}, //15 AlmTmpLv
  {0x00, 0x01}, //16 AlmTmpLvPoint
  {0x00, 0x01}, //17 AlmAllOn
  {0x01, 0x45}, //18 TmpValue
  {0x00, 0x01}, //19 TmpValue_point
  {0x01, 0x4A}, //20 BattMax
  {0x00, 0x02}, //21 BattMaxPoint
  {0x00, 0x52}, //22 BattPct
  {0x00, 0x0A}, //23 PWM_LED_Freq
  {0x00, 0x14}, //24 PWM_LED_Duty
  {0x00, 0x0F}, //25 PWM_Buzz_Freq
  {0x00, 0x14}, //26 PWM_Buzz_Duty
  {0x00, 0x00}, //27 LED_Force
  {0x00, 0x00}, //28 Buzz_Force
  {0x00, 0x00}, //29 BTN_S
  {0x00, 0x01}, //30 BTN_M
  {0x00, 0x00}, //31 BTN_L
  {0x00, 0x01}, //32 S7S_Force
  {0x04, 0xD2}, //33 S7S_Value
  {0x00, 0x02}, //34 S7S_Point
};

uip_ipaddr_t dest_ipaddr;
static uint16_t sendRate;

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/
void
leds_blink(void)
{
  /* Blink all leds that were initially off. */
  leds_toggle(LEDS_GREEN);

  clock_delay(400);

  leds_toggle(LEDS_GREEN);
}
/*---------------------------------------------------------------------------*/
void setting_value(struct setting_msg msg)
{

  if(msg.setting_type == SET_TYPE_RATE && msg.sensor_tittle == SNR_TLE_DEFAULT)
  {
    tempReg[10].HI = ((msg.value >> 8) & 0xff);
    tempReg[10].LO = (msg.value & 0xff);
    LOG_INFO("changing sending rate: %d\n",tempReg[10].HI<<8|tempReg[10].LO);
  }
  else if(msg.setting_type == SET_TYPE_THRESHOLD)
  {
    // LOG_INFO("sensor_tittle %d\n", msg.sensor_tittle);
    switch(msg.sensor_tittle){
      case SNR_TLE_DEFAULT:
        break;
      
      case SNR_TLE_TEMPERATURE:
        tempReg[15].HI = ((msg.value >> 8) & 0xff);
        tempReg[15].LO = (msg.value & 0xff);
        LOG_INFO("changing temperature threshold: %d\n",tempReg[15].HI<<8|tempReg[15].LO);
        break;

      case SNR_TLE_TEMPERATURE_POINT:
        tempReg[16].HI = ((msg.value >> 8) & 0xff);
        tempReg[16].LO = (msg.value & 0xff);
        LOG_INFO("changing temperature threshold point: %d\n",tempReg[16].HI<<8|tempReg[16].LO);
        break;

      case SNR_TLE_TEMPERATURE_ALARM:
        tempReg[17].HI = ((msg.value >> 8) & 0xff);
        tempReg[17].LO = (msg.value & 0xff);
        LOG_INFO("changing temperature threshold alarm: %d\n",tempReg[17].HI<<8|tempReg[17].LO);
        break;

      case SNR_TLE_LED_FORECE:
        tempReg[27].HI = ((msg.value >> 8) & 0xff);
        tempReg[27].LO = (msg.value & 0xff);
        LOG_INFO("changing temperature threshold alarm: %d\n",tempReg[27].HI<<8|tempReg[27].LO);
        break;

      case SNR_TLE_BUZZ_FORCE:
        tempReg[28].HI = ((msg.value >> 8) & 0xff);
        tempReg[28].LO = (msg.value & 0xff);
        LOG_INFO("changing temperature threshold alarm: %d\n",tempReg[28].HI<<8|tempReg[28].LO);
        break;

      case SNR_TLE_S7S_FORCE:
        tempReg[32].HI = ((msg.value >> 8) & 0xff);
        tempReg[32].LO = (msg.value & 0xff);
        LOG_INFO("changing temperature threshold alarm: %d\n",tempReg[32].HI<<8|tempReg[32].LO);
        break;

      case SNR_TLE_S7S_VALUE:
        tempReg[33].HI = ((msg.value >> 8) & 0xff);
        tempReg[33].LO = (msg.value & 0xff);
        LOG_INFO("changing temperature threshold alarm: %d\n",tempReg[33].HI<<8|tempReg[33].LO);
        break;

      default:
        break;
      }
  }
  else{
    return;
  }

}
/*---------------------------------------------------------------------------*/
void
collect_ack_send(const uip_ipaddr_t *sender_addr,
         uint16_t commandId)
{
  // printf("generate ack packet\n");
  struct 
  {
    uint16_t command_id;
    uint16_t command_type;
    uint16_t is_received;
  }ack;

  memset(&ack, 0, sizeof(ack));

  ack.command_id = commandId;
  ack.command_type = CMD_TYPE_ACK;
  ack.is_received = 1;
  // printf("sizeof(ack) %d\n", sizeof(ack));
  LOG_INFO("ack: %u %u %u\n", ack.command_type, ack.command_id, ack.is_received);

  LOG_INFO("send ack\n");
  simple_udp_sendto(&udp_conn, &ack, sizeof(ack), sender_addr);
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
  struct msg{
    uint16_t commandId;
    uint16_t commandType;
  };
  struct msg msg;
  uint16_t sensor_num = 0;

  memset(&msg, 0, sizeof(msg));
  // printf("sizeof(msg) %d\n", sizeof(msg));

  // printf("--------------------recv data-----------------\n");
  // printf("uip_datalen %u\n",datalen);

  memcpy(&msg.commandId, data, sizeof(uint16_t));
  data+=sizeof(uint16_t);
  memcpy(&msg.commandType, data, sizeof(uint16_t));
  data+=sizeof(uint16_t);
  LOG_INFO("msg.commandType %u\n", msg.commandType);
  LOG_INFO("msg.commandId %u\n", msg.commandId);

  if(uip_datalen()>4)
  {
    memcpy(&sensor_num, data, sizeof(uint16_t));
    data+=sizeof(uint16_t);
  }

  struct setting_msg setting_msg[sensor_num];

  if(sensor_num>0)
  {
    for(int i=0; i<sensor_num; i++)
    {
      memcpy(&setting_msg[i].setting_type, data, sizeof(uint16_t));
      data+=sizeof(uint16_t);
      memcpy(&setting_msg[i].sensor_tittle, data, sizeof(uint16_t));
      data+=sizeof(uint16_t);
      memcpy(&setting_msg[i].value, data, sizeof(uint16_t));
      data+=sizeof(uint16_t);
    }
  }
  switch(msg.commandType)
  {
    case CMD_TYPE_CONF:
      LOG_INFO("should send conf\n");
      break;
      
    case CMD_TYPE_SET:
      LOG_INFO("should set value\n");
      if(sensor_num>0)
      {
        for(int i=0; i<sensor_num; i++)
        {
          setting_value(setting_msg[i]);
        }
      }
      break;
    }

  collect_ack_send(sender_addr, msg.commandId);
  // LOG_INFO("Received response '%.*s' from ", datalen, data);
  // LOG_INFO_6ADDR(sender_addr);


// #if LLSEC802154_CONF_ENABLED
//   LOG_INFO_(" LLSEC LV:%d", uipbuf_get_attr(UIPBUF_ATTR_LLSEC_LEVEL));
// #endif
//   LOG_INFO_("\n");

}
/*---------------------------------------------------------------------------*/
void
collect_common_send(void)
{
  static uint16_t seqno;
  struct{
  uint16_t seqno;
  uint16_t parent;
  uint16_t parent_etx;
  uint16_t current_rtmetric;
  uint16_t num_neighbors;
  uint16_t parent_rssi;
  uint16_t temp_value;
  uint16_t ext_tempature_value;  // =real value*100
  uint16_t int_tempature_value;  // =real value*1000
  uint16_t battery;
  uint16_t ext_tempature_point;
  uint16_t btn_s;
  uint16_t btn_m;
  uint16_t btn_l;
  }msg;


  /* struct collect_neighbor *n; */
  uint16_t parent_etx;
  uint16_t rtmetric;
  uint16_t num_neighbors;
  // int16_t battery =0;
  int16_t parent_rssi =0;

  rpl_parent_t *preferred_parent;
  linkaddr_t parent;
  rpl_dag_t *dag;
  // static uint16_t count=0;
  // char string[20];
  // int temp_value;
  // int ain0_value, ain1_value;

  memset(&msg, 0, sizeof(msg));

  linkaddr_copy(&parent, &linkaddr_null);
  parent_etx = 0;

  /* Let's suppose we have only one instance */
  dag = rpl_get_any_dag();
  if(dag != NULL)
  {
    preferred_parent = dag->preferred_parent;
    if(preferred_parent != NULL) {
      uip_ds6_nbr_t *nbr;
      nbr = uip_ds6_nbr_lookup(rpl_parent_get_ipaddr(preferred_parent));
      if(nbr != NULL) {
         //Use parts of the IPv6 address as the parent address, in reversed byte order. 
        parent.u8[LINKADDR_SIZE - 1] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 2];
        parent.u8[LINKADDR_SIZE - 2] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 1];
        // parent_etx = rpl_get_parent_rank((uip_lladdr_t *) uip_ds6_nbr_get_ll(nbr)) / 2;
        parent_etx = rpl_neighbor_get_from_ipaddr(rpl_parent_get_ipaddr(preferred_parent))->rank;
      }
      const struct link_stats *stats= rpl_neighbor_get_link_stats(preferred_parent);
      parent_rssi=stats->rssi;
    }
    rtmetric = dag->rank;
    // LOG_INFO("rtmetric'%u' \n", rtmetric);
    // beacon_interval = (uint16_t) ((2L << dag->instance->dio_intcurrent) / 1000);
    num_neighbors = uip_ds6_nbr_num();
    // LOG_INFO("num_neighbors'%u' \n", num_neighbors);
  } else {
    rtmetric = 0;
    // beacon_interval = 0;
    num_neighbors = 0;
  }

    memcpy(&msg.parent, &parent.u8[LINKADDR_SIZE - 2], 2);
    seqno++;
    if(seqno >=65535) {
      /* Wrap to 128 to identify restarts */
      seqno = 1;
    }
    msg.seqno = seqno;

    msg.parent_etx = parent_etx;
    msg.current_rtmetric = rtmetric;
    msg.num_neighbors = num_neighbors;
    msg.parent_rssi = parent_rssi;
    msg.battery = tempReg[22].HI<<8|tempReg[22].LO;
    msg.ext_tempature_value = tempReg[18].HI<<8|tempReg[18].LO;
    msg.int_tempature_value = (uint16_t)cc2538_temp_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED);
    msg.ext_tempature_point = tempReg[19].HI<<8|tempReg[19].LO;
    msg.btn_s = tempReg[29].HI<<8|tempReg[29].LO;
    msg.btn_m = tempReg[30].HI<<8|tempReg[30].LO;
    msg.btn_l = tempReg[31].HI<<8|tempReg[31].LO;

    // LOG_INFO("parent'%x' \n", msg.parent);
    // LOG_INFO("parent_etx'%u' \n", msg.parent_etx);
    // LOG_INFO("current_rtmetric'%u' \n", msg.current_rtmetric);
    // LOG_INFO("num_neighbors'%u' \n", msg.num_neighbors);
    LOG_INFO("battery'%d' \n", msg.battery);
    // LOG_INFO("parent_rssi'%d' \n\n", parent_rssi);
    LOG_INFO("ext_tempature_point'%d' \n\n", msg.ext_tempature_point);
    LOG_INFO("btn_s'%d' \n", msg.btn_s);
    LOG_INFO("btn_m'%d' \n", msg.btn_m);
    LOG_INFO("btn_l'%d' \n", msg.btn_l);

    simple_udp_sendto(&udp_conn, &msg, sizeof(msg), &dest_ipaddr);
    leds_blink();
}
/*---------------------------------------------------------------------------*/
void ResponseReadPacket(unsigned char* packet)
{
  int16_t address=modbus_get_int16(packet[2], packet[3]);
  int16_t len=modbus_get_int16(packet[4], packet[5]);
  
  unsigned char respPacket[len*2+3];
  memset(&respPacket, 0, sizeof(respPacket));

  // LOG_INFO("ask adress%d len %d\n", address, len);

  respPacket[0] = packet[0];  //address
  respPacket[1] = packet[1];  //function
  respPacket[2] = len*2;      //bytecount
  for(int i =0; i<len; i++)
  {
    respPacket[i*2+3] = tempReg[address+i].HI;
    respPacket[i*2+4] = tempReg[address+i].LO;
  }

  // LOG_INFO("bytecount %d %d %d\n", len, respPacket[2], sizeof(respPacket));
  // for(int i = 0; i < sizeof(respPacket); i++) {
  //   LOG_INFO("%02x", respPacket[i]);
  // }
  // LOG_INFO("\n\r");

  int hasSend= modbus_send_response_packet(respPacket, len*2+3);
  
  if(hasSend!=-1)
  {
    LOG_INFO("send Response success!\n");
  }
}
/*---------------------------------------------------------------------------*/
void ResponseWritePacket(unsigned char* packet)
{
  unsigned char respPacket[6];
  memset(&respPacket, 0, sizeof(respPacket));

  int16_t address=modbus_get_int16(packet[2], packet[3]);

  // LOG_INFO("BEFORE temp_HI: %02x temp_LO: %02x\n", tempReg[address].HI, tempReg[address].LO);

  respPacket[0] = packet[0];  //address
  respPacket[1] = packet[1];  //function
  respPacket[2] = packet[2];  //startAddressHI
  respPacket[3] = packet[3];  //startAddressLO
  tempReg[address].HI = packet[4];
  tempReg[address].LO = packet[5];
  respPacket[4] = tempReg[address].HI;     //dataHI
  respPacket[5] = tempReg[address].LO;     //dataLO

  // LOG_INFO("After temp_HI: %02x temp_LO: %02x\n", tempReg[address].HI, tempReg[address].LO);


  // for(int i = 0; i < sizeof(respPacket); i++) {
  //   printf("%02x", respPacket[i]);
  // }
  // printf("\n\r");

  int hasSend= modbus_send_response_packet(respPacket, 6);
  
  if(hasSend!=-1)
  {
    // LOG_INFO("send Response success!\n");
  }

}
/*---------------------------------------------------------------------------*/
void sendModbusRespPacket(unsigned char* packet)
{

  if(packet[1]==MODBUS_RD_HOLD_REG)
  {
    ResponseReadPacket(packet);
  }
  else if(packet[1]==MODBUS_WR_SINGLE_REG)
  {
    ResponseWritePacket(packet);
  }

}

/*---------------------------------------------------------------------------*/
void readDataFromModbus()
{
  unsigned char data[64];
  int datalen;
  int index=0;
  datalen = modbus_read_data(data);
  unsigned char packet[8];
  st_modbusExceptionCode exceptionCode;
  int hasErr;

  if(datalen <= 0) {
    // LOG_INFO("RX Error...length <= 0\n"); 
  }
  else{

    // LOG_INFO("Read data (%d):", datalen);
    while(index < datalen/8)
    {
      // for(int i = 0; i < 8; i++) {
      //   LOG_INFO("%02x", data[i+index*8]);
      // }
      // LOG_INFO("\n\r");

      packet[0] = data[0+index*8];           //address
      packet[1] = data[1+index*8];          //function
      packet[2] = data[2+index*8];    //startAddressHI
      packet[3] = data[3+index*8];    //startAddressLO
      packet[4] = data[4+index*8];           //countHI
      packet[5] = data[5+index*8];           //countLO
      packet[6] = data[6+index*8];             //crcHI
      packet[7] = data[7+index*8];             //crcLO
      hasErr = modbus_error_check(packet, 3, &exceptionCode, 1);

      if(hasErr==-1)
      {
        LOG_INFO("CRC error\n");
      }
      else{
        sendModbusRespPacket(packet);
      }
      index++;
    }
    clean_rx_data();
  }
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer periodic_timer, send_timer;
  // static uint16_t count=0;
  sendRate = tempReg[10].HI<<8|tempReg[10].LO;
  // static char str[32];
  // uip_ipaddr_t dest_ipaddr;
  int is_coordinator;

  PROCESS_BEGIN();

  is_coordinator = 0;
  if(is_coordinator) {
    NETSTACK_ROUTING.root_start();
  }

  NETSTACK_MAC.on();
  leds_on(LEDS_RED);

  modbus_init();

  /* Initialize UDP connection */
  simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL,
                      UDP_SERVER_PORT, udp_rx_callback);

  etimer_set(&periodic_timer, POLLING_INTERVAL);
  etimer_set(&send_timer, random_rand() % (sendRate*CLOCK_SECOND));
  while(1) {

    PROCESS_WAIT_EVENT(); /* Same thing as PROCESS_YIELD */
    if(etimer_expired(&periodic_timer)) {
      /* Do the work here and restart timer to get it periodic !!! */
      // printf("etimer expired.\n");
      readDataFromModbus(); 
      etimer_restart(&periodic_timer);
    }
    else if(etimer_expired(&send_timer))
    {
      if(NETSTACK_ROUTING.node_is_reachable() && NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr)) {
        /* Send to DAG root */
        // LOG_INFO("Sending request %u to ", count);
        // LOG_INFO_6ADDR(&dest_ipaddr);
        // LOG_INFO_("\n");
        tempReg[12].LO = 0x01; //connected 
        // LOG_INFO("Connected_HI: %02x Connected_LO: %02x\n", tempReg[12].HI, tempReg[12].LO);
        leds_off(LEDS_RED);
        // snprintf(str, sizeof(str), "hello %d", count);
        // simple_udp_sendto(&udp_conn, str, strlen(str), &dest_ipaddr);
        collect_common_send();
      } else {
        leds_on(LEDS_RED);
        tempReg[12].LO = 0x00; //connected 
        // LOG_INFO("Connected_HI: %02x Connected_LO: %02x\n", tempReg[12].HI, tempReg[12].LO);
        LOG_INFO("Not reachable yet\n");
      }
      etimer_set(&send_timer, (sendRate*CLOCK_SECOND) - CLOCK_SECOND + (random_rand() % (2 * CLOCK_SECOND)));
    }

    // PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
    // readDataFromModbus();
    // count++;
    // if(count==sendRate){
    //   if(NETSTACK_ROUTING.node_is_reachable() && NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr)) {
    //     /* Send to DAG root */
    //     // LOG_INFO("Sending request %u to ", count);
    //     // LOG_INFO_6ADDR(&dest_ipaddr);
    //     // LOG_INFO_("\n");
    //     leds_off(LEDS_RED);
    //     // snprintf(str, sizeof(str), "hello %d", count);
    //     // simple_udp_sendto(&udp_conn, str, strlen(str), &dest_ipaddr);
    //     collect_common_send();
    //   } else {
    //     leds_on(LEDS_RED);
    //     LOG_INFO("Not reachable yet\n");
    //   }
    //   count=0;
    // }

    // /* Add some jitter */
    // // etimer_set(&periodic_timer, SEND_INTERVAL
    // //   - CLOCK_SECOND + (random_rand() % (2 * CLOCK_SECOND)));
    // etimer_set(&periodic_timer, POLLING_INTERVAL);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
