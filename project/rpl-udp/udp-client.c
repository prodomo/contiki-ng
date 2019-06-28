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

#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

// #define DEBUG DEBUG_PRINT
// #include "net/ipv6/uip-debug.h"

#define WITH_SERVER_REPLY  1
#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

static struct simple_udp_connection udp_conn;

#define START_INTERVAL		(15 * CLOCK_SECOND)
#define SEND_INTERVAL		  (5 * CLOCK_SECOND)

uint8_t temp_HI =0;
uint8_t temp_LO =0;

uip_ipaddr_t dest_ipaddr;

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
static void
udp_rx_callback(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
  LOG_INFO("Received response '%.*s' from ", datalen, (char *) data);
  LOG_INFO_6ADDR(sender_addr);
#if LLSEC802154_CONF_ENABLED
  LOG_INFO_(" LLSEC LV:%d", uipbuf_get_attr(UIPBUF_ATTR_LLSEC_LEVEL));
#endif
  LOG_INFO_("\n");

}
/*---------------------------------------------------------------------------*/
void
collect_common_send(void)
{
  static uint16_t seqno;
  struct collect_data_msg {
  uint16_t parent;
  uint16_t parent_etx;
  uint16_t current_rtmetric;
  uint16_t num_neighbors;
  uint16_t parent_rssi;
  uint16_t temp_value;
  uint16_t ext_tempature_value;  // =real value*100
  uint16_t int_tempature_value;  // =real value*1000
  uint16_t battery;
  };

  struct {
    uint16_t seqno;
    struct collect_data_msg msg;
  } msg;
  /* struct collect_neighbor *n; */
  uint16_t parent_etx;
  uint16_t rtmetric;
  uint16_t num_neighbors;
  int16_t battery =0;
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
    LOG_INFO("rtmetric'%u' \n", rtmetric);
    // beacon_interval = (uint16_t) ((2L << dag->instance->dio_intcurrent) / 1000);
    num_neighbors = uip_ds6_nbr_num();
    LOG_INFO("num_neighbors'%u' \n", num_neighbors);
  } else {
    rtmetric = 0;
    // beacon_interval = 0;
    num_neighbors = 0;
  }

    memcpy(&msg.msg.parent, &parent.u8[LINKADDR_SIZE - 2], 2);
    seqno++;
    if(seqno >=65535) {
      /* Wrap to 128 to identify restarts */
      seqno = 1;
    }
    msg.seqno = seqno;

    msg.msg.parent_etx = parent_etx;
    msg.msg.current_rtmetric = rtmetric;
    msg.msg.num_neighbors = num_neighbors;
    msg.msg.parent_rssi = parent_rssi;
    msg.msg.battery = 20;
    msg.msg.ext_tempature_value = 20;
    msg.msg.int_tempature_value = 20;

    printf("parent'%x' \n", msg.msg.parent);
    printf("parent_etx'%u' \n", msg.msg.parent_etx);
    printf("current_rtmetric'%u' \n", msg.msg.current_rtmetric);
    printf("num_neighbors'%u' \n", msg.msg.num_neighbors);
    printf("battery'%d' \n", battery);
    printf("parent_rssi'%d' \n\n", parent_rssi);
    simple_udp_sendto(&udp_conn, &msg, sizeof(msg), &dest_ipaddr);
    leds_blink();
}

/*---------------------------------------------------------------------------*/
void sendModbusRespPacket(unsigned char* packet)
{
  int16_t len=modbus_get_int16(packet[4], packet[5]);
  
  unsigned char respPacket[len*2+3];
  memset(&respPacket, 0, sizeof(respPacket));

  if(packet[1]==MODBUS_RD_HOLD_REG)
  {
    int16_t len=modbus_get_int16(packet[4], packet[5]);
    
    unsigned char respPacket[len*2+3];
    memset(&respPacket, 0, sizeof(respPacket));

    respPacket[0] = packet[0];  //address
    respPacket[1] = packet[1];  //function
    respPacket[2] = len*2;      //bytecount
    for(int i =0; i<len; i++)
    {
      respPacket[i*2+3] = 0;
      respPacket[i*2+4] = i;
    }

    printf("bytecount %d %d %d\n", len, respPacket[2], sizeof(respPacket));
    for(int i = 0; i < sizeof(respPacket); i++) {
      printf("%02x", respPacket[i]);
    }
    printf("\n\r");

    int hasSend= modbus_send_response_packet(respPacket, len*2+3);
    
    if(hasSend!=-1)
    {
      printf("send Response success!\n");
    }
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
    printf("RX Error...length <= 0\n"); 
  }
  else{

    printf("Read data (%d):", datalen);
    while(index < datalen/8)
    {
      for(int i = 0; i < 8; i++) {
        printf("%02x", data[i+index*8]);
      }
      printf("\n\r");

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
        printf("CRC error\n");
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
  static struct etimer periodic_timer;
  static unsigned count;
  // static char str[32];
  // uip_ipaddr_t dest_ipaddr;
  int is_coordinator;

  PROCESS_BEGIN();

  is_coordinator = 0;
  if(is_coordinator) {
    NETSTACK_ROUTING.root_start();
  }

  NETSTACK_MAC.on();

  modbus_init();

  /* Initialize UDP connection */
  simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL,
                      UDP_SERVER_PORT, udp_rx_callback);

  etimer_set(&periodic_timer, random_rand() % SEND_INTERVAL);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
    readDataFromModbus();

    if(NETSTACK_ROUTING.node_is_reachable() && NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr)) {
      /* Send to DAG root */
      LOG_INFO("Sending request %u to ", count);
      LOG_INFO_6ADDR(&dest_ipaddr);
      LOG_INFO_("\n");
      leds_off(LEDS_RED);
      // snprintf(str, sizeof(str), "hello %d", count);
      // simple_udp_sendto(&udp_conn, str, strlen(str), &dest_ipaddr);
      collect_common_send();
      count++;
    } else {
      leds_on(LEDS_RED);
      LOG_INFO("Not reachable yet\n");
    }

    /* Add some jitter */
    etimer_set(&periodic_timer, SEND_INTERVAL
      - CLOCK_SECOND + (random_rand() % (2 * CLOCK_SECOND)));
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
