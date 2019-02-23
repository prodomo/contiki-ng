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
#include "command-type.h"

#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

// #define DEBUG DEBUG_PRINT
// #include "net/ipv6/uip-debug.h"

#define WITH_SERVER_REPLY  1
#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

struct simple_udp_connection udp_conn;

#define START_INTERVAL		(15 * CLOCK_SECOND)
#define SEND_INTERVAL		  (30 * CLOCK_SECOND)

uip_ipaddr_t dest_ipaddr;

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client");
AUTOSTART_PROCESSES(&udp_client_process);
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
  printf("ack: %u %u %u\n", ack.command_type, ack.command_id, ack.is_received);

  printf("send ack\n");
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
  printf("sizeof(msg) %d\n", sizeof(msg));

  printf("--------------------recv data-----------------\n");
  printf("uip_datalen %u\n",datalen);

  memcpy(&msg.commandId, data, sizeof(uint16_t));
  data+=sizeof(uint16_t);
  memcpy(&msg.commandType, data, sizeof(uint16_t));
  data+=sizeof(uint16_t);
  printf("msg.commandType %u\n", msg.commandType);
  printf("msg.commandId %u\n", msg.commandId);

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

  collect_ack_send(sender_addr, msg.commandId);
  // LOG_INFO("Received response '%.*s' from ", datalen, data);
  // LOG_INFO_6ADDR(sender_addr);
#if LLSEC802154_CONF_ENABLED
  LOG_INFO_(" LLSEC LV:%d", uipbuf_get_attr(UIPBUF_ATTR_LLSEC_LEVEL));
#endif
  LOG_INFO_("\n");

}
/*---------------------------------------------------------------------------*/
void
collect_common_send(void)
{
  static uint8_t seqno;

  struct collect_view_data_msg {
  uint16_t len;
  uint16_t clock;
  uint16_t timesynch_time;
  uint16_t cpu;
  uint16_t lpm;
  uint16_t transmit;
  uint16_t listen;
  uint16_t parent;
  uint16_t parent_etx;
  uint16_t current_rtmetric;
  uint16_t num_neighbors;
  uint16_t beacon_interval;

  uint16_t sensors[10];
};

  struct {
    uint16_t seqno;
    struct collect_view_data_msg msg;
  } msg;
  /* struct collect_neighbor *n; */
  uint16_t parent_etx;
  uint16_t rtmetric;
  uint16_t num_neighbors;
  uint16_t beacon_interval;
  rpl_parent_t *preferred_parent;
  linkaddr_t parent;
  rpl_dag_t *dag;
  // static uint16_t count=0;
  // char string[20];
  // int temp_value;
  // int ain0_value, ain1_value;

  memset(&msg, 0, sizeof(msg));
  seqno++;
  if(seqno == 0) {
    /* Wrap to 128 to identify restarts */
    seqno = 128;
  }
  msg.seqno = seqno;

  linkaddr_copy(&parent, &linkaddr_null);
  parent_etx = 0;

  /* Let's suppose we have only one instance */
  dag = rpl_get_any_dag();
  if(dag != NULL) {
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
    }
    rtmetric = dag->rank;
    // beacon_interval = (uint16_t) ((2L << dag->instance->dio_intcurrent) / 1000);
    num_neighbors = uip_ds6_nbr_num();
  } else {
    rtmetric = 0;
    beacon_interval = 0;
    num_neighbors = 0;
  }
  // memcpy(msg.msg.parent, parent->u8[LINKADDR_SIZE - 2], 2);
  msg.msg.parent_etx = parent_etx;
  msg.msg.current_rtmetric = rtmetric;
  msg.msg.num_neighbors = num_neighbors;
  msg.msg.beacon_interval = beacon_interval;
  LOG_INFO("parent_etx'%u' \n", msg.msg.parent_etx);
  LOG_INFO("current_rtmetric'%u' \n", msg.msg.current_rtmetric);
  LOG_INFO("num_neighbors'%u' \n", msg.msg.num_neighbors);
  LOG_INFO("beacon_interval'%u' \n", msg.msg.beacon_interval);
  simple_udp_sendto(&udp_conn, &msg, sizeof(msg), &dest_ipaddr);
}

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

  /* Initialize UDP connection */
  simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL,
                      UDP_SERVER_PORT, udp_rx_callback);

  etimer_set(&periodic_timer, random_rand() % SEND_INTERVAL);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));

    if(NETSTACK_ROUTING.node_is_reachable() && NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr)) {
      /* Send to DAG root */
      LOG_INFO("Sending request %u to ", count);
      LOG_INFO_6ADDR(&dest_ipaddr);
      LOG_INFO_("\n");
      // snprintf(str, sizeof(str), "hello %d", count);
      // simple_udp_sendto(&udp_conn, str, strlen(str), &dest_ipaddr);
      collect_common_send();
      count++;
    } else {
      LOG_INFO("Not reachable yet\n");
    }

    /* Add some jitter */
    etimer_set(&periodic_timer, SEND_INTERVAL
      - CLOCK_SECOND + (random_rand() % (2 * CLOCK_SECOND)));
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
