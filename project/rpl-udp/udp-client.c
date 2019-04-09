#include "contiki.h"
#include "net/routing/routing.h"
#include "random.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"

#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

#define WITH_SERVER_REPLY  1
#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

static struct simple_udp_connection udp_conn;

#define START_INTERVAL		(15 * CLOCK_SECOND)
#define SEND_INTERVAL		  (10 * CLOCK_SECOND)

static struct simple_udp_connection udp_conn;
uip_ipaddr_t dest_ipaddr;

static int flag=0;
static int statusCategory=0;
static int status = 0;

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/
void
collect_common_send(void)
{
  static uint16_t seqno;
  struct msg{
    int16_t seqno;
    int16_t statusCategory;
    int16_t status;    
  };
  struct msg msg;
  memset(&msg, 0, sizeof(msg));

  if(seqno>=32766)
  {
    seqno=0;
  }
  seqno++;
  msg.seqno=seqno;

  if( (seqno%10)==1)
  {
    LOG_INFO("flag+1 change status %d", flag);
    flag++;

    if(flag==1)
    {
      statusCategory=10;
      status=11;
    }
    else if(flag==2)
    {
      statusCategory=20;
      status=21;
    }
    else if(flag==3)
    {
      statusCategory=30;
      status=31;
    }
    else if(flag==4)
    {
      statusCategory=30;
      status=32;
    }
    else if(flag==5)
    {
      statusCategory=30;
      status=33;
    }
    else if(flag==6)
    {
      statusCategory=40;
      status=41;
    }
    else if(flag==7)
    {
      statusCategory=50;
      status=51;
    }
    else if(flag==8)
    {
      statusCategory=0;
      status=-1;
    }
    else if(flag==9)
    {
      statusCategory=0;
      status=-2;
    }
    else if(flag==10)
    {
      statusCategory=0;
      status=-3;
      flag=0;
    }
    else
    {

    }
  }
  msg.statusCategory = statusCategory;
  msg.status = status;

  // msg.statusCategory = -1;

  LOG_INFO("Sending packet %u to root! ", seqno);

 
  simple_udp_sendto(&udp_conn, &msg, sizeof(msg), &dest_ipaddr);
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
    LOG_INFO("packet timer timeout %u , %lu !\n", count, RTIMER_NOW());
    count++;

    if(count >=65534)
      count=1;

    if(NETSTACK_ROUTING.node_is_reachable() && NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr)) {
      /* Send to DAG root */
      // LOG_INFO("Sending request %u to ", count);
      // LOG_INFO_6ADDR(&dest_ipaddr);
      // LOG_INFO_("\n");
      // snprintf(str, sizeof(str), "hello %d", count);
      // simple_udp_sendto(&udp_conn, str, strlen(str), &dest_ipaddr);
      collect_common_send();
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
