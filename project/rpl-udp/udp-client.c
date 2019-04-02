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
#include "dev/adc-sensors.h"
#include "dev/servo.h"
#include "net/link-stats.h"
#include "dev/cc2538-sensors.h"

#include "dev/uart.h"
#include "lib/uart1-s7s.h"

#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

// #define DEBUG DEBUG_PRINT
// #include "net/ipv6/uip-debug.h"

#define WITH_SERVER_REPLY  1
#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

struct simple_udp_connection udp_conn;


int user_control_send_period=15;
int argent_period=3;
int send_period = 15;

int temperature_threshold = 50;
//temperature ax+b=y
int temperature_a = 26; 
int temperature_b = 259400;

int battery_threshold = 40;

int urgent_value_on =0;
int urgent_sound_on =0;




#define START_INTERVAL		(send_period/2 * CLOCK_SECOND)
#define SEND_INTERVAL		  (send_period * CLOCK_SECOND)

#define ADC_PIN 5
#define PT100_PIN 4

uip_ipaddr_t dest_ipaddr;


/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client");
AUTOSTART_PROCESSES(&udp_client_process);

//PROCESS(s7s_drive_process, "Driving 7-seg display");
//AUTOSTART_PROCESSES(&s7s_drive_process);
/*---------------------------------------------------------------------------*/
void batt_init()
{
  adc_sensors.configure(ANALOG_AAC_SENSOR, ADC_PIN);
  adc_sensors.configure(ANALOG_VAC_SENSOR, PT100_PIN);
}
/*---------------------------------------------------------------------------*/
int get_batt()
{
  int v = adc_sensors.value(ANALOG_AAC_SENSOR);
  // return v;
  if (v < 32000)
    return -1;
  int r = (v-32400)/100;

  if (r <= 1) {
    return 1;
  }
  else if(r > 100)
  {
    return 100;
  }

  return r;
}
/*---------------------------------------------------------------------------*/
int get_tempature()
{
  int v = adc_sensors.value(ANALOG_VAC_SENSOR);
  // return v;
  int r = (v*temperature_a-temperature_b)/100;


  return r;
}
/*---------------------------------------------------------------------------*/
int
change_TemperatureValue_to_RealValue(int value)
{
  // 13428=50c, 23428=120c

  int result = 0;

  value = value*100;

  if(value>=temperature_b)
  {
    result= (value-temperature_b)/temperature_a;
    printf("%d to result: %d \n",value, result);
    return result;
  }
  else
  {
    result = 0-((temperature_b-value)/temperature_a);
    printf("%d to result: %d \n",value, result);
    return result;
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
  printf("ack: %u %u %u\n", ack.command_type, ack.command_id, ack.is_received);

  printf("send ack\n");
  simple_udp_sendto(&udp_conn, &ack, sizeof(ack), sender_addr);
}
/*---------------------------------------------------------------------------*/
void
set_urgent_sound_onoff(int is_on)
{
  static uint8_t deg = 180;
  static uint8_t sound_pin=7;
  static uint8_t light_pin=6;
  static uint8_t freq =5; //5Hz

  if(is_on==1)
  {
    servo_position_na(SERVO_CHANNEL_7, GPIO_A_NUM, sound_pin, deg, freq);
    servo_position_na(SERVO_CHANNEL_6, GPIO_A_NUM, light_pin, deg, freq);
  }
  else if(is_on==0)
  {
      servo_stop(SERVO_CHANNEL_7, GPIO_A_NUM, sound_pin);
      servo_stop(SERVO_CHANNEL_6, GPIO_A_NUM, light_pin);
  }
  else
  {

  }
}
/*---------------------------------------------------------------------------*/
void setting_value(struct setting_msg msg)
{

  if(msg.setting_type == SET_TYPE_RATE && msg.sensor_tittle == SNR_TLE_DEFAULT)
  {
    send_period = msg.value;
    printf("changing sending rate %u\n", msg.value);
  }
  else if(msg.setting_type == SET_TYPE_THRESHOLD)
  {
    printf("sensor_tittle %d\n", msg.sensor_tittle);
    switch(msg.sensor_tittle){
      case SNR_TLE_DEFAULT:
        break;
      
      case SNR_TLE_TEMPERATURE:
        temperature_threshold = msg.value;
        printf("changing temperature threshold: %d\n",temperature_threshold);
        break;

      case SNR_TLE_TEMPERATURE_A:
        temperature_a = msg.value;
        printf("changing temperature_a: %d\n", temperature_a);
        break;
      
      case SNR_TLE_TEMPERATURE_B:
        temperature_b = msg.value;
        printf("changing temperature_b: %d\n", temperature_b);
        break;

      case SNR_TLE_BATTERY:
        battery_threshold = msg.value;
        printf("changing battery_threshold: %d\n", battery_threshold);
        break;

      // case SNR_TLE_ELE_CURRENT:
      //   printf("changing electric current threshold\n");
      //   break;
      
      // case SNR_TLE_ROTAT_SPEED:
      //   printf("changing rotation speed threshold\n");
      //   break;
      case SNR_TLE_URGENT_VALUE:
        urgent_value_on = msg.value;
        printf("changing urgent_value_on: %d\n", urgent_value_on);
        break;

      case SNR_TLE_URGENT_SOUND:
        urgent_sound_on = msg.value;
        set_urgent_sound_onoff(urgent_sound_on);
        printf("changing urgent_sound_on: %d\n", urgent_sound_on);
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
  switch(msg.commandType)
  {
    case CMD_TYPE_CONF:
      printf("should send conf\n");
      break;
      
    case CMD_TYPE_SET:
      printf("should set value\n");
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
  int16_t battery;
  int16_t parent_rssi =0;

  rpl_parent_t *preferred_parent;
  linkaddr_t parent;
  rpl_dag_t *dag;
  // static uint16_t count=0;
  // char string[20];
  int int_t;
  int ext_t, ext_t_s7s_test;


  memset(&msg, 0, sizeof(msg));
  seqno++;
  if(seqno >=65535) {
    /* Wrap to 128 to identify restarts */
    seqno = 1;
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
        parent_etx = rpl_neighbor_get_from_ipaddr(rpl_parent_get_ipaddr(preferred_parent))->rank;
        const struct link_stats *stats= rpl_neighbor_get_link_stats(preferred_parent);
        parent_rssi=stats->rssi;

        //parent_rssi = rpl_get_parent_link_stats(preferred_parent)->rssi;
      }
    }
    rtmetric = dag->rank;
    num_neighbors = uip_ds6_nbr_num();
  } else {
    rtmetric = 0;
    num_neighbors = 0;
  }
  memcpy(&msg.msg.parent, &parent.u8[LINKADDR_SIZE - 2], 2);
  battery = get_batt();
  
  if(battery < 0)
  {
    msg.msg.battery=0;
  }

  msg.msg.parent_etx = parent_etx;
  msg.msg.current_rtmetric = rtmetric;
  msg.msg.num_neighbors = num_neighbors;
  msg.msg.parent_rssi = (uint16_t)parent_rssi;
  msg.msg.battery = battery;
  msg.msg.ext_tempature_value = (uint16_t)get_tempature();
  msg.msg.int_tempature_value = (uint16_t)cc2538_temp_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED);

  int_t = cc2538_temp_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED)/1000;
  // int_t_s7s_test = cc2538_temp_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED)/100;
  ext_t = get_tempature()/100;
  ext_t_s7s_test = ext_t*10;

  if(ext_t < 0)
  {
    msg.msg.ext_tempature_value=0;
    ext_t_s7s_test=0;
  }


  LOG_INFO("parent'%x' \n", msg.msg.parent);
  LOG_INFO("parent_etx'%u' \n", msg.msg.parent_etx);
  LOG_INFO("current_rtmetric'%u' \n", msg.msg.current_rtmetric);
  LOG_INFO("num_neighbors'%u' \n", msg.msg.num_neighbors);
  LOG_INFO("battery'%d' \n", battery);
  LOG_INFO("parent_rssi'%d' \n", parent_rssi);
  LOG_INFO("ext_tempature_value'%d' \n", ext_t);
  LOG_INFO("int_tempature_value'%d' \n", int_t);

  LOG_INFO("s7s display value='%d/10' \n", ext_t_s7s_test);
  s7s_uart1_display(ext_t_s7s_test);

  if(ext_t>= temperature_threshold && urgent_sound_on==0){
    printf("temperature too high ext '%d'\n", get_tempature()/100);
    printf("temperature_threshold'%d'\n", temperature_threshold);
    send_period=argent_period;
    urgent_sound_on = 1;
    set_urgent_sound_onoff(urgent_sound_on);
  }
  else if(int_t >= temperature_threshold && urgent_sound_on==0){
    printf("temperature too high int \n");
    printf("temperature_threshold'%d'\n", temperature_threshold);
    send_period=argent_period;
    urgent_sound_on = 1;
    set_urgent_sound_onoff(urgent_sound_on);
  }
  else if(urgent_sound_on==1 && int_t < temperature_threshold && ext_t< temperature_threshold)
  {
    send_period=user_control_send_period;
    urgent_sound_on = 0;
    set_urgent_sound_onoff(urgent_sound_on);
    printf("turn back to '%d'\n", send_period);
  }
  else{

  }

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

  batt_init();
  send_period = user_control_send_period;

  // setup s7s
  s7s_uart1_init();
  LOG_INFO("Start s7s driving !\n");
  s7s_uart1_display(1234);

  etimer_set(&periodic_timer, random_rand() % SEND_INTERVAL);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));

    // printf("AC Amps = %d mA\n", adc_sensors.value(ANALOG_AAC_SENSOR));
    // printf("AC voltage = %d V\n", adc_sensors.value(ANALOG_VAC_SENSOR));
    // // printf("battery = %d\n", get_batt());

    // printf("temperature= %d\n", get_tempature());
    // printf("cc2538 temp= %d\n", cc2538_temp_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED));
    LOG_INFO("{asn %02x.%08lx} \n", tsch_current_asn.ms1b, tsch_current_asn.ls4b);
    LOG_INFO("packet timer timeout %u , %lu !\n", count, RTIMER_NOW());

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
      s7s_uart1_display(1234);
    }


    // /* Add some jitter */
    // etimer_set(&periodic_timer, SEND_INTERVAL
    //   - CLOCK_SECOND + (random_rand() % (2 * CLOCK_SECOND)));
    etimer_reset(&periodic_timer);
  }

  PROCESS_END();
}

#if 0
PROCESS_THREAD(s7s_drive_process, ev, data)
{
  static struct etimer s7s_timer;
//  uint8_t cmd[9]={0x79, 0x76, 0x00, 0x30, 0x35, 0x32, 0x33, 0x77, 0x04};
//  uint8_t send_data[20];
  PROCESS_BEGIN();
//  int  tmp_int, j;
  static int ii=9998;
  /* Setup a periodic timer that expires after 10 seconds. */
  etimer_set(&s7s_timer, CLOCK_SECOND/16);

  s7s_uart1_init();

  LOG_INFO("Start s7s driving !\n");

  while(1) {
    //printf("Hello, world\n");

    s7s_uart1_display(ii);

    ii++;
    if (ii>9999)
      ii = 0;
    //memset(send_data, sizeof(send_data), 0);
    /* Wait for the periodic timer to expire and then restart the timer. */
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&s7s_timer));
    etimer_reset(&s7s_timer);
  }

  PROCESS_END();
}
#endif

/*---------------------------------------------------------------------------*/
