/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
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

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "dev/uart.h"

#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
unsigned int
uart1_send_bytes(const unsigned char *s, unsigned int len)
{
  unsigned int i = 0;

  //while(s && *s != 0) {
  while(1) {
    if(i >= len) {
      break;
    }
    uart_write_byte(1, *s++);
    i++;
  }
  return i;
}
/*---------------------------------------------------------------------------*/

void s7s_uart1_init(void){
  uint8_t cmd_reset[1]={0x81};
  uint8_t cmd_br9600[2]={0x7f, 0x02};
  uart1_send_bytes((uint8_t *)cmd_reset, sizeof(cmd_reset));
  uart1_send_bytes((uint8_t *)cmd_br9600, sizeof(cmd_br9600));
}

void s7s_uart1_display(int digit4_point1){
  int j;
  uint8_t cmd_4digit[9]={0x79, 0x76, 0x00, 0x31, 0x35, 0x32, 0x33, 0x77, 0x04};
  uint8_t cmd_3digit[8]={0x79, 0x76, 0x01, 0x31, 0x35, 0x32, 0x77, 0x04};
  uint8_t cmd_2digit[7]={0x79, 0x76, 0x02, 0x32, 0x33, 0x77, 0x04};
//  uint8_t cmd_1digit[6]={0x79, 0x76, 0x03, 0x33, 0x77, 0x04};

    if (digit4_point1 > 1000){
      for (j=4;j>0;j--){
        cmd_4digit[2+j] = digit4_point1%10 + 0x30;
        digit4_point1 = digit4_point1/10;
      } 
      uart1_send_bytes((uint8_t *)cmd_4digit, sizeof(cmd_4digit));
    }else if (digit4_point1 > 100){
      for (j=3;j>0;j--){
        cmd_3digit[2+j] = digit4_point1%10 + 0x30;
        digit4_point1 = digit4_point1/10;
      }  
      uart1_send_bytes((uint8_t *)cmd_3digit, sizeof(cmd_3digit));
    }else if (digit4_point1 > 10){
      for (j=2;j>0;j--){
        cmd_2digit[2+j] = digit4_point1%10 + 0x30;
        digit4_point1 = digit4_point1/10;
      }  
      uart1_send_bytes((uint8_t *)cmd_2digit, sizeof(cmd_2digit));
    }else {
      cmd_2digit[3] = 0x30;
      for (j=1;j>0;j--){
        cmd_2digit[3+j] = digit4_point1%10 + 0x30;
        digit4_point1 = digit4_point1/10;
      }  
      uart1_send_bytes((uint8_t *)cmd_2digit, sizeof(cmd_2digit));
    }
}

void s7s_uart1_off(void){
  uint8_t cmd[1]={0x76};
  uart1_send_bytes((uint8_t *)cmd, sizeof(cmd));
}

/*---------------------------------------------------------------------------*/
