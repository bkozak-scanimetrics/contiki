/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
 *         Implementation of the ContikiMAC power-saving radio duty cycling protocol
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 */

#include "contiki-conf.h"
#include "dev/leds.h"
#include "dev/radio.h"
#include "dev/watchdog.h"
#include "lib/random.h"
#include "net/mac/mac-sequence.h"
#include "net/mac/contikimac/contikimac.h"
#include "net/netstack.h"
#include "net/rime/rime.h"
#include "sys/compower.h"
#include "sys/pt.h"
#include "sys/rtimer.h"
#include "net/mac/contikimac/contikimac-txrx.h"
#include "net/mac/contikimac/contikimac_defs.h"

#include <string.h>

/* Are we currently receiving a burst? */
static int we_are_receiving_burst = 0;

#include <stdio.h>
static struct rtimer rt;
static struct pt pt;

static volatile uint8_t contikimac_is_on = 0;
static volatile uint8_t contikimac_keep_radio_on = 0;

static volatile unsigned char we_are_sending = 0;
static volatile unsigned char radio_is_on = 0;

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTDEBUG(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#define PRINTDEBUG(...)
#endif

#if CONTIKIMAC_CONF_COMPOWER
static struct compower_activity current_packet;
#endif /* CONTIKIMAC_CONF_COMPOWER */

#if WITH_PHASE_OPTIMIZATION

#include "net/mac/phase.h"

#endif /* WITH_PHASE_OPTIMIZATION */

#define DEFAULT_STREAM_TIME (4 * CYCLE_TIME)

#if CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT
static struct timer broadcast_rate_timer;
static int broadcast_rate_counter;
#endif /* CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT */

/*---------------------------------------------------------------------------*/
static int sendStrobes(
		struct cmTxInfo* info,rtimer_clock_t* en_time,int tx_len
	);
static void autoStrobesDone(struct cmTxInfo* inf,rtimer_clock_t eTime,int ret);
static void postStrobes(struct cmTxInfo* inf,rtimer_clock_t eTime,int ret);
static int waitOnACK(int seqno,uint8_t is_broadcast);
/*---------------------------------------------------------------------------*/
static void
on(void)
{
  if(contikimac_is_on && radio_is_on == 0) {
    radio_is_on = 1;
    NETSTACK_RADIO.on();
  }
}
/*---------------------------------------------------------------------------*/
static void
off(void)
{
  if(contikimac_is_on && radio_is_on != 0 &&
     contikimac_keep_radio_on == 0) {
    radio_is_on = 0;
    NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static volatile rtimer_clock_t cycle_start;
static char powercycle(struct rtimer *t, void *ptr);
static void
schedule_powercycle(struct rtimer *t, rtimer_clock_t time)
{
  int r;

  if(contikimac_is_on) {

    if(RTIMER_CLOCK_LT(RTIMER_TIME(t) + time, RTIMER_NOW() + 2)) {
      time = RTIMER_NOW() - RTIMER_TIME(t) + 2;
    }

    r = rtimer_set(t, RTIMER_TIME(t) + time, 1,
                   (void (*)(struct rtimer *, void *))powercycle, NULL);
    if(r != RTIMER_OK) {
      PRINTF("schedule_powercycle: could not set rtimer\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
schedule_powercycle_fixed(struct rtimer *t, rtimer_clock_t fixed_time)
{
  int r;

  if(contikimac_is_on) {

    if(RTIMER_CLOCK_LT(fixed_time, RTIMER_NOW() + 1)) {
      fixed_time = RTIMER_NOW() + 1;
    }

    r = rtimer_set(t, fixed_time, 1,
                   (void (*)(struct rtimer *, void *))powercycle, NULL);
    if(r != RTIMER_OK) {
      PRINTF("schedule_powercycle: could not set rtimer\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
powercycle_turn_radio_off(void)
{
#if CONTIKIMAC_CONF_COMPOWER
  uint8_t was_on = radio_is_on;
#endif /* CONTIKIMAC_CONF_COMPOWER */

  if(we_are_sending == 0 && we_are_receiving_burst == 0) {
    off();
#if CONTIKIMAC_CONF_COMPOWER
    if(was_on && !radio_is_on) {
      compower_accumulate(&compower_idle_activity);
    }
#endif /* CONTIKIMAC_CONF_COMPOWER */
  }
}
/*---------------------------------------------------------------------------*/
static void
powercycle_turn_radio_on(void)
{
  if(we_are_sending == 0 && we_are_receiving_burst == 0) {
    on();
  }
}
/*---------------------------------------------------------------------------*/
static char
powercycle(struct rtimer *t, void *ptr)
{
#if SYNC_CYCLE_STARTS
  static volatile rtimer_clock_t sync_cycle_start;
  static volatile uint8_t sync_cycle_phase;
#endif

  PT_BEGIN(&pt);

#if SYNC_CYCLE_STARTS
  sync_cycle_start = RTIMER_NOW();
#else
  cycle_start = RTIMER_NOW();
#endif

  while(1) {
    static uint8_t packet_seen;
    static uint8_t count;

#if SYNC_CYCLE_STARTS
    /* Compute cycle start when RTIMER_ARCH_SECOND is not a multiple
       of CHANNEL_CHECK_RATE */
    if(sync_cycle_phase++ == NETSTACK_RDC_CHANNEL_CHECK_RATE) {
      sync_cycle_phase = 0;
      sync_cycle_start += RTIMER_ARCH_SECOND;
      cycle_start = sync_cycle_start;
    } else {
#if (RTIMER_ARCH_SECOND * NETSTACK_RDC_CHANNEL_CHECK_RATE) > 65535
      cycle_start = sync_cycle_start + ((unsigned long)(sync_cycle_phase*RTIMER_ARCH_SECOND))/NETSTACK_RDC_CHANNEL_CHECK_RATE;
#else
      cycle_start = sync_cycle_start + (sync_cycle_phase*RTIMER_ARCH_SECOND)/NETSTACK_RDC_CHANNEL_CHECK_RATE;
#endif
    }
#else
    cycle_start += CYCLE_TIME;
#endif

    packet_seen = 0;

    for(count = 0; count < CCA_COUNT_MAX; ++count) {
      if(we_are_sending == 0 && we_are_receiving_burst == 0) {
        powercycle_turn_radio_on();
        /* Check if a packet is seen in the air. If so, we keep the
             radio on for a while (LISTEN_TIME_AFTER_PACKET_DETECTED) to
             be able to receive the packet. We also continuously check
             the radio medium to make sure that we wasn't woken up by a
             false positive: a spurious radio interference that was not
             caused by an incoming packet. */
        if(NETSTACK_RADIO.channel_clear() == 0) {
          packet_seen = 1;
          break;
        }
        powercycle_turn_radio_off();
      }
      schedule_powercycle_fixed(t, RTIMER_NOW() + CCA_SLEEP_TIME);
      PT_YIELD(&pt);
    }

    if(packet_seen) {
      static rtimer_clock_t start;
      static uint8_t silence_periods, periods;
      start = RTIMER_NOW();

      periods = silence_periods = 0;
      while(we_are_sending == 0 && radio_is_on &&
            RTIMER_CLOCK_LT(RTIMER_NOW(),
                            (start + LISTEN_TIME_AFTER_PACKET_DETECTED))) {

        /* Check for a number of consecutive periods of
             non-activity. If we see two such periods, we turn the
             radio off. Also, if a packet has been successfully
             received (as indicated by the
             NETSTACK_RADIO.pending_packet() function), we stop
             snooping. */
#if !RDC_CONF_HARDWARE_CSMA
       /* A cca cycle will disrupt rx on some radios, e.g. mc1322x, rf230 */
       /*TODO: Modify those drivers to just return the internal RSSI when already in rx mode */
        if(NETSTACK_RADIO.channel_clear()) {
          ++silence_periods;
        } else {
          silence_periods = 0;
        }
#endif

        ++periods;

        if(NETSTACK_RADIO.receiving_packet()) {
          silence_periods = 0;
        }
        if(silence_periods > MAX_SILENCE_PERIODS) {
          powercycle_turn_radio_off();
          break;
        }
        if(WITH_FAST_SLEEP &&
            periods > MAX_NONACTIVITY_PERIODS &&
            !(NETSTACK_RADIO.receiving_packet() ||
              NETSTACK_RADIO.pending_packet())) {
          powercycle_turn_radio_off();
          break;
        }
        if(NETSTACK_RADIO.pending_packet()) {
          break;
        }

        schedule_powercycle(t, CCA_CHECK_TIME + CCA_SLEEP_TIME);
        PT_YIELD(&pt);
      }
      if(radio_is_on) {
        if(!(NETSTACK_RADIO.receiving_packet() ||
             NETSTACK_RADIO.pending_packet()) ||
             !RTIMER_CLOCK_LT(RTIMER_NOW(),
                 (start + LISTEN_TIME_AFTER_PACKET_DETECTED))) {
          powercycle_turn_radio_off();
        }
      }
    }

    if(RTIMER_CLOCK_LT(RTIMER_NOW() - cycle_start, CYCLE_TIME - CHECK_TIME * 4)) {
      /* Schedule the next powercycle interrupt, or sleep the mcu
	 until then.  Sleeping will not exit from this interrupt, so
	 ensure an occasional wake cycle or foreground processing will
	 be blocked until a packet is detected */
#if RDC_CONF_MCU_SLEEP
      static uint8_t sleepcycle;
      if((sleepcycle++ < 16) && !we_are_sending && !radio_is_on) {
        rtimer_arch_sleep(CYCLE_TIME - (RTIMER_NOW() - cycle_start));
      } else {
        sleepcycle = 0;
        schedule_powercycle_fixed(t, CYCLE_TIME + cycle_start);
        PT_YIELD(&pt);
      }
#else
      schedule_powercycle_fixed(t, CYCLE_TIME + cycle_start);
      PT_YIELD(&pt);
#endif
    }
  }

  PT_END(&pt);
}
/*---------------------------------------------------------------------------*/
static int
broadcast_rate_drop(void)
{
#if CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT
  if(!timer_expired(&broadcast_rate_timer)) {
    broadcast_rate_counter++;
    if(broadcast_rate_counter < CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT) {
      return 0;
    } else {
      return 1;
    }
  } else {
    timer_set(&broadcast_rate_timer, CLOCK_SECOND);
    broadcast_rate_counter = 0;
    return 0;
  }
#else /* CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT */
  return 0;
#endif /* CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT */
}
/*---------------------------------------------------------------------------*/
static int
send_packet(struct cmTxInfo pInfo)
{
  rtimer_clock_t t0;
  rtimer_clock_t encounter_time = 0;
  uint8_t collisions = 0;
  unsigned short tx_len;
  int ret;


  if(CM_AUTO_STROBES && we_are_sending){
    /* Strobes are being sent in another thread of execution */
    return MAC_TX_COLLISION;
  }

  /* Exit if RDC and radio were explicitly turned off */
   if(!contikimac_is_on && !contikimac_keep_radio_on) {
    PRINTF("contikimac: radio is turned off\n");
    return MAC_TX_ERR_FATAL;
  }

  if(packetbuf_totlen() == 0) {
    PRINTF("contikimac: send_packet data len 0\n");
    return MAC_TX_ERR_FATAL;
  }

#if !NETSTACK_CONF_BRIDGE_MODE
  /* If NETSTACK_CONF_BRIDGE_MODE is set, assume PACKETBUF_ADDR_SENDER is already set. */
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);
#endif
  if(packetbuf_holds_broadcast()) {
    cmSetBroadcast(&pInfo);
    PRINTF("contikimac: send broadcast\n");

    if(broadcast_rate_drop()) {
      return MAC_TX_COLLISION;
    }
  } else {
#if NETSTACK_CONF_WITH_IPV6
    PRINTDEBUG("contikimac: send unicast to %02x%02x:%02x%02x:%02x%02x:%02x%02x\n",
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[1],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[2],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[3],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[4],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[5],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[6],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[7]);
#else /* NETSTACK_CONF_WITH_IPV6 */
    PRINTDEBUG("contikimac: send unicast to %u.%u\n",
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[1]);
#endif /* NETSTACK_CONF_WITH_IPV6 */
  }

  if(!packetbuf_attr(PACKETBUF_ATTR_IS_CREATED_AND_SECURED)) {
    packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
    if(NETSTACK_FRAMER.create_and_secure() < 0) {
      PRINTF("contikimac: framer failed\n");
      return MAC_TX_ERR_FATAL;
    }
  }

  tx_len = packetbuf_totlen();
  NETSTACK_RADIO.prepare(packetbuf_hdrptr(), tx_len);

  #if WITH_PHASE_OPTIMIZATION
  if( !cmNoPhaseDelay(&pInfo)) {
    ret = phase_wait(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                     CYCLE_TIME, GUARD_TIME,
                     pInfo.macCB, pInfo.macPtr, pInfo.buf_list);
    if(ret == PHASE_DEFERRED) {
      return MAC_TX_DEFERRED;
    }
    if(ret != PHASE_UNKNOWN) {
      cmSetRcvrKnown(&pInfo);
    }
  }
  #endif

  /* By setting we_are_sending to one, we ensure that the rtimer
     powercycle interrupt do not interfere with us sending the packet. */
  we_are_sending = 1;

  /* If we have a pending packet in the radio, we should not send now,
     because we will trash the received packet. Instead, we signal
     that we have a collision, which lets the packet be received. This
     packet will be retransmitted later by the MAC protocol
     instread. */
  if(NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet()) {
    we_are_sending = 0;
    PRINTF("contikimac: collision receiving %d, pending %d\n",
           NETSTACK_RADIO.receiving_packet(), NETSTACK_RADIO.pending_packet());
    return MAC_TX_COLLISION;
  }

  /* Switch off the radio to ensure that we didn't start sending while
     the radio was doing a channel check. */
  off();

  /* Send a train of strobes until the receiver answers with an ACK. */

  /* Set contikimac_is_on to one to allow the on() and off() functions
     to control the radio. We restore the old value of
     contikimac_is_on when we are done. */
  cmAssignBitWasOn(&pInfo,contikimac_is_on);
  contikimac_is_on = 1;

#if !RDC_CONF_HARDWARE_CSMA
    /* Check if there are any transmissions by others. */
    /* TODO: why does this give collisions before sending with the mc1322x? */
  if(!cmIsRcvrAwake(&pInfo)) {
    int i;
    for(i = 0; i < CCA_COUNT_MAX_TX; ++i) {
      t0 = RTIMER_NOW();
      on();
#if CCA_CHECK_TIME > 0
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + CCA_CHECK_TIME)) { }
#endif
      if(NETSTACK_RADIO.channel_clear() == 0) {
        collisions++;
        off();
        break;
      }
      off();
      t0 = RTIMER_NOW();
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + CCA_SLEEP_TIME)) { }
    }
  }

  if(collisions > 0) {
    we_are_sending = 0;
    off();
    PRINTF("contikimac: collisions before sending\n");
    contikimac_is_on = cmIsWasOn(&pInfo);
    return MAC_TX_COLLISION;
  }
#endif /* RDC_CONF_HARDWARE_CSMA */

  if((!RDC_HARDWARE_ACK && !cmIsBroadcast(&pInfo)) || CM_AUTO_STROBES) {
    /* Turn radio on to receive expected unicast ack.  Not necessary
       with hardware ack detection, and may trigger an unnecessary cca
       or rx cycle */
     on();
  }

  watchdog_periodic();

  if(CM_AUTO_STROBES){
    /* invalidate rtimer so autoStrobes can use the rtimer */
    autoStrobes_startSend(&pInfo,tx_len,autoStrobesDone);
    ret = MAC_TX_DEFERRED;
  }
  else{
    ret = sendStrobes(&pInfo,&encounter_time,tx_len);
    /* Execute tasks which must happen after strobes are sent */
    postStrobes(&pInfo,encounter_time,ret);
  }

  return ret;
}
/*---------------------------------------------------------------------------*/
static void
qsend_packet(mac_callback_t sent, void *ptr)
{
  struct cmTxInfo pInfo = {sent,ptr,NULL,CMTXRX_NONE};
  int ret = send_packet(pInfo);
  if(ret != MAC_TX_DEFERRED) {
    mac_call_sent_callback(sent, ptr, ret, 1);
  }
}
/*---------------------------------------------------------------------------*/
static void
qsend_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  struct rdc_buf_list *curr;
  struct rdc_buf_list *next;
  struct cmTxInfo pInfo = {sent,ptr,buf_list,CMTXRX_NONE};
  int ret;

  if(buf_list == NULL) {
    return;
  }
  /* Do not send during reception of a burst */
  if(we_are_receiving_burst) {
    /* Prepare the packetbuf for callback */
    queuebuf_to_packetbuf(buf_list->buf);
    /* Return COLLISION so the MAC may try again later */
    mac_call_sent_callback(sent, ptr, MAC_TX_COLLISION, 1);
    return;
  }

  /* Create and secure frames in advance */
  curr = buf_list;
  do {
    next = list_item_next(curr);
    queuebuf_to_packetbuf(curr->buf);
    if(!packetbuf_attr(PACKETBUF_ATTR_IS_CREATED_AND_SECURED)) {
      /* create and secure this frame */
      if(next != NULL) {
        packetbuf_set_attr(PACKETBUF_ATTR_PENDING, 1);
      }
      packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
      if(NETSTACK_FRAMER.create_and_secure() < 0) {
        PRINTF("contikimac: framer failed\n");
        mac_call_sent_callback(sent, ptr, MAC_TX_ERR_FATAL, 1);
        return;
      }

      packetbuf_set_attr(PACKETBUF_ATTR_IS_CREATED_AND_SECURED, 1);
      queuebuf_update_from_packetbuf(curr->buf);
    }
    curr = next;
  } while(next != NULL);

  /* The receiver needs to be awoken before we send */
  cmClrRcvrAwake(&pInfo);
  curr = buf_list;
  do { /* A loop sending a burst of packets from buf_list */
    next = list_item_next(curr);

    /* Prepare the packetbuf */
    queuebuf_to_packetbuf(curr->buf);

    /* Send the current packet */
    pInfo.buf_list = curr;
    ret = send_packet(pInfo);
    if(ret != MAC_TX_DEFERRED) {
      mac_call_sent_callback(sent, ptr, ret, 1);
    }

    if(ret != MAC_TX_OK){
      /* The transmission failed, we stop the burst */
      break;
    }

     /* TODO: this seems to assume that all packets in the list go to the same
     reciever; is this true? */
     cmSetRcvrAwake(&pInfo);
     curr = next;

  } while((next != NULL) && packetbuf_attr(PACKETBUF_ATTR_PENDING));
}
/*---------------------------------------------------------------------------*/
/* Timer callback triggered when receiving a burst, after having
   waited for a next packet for a too long time. Turns the radio off
   and leaves burst reception mode */
static void
recv_burst_off(void *ptr)
{
  if(!(CM_AUTO_STROBES && we_are_sending)){
    /* don't turn off now or we will interrupt the strobe sequence! */
    off();
  }
  we_are_receiving_burst = 0;
}
/*---------------------------------------------------------------------------*/
static void
input_packet(void)
{
  static struct ctimer ct;

  if(CM_AUTO_STROBES && we_are_sending){
    /* Strobes are being sent in another thread of execution */
    return;
  }

  if(!we_are_receiving_burst) {
    off();
  }

  /*  printf("cycle_start 0x%02x 0x%02x\n", cycle_start, cycle_start % CYCLE_TIME);*/

  if(packetbuf_totlen() > 0 && NETSTACK_FRAMER.parse() >= 0) {
    if(packetbuf_datalen() > 0 &&
       packetbuf_totlen() > 0 &&
       (linkaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                     &linkaddr_node_addr) ||
        packetbuf_holds_broadcast())) {
      /* This is a regular packet that is destined to us or to the
         broadcast address. */

      /* If FRAME_PENDING is set, we are receiving a packets in a burst */
      /* TODO To prevent denial-of-sleep attacks, the transceiver should
         be disabled upon receipt of an unauthentic frame. */
      we_are_receiving_burst = packetbuf_attr(PACKETBUF_ATTR_PENDING);
      if(we_are_receiving_burst) {
        on();
        /* Set a timer to turn the radio off in case we do not receive
	   a next packet */
        ctimer_set(&ct, INTER_PACKET_DEADLINE, recv_burst_off, NULL);
      } else {
        off();
        ctimer_stop(&ct);
      }

#if RDC_WITH_DUPLICATE_DETECTION
      /* Check for duplicate packet. */
      if(mac_sequence_is_duplicate()) {
        /* Drop the packet. */
        /*        printf("Drop duplicate ContikiMAC layer packet\n");*/
        return;
      }
      mac_sequence_register_seqno();
#endif /* RDC_WITH_DUPLICATE_DETECTION */

#if CONTIKIMAC_CONF_COMPOWER
      /* Accumulate the power consumption for the packet reception. */
      compower_accumulate(&current_packet);
      /* Convert the accumulated power consumption for the received
         packet to packet attributes so that the higher levels can
         keep track of the amount of energy spent on receiving the
         packet. */
      compower_attrconv(&current_packet);

      /* Clear the accumulated power consumption so that it is ready
         for the next packet. */
      compower_clear(&current_packet);
#endif /* CONTIKIMAC_CONF_COMPOWER */

      PRINTDEBUG("contikimac: data (%u)\n", packetbuf_datalen());
      NETSTACK_MAC.input();
      return;
    } else {
      PRINTDEBUG("contikimac: data not for us\n");
    }
  } else {
    PRINTF("contikimac: failed to parse (%u)\n", packetbuf_totlen());
  }
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  radio_is_on = 0;
  PT_INIT(&pt);

  rtimer_set(&rt, RTIMER_NOW() + CYCLE_TIME, 1,
             (void (*)(struct rtimer *, void *))powercycle, NULL);

  contikimac_is_on = 1;

#if WITH_PHASE_OPTIMIZATION
  phase_init();
#endif /* WITH_PHASE_OPTIMIZATION */

  autoStrobes_init();
}
/*---------------------------------------------------------------------------*/
static int
turn_on(void)
{
  if(contikimac_is_on == 0) {
    contikimac_is_on = 1;
    contikimac_keep_radio_on = 0;
    rtimer_set(&rt, RTIMER_NOW() + CYCLE_TIME, 1,
               (void (*)(struct rtimer *, void *))powercycle, NULL);
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
turn_off(int keep_radio_on)
{
  contikimac_is_on = 0;
  contikimac_keep_radio_on = keep_radio_on;
  if(keep_radio_on) {
    radio_is_on = 1;
    return NETSTACK_RADIO.on();
  } else {
    radio_is_on = 0;
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static unsigned short
duty_cycle(void)
{
  return (1ul * CLOCK_SECOND * CYCLE_TIME) / RTIMER_ARCH_SECOND;
}
/*---------------------------------------------------------------------------*/
uint16_t
contikimac_debug_print(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
sendStrobes(struct cmTxInfo* info,rtimer_clock_t* en_time,int tx_len)
{

  uint8_t got_strobe_ack = 0;
  int collisions = 0;

  rtimer_clock_t wt;
  rtimer_clock_t txtime;
  int strobes;
  rtimer_clock_t t0;
  uint8_t seqno;
  int ret;

  t0 = RTIMER_NOW();
  seqno = packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);

  for(strobes = 0;
      got_strobe_ack == 0 && collisions == 0 &&
      RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + STROBE_TIME); strobes++) {

    watchdog_periodic();

    if(!cmIsBroadcast(info) && (cmIsRcvrAwake(info) || cmIsRcvrKnown(info)) &&
       !RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + MAX_PHASE_STROBE_TIME)) {
      PRINTF("miss to %d\n", packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0]);
      break;
    }

    txtime = RTIMER_NOW();
    ret = NETSTACK_RADIO.transmit(tx_len);

    if(!RDC_CONF_HARDWARE_ACK){
      ret = waitOnACK(seqno,cmIsBroadcast(info));
    }

    /* For radios that block in the transmit routine and detect the
    ACK in hardware */
    if(ret == RADIO_TX_OK) {
      if(!cmIsBroadcast(info)) {
        got_strobe_ack = 1;
        *en_time = txtime;
        break;
      }
    }else if (ret == RADIO_TX_COLLISION) {
      PRINTF("contikimac: collisions while sending\n");
      collisions++;
    }

    if(RDC_CONF_HARDWARE_ACK){
      /*when ACK is handled in software, we've already waited this long*/
      wt = RTIMER_NOW();
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + INTER_PACKET_INTERVAL));
    }

  }

  PRINTF("contikimac: send (strobes=%u, len=%u, %s, %s), done\n", strobes,
         packetbuf_totlen(),
         got_strobe_ack ? "ack" : "no ack",
         collisions ? "collision" : "no collision");

  if(WITH_PHASE_OPTIMIZATION && cmIsRcvrKnown(info) && got_strobe_ack) {
      PRINTF(
          "no miss %d wake-ups %d\n",
          packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0], strobes
        );
  }

  if(collisions > 0) {
    return MAC_TX_COLLISION;
  } else if(!cmIsBroadcast(info) && !got_strobe_ack) {
    return MAC_TX_NOACK;
  }

  return MAC_TX_OK;
}
/*---------------------------------------------------------------------------*/
static void
autoStrobesDone(struct cmTxInfo* inf,rtimer_clock_t eTime,int ret){

  queuebuf_to_packetbuf(inf->buf_list->buf);
  postStrobes(inf,eTime,ret);
  mac_call_sent_callback(inf->macCB,inf->macPtr,ret,1);

  /* Restart powercycle state machine */
  PT_INIT(&pt);
  powercycle(&rt,NULL);


  if(inf->buf_list){
    struct rdc_buf_list *next = list_item_next(inf->buf_list);

    if(next){
      NETSTACK_RDC.send_list(inf->macCB,inf->macPtr,next);
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
postStrobes(struct cmTxInfo* inf,rtimer_clock_t eTime,int ret)
{

  off();

  #if CONTIKIMAC_CONF_COMPOWER
  /* Accumulate the power consumption for the packet transmission. */
  compower_accumulate(&current_packet);

  /* Convert the accumulated power consumption for the transmitted
     packet to packet attributes so that the higher levels can keep
     track of the amount of energy spent on transmitting the
     packet. */
  compower_attrconv(&current_packet);

  /* Clear the accumulated power consumption so that it is ready for
     the next packet. */
  compower_clear(&current_packet);
  #endif /* CONTIKIMAC_CONF_COMPOWER */

  contikimac_is_on = cmIsWasOn(inf);
  we_are_sending = 0;

  #if WITH_PHASE_OPTIMIZATION
  if(!cmIsBroadcast(inf)){
      if( (ret == MAC_TX_OK) && !cmIsRcvrAwake(inf)) {
        phase_update(
            packetbuf_addr(PACKETBUF_ADDR_RECEIVER),eTime, ret
          );
      }
  }
  #endif
}
/*---------------------------------------------------------------------------*/
static int
waitOnACK(int seqno,uint8_t is_broadcast)
{
  int len = 0;
  rtimer_clock_t wt;

  wt = RTIMER_NOW();
  while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + INTER_PACKET_INTERVAL));

  if(!is_broadcast && (NETSTACK_RADIO.receiving_packet() ||
                     NETSTACK_RADIO.pending_packet() ||
                     NETSTACK_RADIO.channel_clear() == 0)) {
    uint8_t ackbuf[ACK_LEN];
    wt = RTIMER_NOW();
    while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + AFTER_ACK_DETECTECT_WAIT_TIME));

    len = NETSTACK_RADIO.read(ackbuf, ACK_LEN);
    if(len == ACK_LEN && seqno == ackbuf[ACK_LEN - 1]) {
      return RADIO_TX_OK;
    } else {
      return RADIO_TX_COLLISION;
    }
  }

  return RADIO_TX_NOACK;
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver contikimac_driver = {
  "ContikiMAC",
  init,
  qsend_packet,
  qsend_list,
  input_packet,
  turn_on,
  turn_off,
  duty_cycle,
};
/*---------------------------------------------------------------------------*/
