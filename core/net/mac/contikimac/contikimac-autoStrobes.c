/*
 * Copyright (c) 2015, Scanimetrics - http://www.scanimetrics.com
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "contikimac-txrx.h"
#if (CM_AUTO_STROBES)
/*---------------------------------------------------------------------------*/
/**
* Includes
**/
#include "contiki-conf.h"
#include "net/mac/contikimac/contikimac.h"
#include "net/mac/contikimac/contikimac_defs.h"
#include "lib/assert.h"
#include "sys/pt.h"
#include "dev/radio.h"
#include "sys/rtimer.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "sys/process.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
/**
* Defines
**/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifndef ATOMIC_SECTION_ENTER
#include "ti-lib.h"

#define ATOMIC_SECTION_ENTER \
    {int _ints_disabled = ti_lib_int_master_disable();

#define ATOMIC_SECTION_EXIT \
    if(!_ints_disabled) ti_lib_int_master_enable();}

#else

#error "ATOMIC_SECTION macros have been provided; we can now remove this code"

#endif /* ATOMIC_SECTION_ENTER */
/*---------------------------------------------------------------------------*/
/**
* Types
**/
struct strobeState{
  struct pt pt;
  unsigned strobes;
  unsigned exitStatus;
  unsigned seqno;

  struct cmTxInfo txInfo;

  unsigned short tx_len;

  rtimer_clock_t strobeT0;

  rtimer_clock_t txTime;
  struct rtimer rt;

  void(*cb)(struct cmTxInfo*,rtimer_clock_t,int);
};
/*---------------------------------------------------------------------------*/
/**
* Data
**/
static volatile bool ackWaiting;
static struct strobeState state;
/*---------------------------------------------------------------------------*/
/**
* Function prototypes
**/
PROCESS(contikimacStrobes_process,"strobes");
static PT_THREAD(sendStrobes_hw(struct rfEvInfo info,enum cmEvent ev));
static void scheduleStrobe(rtimer_clock_t rel);
static void scheduleAckCheckPossible(rtimer_clock_t target,rtimer_clock_t rel);
static void scheduleAckCheck(rtimer_clock_t target,rtimer_clock_t rel);
static void transmitRtimerWrapper(struct rtimer * rt, void* ptr);
static void ccaRtimerWrapper(struct rtimer * rt, void* ptr);
static void runAckCheck(struct rtimer * rt, void* ptr);
static void txDoneCB(int ret,rtimer_clock_t txTime);
static void ccaDoneCB(radio_result_t ret,bool clear,rtimer_clock_t time);
static void ackCB(void);
/*---------------------------------------------------------------------------*/
/**
* Function declarations
**/
void
autoStrobes_startSend(
    struct cmTxInfo* info,unsigned short tx_len,
    void(*cb)(struct cmTxInfo*,rtimer_clock_t,int)
  )
{
  /* stop using rtimer for performing RX */
  rtimer_invalidate();


  memcpy(&(state.txInfo),info,sizeof(*info));
  state.seqno = packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO)&0xFF;
  state.strobes = 0;
  state.tx_len = tx_len;
  state.cb = cb;

  NETSTACK_RFASYNC.startAckDetect(state.seqno);

  if(cmIsBroadcast(&(state.txInfo))){
    state.exitStatus = MAC_TX_OK;
  }
  else{
    state.exitStatus = MAC_TX_NOACK;
  }

  state.strobeT0 = RTIMER_NOW();

  assert(NETSTACK_RFASYNC.transmit);
  assert(NETSTACK_RFASYNC.getCCA);

  PT_INIT(&(state.pt))

  NETSTACK_RFASYNC.transmit(state.tx_len);
}
/*---------------------------------------------------------------------------*/
void
autoStrobes_init(void)
{
  struct asyncRF_callbacks tmp = {txDoneCB,ccaDoneCB,ackCB};

  NETSTACK_RADIO.set_object(RADIO_PARAM_ASYNC_CALLBACKS,&tmp,0);
  process_start(&contikimacStrobes_process,NULL);
}
/*---------------------------------------------------------------------------*/
static
PT_THREAD(sendStrobes_hw(struct rfEvInfo info,enum cmEvent ev))
{
  PT_BEGIN(&(state.pt));

  do{
    state.strobes += 1;
    assert(ev == TX_COMPLETE);

    if(info.status == RADIO_TX_COLLISION){
      /* note: can't use break here due to
      protothread weirdness */
      state.exitStatus = MAC_TX_COLLISION;
      goto exit;
    }
    else if(info.status != RADIO_TX_OK){
      /* Any good way to handle this? */
      PRINTF("rf err\n");
    }
    state.txTime = info.time;

    if(!cmIsBroadcast(&(state.txInfo))){
      PT_YIELD_ON(
          &(state.pt),scheduleAckCheckPossible(info.time,INTER_PACKET_INTERVAL)
        );

      assert(
          ev == ACK_POSSIBLE || ev == ACK_NONE || ev == ACK_RECEIVED
        );

      if(ev == ACK_POSSIBLE){
        PT_YIELD_ON(
            &(state.pt),
            scheduleAckCheck(info.time,AFTER_ACK_DETECTECT_WAIT_TIME)
          );
      }
      assert(ev==ACK_RECEIVED || ev==ACK_NONE || ev==ACK_POSSIBLE);
    }

    if(ev == ACK_RECEIVED){
      /* note: can't use break here due to
      protothread weirdness */
      state.exitStatus = MAC_TX_OK;
      goto exit;
    }


    if( !RTIMER_CLOCK_LT(info.time,(STROBE_TIME+state.strobeT0)) ){
      goto exit;
    }
    PT_YIELD_ON(&(state.pt),scheduleStrobe(INTER_PACKET_INTERVAL));
  }while(1);

  exit:
    assert(state.cb);
    NETSTACK_RFASYNC.stopAckDetect();
    process_poll(&contikimacStrobes_process);

  PT_END(&(state.pt));
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(contikimacStrobes_process,ev,data)
{
  PROCESS_BEGIN();

  while(1){
    PROCESS_WAIT_EVENT();
    if(ev == PROCESS_EVENT_POLL){
      state.cb(&state.txInfo,state.txTime,state.exitStatus);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static void
scheduleStrobe(rtimer_clock_t rel)
{

  int ret = rtimer_setRelTarget(
      &(state.rt),rel,state.txTime,0,transmitRtimerWrapper,NULL
    );

  if(ret == RTIMER_ERR_TIME){
    NETSTACK_RFASYNC.transmit(state.tx_len);
  }
}
/*---------------------------------------------------------------------------*/
static void
scheduleAckCheckPossible(rtimer_clock_t target,rtimer_clock_t rel){
  int ret = rtimer_setRelTarget(
      &(state.rt),rel,target,0,ccaRtimerWrapper,NULL
    );

  if(ret == RTIMER_ERR_TIME){
    NETSTACK_RFASYNC.getCCA();
  }
}
/*---------------------------------------------------------------------------*/
static void
scheduleAckCheck(rtimer_clock_t target,rtimer_clock_t rel)
{
  int ret;

  ackWaiting = true;

  ret = rtimer_setRelTarget(
      &(state.rt),rel,target,0,runAckCheck,NULL
    );

  if(ret == RTIMER_ERR_TIME){
    state.rt.time = RTIMER_NOW();
    runAckCheck(&(state.rt),NULL);
  }
}
/*---------------------------------------------------------------------------*/
static void
transmitRtimerWrapper(struct rtimer * rt, void* ptr)
{
  NETSTACK_RFASYNC.transmit(state.tx_len);
}
/*---------------------------------------------------------------------------*/
static void
ccaRtimerWrapper(struct rtimer * rt, void* ptr)
{
  NETSTACK_RFASYNC.getCCA();
}
/*---------------------------------------------------------------------------*/
static void
txDoneCB(int ret,rtimer_clock_t txTime)
{
  enum cmEvent ev = TX_COMPLETE;
  struct rfEvInfo info = {txTime,ret};

  sendStrobes_hw(info,ev);
}
/*---------------------------------------------------------------------------*/
static void ccaDoneCB(radio_result_t ret,bool clear,rtimer_clock_t time){
  struct rfEvInfo info = {time,ret};

  if(!clear || NETSTACK_RADIO.pending_packet()){
     sendStrobes_hw(info,ACK_POSSIBLE);
  }
  else if(NETSTACK_RFASYNC.ackDetected()){
    sendStrobes_hw(info,ACK_RECEIVED);
  }
  else{
    sendStrobes_hw(info,ACK_NONE);
  }
}
/*---------------------------------------------------------------------------*/
static void
runAckCheck(struct rtimer * rt, void* ptr)
{
  struct rfEvInfo info = {rt->time,0};
  enum cmEvent ev = ACK_NONE;
  bool first = false;

  ATOMIC_SECTION_ENTER;
  if(ackWaiting){
    ackWaiting = false;
    first = true;
  }
  ATOMIC_SECTION_EXIT;

  if(first){
    if(NETSTACK_RFASYNC.ackDetected()){
      ev = ACK_RECEIVED;
    }

    sendStrobes_hw(info,ev);
  }
}
/*---------------------------------------------------------------------------*/
static void
ackCB(void)
{
  struct rfEvInfo info = {RTIMER_NOW(),0};
  bool first = false;

  if(cmIsBroadcast(&(state.txInfo))){
    return;
  }

  ATOMIC_SECTION_ENTER;
  if(ackWaiting){
    rtimer_invalidate();
    ackWaiting = false;
    first = true;
  }
  ATOMIC_SECTION_EXIT;

  if(first){
    sendStrobes_hw(info,ACK_RECEIVED);
  }
}
/*---------------------------------------------------------------------------*/
#endif /* CM_AUTO_STROBES */