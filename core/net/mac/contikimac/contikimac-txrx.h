#ifndef _CONTIKIMAC_TXRX_H_
#define _CONTIKIMAC_TXRX_H_
/**
* Includes
**/
#include "contiki-conf.h"
#include "net/mac/mac.h"
#include "net/mac/rdc.h"
#include "sys/rtimer.h"
#include <stdint.h>
/*---------------------------------------------------------------------------*/
/**
* DEFINES
**/
#define CMTXRX_NONE      0x00
#define CMTXRX_BROADCAST 0x01
#define CMTXRX_RCVRKNOWN 0x02
#define CMTXRX_RCVRAWAKE 0x04
#define CMTXRX_WASON     0x08

#ifdef CM_CONF_AUTO_STROBES
#define CM_AUTO_STROBES CM_CONF_AUTO_STROBES
#else
#define CM_AUTO_STROBES (0)
#endif
/*---------------------------------------------------------------------------*/
/**
* Types
**/
struct cmTxInfo{
  mac_callback_t      macCB;
  void*               macPtr;
  struct rdc_buf_list *buf_list;
  unsigned            flags;
};

struct rfEvInfo{
	rtimer_clock_t time;
	int status;
};

enum cmEvent{
	TX_COMPLETE,ACK_RECEIVED,ACK_NONE,ACK_POSSIBLE
};
/*---------------------------------------------------------------------------*/
/**
* Macros
**/
#define cmIsBroadcast(i)          (!!((i)->flags&CMTXRX_BROADCAST))
#define cmIsRcvrKnown(i)          (!!((i)->flags&CMTXRX_RCVRKNOWN))
#define cmIsRcvrAwake(i)          (!!((i)->flags&CMTXRX_RCVRAWAKE))
#define cmIsWasOn(i)              (!!((i)->flags&CMTXRX_WASON))

#define cmSetBroadcast(i)         ((i)->flags|=CMTXRX_BROADCAST)
#define cmSetRcvrKnown(i)         ((i)->flags|=CMTXRX_RCVRKNOWN)
#define cmSetRcvrAwake(i)         ((i)->flags|=CMTXRX_RCVRAWAKE)
#define cmSetWasOn(i)             ((i)->flags|=CMTXRX_WASON)

#define cmClrBroadcast(i)         ((i)->flags&=~CMTXRX_BROADCAST)
#define cmClrRcvrKnown(i)         ((i)->flags&=~CMTXRX_RCVRKNOWN)
#define cmClrRcvrAwake(i)         ((i)->flags&=~CMTXRX_RCVRAWAKE)
#define cmClrWasOn(i)             ((i)->flags&=~CMTXRX_WASON)

#define cmAssignBitBroadcast(i,v) ((v)?cmSetBroadcast(i):cmClrBroadcast(i))
#define cmAssignBitRcvrKnown(i,v) ((v)?cmSetRcvrKnown(i):cmClrRcvrKnown(i))
#define cmAssignBitRcvrAwake(i,v) ((v)?cmSetRcvrAwake(i):cmClrRcvrAwake(i))
#define cmAssignBitWasOn(i,v)     ((v)?cmSetWasOn(i):cmClrWasOn(i))

#define cmNoPhaseDelay(i)   ((i)->flags&(CMTXRX_BROADCAST|CMTXRX_RCVRAWAKE))
#define cmOnForStrobes(i)   ( \
    (CM_AUTO_STROBES) || \
    (!CONTIKIMAC_HARDWARE_ACK && !cmIsBroadcast(i)) \
  )
/*---------------------------------------------------------------------------*/
/**
* Stubs
**/
#if !CM_AUTO_STROBES
#define autoStrobes_startSend(a,b,c) do{if(c==c);}while(0)
#define autoStrobes_init() {}
#endif

/*---------------------------------------------------------------------------*/
/**
* Prototypes
**/
#if CM_AUTO_STROBES
void autoStrobes_startSend(
    struct cmTxInfo* info,unsigned short tx_len,
    void(*cb)(struct cmTxInfo*,rtimer_clock_t,int)
  );
void autoStrobes_init(void);
#endif

#endif /* _CONTIKIMAC_TXRX_H_ */