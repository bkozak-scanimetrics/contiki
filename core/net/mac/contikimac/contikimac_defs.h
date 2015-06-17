#ifndef _CONTIKIMAC_DEFS_H_
#define _CONTIKIMAC_DEFS_H_
/*---------------------------------------------------------------------------*/
/**
* Includes
**/
#include "contiki-conf.h"
/*---------------------------------------------------------------------------*/
/**
* Defines
**/
/* TX/RX cycles are synchronized with neighbor wake periods */
#ifdef CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION
#define WITH_PHASE_OPTIMIZATION      CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION
#else /* CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION */
#define WITH_PHASE_OPTIMIZATION      1
#endif /* CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION */
/* More aggressive radio sleeping when channel is busy with other traffic */
#ifndef WITH_FAST_SLEEP
#define WITH_FAST_SLEEP              1
#endif
/* Radio does CSMA and autobackoff */
#ifndef RDC_CONF_HARDWARE_CSMA
#define RDC_CONF_HARDWARE_CSMA       0
#endif
/* Radio returns TX_OK/TX_NOACK after autoack wait */
#ifndef RDC_CONF_HARDWARE_ACK
#define RDC_CONF_HARDWARE_ACK        0
#endif

#define RDC_HARDWARE_ACK RDC_CONF_HARDWARE_ACK

/* MCU can sleep during radio off */
#ifndef RDC_CONF_MCU_SLEEP
#define RDC_CONF_MCU_SLEEP           0
#endif

#if NETSTACK_RDC_CHANNEL_CHECK_RATE >= 64
#undef WITH_PHASE_OPTIMIZATION
#define WITH_PHASE_OPTIMIZATION 0
#endif

/* CYCLE_TIME for channel cca checks, in rtimer ticks. */
#ifdef CONTIKIMAC_CONF_CYCLE_TIME
#define CYCLE_TIME (CONTIKIMAC_CONF_CYCLE_TIME)
#else
#define CYCLE_TIME (RTIMER_ARCH_SECOND / NETSTACK_RDC_CHANNEL_CHECK_RATE)
#endif

/* CHANNEL_CHECK_RATE is enforced to be a power of two.
 * If RTIMER_ARCH_SECOND is not also a power of two, there will be an inexact
 * number of channel checks per second due to the truncation of CYCLE_TIME.
 * This will degrade the effectiveness of phase optimization with neighbors
 * that do not have the same truncation error.
 * Define SYNC_CYCLE_STARTS to ensure an integral number of checks per second.
 */
#if RTIMER_ARCH_SECOND & (RTIMER_ARCH_SECOND - 1)
#define SYNC_CYCLE_STARTS                    1
#endif

/* INTER_PACKET_DEADLINE is the maximum time a receiver waits for the
   next packet of a burst when FRAME_PENDING is set. */
#define INTER_PACKET_DEADLINE               CLOCK_SECOND / 32

/* ContikiMAC performs periodic channel checks. Each channel check
   consists of two or more CCA checks. CCA_COUNT_MAX is the number of
   CCAs to be done for each periodic channel check. The default is
   two.*/
#ifdef CONTIKIMAC_CONF_CCA_COUNT_MAX
#define CCA_COUNT_MAX                      (CONTIKIMAC_CONF_CCA_COUNT_MAX)
#else
#define CCA_COUNT_MAX                      2
#endif

/* Before starting a transmission, Contikimac checks the availability
   of the channel with CCA_COUNT_MAX_TX consecutive CCAs */
#ifdef CONTIKIMAC_CONF_CCA_COUNT_MAX_TX
#define CCA_COUNT_MAX_TX                   (CONTIKIMAC_CONF_CCA_COUNT_MAX_TX)
#else
#define CCA_COUNT_MAX_TX                   6
#endif

/* CCA_CHECK_TIME is the time it takes to perform a CCA check. */
/* Note this may be zero. AVRs have 7612 ticks/sec, but block until \
   cca is done */
#ifdef CONTIKIMAC_CONF_CCA_CHECK_TIME
#define CCA_CHECK_TIME                     (CONTIKIMAC_CONF_CCA_CHECK_TIME)
#else
#define CCA_CHECK_TIME                     (RTIMER_ARCH_SECOND / 8192)
#endif

/* CCA_SLEEP_TIME is the time between two successive CCA checks. */
/* Add 1 when rtimer ticks are coarse */
#if RTIMER_ARCH_SECOND > 8000
#define CCA_SLEEP_TIME                     (RTIMER_ARCH_SECOND / 2000)
#else
#define CCA_SLEEP_TIME                     ((RTIMER_ARCH_SECOND / 2000) + 1)
#endif

/* CHECK_TIME is the total time it takes to perform CCA_COUNT_MAX
   CCAs. */
#define CHECK_TIME       (CCA_COUNT_MAX * (CCA_CHECK_TIME + CCA_SLEEP_TIME))

/* CHECK_TIME_TX is the total time it takes to perform CCA_COUNT_MAX_TX
   CCAs. */
#define CHECK_TIME_TX    (CCA_COUNT_MAX_TX * (CCA_CHECK_TIME + CCA_SLEEP_TIME))

/* LISTEN_TIME_AFTER_PACKET_DETECTED is the time that we keep checking
   for activity after a potential packet has been detected by a CCA
   check. */
#define LISTEN_TIME_AFTER_PACKET_DETECTED  (RTIMER_ARCH_SECOND / 80)

/* MAX_SILENCE_PERIODS is the maximum amount of periods (a period is
   CCA_CHECK_TIME + CCA_SLEEP_TIME) that we allow to be silent before
   we turn of the radio. */
#define MAX_SILENCE_PERIODS                5

/* MAX_NONACTIVITY_PERIODS is the maximum number of periods we allow
   the radio to be turned on without any packet being received, when
   WITH_FAST_SLEEP is enabled. */
#define MAX_NONACTIVITY_PERIODS            10



/* STROBE_TIME is the maximum amount of time a transmitted packet
   should be repeatedly transmitted as part of a transmission. */
#define STROBE_TIME                        (CYCLE_TIME + 2 * CHECK_TIME)

/* GUARD_TIME is the time before the expected phase of a neighbor that
   a transmitted should begin transmitting packets. */
#define GUARD_TIME                         (10 * CHECK_TIME + CHECK_TIME_TX)

/* INTER_PACKET_INTERVAL is the interval between two successive
   packet transmissions */

#ifdef CONTIKIMAC_CONF_INTER_PACKET_INTERVAL
#define INTER_PACKET_INTERVAL             CONTIKIMAC_CONF_INTER_PACKET_INTERVAL
#else
#define INTER_PACKET_INTERVAL             (RTIMER_ARCH_SECOND / 2500)
#endif

/* AFTER_ACK_DETECTECT_WAIT_TIME is the time to wait after a potential
   ACK packet has been detected until we can read it out from the
   radio. */
#ifdef CONTIKIMAC_CONF_AFTER_ACK_DETECTECT_WAIT_TIME
#define AFTER_ACK_DETECTECT_WAIT_TIME \
	CONTIKIMAC_CONF_AFTER_ACK_DETECTECT_WAIT_TIME
#else
#define AFTER_ACK_DETECTECT_WAIT_TIME      (RTIMER_ARCH_SECOND / 1500)
#endif

/* MAX_PHASE_STROBE_TIME is the time that we transmit repeated packets
   to a neighbor for which we have a phase lock. */
#define MAX_PHASE_STROBE_TIME              (RTIMER_ARCH_SECOND / 60)

#define ACK_LEN 3

#endif //_CONTIKIMAC_DEFS_H_
