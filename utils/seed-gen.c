/*
 * Volvo P3 CEM hash collision finder
 *
 * (c) 2022 Vitaly Mayatskikh <vitaly@gravicappa.info>
 *
 * Finds all matching pins for a given seed+key match
 *
 * $ gcc -Ofast -o seed-gen seed-gen.c
 * $ ./seed-gen --seed "8f ca 96" --key "19 30 48" --pps 866
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <byteswap.h>
#include <time.h>
#include <string.h>
#include <getopt.h>
#include <limits.h>

unsigned char b2b[100] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29,
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99
};

unsigned char bin_to_bcd(unsigned char value)
{
	return b2b[value];
}

unsigned int p3_hash(unsigned int k, int w, unsigned int n)
{
	unsigned int m = 0x1212050;
	int i;

	for (i = 0; i < w; i++, n >>= 1, k >>= 1) {
		if ((n ^ k) & 0x1)
			n ^= m;
	}

	return n;
}

void p3_hash_shuffle(unsigned int _n, unsigned char *hash)
{
	unsigned n = _n;
	struct { unsigned int n0:4, n1:4, n2:4, n3:4, n4:4, n5:4, n6:4, n7:4; } *out = (void *)&n;

	hash[0] = 0x10 * out->n2 + out->n1;
	hash[1] = 0x10 * out->n3 + out->n5;
	hash[2] = 0x10 * out->n0 + out->n4;
}

static struct option const long_opts[] = {
	{"seed", required_argument, NULL, 's'},
	{"key", required_argument, NULL, 'k'},
	{"pps", required_argument, NULL, 'p'},
	{"help", no_argument, NULL, 'h'},
	{0, 0, 0, 0 }
};

void die(void)
{
	printf("./seed-gen --seed \"AA BB CC\" --key \"XX YY ZZ\" <--pps N>\n");
	exit(1);
}

int main(int argc, char *argv[])
{
	int opt, ret, got_seed = 0, got_key = 0;
	unsigned char seed[3] = { };
	unsigned char key0[4] = { };
	unsigned char key1[4] = { };
	unsigned char pin[6] = { };
	unsigned int i, j = 0;
	unsigned int n[4];
	long l = 0;
	int p0, p1, p2, p3, p4;
	int pps = 866;

	while ((opt = getopt_long(argc, argv, ":spk", long_opts, NULL)) != -1) {
		switch (opt) {
		case 's':
			ret = sscanf(optarg, "%02hhx %02hhx %02hhx", seed + 0, seed + 1, seed + 2);
			if (ret != 3)
				die();
			got_seed++;
			break;
		case 'k':
			ret = sscanf(optarg, "%02hhx %02hhx %02hhx", key0 + 0, key0 + 1, key0 + 2);
			if (ret != 3)
				die();
			got_key++;
			break;
		case 'p':
			ret = strtoul(optarg, NULL, 10);
			if (ret <= 0 || ret == ULONG_MAX)
				die();
			pps = ret;
			break;
		case 'h':
		default:
			die();
			break;
		}
	}

	if (!got_seed || !got_key)
		die();

	printf("Searching all hash collisions for SEED %02x %02x %02x, KEY %02x %02x %02x at %d tries per second\n",
	       seed[0], seed[1], seed[2], key0[0], key0[1], key0[2], pps);

	for (p0 = 0; p0 < 100; p0++) {
		pin[0] = bin_to_bcd(p0);
		n[0] = p3_hash((seed[0] << 0) | (seed[1] << 8) | (seed[2] << 16) | (pin[0] << 24), 32, 0xc541a9);
		for (p1 = 0; p1 < 100; p1++) {
			pin[1] = bin_to_bcd(p1);
			n[1] = p3_hash(pin[1], 8, n[0]);
			for (p2 = 0; p2 < 100; p2++) {
				pin[2] = bin_to_bcd(p2);
				n[2] = p3_hash(pin[2], 8, n[1]);
				for (p3 = 0; p3 < 100; p3++) {
					pin[3] = bin_to_bcd(p3);
					n[3] = p3_hash(pin[3], 8, n[2]);
					for (p4 = 0; p4 < 100; p4++) {
						pin[4] = bin_to_bcd(p4);
						n[4] = p3_hash(pin[4], 8, n[3]);

						p3_hash_shuffle(n[4], key1);
						l++;
						if (*(int *)&key0 == *(int *)&key1) {
							printf("%7.2f hrs [%7d] PIN %02x %02x %02x %02x %02x\n",
							       l / pps / 3600.0,
							       ++j, pin[0], pin[1], pin[2], pin[3], pin[4]);
						}
					}
				}
			}
		}
	}

	return 0;
}
