/* SPDX-License-Identifier: GPL-2.0-only */
/* wmrom CLI: load a flat image through the WM ROM downloader. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>

#define DEV "/dev/wmrom"
struct load_req { uint64_t user_ptr; uint32_t len; uint32_t entry; };
struct reg { uint32_t off; uint32_t val; };
#define WMROM_IOC_MAGIC 'R'
#define WMROM_IOC_LOAD		_IOW(WMROM_IOC_MAGIC, 1, struct load_req)
#define WMROM_IOC_FWSTATE	_IOR(WMROM_IOC_MAGIC, 2, uint32_t)
#define WMROM_IOC_RESTART	_IO(WMROM_IOC_MAGIC, 3)
#define WMROM_IOC_BAR_READ	_IOWR(WMROM_IOC_MAGIC, 4, struct reg)
#define WMROM_IOC_PWRCYCLE	_IO(WMROM_IOC_MAGIC, 5)

static int open_dev(void)
{
	int fd = open(DEV, O_RDWR);

	if (fd < 0) {
		perror("open " DEV);
		fprintf(stderr, "load wmrom.ko (mt_wifi must be unloaded)\n");
		exit(1);
	}
	return fd;
}

int main(int argc, char **argv)
{
	int fd = open_dev();
	uint32_t st;

	if (argc == 2 && !strcmp(argv[1], "fwstate")) {
		if (ioctl(fd, WMROM_IOC_FWSTATE, &st)) { perror("FWSTATE"); return 1; }
		printf("fw_state %u\n", st);
	} else if (argc == 2 && !strcmp(argv[1], "restart")) {
		if (ioctl(fd, WMROM_IOC_RESTART)) { perror("RESTART"); return 1; }
		printf("restart ok\n");
	} else if (argc == 2 && !strcmp(argv[1], "pwrcycle")) {
		if (ioctl(fd, WMROM_IOC_PWRCYCLE)) { perror("PWRCYCLE"); return 1; }
		if (ioctl(fd, WMROM_IOC_FWSTATE, &st) == 0)
			printf("pwrcycle ok, fw_state %u\n", st);
		else
			printf("pwrcycle ok\n");
	} else if (argc == 3 && !strcmp(argv[1], "reg")) {
		struct reg r = { .off = (uint32_t)strtoul(argv[2], NULL, 0) };
		if (ioctl(fd, WMROM_IOC_BAR_READ, &r)) { perror("BAR_READ"); return 1; }
		printf("0x%06x = 0x%08x\n", r.off, r.val);
	} else if (argc == 3 && !strcmp(argv[1], "load")) {
		FILE *f = fopen(argv[2], "rb");
		long len;
		void *buf;
		struct load_req req = { 0 };

		if (!f) { perror(argv[2]); return 1; }
		fseek(f, 0, SEEK_END); len = ftell(f); fseek(f, 0, SEEK_SET);
		if (len <= 0 || len > 8 * 1024 * 1024) { fprintf(stderr, "bad size\n"); return 1; }
		buf = malloc(len);
		if (!buf || fread(buf, 1, len, f) != (size_t)len) { fprintf(stderr, "read fail\n"); return 1; }
		fclose(f);
		req.user_ptr = (uint64_t)(uintptr_t)buf;
		req.len = (uint32_t)len;
		if (ioctl(fd, WMROM_IOC_LOAD, &req)) { perror("LOAD"); return 1; }
		printf("loaded %ld bytes\n", len);
	} else {
		fprintf(stderr, "usage: %s fwstate|restart|pwrcycle|reg OFF|load FILE\n", argv[0]);
		return 2;
	}
	close(fd);
	return 0;
}
