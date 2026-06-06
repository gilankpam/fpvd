// feeder.c — paced probe traffic generator for the fpvd probe link.
// Sends seq-numbered UDP datagrams to 127.0.0.1:<port> at a fixed rate, for a
// dedicated FEC-off wfb_tx to inject at a fixed MCS on its own radio_port.
//
// Usage: probe-feeder <port> <pps> <size> [duration_s]
// Wire: [0..3]='PRB0'  [4..11]=big-endian uint64 seq  [12..]=0xA5 fill
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: %s <port> <pps> <size> [duration_s]\n", argv[0]);
        return 2;
    }
    int port = atoi(argv[1]);
    int pps  = atoi(argv[2]);
    int size = atoi(argv[3]);
    long dur = argc > 4 ? atol(argv[4]) : 0;
    if (size < 12) size = 12;
    if (pps  < 1)  pps  = 1;

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("socket"); return 1; }
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port   = htons(port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(fd, (struct sockaddr *)&a, sizeof a) < 0) { perror("connect"); return 1; }

    unsigned char *buf = malloc(size);
    memset(buf, 0xA5, size);
    memcpy(buf, "PRB0", 4);

    long ns = 1000000000L / pps;
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint64_t seq = 0;
    time_t t0 = time(NULL);

    for (;;) {
        for (int i = 0; i < 8; i++) buf[4 + i] = (seq >> (56 - 8 * i)) & 0xff;
        (void)send(fd, buf, size, 0);
        seq++;
        t.tv_nsec += ns;
        while (t.tv_nsec >= 1000000000L) { t.tv_nsec -= 1000000000L; t.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
        if (dur && (time(NULL) - t0) >= dur) break;
    }
    return 0;
}
