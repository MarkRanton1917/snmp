#ifndef _W5500_NETIF_HARDWARE_H_
#define _W5500_NETIF_HARDWARE_H_

#define CS_PIN 5
#define SCLK_PIN 18
#define MOSI_PIN 23
#define MISO_PIN 19
#define SCLK_FREQ 10000000

void w5500InitHardware(void);

#endif