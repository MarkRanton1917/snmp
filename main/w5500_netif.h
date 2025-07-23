#ifndef _W5500IF_H_
#define _W5500IF_H_

#include <lwip/err.h>
#include <lwip/netif.h>

#define IFNAME0 'e'
#define IFNAME1 '0'
#define TX_TIMEOUT 100

err_t w5500Init(struct netif *netif);

#endif