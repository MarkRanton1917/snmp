#include "w5500.h"
#include <freertos/semphr.h>
#include <lwip/err.h>
#include <lwip/ip4_addr.h>
#include <lwip/netif.h>

class EthW5500
{
private:
    struct netif netif_;
    W5500 w5500_;
    const ip4_addr_t deviceIp_;
    const ip4_addr_t netmask_;
    const ip4_addr_t gatewayIp_;

public:
    EthW5500(W5500 w5500, const ip4_addr_t deviceIp, const ip4_addr_t netmask, const ip4_addr_t gatewayIp);
    void begin();
    void maintain();
};