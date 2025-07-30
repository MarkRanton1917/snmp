#include "eth.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <lwip/dhcp.h>
#include <lwip/etharp.h>
#include <lwip/netifapi.h>
#include <lwip/pbuf.h>
#include <lwip/snmp.h>
#include <lwip/stats.h>
#include <lwip/tcpip.h>

#include <utility>

#define TICK2MS(tick) ((tick) / portTICK_PERIOD_MS)
#define TX_ADDR(x) (((uint32_t)(x) << 8) + (WIZCHIP_TXBUF_BLOCK(0) << 3))
#define RX_ADDR(x) (((uint32_t)(x) << 8) + (WIZCHIP_RXBUF_BLOCK(0) << 3))

static void reopen(int w5500Id);
static err_t input(struct netif *netif, struct pbuf **p);
static err_t output(struct netif *netif, struct pbuf *p);
static err_t init(struct netif *);

W5500 *regW5500Addr[10] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
SemaphoreHandle_t regW5500Sem[10];

EthW5500::EthW5500(W5500 w5500, 
				   const ip4_addr_t deviceIp, const ip4_addr_t netmask, const ip4_addr_t gatewayIp)
    : w5500_(std::move(w5500)), deviceIp_(deviceIp), netmask_(netmask), gatewayIp_(gatewayIp)
{
    int i;
    for (i = 0; i < 10; i++)
    {
        if (!regW5500Addr[i])
        {
            regW5500Addr[i] = &this->w5500_;
            this->netif_.name[0] = 'e';
            this->netif_.name[1] = '0' + i;
            regW5500Sem[i] = xSemaphoreCreateMutex();
            break;
        }
    }
    ESP_ERROR_CHECK(i == 10);
}

void EthW5500::begin()
{
    uint8_t mac[6];

    this->w5500_.begin();
    this->w5500_.getMac(mac);

    netif_add(&this->netif_, &this->deviceIp_, &this->netmask_, &this->gatewayIp_, 
    		  (void *)mac, &init, &tcpip_input);
    netif_set_up(&this->netif_);
    dhcp_inform(&this->netif_);
}

void EthW5500::maintain()
{
    struct pbuf *p;
    if (input(&this->netif_, &p) == ERR_OK)
        if (this->netif_.input(p, &this->netif_) != ERR_OK)
            pbuf_free(p);
    vTaskDelay(pdMS_TO_TICKS(1));
}

static void reopen(int w5500Id)
{
    W5500 *w5500 = regW5500Addr[w5500Id];
    w5500->setSn_CR(0, Sn_CR_CLOSE);
    while (w5500->getSn_SR(0) != SOCK_CLOSED)
        ;

    w5500->setSn_IR(0, 0xFF);
    w5500->setSn_CR(0, Sn_CR_OPEN);
    while (w5500->getSn_SR(0) != SOCK_MACRAW)
        ;
}

static err_t output(struct netif *netif, struct pbuf *p)
{
    int w5500Id = netif->name[1] - '0';
    W5500 *w5500 = regW5500Addr[w5500Id];

    if (xSemaphoreTake(regW5500Sem[w5500Id], portMAX_DELAY) == pdTRUE)
    {
        TickType_t tick;
        uint16_t curLen, ptr;
        struct pbuf *q = p;

        if (!(w5500->getSn_SR(0) & SOCK_MACRAW))
            goto ERROR;

        w5500->setSn_IR(0, Sn_IR_SENDOK);

        ptr = w5500->getSn_TX_WR(0);

        while (q)
        {
            curLen = q->len;
            if (curLen > w5500->getSn_TX_FSR(0))
            {
                reopen(w5500Id);
                goto ERROR;
            }
            w5500->writeBurst(TX_ADDR(ptr), (uint8_t *)q->payload, curLen);

            ptr += curLen;
            q = q->next;
        }

        w5500->setSn_TX_WR(0, ptr);

        tick = xTaskGetTickCount();
        w5500->setSn_CR(0, Sn_CR_SEND);
        while (w5500->getSn_CR(0))
            ;
        while (!(w5500->getSn_IR(0) & Sn_IR_SENDOK))
            if (TICK2MS(xTaskGetTickCount() - tick) > 10)
            {
                reopen(w5500Id);
                goto ERROR;
            }
        w5500->setSn_IR(0, Sn_IR_SENDOK);

        MIB2_STATS_NETIF_ADD(netif, ifoutoctets, p->tot_len);
        if (((u8_t *)p->payload)[0] & 1)
            MIB2_STATS_NETIF_INC(netif, ifoutnucastpkts);
        else
            MIB2_STATS_NETIF_INC(netif, ifoutucastpkts);
        LINK_STATS_INC(link.xmit);

        xSemaphoreGive(regW5500Sem[w5500Id]);
        return ERR_OK;
    }
    return ERR_IF;
ERROR:
    xSemaphoreGive(regW5500Sem[w5500Id]);
    return ERR_IF;
}

static err_t input(struct netif *netif, struct pbuf **p)
{

    uint16_t frameLen, ptr;
    int w5500Id = netif->name[1] - '0';
    W5500 *w5500 = regW5500Addr[w5500Id];

    if (xSemaphoreTake(regW5500Sem[w5500Id], 10) == pdTRUE)
    {
        bool lw5500 = w5500->connected();
        bool llwip = netif_is_link_up(netif);

        if (lw5500 && !llwip)
            netif_set_link_up(netif);
        else if (!lw5500)
        {
            if (llwip)
                netif_set_link_down(netif);
            goto ERROR;
        }

        if (!(w5500->getSn_SR(0) & SOCK_MACRAW))
        {
            reopen(w5500Id);
            goto ERROR;
        }

        if (!(w5500->getSn_IR(0) & Sn_IR_RECV))
            goto ERROR;

        w5500->setSn_IR(0, Sn_IR_RECV);
        ptr = w5500->getSn_RX_RD(0);

        w5500->readBurst(RX_ADDR(ptr), (uint8_t *)&frameLen, 2);
        if (!(frameLen = (((frameLen & 0xFF) << 8) | (frameLen >> 8)) - 2))
            goto ERROR;

        ptr += 2;

        if (frameLen > 1514)
        {
            reopen(w5500Id);
            goto ERROR;
        }

        if ((*p = pbuf_alloc(PBUF_RAW, frameLen, PBUF_POOL)) == NULL)
        {
            ptr += frameLen;
            w5500->setSn_RX_RD(0, ptr);
            w5500->setSn_CR(0, Sn_CR_RECV);
            while (w5500->getSn_CR(0))
                ;

            LINK_STATS_INC(link.memerr);
            LINK_STATS_INC(link.drop);
            MIB2_STATS_NETIF_INC(netif, ifindiscards);
            goto ERROR;
        }

        w5500->readBurst(RX_ADDR(ptr), (uint8_t *)(*p)->payload, frameLen);
        w5500->setSn_RX_RD(0, ptr);
        w5500->setSn_CR(0, Sn_CR_RECV);
        while (w5500->getSn_CR(0))
            ;

        MIB2_STATS_NETIF_ADD(netif, ifinoctets, (*p)->tot_len);
        if (((uint8_t *)(*p)->payload)[0] & 1)
            MIB2_STATS_NETIF_INC(netif, ifinnucastpkts);
        else
            MIB2_STATS_NETIF_INC(netif, ifinucastpkts);
        LINK_STATS_INC(link.recv);

        xSemaphoreGive(regW5500Sem[w5500Id]);
        return ERR_OK;
    }
    return ERR_IF;
ERROR:
    xSemaphoreGive(regW5500Sem[w5500Id]);
    return ERR_IF;
}

static err_t init(struct netif *netif)
{
    int i;
    for (i = 0; i < 6; i++)
        netif->hwaddr[i] = ((uint8_t *)netif->state)[i];
    netif->hwaddr_len = ETHARP_HWADDR_LEN;
    netif->output = etharp_output;
    netif->linkoutput = output;
    netif->mtu = 1500;
    netif->flags |= NETIF_FLAG_ETHARP | NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHERNET;

    return ERR_OK;
}