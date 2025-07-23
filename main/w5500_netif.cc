#include "w5500_netif.h"
#include "w5500_netif_hardware.h"
#include "wizchip_conf.h"
#include "print.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <lwip/netifapi.h>
#include <lwip/pbuf.h>
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include <netif/etharp.h>

#define TICK2MS(tick) ((tick) / portTICK_PERIOD_MS)
#define TX_ADDR(x) (((uint32_t)(x) << 8) + (WIZCHIP_TXBUF_BLOCK(0) << 3))
#define RX_ADDR(x) (((uint32_t)(x) << 8) + (WIZCHIP_RXBUF_BLOCK(0) << 3))

static struct netif *w5500netif;
static SemaphoreHandle_t w5500ifMutexHandle;

static void w5500Reopen() {
  setSn_CR(0, Sn_CR_CLOSE);
  while (getSn_SR(0) != SOCK_CLOSED)
    ;
  setSn_IR(0, 0xFF);
  setSn_CR(0, Sn_CR_OPEN);
  while (getSn_SR(0) != SOCK_MACRAW)
    ;
}

err_t w5500LinkOutput(struct netif *netif, struct pbuf *p) {
  if (xSemaphoreTake(w5500ifMutexHandle, portMAX_DELAY) == pdTRUE) {
    TickType_t tick;
    uint16_t curLen, ptr;
    struct pbuf *q = p;
    (void)netif;

    if (!(getSn_SR(0) & SOCK_MACRAW))
      goto ERROR;

    setSn_IR(0, Sn_IR_SENDOK);

    ptr = getSn_TX_WR(0);

    while (q) {
      curLen = q->len;
      if (curLen > getSn_TX_FSR(0)) {
        w5500Reopen();
        goto ERROR;
      }
      WIZCHIP_WRITE_BUF(TX_ADDR(ptr), (uint8_t*)q->payload, curLen);

      ptr += curLen;
      q = q->next;
    }

    setSn_TX_WR(0, ptr);

    tick = xTaskGetTickCount();
    setSn_CR(0, Sn_CR_SEND);
    while (getSn_CR(0))
      ;
    while (!(getSn_IR(0) & Sn_IR_SENDOK))
      if (TICK2MS(xTaskGetTickCount() - tick) > TX_TIMEOUT) {
        w5500Reopen();
        goto ERROR;
      }
    setSn_IR(0, Sn_IR_SENDOK);

    MIB2_STATS_NETIF_ADD(w5500netif, ifoutoctets, p->tot_len);
    if (((u8_t *)p->payload)[0] & 1) {
      /* broadcast or multicast packet*/
      MIB2_STATS_NETIF_INC(w5500netif, ifoutnucastpkts);
    } else {
      /* unicast packet */
      MIB2_STATS_NETIF_INC(w5500netif, ifoutucastpkts);
    }
    LINK_STATS_INC(link.xmit);

    xSemaphoreGive(w5500ifMutexHandle);
    return ERR_OK;
  }
  return ERR_IF;
ERROR:
  xSemaphoreGive(w5500ifMutexHandle);
  return ERR_IF;
}

static err_t w5500Input(struct pbuf **p) {
  
  uint16_t frameLen, ptr;

  if (xSemaphoreTake(w5500ifMutexHandle, 10) == pdTRUE) {
    bool lw5500 = wizphy_getphylink();
    bool llwip = netif_is_link_up(w5500netif);

    if (lw5500 && !llwip)
      netifapi_netif_set_link_up(w5500netif);
    else if (!lw5500 && llwip) {
      netifapi_netif_set_link_down(w5500netif);
      print(LOG_LEVEL_DEBUG, "No link\n");
      goto ERROR;
    }

    if (!(getSn_SR(0) & SOCK_MACRAW)) {
      print(LOG_LEVEL_DEBUG, "No socket\n");
      goto ERROR;
    }

    if (!(getSn_IR(0) & Sn_IR_RECV)) {
      print(LOG_LEVEL_DEBUG, "No data\n");
      goto ERROR;
    }
    setSn_IR(0, Sn_IR_RECV);
    print(LOG_LEVEL_DEBUG, "Ok\n");
    ptr = getSn_RX_RD(0);

    WIZCHIP_READ_BUF(RX_ADDR(ptr), (uint8_t *)&frameLen, 2);
    if (!(frameLen = (((frameLen & 0xFF) << 8) | (frameLen >> 8)) - 2))
      goto ERROR;
    ptr += 2;

    if (frameLen > 1514) {
      w5500Reopen();
      goto ERROR;
    }

    if ((*p = pbuf_alloc(PBUF_RAW, (frameLen), PBUF_RAM)) == NULL) {
      ptr += frameLen;
      setSn_RX_RD(0, ptr);
      setSn_CR(0, Sn_CR_RECV);
      while (getSn_CR(0))
        ;
      LINK_STATS_INC(link.memerr);
      LINK_STATS_INC(link.drop);
      MIB2_STATS_NETIF_INC(w5500netif, ifindiscards);
      goto ERROR;
    }

    WIZCHIP_READ_BUF(RX_ADDR(ptr), (uint8_t*)(*p)->payload, frameLen);
    setSn_RX_RD(0, ptr);
    setSn_CR(0, Sn_CR_RECV);
    while (getSn_CR(0))
      ;

    MIB2_STATS_NETIF_ADD(w5500netif, ifinoctets, (*p)->tot_len);
    if (((uint8_t *)(*p)->payload)[0] & 1) {
      /* broadcast or multicast packet*/
      MIB2_STATS_NETIF_INC(w5500netif, ifinnucastpkts);
    } else {
      /* unicast packet*/
      MIB2_STATS_NETIF_INC(w5500netif, ifinucastpkts);
    }

    LINK_STATS_INC(link.recv);
    xSemaphoreGive(w5500ifMutexHandle);
    return ERR_OK;
  }
  print(LOG_LEVEL_DEBUG, "No mutex\n");
  return ERR_IF;
ERROR:
  xSemaphoreGive(w5500ifMutexHandle);
  return ERR_IF;
}

static void w5500InputTask(void *arg) {
  (void)arg;
  struct pbuf *p;
  for (;;)
    if (w5500Input(&p) == ERR_OK)
      if (w5500netif->input(p, w5500netif) != ERR_OK)
        pbuf_free(p);
}

err_t w5500Init(struct netif *netif) {
  uint8_t i;
  uint8_t protocol = Sn_MR_MACRAW;
  uint8_t flag = Sn_MR_MFEN | Sn_MR_MMB | Sn_MR_MIP6B;
  uint8_t rx_tx_buff_sizes[] = {16, 0, 0, 0, 0, 0, 0, 0};
  uint8_t mac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

  w5500InitHardware();
  wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);
  setSn_MR(0, (protocol | flag));
  setSn_IR(0, 0xFF);
  setSHAR(mac);
  setSn_CR(0, Sn_CR_OPEN);
  while (getSn_SR(0) != SOCK_MACRAW);

  for (i = 0; i < 6; i++)
    netif->hwaddr[i] = mac[i];
  netif->hwaddr_len = ETHARP_HWADDR_LEN;
  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  netif->output = etharp_output;
  netif->linkoutput = w5500LinkOutput;
  netif->mtu = 1500;
  netif->flags |=
      NETIF_FLAG_ETHARP | NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHERNET;
  w5500netif = netif;

  if (wizphy_getphylink())
    netif->flags |= NETIF_FLAG_LINK_UP;

  w5500ifMutexHandle = xSemaphoreCreateMutex();
  xTaskCreate(w5500InputTask, "w5500InTask", 2048, NULL, 1, NULL);

  return ERR_OK;
}