#include "w5500_netif.h"
#include "print.h"
#include "main.h"

#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <lwip/api.h>
#include <lwip/apps/snmp.h>
#include <lwip/apps/snmp_mib2.h>
#include <lwip/ip4_addr.h>
#include <lwip/dhcp.h>
#include <lwip/netif.h>
#include <lwip/netifapi.h>
#include <lwip/tcpip.h>
#include <lwip/udp.h>

#include <string.h>

//#define WORK 

#ifdef WORK
#define DEVICEIP {PP_HTONL(LWIP_MAKEU32(192, 168, 0, 44))}
#define NETMASK  {PP_HTONL(LWIP_MAKEU32(255, 255, 255, 0))}
#define GATEWAY  {PP_HTONL(LWIP_MAKEU32(192, 168, 0, 1))}
#define CLIENTIP {PP_HTONL(LWIP_MAKEU32(192, 168, 0, 40))} 
#else
#define DEVICEIP {PP_HTONL(LWIP_MAKEU32(192, 168, 1, 144))}
#define NETMASK  {PP_HTONL(LWIP_MAKEU32(255, 255, 255, 0))}
#define GATEWAY  {PP_HTONL(LWIP_MAKEU32(192, 168, 1, 1))}
#define CLIENTIP {PP_HTONL(LWIP_MAKEU32(192, 168, 1, 152))} 
#endif

static void lwipInit(void *);
static void testAssert(const char *, int);

SemaphoreHandle_t initSemaphoreHandle;
struct netif eth0;
struct snmp_obj_id enterpriseOid = {.len = 7,
                                    .id = {1, 3, 6, 1, 4, 1, 12345}};
static u8_t sysLocation[32] = "lwIP development PC";
static u8_t sysContact[32] = "root";

void lwipInit(void *params) {

  const ip4_addr_t 
    deviceIp = DEVICEIP,
    netmask = NETMASK,
    gatewayIp = GATEWAY;
  const ip_addr_t clientIp = CLIENTIP;

  netif_add(&eth0, &deviceIp, &netmask, &gatewayIp, 
            NULL, &w5500Init, &tcpip_input);

  netif_set_default(&eth0);
  netif_set_up(&eth0);

  snmp_threadsync_init(&snmp_mib2_lwip_locks, snmp_mib2_lwip_synchronizer);
  snmp_mib2_set_syscontact(sysContact, NULL, sizeof(sysContact));
  snmp_mib2_set_syslocation(sysLocation, NULL, sizeof(sysLocation));
  snmp_mib2_set_sysdescr((const u8_t *)"Ephphatha converter", NULL);
  snmp_set_device_enterprise_oid(&enterpriseOid);
  snmp_trap_dst_ip_set(0, &clientIp);
  snmp_trap_dst_enable(0, 1);

  xSemaphoreGive(initSemaphoreHandle);
}

extern "C" void app_main() {
  err_t err;
  int i = 0;
  initSemaphoreHandle = xSemaphoreCreateBinary();
  
  initArduino();
  printInit();
  tcpip_init(lwipInit, NULL);
  xSemaphoreTake(initSemaphoreHandle, portMAX_DELAY);
  print(LOG_LEVEL_APP, "Inited!\n");
  
  dhcp_inform(&eth0);
  err = dhcp_start(&eth0);
  testAssert("Unable to start DHCP!", err == ERR_OK);
  print(LOG_LEVEL_APP, "DHCP started!\n");
  snmp_init();
  print(LOG_LEVEL_APP, "SNMP started!\n");

  while (1) 
  {
    print(LOG_LEVEL_APP, "Still working! (%d)\n", i++);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void testAssert(const char *message, int flag) {
  if (!flag) {
    printAssert(message);
    while (1)
      vTaskDelay(pdMS_TO_TICKS(1));
  }
}