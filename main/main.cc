#include "main.h"
#include "eth.h"
#include "print.h"
#include "snmpv3.h"

#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <lwip/api.h>
#include <lwip/apps/snmp.h>
#include <lwip/apps/snmp_mib2.h>
#include <lwip/apps/snmp_snmpv2_usm.h>
#include <lwip/apps/snmp_snmpv2_framework.h>
#include <lwip/dhcp.h>
#include <lwip/ip4_addr.h>
#include <lwip/netif.h>
#include <lwip/netifapi.h>
#include <lwip/tcpip.h>
#include <lwip/udp.h>

#include <string.h>

#define WORK

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

static void maintainTask(void *arg);

static const struct snmp_mib *mibs[] = {
  &mib2,
  &snmpframeworkmib,
  &snmpusmmib
};

static struct snmp_obj_id enterpriseOid = {.len = 7, .id = {1, 3, 6, 1, 4, 1, 12345}};
static u8_t sysLocation[32] = "lwIP development PC";
static u8_t sysContact[32] = "root";

extern "C" void app_main()
{
    int i = 0;
    const ip4_addr_t 
		deviceIp = DEVICEIP, 
		netmask = NETMASK, 
		gatewayIp = GATEWAY, 
		clientIp = CLIENTIP;
    SemaphoreHandle_t spi2Sem = xSemaphoreCreateMutex();

    W5500 w5500(SPI2_HOST, GPIO_NUM_19, GPIO_NUM_23, GPIO_NUM_18, GPIO_NUM_5, GPIO_NUM_NC, 
		10000000, spi2Sem);
    EthW5500 eth0(w5500, deviceIp, netmask, gatewayIp);

    initArduino();
    printInit();
    eth0.begin();
    xTaskCreate(maintainTask, "w5500Task", 1024, (void *)&eth0, 1, NULL);

    snmp_threadsync_init(&snmp_mib2_lwip_locks, snmp_mib2_lwip_synchronizer);
    snmp_mib2_set_syscontact(sysContact, NULL, sizeof(sysContact));
    snmp_mib2_set_syslocation(sysLocation, NULL, sizeof(sysLocation));
    snmp_mib2_set_sysdescr((const u8_t *)"Ephphatha converter", NULL);
    snmp_set_device_enterprise_oid(&enterpriseOid);
    snmp_trap_dst_ip_set(0, &clientIp);
    snmp_trap_dst_enable(0, 1);

    tcpip_init(NULL, NULL);
    print(LOG_LEVEL_APP, "Inited!\n");
    snmp_init();
    snmpv3_dummy_init();
    snmp_set_mibs(mibs, LWIP_ARRAYSIZE(mibs));
    print(LOG_LEVEL_APP, "SNMP started!\n");

    while (1)
    {
        print(LOG_LEVEL_APP, "Still working! (%d)\n", i++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void maintainTask(void *arg)
{
    for (;;)
        ((EthW5500 *)arg)->maintain();
}