#include "w5500_netif_hardware.h"
#include "wizchip_conf.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <cstring>

static void w5500Select(void);
static void w5500Unselect(void);
static void w5500ReadBuff(uint8_t*, uint16_t);
static void w5500WriteBuff(uint8_t*, uint16_t);
static uint8_t w5500ReadByte(void);
static void w5500WriteByte(uint8_t);
static void w5500EnterCritical(void);
static void w5500ExitCritical(void);

spi_device_handle_t spi;

void w5500InitHardware() {

  spi_bus_config_t buscfg = 
  {
    .mosi_io_num=MOSI_PIN,
    .miso_io_num=MISO_PIN,
    .sclk_io_num=SCLK_PIN,
    .quadwp_io_num=-1,
    .quadhd_io_num=-1,
    .max_transfer_sz=64
  };

  spi_device_interface_config_t devcfg = 
  {
    .mode=3,
    .clock_speed_hz=SCLK_FREQ,
    .spics_io_num=-1,
    .flags = SPI_DEVICE_HALFDUPLEX,
    .queue_size=1
  };

  gpio_set_direction((gpio_num_t)CS_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)CS_PIN, 1);
  spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_DISABLED);
  spi_bus_add_device(SPI3_HOST, &devcfg, &spi);
  reg_wizchip_cs_cbfunc(w5500Select, w5500Unselect);
  reg_wizchip_spi_cbfunc(w5500ReadByte, w5500WriteByte);
  reg_wizchip_spiburst_cbfunc(w5500ReadBuff, w5500WriteBuff);
  reg_wizchip_cris_cbfunc(w5500EnterCritical, w5500ExitCritical);
}

static void w5500Select(void) {
  gpio_set_level((gpio_num_t)CS_PIN, 0);
}

static void w5500Unselect(void) {
  gpio_set_level((gpio_num_t)CS_PIN, 1);
}

static void w5500ReadBuff(uint8_t *buff, uint16_t len) {
  for(int i = 0; i < len; i++)
    buff[i] = w5500ReadByte();
}

static void w5500WriteBuff(uint8_t *buff, uint16_t len) {
  for(int i = 0; i < len; i++)
    w5500WriteByte(buff[i]);
}

static uint8_t w5500ReadByte(void) {
  uint32_t byte;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); 
  t.rxlength = 8;
  t.rx_buffer = &byte;
  spi_device_polling_transmit(spi, &t);
  return (uint8_t)byte;
}

static void w5500WriteByte(uint8_t byte) { 
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); 
  t.length = 8;
  t.tx_buffer = (uint32_t*)&byte;
  spi_device_polling_transmit(spi, &t);
}

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

static void w5500EnterCritical(void) {
  //taskENTER_CRITICAL(&spinlock);
}

static void w5500ExitCritical(void) {
  //taskEXIT_CRITICAL(&spinlock);
}