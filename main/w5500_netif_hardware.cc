#include "w5500_netif_hardware.h"
#include "wizchip_conf.h"

#include <SPI.h>
#include <Arduino.h>

static void w5500Select(void);
static void w5500Unselect(void);
static void w5500ReadBuff(uint8_t*, uint16_t);
static void w5500WriteBuff(uint8_t*, uint16_t);
static uint8_t w5500ReadByte(void);
static void w5500WriteByte(uint8_t);
static void w5500EnterCritical(void);
static void w5500ExitCritical(void);

static SPIClass w5500spi(VSPI);
static SPISettings spiSettings(SCLK_FREQ, MSBFIRST, SPI_MODE3);

void w5500InitHardware() {
  
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  w5500spi.begin(SCLK_PIN, MISO_PIN, MOSI_PIN, -1);

  reg_wizchip_cs_cbfunc(w5500Select, w5500Unselect);
  reg_wizchip_spi_cbfunc(w5500ReadByte, w5500WriteByte);
  reg_wizchip_spiburst_cbfunc(w5500ReadBuff, w5500WriteBuff);
  reg_wizchip_cris_cbfunc(w5500EnterCritical, w5500ExitCritical);
}

static void w5500Select(void) {
  digitalWrite(CS_PIN, LOW);
}

static void w5500Unselect(void) {
  digitalWrite(CS_PIN, HIGH);
}

static void w5500ReadBuff(uint8_t *buff, uint16_t len) {
  w5500spi.transferBytes(NULL, buff, len);
}

static void w5500WriteBuff(uint8_t *buff, uint16_t len) {
  w5500spi.transferBytes(buff, NULL, len);
}

static uint8_t w5500ReadByte(void) {
  uint8_t ret;
  ret = w5500spi.transfer(0);
  return ret;
}

static void w5500WriteByte(uint8_t byte) { 
  w5500spi.transfer(byte);
}

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

static void w5500EnterCritical(void) {
  taskENTER_CRITICAL(&spinlock);
  w5500spi.beginTransaction(spiSettings);
}

static void w5500ExitCritical(void) {
  w5500spi.endTransaction();
  taskEXIT_CRITICAL(&spinlock);
}