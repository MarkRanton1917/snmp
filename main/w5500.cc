#include "w5500.h"
#include <cstring>
#include <esp_mac.h>
#include <freertos/task.h>

W5500::W5500(spi_host_device_t spi, 
             gpio_num_t miso, gpio_num_t mosi, gpio_num_t sclk, gpio_num_t cs, gpio_num_t rst,
             uint32_t sclkFreq, SemaphoreHandle_t mux)
    : cs_(cs), rst_(rst), mux_(mux), portMux_(portMUX_INITIALIZER_UNLOCKED)
{
    spi_bus_config_t buscfg;
    spi_device_interface_config_t devcfg;

    memset(&buscfg, 0, sizeof(buscfg));
    memset(&devcfg, 0, sizeof(devcfg));

    buscfg.mosi_io_num = mosi;
    buscfg.miso_io_num = miso;
    buscfg.sclk_io_num = sclk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 64;

    devcfg.mode = 3;
    devcfg.clock_speed_hz = sclkFreq;
    devcfg.spics_io_num = -1;
    devcfg.flags = SPI_DEVICE_HALFDUPLEX;
    devcfg.queue_size = 1;

    gpio_set_direction(cs, GPIO_MODE_OUTPUT);
    gpio_set_level(cs, 1);
    spi_bus_initialize(spi, &buscfg, SPI_DMA_DISABLED);
    spi_bus_add_device(spi, &devcfg, &spi_);
}

void W5500::begin()
{
    int i = 0;
    uint8_t protocol = Sn_MR_MACRAW;
    uint8_t flag = Sn_MR_MFEN | Sn_MR_MMB | Sn_MR_MIP6B;
    uint8_t rxTxSizes[] = {16, 0, 0, 0, 0, 0, 0, 0};

    esp_read_mac(this->mac_, ESP_MAC_ETH);

    this->swReset();

    for (i = 0; i < 8; i++)
    {
        this->setSn_TXBUF_SIZE(i, rxTxSizes[i]);
        this->setSn_RXBUF_SIZE(i, rxTxSizes[i]);
    }
    this->setSn_MR(0, (protocol | flag));
    this->setSn_IR(0, 0xFF);
    this->setSHAR(this->mac_);
    this->setSn_CR(0, Sn_CR_OPEN);
    while (this->getSn_SR(0) != SOCK_MACRAW)
        ;
}

void W5500::spiWriteBurst(uint8_t *vals, uint16_t len)
{
    for (int i = 0; i < len; i++)
        this->spiWriteByte(vals[i]);
}

void W5500::spiReadBurst(uint8_t *vals, uint16_t len)
{
    for (int i = 0; i < len; i++)
        vals[i] = this->spiReadByte();
}

void W5500::spiWriteByte(uint8_t val)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = (uint32_t *)&val;
    spi_device_polling_transmit(this->spi_, &t);
}

uint8_t W5500::spiReadByte()
{
    uint32_t byte;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.rxlength = 8;
    t.rx_buffer = &byte;
    spi_device_polling_transmit(this->spi_, &t);
    return (uint8_t)byte;
}

void W5500::spiLock()
{
    xSemaphoreTake(this->mux_, portMAX_DELAY);
    taskENTER_CRITICAL(&this->portMux_);
}

void W5500::spiUnlock()
{
    xSemaphoreGive(this->mux_);
    taskEXIT_CRITICAL(&this->portMux_);
}

void W5500::spiSelect()
{
    gpio_set_level(this->cs_, 0);
}

void W5500::spiDeselect()
{
    gpio_set_level(this->cs_, 1);
}

uint8_t W5500::readByte(uint32_t addr)
{
    uint8_t ret;
    uint8_t spi_data[3];

    this->spiLock();
    this->spiSelect();

    addr |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);

    spi_data[0] = (addr & 0x00FF0000) >> 16;
    spi_data[1] = (addr & 0x0000FF00) >> 8;
    spi_data[2] = (addr & 0x000000FF) >> 0;

    this->spiWriteBurst(spi_data, 3);
    ret = this->spiReadByte();

    this->spiDeselect();
    this->spiUnlock();

    return ret;
}

void W5500::writeByte(uint32_t addr, uint8_t val)
{
    uint8_t spi_data[4];

    this->spiLock();
    this->spiSelect();

    addr |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);

    spi_data[0] = (addr & 0x00FF0000) >> 16;
    spi_data[1] = (addr & 0x0000FF00) >> 8;
    spi_data[2] = (addr & 0x000000FF) >> 0;
    spi_data[3] = val;

    this->spiWriteBurst(spi_data, 4);

    this->spiDeselect();
    this->spiUnlock();
}

void W5500::readBurst(uint32_t addr, uint8_t *dest, uint16_t len)
{
    uint8_t spi_data[3];

    this->spiLock();
    this->spiSelect();

    addr |= (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_);

    spi_data[0] = (addr & 0x00FF0000) >> 16;
    spi_data[1] = (addr & 0x0000FF00) >> 8;
    spi_data[2] = (addr & 0x000000FF) >> 0;
    this->spiWriteBurst(spi_data, 3);
    this->spiReadBurst(dest, len);

    this->spiDeselect();
    this->spiUnlock();
}

void W5500::writeBurst(uint32_t addr, uint8_t *vals, uint16_t len)
{
    uint8_t spi_data[3];

    this->spiLock();
    this->spiSelect();

    addr |= (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_);

    spi_data[0] = (addr & 0x00FF0000) >> 16;
    spi_data[1] = (addr & 0x0000FF00) >> 8;
    spi_data[2] = (addr & 0x000000FF) >> 0;
    this->spiWriteBurst(spi_data, 3);
    this->spiWriteBurst(vals, len);

    this->spiDeselect();
    this->spiUnlock();
}

void W5500::swReset()
{
    uint8_t gw[4], sn[4], sip[4];
    uint8_t mac[6];
    uint8_t tmp = this->getPHYCFGR();

    tmp &= PHYCFGR_RST;
    this->setPHYCFGR(tmp);
    tmp = this->getPHYCFGR();
    tmp |= ~PHYCFGR_RST;
    this->setPHYCFGR(tmp);

    this->getSHAR(mac);
    this->getGAR(gw);
    this->getSUBR(sn);
    this->getSIPR(sip);
    this->setMR(MR_RST);
    this->getMR();
    this->setSHAR(mac);
    this->setGAR(gw);
    this->setSUBR(sn);
    this->setSIPR(sip);
}

void W5500::hwReset()
{
    gpio_set_level(this->rst_, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(this->rst_, 1);
    this->begin();
}

bool W5500::connected()
{
    if (this->getPHYCFGR() & PHYCFGR_LNK_ON)
        return true;
    return false;
}

void W5500::getMac(uint8_t *mac)
{
    int i;
    for (i = 0; i < 6; i++)
        mac[i] = this->mac_[i];
}

uint16_t W5500::getSn_TX_FSR(uint8_t sn)
{
    uint16_t val = 0, val1 = 0;

    do
    {
        val1 = this->readByte(Sn_TX_FSR(sn));
        val1 = (val1 << 8) + this->readByte(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn), 1));
        if (val1 != 0)
        {
            val = this->readByte(Sn_TX_FSR(sn));
            val = (val << 8) + this->readByte(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn), 1));
        }
    } while (val != val1);
    return val;
}

uint16_t W5500::getSn_RX_RSR(uint8_t sn)
{
    uint16_t val = 0, val1 = 0;

    do
    {
        val1 = this->readByte(Sn_RX_RSR(sn));
        val1 = (val1 << 8) + this->readByte(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn), 1));
        if (val1 != 0)
        {
            val = this->readByte(Sn_RX_RSR(sn));
            val = (val << 8) + this->readByte(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn), 1));
        }
    } while (val != val1);
    return val;
}