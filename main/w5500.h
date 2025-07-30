#include <cstdint>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <freertos/semphr.h>

#define _W5500_IO_BASE_ 0x00000000

#define _W5500_SPI_READ_ (0x00 << 2)  //< SPI interface Read operation in Control Phase
#define _W5500_SPI_WRITE_ (0x01 << 2) //< SPI interface Write operation in Control Phase

#define WIZCHIP_CREG_BLOCK 0x00            //< Common register block
#define WIZCHIP_SREG_BLOCK(N) (1 + 4 * N)  //< Socket N register block
#define WIZCHIP_TXBUF_BLOCK(N) (2 + 4 * N) //< Socket N Tx buffer address block
#define WIZCHIP_RXBUF_BLOCK(N) (3 + 4 * N) //< Socket N Rx buffer address block

#define _W5500_SPI_VDM_OP_ 0x00
#define _W5500_SPI_FDM_OP_LEN1_ 0x01
#define _W5500_SPI_FDM_OP_LEN2_ 0x02
#define _W5500_SPI_FDM_OP_LEN4_ 0x03

#define WIZCHIP_OFFSET_INC(ADDR, N) (ADDR + (N << 8))

#define MR (_W5500_IO_BASE_ + (0x0000 << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define GAR (_W5500_IO_BASE_ + (0x0001 << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define SUBR (_W5500_IO_BASE_ + (0x0005 << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define SHAR (_W5500_IO_BASE_ + (0x0009 << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define SIPR (_W5500_IO_BASE_ + (0x000F << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define INTLEVEL (_W5500_IO_BASE_ + (0x0013 << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define IR (_W5500_IO_BASE_ + (0x0015 << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define _IMR_ (_W5500_IO_BASE_ + (0x0016 << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define SIR (_W5500_IO_BASE_ + (0x0017 << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define SIMR (_W5500_IO_BASE_ + (0x0018 << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define _RTR_ (_W5500_IO_BASE_ + (0x0019 << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define _RCR_ (_W5500_IO_BASE_ + (0x001B << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define PTIMER (_W5500_IO_BASE_ + (0x001C << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define PMAGIC (_W5500_IO_BASE_ + (0x001D << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define PHAR (_W5500_IO_BASE_ + (0x001E << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define PSID (_W5500_IO_BASE_ + (0x0024 << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define PMRU (_W5500_IO_BASE_ + (0x0026 << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define UIPR (_W5500_IO_BASE_ + (0x0028 << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define UPORTR (_W5500_IO_BASE_ + (0x002C << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define PHYCFGR (_W5500_IO_BASE_ + (0x002E << 8) + (WIZCHIP_CREG_BLOCK << 3))
#define VERSIONR (_W5500_IO_BASE_ + (0x0039 << 8) + (WIZCHIP_CREG_BLOCK << 3))

#define Sn_MR(N) (_W5500_IO_BASE_ + (0x0000 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_CR(N) (_W5500_IO_BASE_ + (0x0001 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_IR(N) (_W5500_IO_BASE_ + (0x0002 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_SR(N) (_W5500_IO_BASE_ + (0x0003 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_PORT(N) (_W5500_IO_BASE_ + (0x0004 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_DHAR(N) (_W5500_IO_BASE_ + (0x0006 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_DIPR(N) (_W5500_IO_BASE_ + (0x000C << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_DPORT(N) (_W5500_IO_BASE_ + (0x0010 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_MSSR(N) (_W5500_IO_BASE_ + (0x0012 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_TOS(N) (_W5500_IO_BASE_ + (0x0015 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_TTL(N) (_W5500_IO_BASE_ + (0x0016 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_RXBUF_SIZE(N) (_W5500_IO_BASE_ + (0x001E << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_TXBUF_SIZE(N) (_W5500_IO_BASE_ + (0x001F << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_TX_FSR(N) (_W5500_IO_BASE_ + (0x0020 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_TX_RD(N) (_W5500_IO_BASE_ + (0x0022 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_TX_WR(N) (_W5500_IO_BASE_ + (0x0024 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_RX_RSR(N) (_W5500_IO_BASE_ + (0x0026 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_RX_RD(N) (_W5500_IO_BASE_ + (0x0028 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_RX_WR(N) (_W5500_IO_BASE_ + (0x002A << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_IMR(N) (_W5500_IO_BASE_ + (0x002C << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_FRAG(N) (_W5500_IO_BASE_ + (0x002D << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
#define Sn_KPALVTR(N) (_W5500_IO_BASE_ + (0x002F << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

#define MR_RST 0x80
#define MR_WOL 0x20
#define MR_PB 0x10
#define MR_PPPOE 0x08
#define MR_FARP 0x02

#define IR_CONFLICT 0x80
#define IR_UNREACH 0x40
#define IR_PPPoE 0x20
#define IR_MP 0x10

#define PHYCFGR_RST ~(1 << 7) //< For PHY reset, must operate AND mask.
#define PHYCFGR_OPMD (1 << 6) // Configre PHY with OPMDC value
#define PHYCFGR_OPMDC_ALLA (7 << 3)
#define PHYCFGR_OPMDC_PDOWN (6 << 3)
#define PHYCFGR_OPMDC_NA (5 << 3)
#define PHYCFGR_OPMDC_100FA (4 << 3)
#define PHYCFGR_OPMDC_100F (3 << 3)
#define PHYCFGR_OPMDC_100H (2 << 3)
#define PHYCFGR_OPMDC_10F (1 << 3)
#define PHYCFGR_OPMDC_10H (0 << 3)
#define PHYCFGR_DPX_FULL (1 << 2)
#define PHYCFGR_DPX_HALF (0 << 2)
#define PHYCFGR_SPD_100 (1 << 1)
#define PHYCFGR_SPD_10 (0 << 1)
#define PHYCFGR_LNK_ON (1 << 0)
#define PHYCFGR_LNK_OFF (0 << 0)

#define IM_IR7 0x80
#define IM_IR6 0x40
#define IM_IR5 0x20
#define IM_IR4 0x10

#define Sn_MR_MULTI 0x80
#define Sn_MR_BCASTB 0x40
#define Sn_MR_ND 0x20
#define Sn_MR_UCASTB 0x10
#define Sn_MR_MACRAW 0x04
#define Sn_MR_IPRAW 0x03
#define Sn_MR_UDP 0x02
#define Sn_MR_TCP 0x01
#define Sn_MR_CLOSE 0x00
#define Sn_MR_MFEN Sn_MR_MULTI
#define Sn_MR_MMB Sn_MR_ND
#define Sn_MR_MIP6B Sn_MR_UCASTB
#define Sn_MR_MC Sn_MR_ND

#define SOCK_STREAM Sn_MR_TCP
#define SOCK_DGRAM Sn_MR_UDP

#define Sn_CR_OPEN 0x01
#define Sn_CR_LISTEN 0x02
#define Sn_CR_CONNECT 0x04
#define Sn_CR_DISCON 0x08
#define Sn_CR_CLOSE 0x10
#define Sn_CR_SEND 0x20
#define Sn_CR_SEND_MAC 0x21
#define Sn_CR_SEND_KEEP 0x22
#define Sn_CR_RECV 0x40

#define Sn_IR_SENDOK 0x10
#define Sn_IR_TIMEOUT 0x08
#define Sn_IR_RECV 0x04
#define Sn_IR_DISCON 0x02
#define Sn_IR_CON 0x01

#define SOCK_CLOSED 0x00
#define SOCK_INIT 0x13
#define SOCK_LISTEN 0x14
#define SOCK_SYNSENT 0x15
#define SOCK_SYNRECV 0x16
#define SOCK_ESTABLISHED 0x17
#define SOCK_FIN_WAIT 0x18
#define SOCK_CLOSING 0x1A
#define SOCK_TIME_WAIT 0x1B
#define SOCK_CLOSE_WAIT 0x1C
#define SOCK_LAST_ACK 0x1D
#define SOCK_UDP 0x22
#define SOCK_IPRAW 0x32
#define SOCK_MACRAW 0x42

#define IPPROTO_IP 0    //< Dummy for IP
#define IPPROTO_ICMP 1  //< Control message protocol
#define IPPROTO_IGMP 2  //< Internet group management protocol
#define IPPROTO_GGP 3   //< Gateway^2 (deprecated)
#define IPPROTO_TCP 6   //< TCP
#define IPPROTO_PUP 12  //< PUP
#define IPPROTO_UDP 17  //< UDP
#define IPPROTO_IDP 22  //< XNS idp
#define IPPROTO_ND 77   //< UNOFFICIAL net disk protocol
#define IPPROTO_RAW 255 //< Raw IP packet

class W5500
{
private:
    void spiWriteByte(uint8_t val);
    uint8_t spiReadByte();
    void spiWriteBurst(uint8_t *vals, uint16_t len);
    void spiReadBurst(uint8_t *dest, uint16_t len);
    void spiSelect();
    void spiDeselect();
    void spiLock();
    void spiUnlock();

    spi_device_handle_t spi_;
    gpio_num_t cs_;
    gpio_num_t rst_;
    uint8_t mac_[6];
    SemaphoreHandle_t mux_;
    portMUX_TYPE portMux_;

public:
    W5500(spi_host_device_t spi, gpio_num_t miso, gpio_num_t mosi, gpio_num_t sclk, gpio_num_t cs, gpio_num_t rst,
          uint32_t sclk_freq, SemaphoreHandle_t mux);
    void begin();
    void swReset();
    void hwReset();
    bool connected();

    void getMac(uint8_t *mac);
    uint16_t getSn_TX_FSR(uint8_t sn);
    uint16_t getSn_RX_RSR(uint8_t sn);

    void writeByte(uint32_t addr, uint8_t val);
    uint8_t readByte(uint32_t addr);
    void writeBurst(uint32_t addr, uint8_t *vals, uint16_t len);
    void readBurst(uint32_t addr, uint8_t *dest, uint16_t len);

    void setMR(uint8_t mr)
    {
        this->writeByte(MR, mr);
    }

    uint8_t getMR()
    {
        return this->readByte(MR);
    }

    void setGAR(uint8_t *gar)
    {
        this->writeBurst(GAR, gar, 4);
    }

    void getGAR(uint8_t *gar)
    {
        this->readBurst(GAR, gar, 4);
    }

    void setSUBR(uint8_t *subr)
    {
        this->writeBurst(SUBR, subr, 4);
    }

    void getSUBR(uint8_t *subr)
    {
        this->readBurst(SUBR, subr, 4);
    }

    void setSHAR(uint8_t *shar)
    {
        this->writeBurst(SHAR, shar, 6);
    }

    void getSHAR(uint8_t *shar)
    {
        this->readBurst(SHAR, shar, 6);
    }

    void setSIPR(uint8_t *sipr)
    {
        this->writeBurst(SIPR, sipr, 4);
    }

    void getSIPR(uint8_t *sipr)
    {
        this->readBurst(SIPR, sipr, 4);
    }

    void setINTLEVEL(uint16_t intlevel)
    {
        this->writeByte(INTLEVEL, (uint8_t)(intlevel >> 8));
        this->writeByte(WIZCHIP_OFFSET_INC(INTLEVEL, 1), (uint8_t)intlevel);
    }

    uint16_t getINTLEVEL()
    {
        return (((uint16_t)this->readByte(INTLEVEL) << 8) + this->readByte(WIZCHIP_OFFSET_INC(INTLEVEL, 1)));
    }

    void setIR(uint8_t ir)
    {
        this->writeByte(IR, (ir & 0xF0));
    }

    uint8_t getIR()
    {
        return this->readByte(IR) & 0xF0;
    }

    void setIMR(uint8_t imr)
    {
        this->writeByte(_IMR_, imr);
    }

    uint8_t getIMR()
    {
        return this->readByte(_IMR_);
    }

    void setSIR(uint8_t sir)
    {
        this->writeByte(SIR, sir);
    }

    uint8_t getSIR()
    {
        return this->readByte(SIR);
    }

    void setSIMR(uint8_t simr)
    {
        this->writeByte(SIMR, simr);
    }

    uint8_t getSIMR()
    {
        return this->readByte(SIMR);
    }

    void setRTR(uint16_t rtr)
    {
        this->writeByte(_RTR_, (uint8_t)(rtr >> 8));
        this->writeByte(WIZCHIP_OFFSET_INC(_RTR_, 1), (uint8_t)rtr);
    }

    uint16_t getRTR()
    {
        return (((uint16_t)this->readByte(_RTR_) << 8) + this->readByte(WIZCHIP_OFFSET_INC(_RTR_, 1)));
    }

    void setRCR(uint8_t rcr)
    {
        this->writeByte(_RCR_, rcr);
    }

    uint8_t getRCR()
    {
        return this->readByte(_RCR_);
    }

    void setPTIMER(uint8_t ptimer)
    {
        this->writeByte(PTIMER, ptimer);
    }

    uint8_t getPTIMER()
    {
        return this->readByte(PTIMER);
    }

    void setPMAGIC(uint8_t pmagic)
    {
        this->writeByte(PMAGIC, pmagic);
    }

    uint8_t getPMAGIC()
    {
        return this->readByte(PMAGIC);
    }

    void setPHAR(uint8_t *phar)
    {
        this->writeBurst(PHAR, phar, 6);
    }

    void getPHAR(uint8_t *phar)
    {
        this->readBurst(PHAR, phar, 6);
    }

    void setPSID(uint16_t psid)
    {
        this->writeByte(PSID, (uint8_t)(psid >> 8));
        this->writeByte(WIZCHIP_OFFSET_INC(PSID, 1), (uint8_t)psid);
    }

    uint16_t getPSID()
    {
        return (((uint16_t)this->readByte(PSID) << 8) + this->readByte(WIZCHIP_OFFSET_INC(PSID, 1)));
    }

    void setPMRU(uint16_t pmru)
    {
        this->writeByte(PMRU, (uint8_t)(pmru >> 8));
        this->writeByte(WIZCHIP_OFFSET_INC(PMRU, 1), (uint8_t)pmru);
    }

    uint16_t getPMRU()
    {
        return (((uint16_t)this->readByte(PMRU) << 8) + this->readByte(WIZCHIP_OFFSET_INC(PMRU, 1)));
    }

    void getUIPR(uint8_t *uipr)
    {
        this->readBurst(UIPR, uipr, 4);
    }

    uint16_t getUPORTR()
    {
        return (((uint16_t)this->readByte(UPORTR) << 8) + this->readByte(WIZCHIP_OFFSET_INC(UPORTR, 1)));
    }

    void setPHYCFGR(uint8_t phycfgr)
    {
        this->writeByte(PHYCFGR, phycfgr);
    }

    uint8_t getPHYCFGR()
    {
        return this->readByte(PHYCFGR);
    }

    uint8_t getVERSIONR()
    {
        return this->readByte(VERSIONR);
    }

    void setSn_MR(uint8_t sn, uint8_t mr)
    {
        this->writeByte(Sn_MR(sn), mr);
    }

    uint8_t getSn_MR(uint8_t sn)
    {
        return this->readByte(Sn_MR(sn));
    }

    void setSn_CR(uint8_t sn, uint8_t cr)
    {
        this->writeByte(Sn_CR(sn), cr);
    }

    uint8_t getSn_CR(uint8_t sn)
    {
        return this->readByte(Sn_CR(sn));
    }

    void setSn_IR(uint8_t sn, uint8_t ir)
    {
        this->writeByte(Sn_IR(sn), (ir & 0x1F));
    }

    uint8_t getSn_IR(uint8_t sn)
    {
        return (this->readByte(Sn_IR(sn)) & 0x1F);
    }

    void setSn_IMR(uint8_t sn, uint8_t imr)
    {
        this->writeByte(Sn_IMR(sn), (imr & 0x1F));
    }

    uint8_t getSn_IMR(uint8_t sn)
    {
        return (this->readByte(Sn_IMR(sn)) & 0x1F);
    }

    uint8_t getSn_SR(uint8_t sn)
    {
        return this->readByte(Sn_SR(sn));
    }

    void setSn_PORT(uint8_t sn, uint16_t port)
    {
        this->writeByte(Sn_PORT(sn), (uint8_t)(port >> 8));
        this->writeByte(WIZCHIP_OFFSET_INC(Sn_PORT(sn), 1), (uint8_t)port);
    }

    uint16_t getSn_PORT(uint8_t sn)
    {
        return (((uint16_t)this->readByte(Sn_PORT(sn)) << 8) + this->readByte(WIZCHIP_OFFSET_INC(Sn_PORT(sn), 1)));
    }

    void setSn_DHAR(uint8_t sn, uint8_t *dhar)
    {
        this->writeBurst(Sn_DHAR(sn), dhar, 6);
    }

    void getSn_DHAR(uint8_t sn, uint8_t *dhar)
    {
        this->readBurst(Sn_DHAR(sn), dhar, 6);
    }

    void setSn_DIPR(uint8_t sn, uint8_t *dipr)
    {
        this->writeBurst(Sn_DIPR(sn), dipr, 4);
    }

    void getSn_DIPR(uint8_t sn, uint8_t *dipr)
    {
        this->readBurst(Sn_DIPR(sn), dipr, 4);
    }

    void setSn_DPORT(uint8_t sn, uint16_t dport)
    {
        this->writeByte(Sn_DPORT(sn), (uint8_t)(dport >> 8));
        this->writeByte(WIZCHIP_OFFSET_INC(Sn_DPORT(sn), 1), (uint8_t)dport);
    }

    uint16_t getSn_DPORT(uint8_t sn)
    {
        return (((uint16_t)this->readByte(Sn_DPORT(sn)) << 8) + this->readByte(WIZCHIP_OFFSET_INC(Sn_DPORT(sn), 1)));
    }

    void setSn_MSSR(uint8_t sn, uint16_t mss)
    {
        this->writeByte(Sn_MSSR(sn), (uint8_t)(mss >> 8));
        this->writeByte(WIZCHIP_OFFSET_INC(Sn_MSSR(sn), 1), (uint8_t)mss);
    }

    uint16_t getSn_MSSR(uint8_t sn)
    {
        return (((uint16_t)this->readByte(Sn_MSSR(sn)) << 8) + this->readByte(WIZCHIP_OFFSET_INC(Sn_MSSR(sn), 1)));
    }

    void setSn_TOS(uint8_t sn, uint8_t tos)
    {
        this->writeByte(Sn_TOS(sn), tos);
    }

    uint8_t getSn_TOS(uint8_t sn)
    {
        return this->readByte(Sn_TOS(sn));
    }

    void setSn_TTL(uint8_t sn, uint8_t ttl)
    {
        this->writeByte(Sn_TTL(sn), ttl);
    }

    uint8_t getSn_TTL(uint8_t sn)
    {
        return this->readByte(Sn_TTL(sn));
    }

    void setSn_RXBUF_SIZE(uint8_t sn, uint8_t rxbufsize)
    {
        this->writeByte(Sn_RXBUF_SIZE(sn), rxbufsize);
    }

    uint8_t getSn_RXBUF_SIZE(uint8_t sn)
    {
        return this->readByte(Sn_RXBUF_SIZE(sn));
    }

    void setSn_TXBUF_SIZE(uint8_t sn, uint8_t txbufsize)
    {
        this->writeByte(Sn_TXBUF_SIZE(sn), txbufsize);
    }

    uint8_t getSn_TXBUF_SIZE(uint8_t sn)
    {
        return this->readByte(Sn_TXBUF_SIZE(sn));
    }

    uint16_t getSn_TX_RD(uint8_t sn)
    {
        return (((uint16_t)this->readByte(Sn_TX_RD(sn)) << 8) + this->readByte(WIZCHIP_OFFSET_INC(Sn_TX_RD(sn), 1)));
    }

    void setSn_TX_WR(uint8_t sn, uint16_t txwr)
    {
        this->writeByte(Sn_TX_WR(sn), (uint8_t)(txwr >> 8));
        this->writeByte(WIZCHIP_OFFSET_INC(Sn_TX_WR(sn), 1), (uint8_t)txwr);
    }

    uint16_t getSn_TX_WR(uint8_t sn)
    {
        return (((uint16_t)this->readByte(Sn_TX_WR(sn)) << 8) + this->readByte(WIZCHIP_OFFSET_INC(Sn_TX_WR(sn), 1)));
    }

    void setSn_RX_RD(uint8_t sn, uint16_t rxrd)
    {
        this->writeByte(Sn_RX_RD(sn), (uint8_t)(rxrd >> 8));
        this->writeByte(WIZCHIP_OFFSET_INC(Sn_RX_RD(sn), 1), (uint8_t)rxrd);
    }

    uint16_t getSn_RX_RD(uint8_t sn)
    {
        return (((uint16_t)this->readByte(Sn_RX_RD(sn)) << 8) + this->readByte(WIZCHIP_OFFSET_INC(Sn_RX_RD(sn), 1)));
    }

    uint16_t getSn_RX_WR(uint8_t sn)
    {
        return (((uint16_t)this->readByte(Sn_RX_WR(sn)) << 8) + this->readByte(WIZCHIP_OFFSET_INC(Sn_RX_WR(sn), 1)));
    }

    void setSn_FRAG(uint8_t sn, uint16_t frag)
    {
        this->writeByte(Sn_FRAG(sn), (uint8_t)(frag >> 8));
        this->writeByte(WIZCHIP_OFFSET_INC(Sn_FRAG(sn), 1), (uint8_t)frag);
    }

    uint16_t getSn_FRAG(uint8_t sn)
    {
        return (((uint16_t)this->readByte(Sn_FRAG(sn)) << 8) + this->readByte(WIZCHIP_OFFSET_INC(Sn_FRAG(sn), 1)));
    }

    void setSn_KPALVTR(uint8_t sn, uint8_t kpalvt)
    {
        this->writeByte(Sn_KPALVTR(sn), kpalvt);
    }

    uint8_t getSn_KPALVTR(uint8_t sn)
    {
        return this->readByte(Sn_KPALVTR(sn));
    }

    uint16_t getSn_RxMAX(uint8_t sn)
    {
        return (((uint16_t)this->getSn_RXBUF_SIZE(sn)) << 10);
    }

    uint16_t getSn_TxMAX(uint8_t sn)
    {
        return (((uint16_t)this->getSn_TXBUF_SIZE(sn)) << 10);
    }
};