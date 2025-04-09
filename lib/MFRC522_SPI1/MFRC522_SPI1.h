#ifndef MFRC522_SPI1_H
#define MFRC522_SPI1_H

#include <Arduino.h>
#include <hardware/spi.h> // Thư viện SPI cho RP2040
#include <MFRC522.h>

class MFRC522_SPI1 : public MFRC522 {
public:
    MFRC522_SPI1(byte ssPin, byte rstPin, spi_inst_t *spiInstance, byte sck, byte mosi, byte miso);

    void begin(uint32_t spiSpeed);
    void PCD_WriteRegister(byte reg, byte value);
    byte PCD_ReadRegister(byte reg);

    // Override PCD_Init để tránh gọi SPI.begin()
    void PCD_Init();

private:
    spi_inst_t* spiInstance;
    byte _sck, _mosi, _miso, _ssPin;
};

#endif
