/**
 * This is a driver library for VS1053 MP3 Codec Breakout
 * (Ogg Vorbis / MP3 / AAC / WMA / FLAC / MIDI Audio Codec Chip).
 * Adapted for Espressif ESP8266 and ESP32 boards.
 *
 * version 1.0.1
 *
 * Licensed under GNU GPLv3 <http://gplv3.fsf.org/>
 * Copyright © 2017
 *
 * @authors baldram, edzelf, MagicCube, maniacbug
 *
 * Development log:
 *  - 2011: initial VS1053 Arduino library
 *          originally written by J. Coliz (github: @maniacbug),
 *  - 2016: refactored and integrated into Esp-radio sketch
 *          by Ed Smallenburg (github: @edzelf)
 *  - 2017: refactored to use as PlatformIO library
 *          by Marcin Szalomski (github: @baldram | twitter: @baldram)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License or later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VS1053_H
#define VS1053_H

#include <Arduino.h>
#include <SPI.h>
#include "ConsoleLogger.h"

#include "patches/vs1053b-patches.h"

enum VS1053_I2S_RATE {
    VS1053_I2S_RATE_192_KHZ,
    VS1053_I2S_RATE_96_KHZ,
    VS1053_I2S_RATE_48_KHZ
};

class VS1053 {
private:
    uint8_t cs_pin;                         // Pin where CS line is connected
    uint8_t dcs_pin;                        // Pin where DCS line is connected
    uint8_t dreq_pin;                       // Pin where DREQ line is connected
    uint8_t curvol;                         // Current volume setting 0..100%
    int8_t  curbalance = 0;                 // Current balance setting -100..100
                                            // (-100 = right channel silent, 100 = left channel silent)
    const uint8_t vs1053_chunk_size = 32;
    // SCI Register
    const uint8_t SCI_MODE = 0x0;
    const uint8_t SCI_STATUS = 0x1;
    const uint8_t SCI_BASS = 0x2;
    const uint8_t SCI_CLOCKF = 0x3;
    const uint8_t SCI_DECODE_TIME = 0x4;        // current decoded time in full seconds
    const uint8_t SCI_AUDATA = 0x5;
    const uint8_t SCI_WRAM = 0x6;
    const uint8_t SCI_WRAMADDR = 0x7;
    const uint8_t SCI_AIADDR = 0xA;
    const uint8_t SCI_VOL = 0xB;
    const uint8_t SCI_AICTRL0 = 0xC;
    const uint8_t SCI_AICTRL1 = 0xD;
    const uint8_t SCI_num_registers = 0xF;
    // SCI_MODE bits
    const uint8_t SM_SDINEW = 11;           // Bitnumber in SCI_MODE always on
    const uint8_t SM_RESET = 2;             // Bitnumber in SCI_MODE soft reset
    const uint8_t SM_CANCEL = 3;            // Bitnumber in SCI_MODE cancel song
    const uint8_t SM_TESTS = 5;             // Bitnumber in SCI_MODE for tests
    const uint8_t SM_LINE1 = 14;            // Bitnumber in SCI_MODE for Line input
    const uint8_t SM_STREAM = 6;            // Bitnumber in SCI_MODE for Streaming Mode

    const uint16_t ADDR_REG_GPIO_DDR_RW = 0xc017;
    const uint16_t ADDR_REG_GPIO_VAL_R = 0xc018;
    const uint16_t ADDR_REG_GPIO_ODATA_RW = 0xc019;
    const uint16_t ADDR_REG_I2S_CONFIG_RW = 0xc040;

    SPISettings VS1053_SPI;                 // SPI settings for this slave
    uint8_t endFillByte;                    // Byte to send when stopping song
public:
    VS1053(uint8_t _cs_pin, uint8_t _dcs_pin, uint8_t _dreq_pin);

    uint16_t readRegister(uint8_t _reg) const;
    void writeRegister(uint8_t _reg, uint16_t _value) const;
    uint16_t read_register(uint8_t _reg) const;
    void write_register(uint8_t _reg, uint16_t _value) const;

    void begin();
    void startSong();
    void playChunk(uint8_t *data, size_t len);
    void stopSong();
    void setVolume(uint8_t vol);
    void setBalance(int8_t balance);
    void setTone(uint8_t *rtone);
    uint8_t getVolume();
    int8_t getBalance();
    void printDetails(const char *header);
    void softReset();
    bool testComm(const char *header);
    inline bool data_request() const {
        return (digitalRead(dreq_pin) == HIGH);
    }
    void adjustRate(long ppm2);
    void streamModeOn();
    void streamModeOff();
    void switchToMp3Mode();
    void disableI2sOut();
    void enableI2sOut(VS1053_I2S_RATE i2sRate = VS1053_I2S_RATE_48_KHZ);
    bool isChipConnected();
    uint16_t getChipVersion();
    uint16_t getDecodedTime();
    void clearDecodedTime();
    void loadUserCode(const unsigned short* plugin, unsigned short plugin_size);
    void loadDefaultVs1053Patches();

protected:
    inline void await_data_request() const {
        while (!digitalRead(dreq_pin)) {
            yield();                        // Very short delay
        }
    }

    inline void control_mode_on() const {
        SPI.beginTransaction(VS1053_SPI);   // Prevent other SPI users
        digitalWrite(dcs_pin, HIGH);        // Bring slave in control mode
        digitalWrite(cs_pin, LOW);
    }

    inline void control_mode_off() const {
        digitalWrite(cs_pin, HIGH);         // End control mode
        SPI.endTransaction();               // Allow other SPI users
    }

    inline void data_mode_on() const {
        SPI.beginTransaction(VS1053_SPI);   // Prevent other SPI users
        digitalWrite(cs_pin, HIGH);         // Bring slave in data mode
        digitalWrite(dcs_pin, LOW);
    }

    inline void data_mode_off() const {
        digitalWrite(dcs_pin, HIGH);        // End data mode
        SPI.endTransaction();               // Allow other SPI users
    }

    void sdi_send_buffer(uint8_t *data, size_t len);
    void sdi_send_fillers(size_t length);
    void wram_write(uint16_t address, uint16_t data);
    uint16_t wram_read(uint16_t address);
};
#endif