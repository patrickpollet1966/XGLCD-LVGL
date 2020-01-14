/*
 XGLCD-LVGL library 
 LCD library for LCD modules with RA8875 driver using LittleVGL

 Based on XVGL from XGraph
 Copyright (c) 2016-2019 DELCOMp bvba / UVee bvba
 2sd (a t) delcomp (d o t) com
 
 Targets: RA8875 LCD module with FT5206 capacitive touchscreen (Resitive touchscreen managed directly by RA8875 code is commented)
 
 Optimized code is used to control the RA8875 chip with the hardware SPI bus of the CPU boards:
 - Arduino MKR family
 - ESP32
 - Pycom
 (- Particle Proton/Electron)
 - Particle Argon/Boron
 - nRF528xx
 - Arduino headers (note that the library is 32-bit optimised, it will run slow on 8-bit cpu's)
 
 This library contains functions:
 - generic for all RA8875 features using RA8875 hardware acceleration and SPI-bus optimalisations for using LittleVGL
 - low-level driver for Littlevgl
 
 This library is based on the RA8875 library from https://github.com/sumotoy/RA8875
 
 License:GNU General Public License v3.0
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 SPI BUS WARNINGS:
 The LCD driver and SDCard socket are routed to the same SPI bus. Interrupts are NOT disabled during SPI bus
 transmissions because this results in hang ups on some CPU modules due to the prolonged no-interrupt
 period this causes.
 Make sure your app does NOT interrupt routines to use the SPI bus as this will cause problems with the LCD & SDCard functions
 
 WARNING: SPI_HAS_TRANSACTIONS is not used. This is because of the embedded SPI code to maximalise the
 SPI bus speed and thus the LCD drawing speed. Just make sure the LCD SPI bus is not used in interrupt routines.
 
 WARNING: SD.begin(XG_SD_CS) MUST be added AFTER tft.begin()
 If not included you might get weird behavior depending on the brand of the inserted SDCard.
*/

/* To make the compiled code smaller this library includes only the minimal code to use RA8875 with LittleVGL it's not a full RA8875 library
*/

#ifndef _XGLCD_H_
#define _XGLCD_H_

/* BEGIN OF USER DEFINES ************************************************************************************/
// If you do not want to use LCD auto-detection, manual enable one (and only one) of below LCD types
// #define XG_LCD_50             // 5.0" 800x480 LCD
// #define XG_LCD_56             // 5.6" 640x480 LCD
// #define XG_LCD_90             // 7.0" 800x480 LCD
// #define XG_LCD_90             // 9.0" 800x480 LCD
/* END OF USER DEFINES ************************************************************************************/


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            HOUSEKEEPING                                     +
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <stdio.h>
//XG// #include "Print.h"

#if defined(_FORCE_PROGMEM__)                                                   // Disable PROGMEM which is not needed on 32-bit CPU's with a lot of RAM
    #undef _FORCE_PROGMEM__
#endif
#define __PRGMTAG_

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            IO PIN ABSTRACTION                               +
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// LCD DETECTION
#if !defined(XG_LCD_50) && !defined(XG_LCD_56) && !defined(XG_LCD_90)
    #define XG_LCD_AUTO
#endif

// CPU module auto-detection based on the user selected board type in the Arduino IDE
#if defined(ARDUINO_ARCH_SAMD)                                                  // Arduino MKR family (and all other SAMD modules, but only the MKR will work)
    #define XG_CPU_MKR
#elif defined(ARDUINO_ESP32_DEV)                                                // ESP32 Development Kit C / Pycom modules (without Python of course)
// Pycom modules (the latest only) have 8Mbit Flash and 4Mbit PSRAM
    #define XG_CPU_ESP32
#elif defined(PLATFORM_ID)                                                      // Particle Family, this library should be used in the Particle IDE not in the Arduino IDE
// PLATFORM_PHOTON_PRODUCTION
// PLATFORM_ELECTRON_PRODUCTION
// PLATFORM_ARGON -> NRF5 based
// PLATFORM_BORON -> NRF5 based
// PLATFORM_XENON
    #define XG_CPU_PARTICLE
#elif defined(FANSTEL)                                                          // Fanstel module
//#elif defined(ARDUINO_ARCH_NRF5)                                              // Nordic NRF5x family
    #define XG_CPU_FANSTEL
#else                                                                           // All other boards presume the Arduino headers are used, 32-bit boards work better, generic SPI commands are used
    #define XG_CPU_ARDUINO
#endif

// Max SPI speed RA8875 (with system clock @ 60MHz):
// Write speed: 20MHz
// Read speed: 4Mhz
// The maximum read speed is theoretically 10MHz. At that speed it fails for example during touchscreen
// coordinate reads. The read speed should never be higher then 4Mhz.

#if defined(XG_CPU_MKR)                                                         // Arduino MKR family

    #include <Arduino.h>
    #include <pins_arduino.h>
    #include <SPI.h>

    #define SPI_SPEED_WRITE                 4                                   // 12 MHz which is the max for the __SMAD21G18A__ CPU (bug in chip cause 2 should be possible)
    #define SPI_SPEED_READ                  16                                  // 3 MHz @ 48MHz CPU clock
    #define SPI_SPEED_SLOW                  32                                  // must be slower then 2 MHz before PLL is configured -> 1.5MHz @ 48MHz CPU clock
    #define SPI_MODE_LCD                    SPI_MODE3

    //#define _spiwrite(c)                    _writeMKRSPI(c)
    //#define _spiwrite16(d)                  _writeMKRSPI16(d)
    #define _spiwrite(c)                    {SERCOM1->SPI.DATA.bit.DATA = c;while(SERCOM1->SPI.INTFLAG.bit.TXC == 0);}
    #define _spiwrite16(c)                  {SERCOM1->SPI.DATA.bit.DATA = c>>8;SERCOM1->SPI.DATA.bit.DATA = c & 0xff;while(SERCOM1->SPI.INTFLAG.bit.TXC == 0);}
    #define _spiwritedma(wbuf, length)      {uint16_t i;for (i=0; i<length; i++) {while(SERCOM1->SPI.INTFLAG.bit.DRE == 0);SERCOM1->SPI.DATA.bit.DATA = buf[i];}while(SERCOM1->SPI.INTFLAG.bit.TXC == 0);}

    #define _spiread(r)                     r = SPI.transfer(0x00);
    #define _spibegin()                     SPI.begin()
    #define _spisetDataMode(datamode)       SPI.setDataMode(datamode)
    #define _spisetBitOrder(order)          SPI.setBitOrder(order)
    #define _spisetSpeed(s)                 SPI.setClockDivider(s)

    #define _spiCSLow                       PORT->Group[g_APinDescription[XG_PIN_LCD_CS].ulPort].OUTCLR.reg = (1UL << g_APinDescription[XG_PIN_LCD_CS].ulPin)
    #define _spiCSHigh                      PORT->Group[g_APinDescription[XG_PIN_LCD_CS].ulPort].OUTSET.reg = (1UL << g_APinDescription[XG_PIN_LCD_CS].ulPin)

    //LED = 6 (LED_BUILTIN)
    #define XG_PIN_BL (0u)
    #define XG_PIN_DC (1u)
    #define XG_PIN_LCD_RESET (4u)
    #define XG_PIN_SD_CS (3u)
    #define XG_PIN_TP_CS PIN_A3
    #define XG_PIN_TP_IRQ PIN_A2
    #define XG_PIN_LCD_CS (5u)
    #define XG_PIN_MOSI (8u)
    #define XG_PIN_MISO (10u)
    #define XG_PIN_SCK (9u)

    #define XG_PIN_MISO2 PIN_A4
    #define XG_PIN_MOSI2 PIN_A5
    #define XG_PIN_SCK2 PIN_A6
    #define XG_PIN_CS2 (2u)
    #define XG_PIN_D6 (6u)                                                      // also LED
    #define XG_PIN_D7 (7u)
    #define XG_PIN_SDA (11u)
    #define XG_PIN_SCL (12u)
    #define XG_PIN_RX (13u)
    #define XG_PIN_TX (14u)

    #define XG_PIN_A0 A4
    #define XG_PIN_A1 A5
    #define XG_PIN_A2 A6

#elif defined(XG_CPU_ESP32)                                                     // ESP32 Development Kit C / Pycom modules (without Python of course)

    #include <Arduino.h>
    #include <SPI.h>
    #include "soc/spi_struct.h"
    #include "driver/spi_common.h"
    #include "soc/dport_reg.h"

    #define SPI_SPEED_WRITE                 12000000
    #define SPI_SPEED_READ                  4000000
    #define SPI_SPEED_SLOW                  2000000
    #define SPI_MODE_LCD                    SPI_MODE3

/*  #define _spiwrite(c)                    SPI.write(c)
    #define _spiwrite16(d)                  SPI.write16(d)

    #define _spiread(r)                     r = SPI.transfer(0x00);
    #define _spibegin()                     SPI.begin()
    #define _spisetDataMode(datamode)       SPI.setDataMode(datamode)
    #define _spisetBitOrder(order)          SPI.setBitOrder(order)
    #define _spisetSpeed                    SPI.setFrequency(_spi_speed)
*/
    #define _spiwrite(c)                    {\
                                            dev->mosi_dlen.usr_mosi_dbitlen = 7;\
                                            dev->data_buf[0] = c;\
                                            dev->cmd.usr = 1;\
                                            while(dev->cmd.usr);\
                                            }
    #define _spiwrite16(d)                  {\
                                            dev->mosi_dlen.usr_mosi_dbitlen = 15;\
                                            dev->data_buf[0] = ((d) >> 8) | ((d) << 8);\
                                            dev->cmd.usr = 1;\
                                            while(dev->cmd.usr);\
                                            }
    #define _spiwrite24(r,d)                {\
                                            dev->mosi_dlen.usr_mosi_dbitlen = 23;\
                                            dev->data_buf[0] = ((d) && 0xff00) | ((d) << 16) | r;\
                                            dev->cmd.usr = 1;\
                                            while(dev->cmd.usr);\
                                            }
    // LittleVGL supports a double buffer with the LV_VDB_DOUBLE macro and DMA support
    // BUT, this requires a call to lv_flush_ready() when the SPI DMA transfer is finished, and that requires an interrupt routine
    // For now, that is not supported in this class (mixing C/C++ code), so double buffering is disabled
    // Note that the expected speed profit is limited because the display frame calculation is rather fast on an ESP32
    #define _spixwritedma(c, wbuf, len)     {\
                                            dev->dma_conf.val |= SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST;\
                                            dev->dma_out_link.start=0;\
                                            dev->dma_conf.val &= ~(SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST);\
                                            uint32_t n=0;\
                                            uint8_t cmd = c;\
                                            dmadesc[n].size = 1;\
                                            dmadesc[n].length = 1;\
                                            dmadesc[n].eof = 0;\
                                            dmadesc[n].buf = (uint8_t *)&cmd;\
                                            dmadesc[n].qe.stqe_next = &dmadesc[n+1];\
                                            n++;\
                                            len *=2;\
                                            dev->mosi_dlen.usr_mosi_dbitlen = len*8-1+8;\
                                            uint32_t dmachunklen;\
                                            while (len) {\
                                                dmachunklen = len;\
                                                if (dmachunklen > SPI_MAX_DMA_LEN) dmachunklen = SPI_MAX_DMA_LEN;\
                                                dmadesc[n].size = dmachunklen;\
                                                dmadesc[n].length = dmachunklen;\
                                                dmadesc[n].eof = 0;\
                                                dmadesc[n].buf = (uint8_t *)wbuf;\
                                                dmadesc[n].qe.stqe_next = &dmadesc[n+1];\
                                                len -= dmachunklen;\
                                                wbuf += dmachunklen/2;\
                                                n++;\
                                            }\
                                            dmadesc[n - 1].eof = 1;\
                                            dmadesc[n - 1].qe.stqe_next = NULL;\
                                            dev->dma_out_link.start=1;\
                                            dev->user.usr_miso=0;\
                                            dev->cmd.usr = 1;\
                                            while(dev->cmd.usr);\
                                            }
    // delayMicroseconds(50); is removed: TBC
    #define _spixread(x, r)                 {\
                                            dev->mosi_dlen.usr_mosi_dbitlen = 15;\
                                            dev->data_buf[0] = x;\
                                            dev->cmd.usr = 1;\
                                            while(dev->cmd.usr);\
                                            r = dev->data_buf[0] >> 8;\
                                            }
    #define _spixbread(x, r, b)             {\
                                            dev->mosi_dlen.usr_mosi_dbitlen = b-1+8;\
                                            dev->miso_dlen.usr_miso_dbitlen = b-1+8;\
                                            dev->data_buf[0] = x;\
                                            dev->data_buf[1] = 0x00;\
                                            dev->cmd.usr = 1;\
                                            while(dev->cmd.usr);\
                                            r = (dev->data_buf[0]>>8) + (dev->data_buf[1]<<24);\
                                            dev->miso_dlen.usr_miso_dbitlen = 15;\
                                            }
    #define _spibegin()                     {SPI.begin();}
    #define _spisetDataMode(datamode)       SPI.setDataMode(datamode)
    #define _spisetBitOrder(order)          SPI.setBitOrder(order)
    #define _spisetSpeed(s)                 SPI.setFrequency(s)
    #define _spisetBitLen                   {\
                                            SPI.setHwCs(true);\
                                            dev->mosi_dlen.usr_mosi_dbitlen = 15;\
                                            dev->miso_dlen.usr_miso_dbitlen = 15;\
                                            dev->user2.val = 0;\
                                            DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_DMA_CLK_EN);\
                                            DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI_DMA_RST);\
                                            DPORT_SET_PERI_REG_BITS(DPORT_SPI_DMA_CHAN_SEL_REG, 3, 2, 4);\
                                            dev->dma_conf.out_data_burst_en = 1;\
                                            dev->dma_conf.indscr_burst_en = 1;\
                                            dev->dma_conf.outdscr_burst_en = 1;\
                                            dev->dma_in_link.addr = 0;\
                                            dev->dma_in_link.start = 0;\
                                            dev->dma_out_link.addr = (uint32_t)(&dmadesc[0]) & 0xFFFFF;\
                                            for (uint32_t n=0; n<32; n++) {\
                                                dmadesc[n].offset = 0;\
                                                dmadesc[n].sosf = 0;\
                                                dmadesc[n].owner = 1;\
                                            }\
                                            }

    #define _spiCSLow
    #define _spiCSHigh
    //#define _spiCSLow                       GPIO.out_w1tc = 0x20
    //#define _spiCSHigh                      GPIO.out_w1ts = 0x20

    //#define XG_PIN_BL 17
    #define XG_PIN_DC 16
    #define XG_PIN_LCD_RESET 17 //34
    #define XG_PIN_SD_CS 4
    #define XG_PIN_TP_CS 33
    #define XG_PIN_TP_IRQ 27
    #define XG_PIN_LCD_CS SS
    #define XG_PIN_MOSI MOSI
    #define XG_PIN_MISO MISO
    #define XG_PIN_SCK SCK

    //#define XG_PIN_MISO2 12
    //#define XG_PIN_MOSI2 13
    #define XG_PIN_SCK2 14
    #define XG_PIN_CS2 15
    #define XG_PIN_D6 25
    #define XG_PIN_D7 26
    #define XG_PIN_SDA SDA
    #define XG_PIN_SCL SCL
    #define XG_PIN_RX RX
    #define XG_PIN_TX TX

    #define XG_PIN_A0 A14
    #define XG_PIN_A1 A13
    #define XG_PIN_A2 A15

#elif defined(XG_CPU_PARTICLE)                                                  // Particle Family, this library should be used in the Particle IDE not in the Arduino IDE
3
    #include <Arduino.h>
    #define SPI_SPEED_WRITE                 SPI_CLOCK_DIV4                      // 15 MHz (divider /6 does not work) (SPI base clock = CPU clock / 2)
    #define SPI_SPEED_READ                  SPI_CLOCK_DIV16                     // 3.75MHz / Theoretical: 7.5 MHz @ 120MHz CPU clock
    #define SPI_SPEED_SLOW                  SPI_CLOCK_DIV32                     // must be slower then 2 MHz before PLL is configured -> 1.82MHz @ 120MHz CPU clock
    #define SPI_MODE_LCD                    SPI_MODE3

    #define _spiwrite(c)                    SPI.transfer(c)
    #define _spiwrite16(d)                  {SPI.transfer(d >> 8);SPI.transfer(d & 0xFF);}
    #define _spiwritedma(wbuf, length)      SPI.transfer(wbuf, NULL, length, NULL);

    #define _spiread(r)                     r = SPI.transfer(0x00);
    #define _spibegin()                     SPI.begin()
    //#if defined(SPARK) (TO BE FIXED, after testing with Particle parts)
    //    if (datamode == SPI_MODE3) datamode = SPI_MODE0;
    //#endif
    #define _spisetDataMode(datamode)       SPI.setDataMode(datamode)
    #define _spisetBitOrder(order)          SPI.setBitOrder(order)
    #define _spisetSpeed(s)                 SPI.setClockDivider(s)

    #define _spiCSLow                       pinResetFast(XG_PIN_LCD_CS)
    #define _spiCSHigh                      pinSetFast(XG_PIN_LCD_CS)

    #define LED_BUILTIN D7                                                      //Shared with BL
    //LED = 21/22/23 (RGB)
    #define XG_PIN_BL D7                                                        //D7 shared with LED
    #define XG_PIN_DC A7                                                        //D15 = 17 / Also WKP on Duo & Photon
    #define XG_PIN_LCD_RESET D6
    #define XG_PIN_SD_CS A6                                                     //D14 = 16 = DAC1 on Photon
    #define XG_PIN_TP_CS A0                                                     //D8 = 10
    #define XG_PIN_TP_IRQ A1                                                    //D9 = 11
    #define XG_PIN_LCD_CS A2                                                    //D10 = 12 = DAC1 on Duo
    #define XG_PIN_MOSI MOSI                                                    //D13 = 15
    #define XG_PIN_MISO MISO                                                    //D12 = 14
    #define XG_PIN_SCK SCK                                                      //D11 = 13 = DAC2

    #define XG_PIN_MISO2 D3
    #define XG_PIN_MOSI2 D2
    #define XG_PIN_SCK2 D4
    #define XG_PIN_CS2 D5
    #define XG_PIN_D6
    #define XG_PIN_D7
    #define XG_PIN_SDA SDA                                                      //D0
    #define XG_PIN_SCL SCL                                                      //D1
    #define XG_PIN_RX RX                                                        //D16 = 18
    #define XG_PIN_TX TX                                                        //D17 = 19

    #define XG_PIN_A0 A0                                                        // TP CS
    #define XG_PIN_A1 A7                                                        // DC
    //#define XG_PIN_A2

#elif defined(XG_CPU_FANSTEL)                                                   // Fanstel module
4
    #include <Arduino.h>
    #include <SPI_RF.h>
    #define SPI_SPEED_WRITE                 SPI_8M                              // 8 MHz // SPI_8M is defined but 8MHz is out-of-spec for the nRF51822, is it ?
    #define SPI_SPEED_READ                  SPI_4M                              // 4 MHz
    #define SPI_SPEED_SLOW                  SPI_2M                              // must be slower then 2 MHz before PLL is configured
    #define SPI_MODE_LCD                    SPI_MODE2

    #define _spiwrite(c)                    SPI_RF.write(c)
    #define _spiwrite16(d)                  SPI_RF.write16(d)
    #define _spiwritedma(wbuf, length)      SPI_RF.write(wbuf, length)

    #define _spiread(r)                     r = SPI_RF.transfer(0x00);
    #define _spibegin()                     SPI_RF.begin()
    #define _spisetDataMode(datamode)       SPI_RF.setSPIMode(datamode)
    #define _spisetBitOrder(order)          SPI_RF.setBitORDER(order)
    #define _spisetSpeed(s)                 SPI_RF.setFrequency(s)

    #define _spiCSLow                       NRF_GPIO->OUTCLR = (1ul << P0_10)
    #define _spiCSHigh                      NRF_GPIO->OUTSET = (1ul << P0_10)
    /*
     #define SS XG_PIN_LCD_CS                                                    // needed for SdFat
     #ifndef ENABLE_MBED_FS
     #undef O_RDONLY                                                             // These defines are used in the mbed file system
     #undef O_WRONLY                                                             // but also used SdFat. By undef'ing them SdFat works, but the mbed file system
     #undef O_RDWR                                                               // cant be used anymore
     #undef O_ACCMODE                                                            // If your app needs the mbed file system (but not the sdcard) then
     #undef O_APPEND                                                             // use the macro ENABLE_MBED_FS to disable this behavior
     #undef O_SYNC
     #undef O_TRUNC
     #undef O_CREAT
     #undef O_EXCL
     #endif
     #define ARDUINO_FILE_USES_STREAM 0                                          // mbed does not support streams properly
     */

    #define LED_BUILTIN D13                                                     //P0_15 (Pin_nRF51822_to_Arduino should be used)
    #define XG_PIN_BL D4                                                        //D4 - P0_21
    // TEMP TEMP TEMP only for R2 pcb cause DC swapped with TP_IRQ on R3 pcb
    #define XG_PIN_DC D1                                                        //D1 Shared with TX - P0_9 (cores\RBL_nRF51822\pin_transform.cpp)
    #define XG_PIN_LCD_RESET D7                                                 //D7 - P0_17
    #define XG_PIN_SD_CS A5                                                     //D14 - P0_29
    #define XG_PIN_TP_CS D6                                                     //D6 - P0_16
    #define XG_PIN_TP_IRQ D5                                                    //D5 - P0_23
    #define XG_PIN_LCD_CS D2                                                    //D2 - SDA - P0_10
    #define XG_PIN_MOSI A3                                                      //A3 - D11 - P0_12
    #define XG_PIN_MISO A4                                                      //A4 - D12 - P0_13
    #define XG_PIN_SCK SCK                                                      //D3 - SCL - P0_8

    #define XG_PIN_MISO2
    #define XG_PIN_MOSI2
    #define XG_PIN_SCK2
    #define XG_PIN_CS2
    #define XG_PIN_D6
    #define XG_PIN_D7
    #define XG_PIN_SDA
    #define XG_PIN_SCL
    #define XG_PIN_RX D0
    #define XG_PIN_TX D1

    //#define XG_PIN_A0
    //#define XG_PIN_A1
    //#define XG_PIN_A2

#elif defined(XG_CPU_ARDUINO)                                                   // All other boards presume the Arduino headers are used, 32-bit boards work better, generic SPI commands are used
5
    #include <Arduino.h>
    #include <SPI.h>

    #define SPI_SPEED_WRITE                 SPI_CLOCK_DIV2                      // 8MHz with a 16MHz base clock
    #define SPI_SPEED_READ                  SPI_CLOCK_DIV4                      // 4MHz
    #define SPI_SPEED_SLOW                  SPI_CLOCK_DIV8                      // 2MHz
    #define SPI_MODE_LCD                    SPI_MODE3

    #define _spiwrite(c)                    SPI.transfer(c)
    #define _spiwrite16(d)                  SPI.transfer16(d)

    #define _spiread(r)                     r = SPI.transfer(0x00);
    #define _spibegin()                     SPI.begin()
    #define _spisetDataMode(datamode)       SPI.setDataMode(datamode)
    #define _spisetBitOrder(order)          SPI.setBitOrder(order)
    #define _spisetSpeed(s)                 setClockDivider(s)

    #define _spiCSLow                       digitalWrite(XG_PIN_LCD_CS, LOW)
    #define _spiCSHigh                      digitalWrite(XG_PIN_LCD_CS, HIGH)

    #define XG_PIN_BL (8)
    #define XG_PIN_DC (7)
    #define XG_PIN_LCD_RESET (9)
    #define XG_PIN_SD_CS (5)
    #define XG_PIN_TP_CS (6)
    #define XG_PIN_TP_IRQ(4)
    #define XG_PIN_LCD_CS PIN_SPI_SS
    #define XG_PIN_MOSI PIN_SPI_MOSI
    #define XG_PIN_MISO PIN_SPI_MISO
    #define XG_PIN_SCK PIN_SPI_SCK

#else                                                                           // Safeguard, should never occur
    #error "Your CPU board is not supported on your X-Graph LCD module."
#endif

// Macro's to abstract the Serial object (usefull to easily disable serial/debugging output)
#define Sbegin(a) Serial.begin(a);
#define Sprintln(a) Serial.println(a)
#define Sprintln2(a,b) Serial.println(a,b)
#define Sprint(a) Serial.print(a)


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            TOUCHSCREEN CALIBRATION                          +
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Not all touchscreens are the same and they might need calibration. This can be
// testen by loading the touchPaint.ino example. Just try to paint all over the screen!
// If you have space on one or more sides you need to calibrate the touchscreen.
// To perform the touch screen calibration, load libTouchSCalibration.ino and open the serial terminal:
// TOUCSRCAL_XLOW = the lowest value of x by touching the top/left corner of your tft
// TOUCSRCAL_YLOW = the lowest value of y by touching the top/left corner of your tft
// TOUCSRCAL_XHIGH = the highest value of x by touching the lower/bottom corner of your tft
// TOUCSRCAL_XHIGH = the highest value of y by touching the lower/bottom corner of your tft
/*#ifdef XG_LCD_50
    #define TOUCSRCAL_XLOW              60 //70
    #define TOUCSRCAL_YLOW              116 //111
    #define TOUCSRCAL_XHIGH             939 //895
    #define TOUCSRCAL_YHIGH             931 //880
#elif defined(XG_LCD_90)
    #define TOUCSRCAL_XLOW              967
    #define TOUCSRCAL_YLOW              941
    #define TOUCSRCAL_XHIGH             43
    #define TOUCSRCAL_YHIGH             84
#elif defined(XG_LCD_24)
    #define TOUCSRCAL_XLOW              70
    #define TOUCSRCAL_YLOW              111
    #define TOUCSRCAL_XHIGH             895
    #define TOUCSRCAL_YHIGH             880
#else
    #define TOUCSRCAL_AUTO
#endif */
#define TOUCSRCAL_XLOW                  43
#define TOUCSRCAL_YLOW                  84
#define TOUCSRCAL_XHIGH                 967
#define TOUCSRCAL_YHIGH                 941

// Global defines
#define CENTER                          9998
#define ARC_ANGLE_MAX                   360
#define ARC_ANGLE_OFFSET                -90
#define ANGLE_OFFSET                    -90


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            RA8875 Registers                                 +
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define RA8875_PWRR                     0x01                                    // PWRR [0x01] Power and Display Control Register
#define RA8875_PWRR_DISPON              0x80                                    // LCD Display off: 0 = off / 1 = on
#define RA8875_PWRR_DISPOFF             0x00
#define RA8875_PWRR_SLEEP               0x02                                    // Sleep mode: 0 = normal / 1 = sleep
#define RA8875_PWRR_NORMAL              0x00
#define RA8875_PWRR_SOFTRESET           0x01                                    // Software reset; 0 = nothing / 1 = reset

// Data to write in memory corresponding to the setting of MWCR1[3:2]. Continuous data write cycle can be accepted in bulk data write case.
#define RA8875_MRWC                     0x02                                    // MRWC [0x02] Memory Read/Write Command
#define RA8875_CMDWRITE                 0x8000
#define RA8875_CMDREAD                  0xC0
#define RA8875_DATAWRITE                0x00
#define RA8875_DATAREAD                 0x40
#define RA8875_STATREG                  0x40

#define RA8875_PCSR                     0x04                                    // PCSR [0x04] Pixel Clock Setting Register
#define RA8875_PCSR_PDATR               0x00                                    // PCLK Inversion: 0 = PDAT at PLCK rising / 1 = PDAT at PLCK falling
#define RA8875_PCSR_PDATL               0x80
#define RA8875_PCSR_CLK                 0x00                                    // PCLK period = System Clock period
#define RA8875_PCSR_2CLK                0x01                                    //             = 2x System Clock period
#define RA8875_PCSR_4CLK                0x02                                    //             = 4x System Clock period
#define RA8875_PCSR_8CLK                0x03                                    //             = 8x System Clock period

#define RA8875_SROC                     0x05                                    // SROC [0x05] Serial Flash/ROM Configuration (not used)
#define RA8875_SFCLR                    0x06                                    // SFCLR [0x06] Serial Flash/ROM CLK (not used)

#define RA8875_SYSR                     0x10                                    // SYSR [0x10] System Configuration Register
#define RA8875_SYSR_8BPP                0x00                                    // Color Depth = 256 colors
#define RA8875_SYSR_16BPP               0x0C                                    //             = 65k colors
#define RA8875_SYSR_MCU8                0x00                                    // MCU Interface = 8-bit
#define RA8875_SYSR_MCU16               0x03                                    //               = 16-bit

#define RA8875_HDWR                     0x14                                    // HDWR [0x14] LCD Horizontal Display Width Register (bits 6->0 and max = 0x63 -> (0x63+1)*8 = 800)
#define RA8875_HNDFTR                   0x15                                    // HNDFTR [0x15] Horizontal Non-Display Period Fine Tuning Option Register = bits 3->0
#define RA8875_HNDFTR_DE_HIGH           0x00                                    // DE polarity = high
#define RA8875_HNDFTR_DE_LOW            0x80                                    //             = low
#define RA8875_HNDR                     0x16                                    // HNDR [0x016] LCD Horizontal Non-Display Period Register (bits 4->0 -> (HNDR + 1) * 8)
#define RA8875_HSTR                     0x17                                    // HSTR [0x17] HSYNC Start Position Register  (bits 4->0 -> (HSTR + 1) * 8)
#define RA8875_HPWR                     0x18                                    // HPWR [0x18] HSYNC Pulse Width Register (bits 4->0 -> (HPWR + 1) * 8)
#define RA8875_HPWR_LOW                 0x00                                    // HSYNC Polarity 0 = low
#define RA8875_HPWR_HIGH                0x80                                    //                 1 = high

#define RA8875_VDHR0                    0x19                                    // LCD Vertical Display Height Register 0
#define RA8875_VDHR1                    0x1A                                    // LCD Vertical Display Height Register 1
#define RA8875_VNDR0                    0x1B                                    // LCD Vertical Non-Display Period Register 0
#define RA8875_VNDR1                    0x1C                                    // LCD Vertical Non-Display Period Register 1
#define RA8875_VSTR0                    0x1D                                    // VSYNC Start Position Register 0
#define RA8875_VSTR1                    0x1E                                    // VSYNC Start Position Register 1
#define RA8875_VPWR                     0x1F                                    // VSYNC Pulse Width Register
#define RA8875_VPWR_LOW                 0x00
#define RA8875_VPWR_HIGH                0x80

#define RA8875_DPCR                     0x20                                    // DPCR [0x20] Display Configuration Register
#define RA8875_DPCR_ONE_LAYER           0x00                                    // 1 layer
#define RA8875_DPCR_TWO_LAYERS          0x80                                    // 2 layers
#define RA8875_DPCR_HDIR_NORMAL         0x00                                    // Horizontal scan direction SEG0 -> SEG(n-1)
#define RA8875_DPCR_HDIR_REVERSE        0x08                                    // reversed
#define RA8875_DPCR_VDIR_NORMAL         0x00                                    // Vertical scan direction COM0 -> COM(n-1)
#define RA8875_DPCR_VDIR_REVERSE        0x04                                    // reversed

#define RA8875_FNCR0                    0x21                                    // FNCR0 [0x21] Font Control Register 0
#define RA8875_FNCR0_CG_MASK            0xa0
#define RA8875_FNCR0_CGROM              0x00                                    // Font selection in text mode: CGROM
#define RA8875_FNCR0_CGRAM              0x80                                    //                              CGRAM
#define RA8875_FNCR0_INTERNAL_CGROM     0x00                                    // External or Internal CGROM: Internal (RA8875_SFRSET = 0)
#define RA8857_FNCR0_EXTERNAL_CGROM     0x20                                    //                           : External (RA8875_FWTSET bit 6,7 = 0)
#define RA8875_FNCR0_8859_MASK          0x03
#define RA8857_FNCR0_8859_1             0x00                                    // Font Selection for internal CGROM
#define RA8857_FNCR0_8859_2             0x01
#define RA8857_FNCR0_8859_3             0x02
#define RA8857_FNCR0_8859_4             0x03

#define RA8875_FNCR1                    0x22                                    // FNCR1 [0x22] Font Control Register 1
#define RA8875_FNCR1_ALIGNMENT_OFF      0x00                                    // Full Alignment disabled
#define RA8875_FNCR1_ALIGNMENT_ON       0x80                                    //                enabled
#define RA8875_FNCR1_TRANSPARENT_OFF    0x00                                    // Font Transparency disabled
#define RA8875_FNCR1_TRANSPARENT_ON     0x40                                    //                   enabled
#define RA8875_FNCR1_NORMAL             0x00                                    // Font Rotation disabled
#define RA8875_FNCR1_90DEGREES          0x10                                    //               90 degrees
#define RA8875_FNCR1_SCALE_MASK         0x0f
#define RA8875_FNCR1_SCALE_HOR_1        0x00                                    // Font Horizontal Enlargment disabled
#define RA8875_FNCR1_SCALE_HOR_2        0x04                                    //                            x2
#define RA8875_FNCR1_SCALE_HOR_3        0x08                                    //                            x3
#define RA8875_FNCR1_SCALE_HOR_4        0x0c                                    //                            x4
#define RA8875_FNCR1_SCALE_VER_1        0x00                                    // Font Vertical Enlargment disabled
#define RA8875_FNCR1_SCALE_VER_2        0x01                                    //                          x2
#define RA8875_FNCR1_SCALE_VER_3        0x02                                    //                          x3
#define RA8875_FNCR1_SCALE_VER_4        0x03                                    //                          x4

#define RA8875_CGSR                     0x23                                    // CGSR [0x23] CGRAM Select Register
#define RA8875_HOFS0                    0x24                                    // HOFS0 [0x24] Horizontal Scroll Offset Register 0
#define RA8875_HOFS1                    0x25                                    // HOFS1 [0x25] Horizontal Scroll Offset Register 1 (bits 2->0)
#define RA8875_VOFS0                    0x26                                    // VOFS0 [0x26] Vertical Scroll Offset Register 0
#define RA8875_VOFS1                    0x27                                    // VOFS1 [0x27] Vertical Scroll Offset Register 1 (bits 1->0)
#define RA8875_FLDR                     0x29                                    // FLDR [0x29] Font Line Distance Setting Register (bits 4->0)

#define RA8875_F_CURXL                  0x2A                                    // CURXL [0x2A] Font Write Cursor Horizontal Position Register 0
#define RA8875_F_CURXH                  0x2B                                    // CURXH [0x2B] Font Write Cursor Horizontal Position Register 1 (bits 1->0)
#define RA8875_F_CURYL                  0x2C                                    // CURYL [0x2C] Font Write Cursor Vertical Position Register 0
#define RA8875_F_CURYH                  0x2D                                    // CURYH [0x2D] Font Write Cursor Vertical Position Register 1 (bit 0)

#define RA8875_FWTSET                   0x2E                                    // FWTSET [0x2E] Font Write Type Setting Register
#define RA8875_FWTSET_SIZE_MASK         0xc0
#define RA8875_FWTSET_16X16             0x00                                    // Font set: 16x16 (full) - 8x16 (half) - nx16 (var)
#define RA8875_FWTSET_24X24             0x40                                    // Font set: 24x24 (full) - 12x24 (half) - nx24 (var)
#define RA8875_FWTSET_32X32             0x80                                    // Font set: 32x32 (full) - 16x32 (half) - nx32 (var)
#define RA8875_FWTSET_WIDTH_MASK        0x3F                                    // Font to Font Width Setting (0 to 63 pixels)
#define RA8875_SFRSET                   0x2F                                    // SFRSET [0x0F] Serial Font ROM Setting

#define RA8875_HSAW0                    0x30                                    // HSAW0 [0x30] Horizontal Start Point 0 of Active Window
#define RA8875_HSAW1                    0x31                                    // HSAW1 [0x31] Horizontal Start Point 1 of Active Window
#define RA8875_VSAW0                    0x32                                    // VSAW0 [0x32] Vertical   Start Point 0 of Active Window
#define RA8875_VSAW1                    0x33                                    // VSAW1 [0x32] Vertical   Start Point 1 of Active Window
#define RA8875_HEAW0                    0x34                                    // HEAW0 [0x34] Horizontal End   Point 0 of Active Window
#define RA8875_HEAW1                    0x35                                    // HEAW1 [0x35] Horizontal End   Point 1 of Active Window
#define RA8875_VEAW0                    0x36                                    // VEAW0 [0x36] Vertical   End   Point of Active Window 0
#define RA8875_VEAW1                    0x37                                    // VEAW1 [0x37] Vertical   End   Point of Active Window 1

#define RA8875_HSSW0                    0x38                                    // HSSW0 [0x38] Horizontal Start Point 0 of Scroll Window
#define RA8875_HSSW1                    0x39                                    // HSSW1 [0x39] Horizontal Start Point 1 of Scroll Window
#define RA8875_VSSW0                    0x3A                                    // VSSW0 [0x3A] Vertical      Start Point 0 of Scroll Window
#define RA8875_VSSW1                    0x3B                                    // VSSW1 [0x3B] Vertical      Start Point 1 of Scroll Window
#define RA8875_HESW0                    0x3C                                    // HESW0 [0x3C] Horizontal End   Point 0 of Scroll Window
#define RA8875_HESW1                    0x3D                                    // HESW1 [0x3D] Horizontal End   Point 1 of Scroll Window
#define RA8875_VESW0                    0x3E                                    // VESW0 [0x3E] Vertical      End   Point 0 of Scroll Window
#define RA8875_VESW1                    0x3F                                    // VESW1 [0x3F] Vertical      End   Point 1 of Scroll Window
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                    Cursor Setting Registers
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Memory Write Control Register 0   [0x40]
 ----- Bit 7 (Select Mode)
 0: Graphic Mode
 1: Text Mode
 ----- Bit 6 (Font Write Cursor/ Memory Write Cursor Enable)
 0: Font write cursor/ Memory Write Cursor is not visible
 1: Font write cursor/ Memory Write Cursor is visible
 ----- Bit 5 (Font Write Cursor/ Memory Write Cursor Blink Enable)
 0: Normal display
 1: Blink display
 ----- Bit 4 (na)
 ----- Bit 3,2 (Memory Write Direction (Only for Graphic Mode)
 00: Left -> Right then Top -> Down
 01: Right -> Left then Top -> Down
 10: Top -> Down then Left -> Right
 11: Down -> Top then Left -> Right
 ----- Bit 1 (Memory Write Cursor Auto-Increase Disable)
 0: Cursor auto-increases when memory write
 1: Cursor doesn’t auto-increases when memory write
 ----- Bit 0(Memory Read Cursor Auto-Increase Disable)
 0: Cursor auto-increases when memory read
 1: Cursor doesn’t auto-increases when memory read */
#define RA8875_MWCR0                    0x40                                    //Memory Write Control Register 0
#define RA8875_MWCR0_GFXMODE            0x00
#define RA8875_MWCR0_TXTMODE            0x80
#define RA8875_MWCR0_NO_CURSOR          0x00
#define RA8875_MWCR0_CURSOR             0x40
#define RA8875_MWCR0_CURSOR_NORMAL      0x00
#define RA8875_MWCR0_CURSOR_BLINK       0x20
#define RA8875_MWCR0_MEMWRDIR_MASK      0x0c
#define RA8875_MWCR0_MEMWRDIR_LT        0x00
#define RA8875_MWCR0_MEMWRDIR_RT        0x04
#define RA8875_MWCR0_MEMWRDIR_TL        0x08
#define RA8875_MWCR0_MEMWRDIR_DL        0x0c
#define RA8875_MWCR0_MEMWR_CUR_INC      0x00
#define RA8875_MWCR0_MEMWR_NO_INC       0x02
#define RA8875_MWCR0_MEMRD_CUR_INC      0x00
#define RA8875_MWCR0_MEMRD_NO_INC       0x01
/* Memory Write Control Register 1   [0x41]
 ----- Bit 7 (Graphic Cursor Enable)
 0:disable, 1:enable
 ----- Bit 6,5,4 (Graphic Cursor Selection)
 000: Graphic Cursor Set 1
 ...
 111: Graphic Cursor Set 8
 ----- Bit 3,2 (Write Destination Selection)
 00: Layer 1~2
 01: CGRAM
 10: Graphic Cursor
 11: Pattern
 Note : When CGRAM is selected , RA8875_FNCR0 bit 7 must be set as 0.
 ----- Bit 1 (na)
 ----- Bit 0 (Layer No. for Read/Write Selection)
 When resolution =< 480x400 or color depth = 8bpp:
 0: Layer 1
 1: Layer 2
 When resolution > 480x400 and color depth > 8bpp:
 na */
#define RA8875_MWCR1                    0x41                                    //Memory Write Control Register 1
/*
 from 0 to 255
 */
#define RA8875_BTCR                     0x44                                    //Blink Time Control Register
/* Memory Read Cursor Direction      [0x45]
 ----- Bit 7,6,5,4,3,2(na)
 ----- Bit 1,0(Memory Read Direction (Only for Graphic Mode))
 00: Left -> Right then Top -> Down
 01: Right -> Left then Top -> Down
 10: Top -> Down then Left -> Right
 11: Down -> Top then Left -> Right */
#define RA8875_MRCD                     0x45                                    //Memory Read Cursor Direction
#define RA8875_CURH0                    0x46                                    //Memory Write Cursor Horizontal Position Register 0
#define RA8875_CURH1                    0x47                                    //Memory Write Cursor Horizontal Position Register 1
#define RA8875_CURV0                    0x48                                    //Memory Write Cursor Vertical Position Register 0
#define RA8875_CURV1                    0x49                                    //Memory Write Cursor Vertical Position Register 1

#define RA8875_RCURH0                   0x4A                                    //Memory Read Cursor Horizontal Position Register 0
#define RA8875_RCURH1                   0x4B                                    //Memory Read Cursor Horizontal Position Register 1
#define RA8875_RCURV0                   0x4C                                    //Memory Read Cursor Vertical Position Register 0
#define RA8875_RCURV1                   0x4D                                    //Memory Read Cursor Vertical Position Register 1


/* Font Write Cursor and Memory Write Cursor Horizontal Size  [0x4E]
 ----- Bit 7,6,5 (na)
 ----- Bit 4,0(Font Write Cursor Horizontal Size in pixels) */
#define RA8875_CURHS                    0x4E                                    //Font Write Cursor and Memory Write Cursor Horizontal Size Register
/* Font Write Cursor Vertical Size Register   [0x4F]
 ----- Bit 7,6,5 (na)
 ----- Bit 4,0(Font Write Cursor Vertical Size in pixels) */
#define RA8875_CURVS                    0x4F                                    //Font Write Cursor Vertical Size Register
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//              Block Transfer Engine(BTE) Control Registers
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define RA8875_BECR0                    0x50                                    //BTE Function Control Register 0
#define RA8875_BECR1                    0x51                                    //BTE Function Control Register 1
/* Layer Transparency Register 0     [0x52]
 ----- Bit 7,6 (Layer1/2 Scroll Mode)
 00: Layer 1/2 scroll simultaneously
 01: Only Layer 1 scroll
 10: Only Layer 2 scroll
 11: Buffer scroll (using Layer 2 as scroll buffer)
 ----- Bit 5 (Floating Windows Transparency Display With BGTR)
 0:disable, 1:enable
 ----- Bit 4,3 (na)
 ----- Bit 2,1,0(Layer1/2 Display Mode)
 000: Only Layer 1 is visible
 001: Only Layer 2 is visible
 010: Lighten-overlay mode
 011: Transparent mode
 100: Boolean OR
 101: Boolean AND
 110: Floating window mode
 111: Reserve */
#define RA8875_LTPR0                    0x52                                    //Layer Transparency Register 0
/* Layer Transparency Register 1     [0x53]
 ----- Bit 7,6,5,4 (Layer Transparency Setting for Layer 2)
 0000: Total display
 0001: 7/8 display
 0010: 3/4 display
 0011: 5/8 display
 0100: 1/2 display
 0101: 3/8 display
 0110: 1/4 display
 0111: 1/8 display
 1000: Display disable
 ----- Bit 3,2,1,0 (Layer Transparency Setting for Layer 1)
 0000: Total display
 0001: 7/8 display
 0010: 3/4 display
 0011: 5/8 display
 0100: 1/2 display
 0101: 3/8 display
 0110: 1/4 display
 0111: 1/8 display
 1000: Display disable */
#define RA8875_LTPR1                    0x53                                    //Layer Transparency Register 1
#define RA8875_HSBE0                    0x54                                    //Horizontal Source Point 0 of BTE
#define RA8875_HSBE1                    0x55                                    //Horizontal Source Point 1 of BTE
#define RA8875_VSBE0                    0x56                                    //Vertical Source Point 0 of BTE
#define RA8875_VSBE1                    0x57                                    //Vertical Source Point 1 of BTE
#define RA8875_HDBE0                    0x58                                    //Horizontal Destination Point 0 of BTE
#define RA8875_HDBE1                    0x59                                    //Horizontal Destination Point 1 of BTE
#define RA8875_VDBE0                    0x5A                                    //Vertical Destination Point 0 of BTE
#define RA8875_VDBE1                    0x5B                                    //Vertical Destination Point 1 of BTE
#define RA8875_BEWR0                    0x5C                                    //BTE Width Register 0
#define RA8875_BEWR1                    0x5D                                    //BTE Width Register 1
#define RA8875_BEHR0                    0x5E                                    //BTE Height Register 0
#define RA8875_BEHR1                    0x5F                                    //BTE Height Register 1

/* Pattern Set No for BTE            [0x66]
 ----- Bit 7 (Pattern Format)
 0: 8x8
 1: 16x16
 ----- Bit 6,5,4 (na)
 ----- Bit 3,2,1,0 (Pattern Set No)
 If pattern Format = 8x8 then Pattern Set [3:0]
 If pattern Format = 16x16 then Pattern Set [1:0] is valid */
#define RA8875_PTNO                     0x66                                    //Pattern Set No for BTE

//BTE Raster OPerations - there's 16 possible operations but these are the main ones likely to be useful
#define RA8875_BTEROP_SOURCE            0xC0                                    //Overwrite dest with source (no mixing) *****THIS IS THE DEFAULT OPTION****
#define RA8875_BTEROP_BLACK             0x00                                    //all black
#define RA8875_BTEROP_WHITE             0xf0                                    //all white
#define RA8875_BTEROP_DEST              0xA0                                    //destination unchanged
#define RA8875_BTEROP_ADD               0xE0                                    //ADD (brighter)
#define RA8875_BTEROP_SUBTRACT          0x20                                    //SUBTRACT (darker)
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            Color Registers
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define RA8875_BGCR0                    0x60                                    //Background Color Register 0 (R)
#define RA8875_BGCR1                    0x61                                    //Background Color Register 1 (G)
#define RA8875_BGCR2                    0x62                                    //Background Color Register 2 (B)
#define RA8875_FGCR0                    0x63                                    //Foreground Color Register 0 (R)
#define RA8875_FGCR1                    0x64                                    //Foreground Color Register 1 (G)
#define RA8875_FGCR2                    0x65                                    //Foreground Color Register 2 (B)
#define RA8875_BGTR0                    0x67                                    //Background Color Register for Transparent 0 (R)
#define RA8875_BGTR1                    0x68                                    //Background Color Register for Transparent 1 (G)
#define RA8875_BGTR2                    0x69                                    //Background Color Register for Transparent 2 (B)

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            TOUCH SCREEN REGISTERS
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define RA8875_TPCR0                    0x70                                    //Touch Panel Control Register 0
#define RA8875_TPCR0_ENABLE             0x80
#define RA8875_TPCR0_DISABLE            0x00
#define RA8875_TPCR0_WAIT_512CLK        0x00
#define RA8875_TPCR0_WAIT_1024CLK       0x10
#define RA8875_TPCR0_WAIT_2048CLK       0x20
#define RA8875_TPCR0_WAIT_4096CLK       0x30
#define RA8875_TPCR0_WAIT_8192CLK       0x40
#define RA8875_TPCR0_WAIT_16384CLK      0x50
#define RA8875_TPCR0_WAIT_32768CLK      0x60
#define RA8875_TPCR0_WAIT_65536CLK      0x70
#define RA8875_TPCR0_WAKEENABLE         0x08
#define RA8875_TPCR0_WAKEDISABLE        0x00
#define RA8875_TPCR0_ADCCLK_DIV1        0x00
#define RA8875_TPCR0_ADCCLK_DIV2        0x01
#define RA8875_TPCR0_ADCCLK_DIV4        0x02
#define RA8875_TPCR0_ADCCLK_DIV8        0x03
#define RA8875_TPCR0_ADCCLK_DIV16       0x04
#define RA8875_TPCR0_ADCCLK_DIV32       0x05
#define RA8875_TPCR0_ADCCLK_DIV64       0x06
#define RA8875_TPCR0_ADCCLK_DIV128      0x07
/*
 bits
 0,1: 00(idle), 10(TP event), 01(latch x) , 11(latch y)
 2:   1(Debounce enabled), 0(debounce disabled)
 3,4: NA
 5:   0(internal VREF), 1(Ext VREF)
 6:   0(AUTO mode, 1(MANUAL mode)
 */
#define RA8875_TPCR1                    0x71                                    //Touch Panel Control Register 1
#define RA8875_TPCR1_AUTO               0x00
#define RA8875_TPCR1_MANUAL             0x40
#define RA8875_TPCR1_VREFINT            0x00
#define RA8875_TPCR1_VREFEXT            0x20
#define RA8875_TPCR1_DEBOUNCE           0x04
#define RA8875_TPCR1_NODEBOUNCE         0x00
#define RA8875_TPCR1_IDLE               0x00
#define RA8875_TPCR1_WAIT               0x01
#define RA8875_TPCR1_LATCHX             0x02
#define RA8875_TPCR1_LATCHY             0x03

#define RA8875_TPXH                     0x72                                    //Touch Panel X High Byte Data Register
#define RA8875_TPYH                     0x73                                    //Touch Panel Y High Byte Data Register
#define RA8875_TPXYL                    0x74                                    //Touch Panel X/Y Low Byte Data Register
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            Graphic Cursor Setting Registers
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define RA8875_GCHP0                    0x80                                    //Graphic Cursor Horizontal Position Register 0
#define RA8875_GCHP1                    0x81                                    //Graphic Cursor Horizontal Position Register 1
#define RA8875_GCVP0                    0x82                                    //Graphic Cursor Vertical Position Register 0
#define RA8875_GCVP1                    0x83                                    //Graphic Cursor Vertical Position Register 0
#define RA8875_GCC0                     0x84                                    //Graphic Cursor Color 0
#define RA8875_GCC1                     0x85                                    //Graphic Cursor Color 1
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            PLL Setting Registers
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define RA8875_PLLC1                    0x88                                    //PLL Control Register 1
#define RA8875_PLLC1_PLLDIV2            0x80
#define RA8875_PLLC1_PLLDIV1            0x00
#define RA8875_PLLC2                    0x89                                    //PLL Control Register 2
#define RA8875_PLLC2_DIV1               0x00
#define RA8875_PLLC2_DIV2               0x01
#define RA8875_PLLC2_DIV4               0x02
#define RA8875_PLLC2_DIV8               0x03
#define RA8875_PLLC2_DIV16              0x04
#define RA8875_PLLC2_DIV32              0x05
#define RA8875_PLLC2_DIV64              0x06
#define RA8875_PLLC2_DIV128             0x07

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            PWM Control Registers
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define RA8875_P1CR                     0x8A                                    //PWM1 Control Register
#define RA8875_P1CR_ENABLE              0x80
#define RA8875_P1CR_DISABLE             0x00
#define RA8875_P1CR_CLKOUT              0x10
#define RA8875_P1CR_PWMOUT              0x00
#define RA8875_P1DCR                    0x8B                                    //PWM1 Duty Cycle Register

#define RA8875_P2CR                     0x8C                                    //PWM2 Control Register
#define RA8875_P2CR_ENABLE              0x80
#define RA8875_P2CR_DISABLE             0x00
#define RA8875_P2CR_CLKOUT              0x10
#define RA8875_P2CR_PWMOUT              0x00
#define RA8875_P2DCR                    0x8D                                    //PWM2 Control Register

#define RA8875_PxCR_ENABLE              0x80
#define RA8875_PxCR_DISABLE             0x00
#define RA8875_PxCR_CLKOUT              0x10
#define RA8875_PxCR_PWMOUT              0x00

#define RA8875_PWM_CLK_DIV1             0x00
#define RA8875_PWM_CLK_DIV2             0x01
#define RA8875_PWM_CLK_DIV4             0x02
#define RA8875_PWM_CLK_DIV8             0x03
#define RA8875_PWM_CLK_DIV16            0x04
#define RA8875_PWM_CLK_DIV32            0x05
#define RA8875_PWM_CLK_DIV64            0x06
#define RA8875_PWM_CLK_DIV128           0x07
#define RA8875_PWM_CLK_DIV256           0x08
#define RA8875_PWM_CLK_DIV512           0x09
#define RA8875_PWM_CLK_DIV1024          0x0A
#define RA8875_PWM_CLK_DIV2048          0x0B
#define RA8875_PWM_CLK_DIV4096          0x0C
#define RA8875_PWM_CLK_DIV8192          0x0D
#define RA8875_PWM_CLK_DIV16384         0x0E
#define RA8875_PWM_CLK_DIV32768         0x0F

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            Memory Clear
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Memory Clear Control Register     [0x8E]
 ----- Bit 7 (Memory Clear Function)
 0: End or Stop (if read this bit and it's 0, clear completed)
 1: Start the memory clear function
 ----- Bit 6 (Memory Clear Area Setting)
 0: Clear the full window (ref. RA8875_HDWR,RA8875_VDHR0,RA8875_VDHR0ì1)
 1: Clear the active window
 ----- Bit 5,4,3,2,1,0 (na)  */
#define RA8875_MCLR                     0x8E                                    //Memory Clear Control Register
#define RA8875_MCLR_START               0x80
#define RA8875_MCLR_STOP                0x00
#define RA8875_MCLR_READSTATUS          0x80
#define RA8875_MCLR_FULL                0x00
#define RA8875_MCLR_ACTIVE              0x40

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            Drawing Control Registers
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define RA8875_DCR                      0x90                                    //Draw Line/Circle/Square Control Register
#define RA8875_DCR_LINESQUTRI_START     0x80
#define RA8875_DCR_LINESQUTRI_STOP      0x00
#define RA8875_DCR_LINESQUTRI_STATUS    0x80
#define RA8875_DCR_CIRCLE_START         0x40
#define RA8875_DCR_CIRCLE_STATUS        0x40
#define RA8875_DCR_CIRCLE_STOP          0x00
#define RA8875_DCR_FILL                 0x20
#define RA8875_DCR_NOFILL               0x00
#define RA8875_DCR_DRAWLINE             0x00
#define RA8875_DCR_DRAWTRIANGLE         0x01
#define RA8875_DCR_DRAWSQUARE           0x10

#define RA8875_DLHSR0                   0x91                                    //Draw Line/Square Horizontal Start Address Register0
#define RA8875_DLHSR1                   0x92                                    //Draw Line/Square Horizontal Start Address Register1
#define RA8875_DLVSR0                   0x93                                    //Draw Line/Square Vertical Start Address Register0
#define RA8875_DLVSR1                   0x94                                    //Draw Line/Square Vertical Start Address Register1
#define RA8875_DLHER0                   0x95                                    //Draw Line/Square Horizontal End Address Register0
#define RA8875_DLHER1                   0x96                                    //Draw Line/Square Horizontal End Address Register1
#define RA8875_DLVER0                   0x97                                    //Draw Line/Square Vertical End Address Register0
#define RA8875_DLVER1                   0x98                                    //Draw Line/Square Vertical End Address Register0

#define RA8875_DCHR0                    0x99                                    //Draw Circle Center Horizontal Address Register0
#define RA8875_DCHR1                    0x9A                                    //Draw Circle Center Horizontal Address Register1
#define RA8875_DCVR0                    0x9B                                    //Draw Circle Center Vertical Address Register0
#define RA8875_DCVR1                    0x9C                                    //Draw Circle Center Vertical Address Register1
#define RA8875_DCRR                     0x9D                                    //Draw Circle Radius Register

#define RA8875_ELLIPSE                  0xA0                                    //Draw Ellipse/Ellipse Curve/Circle Square Control Register
#define RA8875_ELLIPSE_STATUS           0x80

#define RA8875_ELL_A0                   0xA1                                    //Draw Ellipse/Circle Square Long axis Setting Register0
#define RA8875_ELL_A1                   0xA2                                    //Draw Ellipse/Circle Square Long axis Setting Register1
#define RA8875_ELL_B0                   0xA3                                    //Draw Ellipse/Circle Square Short axis Setting Register0
#define RA8875_ELL_B1                   0xA4                                    //Draw Ellipse/Circle Square Short axis Setting Register1

#define RA8875_DEHR0                    0xA5                                    //Draw Ellipse/Circle Square Center Horizontal Address Register0
#define RA8875_DEHR1                    0xA6                                    //Draw Ellipse/Circle Square Center Horizontal Address Register1
#define RA8875_DEVR0                    0xA7                                    //Draw Ellipse/Circle Square Center Vertical Address Register0
#define RA8875_DEVR1                    0xA8                                    //Draw Ellipse/Circle Square Center Vertical Address Register1

#define RA8875_DTPH0                    0xA9                                    //Draw Triangle Point 2 Horizontal Address Register0
#define RA8875_DTPH1                    0xAA                                    //Draw Triangle Point 2 Horizontal Address Register1
#define RA8875_DTPV0                    0xAB                                    //Draw Triangle Point 2 Vertical Address Register0
#define RA8875_DTPV1                    0xAC                                    //Draw Triangle Point 2 Vertical Address Register1

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            DMA REGISTERS
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define RA8875_SSAR0                    0xB0                                    //Source Starting Address REG 0
#define RA8875_SSAR1                    0xB1                                    //Source Starting Address REG 1
#define RA8875_SSAR2                    0xB2                                    //Source Starting Address REG 2
//#define RA8875_????                   0xB3                                    //???????????

#define RA8875_DTNR0                    0xB4                                    //Block Width REG 0(BWR0) / DMA Transfer Number REG 0
#define RA8875_BWR1                     0xB5                                    //Block Width REG 1
#define RA8875_DTNR1                    0xB6                                    //Block Height REG 0(BHR0) /DMA Transfer Number REG 1
#define RA8875_BHR1                     0xB7                                    //Block Height REG 1
#define RA8875_DTNR2                    0xB8                                    //Source Picture Width REG 0(SPWR0) / DMA Transfer Number REG 2
#define RA8875_SPWR1                    0xB9                                    //Source Picture Width REG 1
#define RA8875_DMACR                    0xBF                                    //DMA Configuration REG


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            GPIO
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define RA8875_GPI                      0x12
#define RA8875_GPO                      0x13
#define RA8875_GPIOX                    0xC7
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            KEY-MATRIX
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define RA8875_KSCR1                    0xC0                                    //Key-Scan Control Register 1 (KSCR1)
#define RA8875_KSCR2                    0xC1                                    //Key-Scan Controller Register 2 (KSCR2)
#define RA8875_KSDR0                    0xC2                                    //Key-Scan Data Register (KSDR0)
#define RA8875_KSDR1                    0xC3                                    //Key-Scan Data Register (KSDR1)
#define RA8875_KSDR2                    0xC4                                    //Key-Scan Data Register (KSDR2)
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                         Interrupt Control Registers
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*    Interrupt Control Register1          [0xF0]
 7,6,5: (na)
 4: KEYSCAN Interrupt Enable Bit
 3: DMA Interrupt Enable Bit
 2: TOUCH Panel Interrupt Enable Bit
 1: BTE Process Complete Interrupt Enable Bit
 0:
 When MCU-relative BTE operation is selected(*1) and BTE
 Function is Enabled(REG[50h] Bit7 = 1), this bit is used to
 Enable the BTE Interrupt for MCU R/W:
 0 : Disable BTE interrupt for MCU R/W.
 1 : Enable BTE interrupt for MCU R/W.
 When the BTE Function is Disabled, this bit is used to
 Enable the Interrupt of Font Write Function:
 0 : Disable font write interrupt.
 1 : Enable font write interrupt.
 */
#define RA8875_INTC1                    0xF0                                    //Interrupt Control Register1
#define RA8875_INTC2                    0xF1                                    //Interrupt Control Register2
#define RA8875_INTCx_KEY                0x10
#define RA8875_INTCx_DMA                0x08
#define RA8875_INTCx_TP                 0x04
#define RA8875_INTCx_BTE                0x02

#define RA8875_ENABLE_INT_TP            ((uint8_t)(1<<2))
#define RA8875_DISABLE_INT_TP           ((uint8_t)(0<<2))

/* Touch Panel Enable/Disable Reg TPCR0[7] */
// #define TP_ENABLE                       ((uint8_t)(1<<7))
// #define TP_DISABLE                      ((uint8_t)(0<<7))
// #define TP_MODE_AUTO                    ((uint8_t)(0<<6))
// #define TP_MODE_MANUAL                  ((uint8_t)(1<<6))
// #define TP_DEBOUNCE_OFF                 ((uint8_t)(0<<2))
// #define TP_DEBOUNCE_ON                  ((uint8_t)(1<<2))


// #define TP_ADC_CLKDIV_1                 0
// #define TP_ADC_CLKDIV_2                 1
// #define TP_ADC_CLKDIV_4                 2
// #define TP_ADC_CLKDIV_8                 3
// #define TP_ADC_CLKDIV_16                4
// #define TP_ADC_CLKDIV_32                5
// #define TP_ADC_CLKDIV_64                6
// #define TP_ADC_CLKDIV_128               7
// #define TP_ADC_SAMPLE_512_CLKS          ((uint8_t)(0<<4))
// #define TP_ADC_SAMPLE_1024_CLKS         ((uint8_t)(1<<4))
// #define TP_ADC_SAMPLE_2048_CLKS         ((uint8_t)(2<<4))
// #define TP_ADC_SAMPLE_4096_CLKS         ((uint8_t)(3<<4))
// #define TP_ADC_SAMPLE_8192_CLKS         ((uint8_t)(4<<4))
// #define TP_ADC_SAMPLE_16384_CLKS        ((uint8_t)(5<<4))
// #define TP_ADC_SAMPLE_32768_CLKS        ((uint8_t)(6<<4))
// #define TP_ADC_SAMPLE_65536_CLKS        ((uint8_t)(7<<4))


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            COLOR DEFINES                                    +
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static const uint8_t _RA8875colorMask[6] = {11,5,0,13,8,3};                     //for color masking, first 3 byte for 65K

// // Color RA8875 compatibility
const uint16_t COLOR_BLACK              = 0x0000;
const uint16_t COLOR_WHITE              = 0xFFFF;

// const uint16_t COLOR_RED                = 0xF800;
// const uint16_t COLOR_GREEN              = 0x07E0;
// const uint16_t COLOR_BLUE               = 0x001F;

// const uint16_t COLOR_CYAN               = COLOR_GREEN | COLOR_BLUE;//0x07FF;
// const uint16_t COLOR_MAGENTA            = 0xF81F;
// const uint16_t COLOR_YELLOW             = COLOR_RED | COLOR_GREEN;//0xFFE0;
// const uint16_t COLOR_LIGHT_GREY         = 0xB5B2; // the experimentalist
// const uint16_t COLOR_LIGHT_ORANGE       = 0xFC80; // the experimentalist
// const uint16_t COLOR_DARK_ORANGE        = 0xFB60; // the experimentalist
// const uint16_t COLOR_PINK               = 0xFCFF; // M.Sandercock
// const uint16_t COLOR_PURPLE             = 0x8017; // M.Sandercock
// const uint16_t COLOR_GRAYSCALE          = 2113;//grayscale30 = RA8875_GRAYSCALE*30

// Colors UTFT compatibility
// const uint16_t VGA_BLACK                = 0x0000;
// const uint16_t VGA_WHITE                = 0xFFFF;
// const uint16_t VGA_RED                  = 0xF800;
// const uint16_t VGA_GREEN                = 0x0400;
// const uint16_t VGA_BLUE                 = 0x001F;
// const uint16_t VGA_SILVER               = 0xC618;
// const uint16_t VGA_GRAY                 = 0x8410;
// const uint16_t VGA_MAROON               = 0x8000;
// const uint16_t VGA_YELLOW               = 0xFFE0;
// const uint16_t VGA_OLIVE                = 0x8400;
// const uint16_t VGA_LIME                 = 0x07E0;
// const uint16_t VGA_AQUA                 = 0x07FF;
// const uint16_t VGA_TEAL                 = 0x0410;
// const uint16_t VGA_NAVY                 = 0x0010;
// const uint16_t VGA_FUCHSIA              = 0xF81F;
// const uint16_t VGA_PURPLE               = 0x8010;
// const uint16_t VGA_TRANSPARENT          = 0xFFFFFFFF;

// Adafruit Color Compatibility
#define RA8875_BLACK                    0x0000
#define RA8875_BLUE                     0x001F
#define RA8875_RED                      0xF800
#define RA8875_GREEN                    0x07E0
#define RA8875_CYAN                     0x07FF
#define RA8875_MAGENTA                  0xF81F
#define RA8875_YELLOW                   0xFFE0
#define RA8875_WHITE                    0xFFFF





//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//         MACROS & STRUCTS                                                    +
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef bitRead
    #define bitRead(a,b) ((a) & (1<<(b)))
#endif
#ifndef bitWrite
    #define __bitSet(value, bit) ((value) |= (1UL << (bit)))
    #define __bitClear(value, bit) ((value) &= ~(1UL << (bit)))
    #define bitWrite(value, bit, bitvalue) (bitvalue ? __bitSet(value, bit) : __bitClear(value, bit))
#endif
#ifndef PI
    #define PI 3.14159265358979323846
#endif

typedef struct {
    const uint8_t   *data;
    uint8_t         image_width;
    int             image_datalen;
} tImage;

typedef struct {
    uint8_t         char_code;
    const tImage    *image;
} tChar;

typedef struct {
    uint8_t         length;
    const tChar     *chars;
    uint8_t         font_width;
    uint8_t         font_height;
    bool            rle;
} tFont;

#if !defined(swapvals)
    #define swapvals(a, b) { typeof(a) t = a; a = b; b = t; }
#endif

enum RA8875tcursor          { NOCURSOR=0, IBEAM, UNDER, BLOCK };
enum RA8875fontCoding       { ISO_IEC_8859_1, ISO_IEC_8859_2, ISO_IEC_8859_3, ISO_IEC_8859_4 };
enum RA8875boolean          { LAYER1, LAYER2, TRANSPARENT, LIGHTEN, OR, AND, FLOATING };
enum RA8875writes           { L1=0, L2, CGRAM, PATTERN, CURSOR };
enum RA8875scrollMode       { SIMULTANEOUS, LAYER1ONLY, LAYER2ONLY, BUFFERED };
enum RA8875pattern          { P8X8, P16X16 };
enum RA8875btedatam         { CONT, RECT };
enum RA8875btelayer         { SOURCE, DEST };
enum RA8875intlist          { BTE=1,TOUCH=2, DMA=3, KEY=4 };



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                            DECLARATIONS                                     +
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class XGLCD  {
  public:
    XGLCD(void);
    void        begin(void);
    void        displayOn(boolean on);
    void        softReset(void);
    void        sleep(boolean sleep);
    void        GPIOX(boolean on);
    void        setSPI();

#if defined(XG_CPU_ESP32)
    spi_dev_t * dev = (volatile spi_dev_t *)(DR_REG_SPI3_BASE);                 // VSPI = SPI3 = default SPI port
    lldesc_t dmadesc[32];                                                       // max dma size = 32 * 4096 words = 128kByte
#endif
    
    // Windows
    void        setActiveWindow(void);
    void        setActiveWindow(int16_t XL,int16_t XR,int16_t YT,int16_t YB);
    void        getActiveWindow(int16_t &XL,int16_t &XR ,int16_t &YT ,int16_t &YB);
    void        clearMemory(bool stop=false);
    void        clearActiveWindow(bool full=false);
    uint16_t    width(bool absolute=false) const;
    uint16_t    height(bool absolute=false) const;
    void        setRotation(uint8_t rotation);
    uint8_t     getRotation();
    boolean     isPortrait(void);

    void        fillWindow(uint16_t color=COLOR_BLACK);
    void        clearScreen(uint16_t color=COLOR_BLACK);

    void        setXY(int16_t x, int16_t y);
    void        setX(int16_t x);
    void        setY(int16_t y) ;
    

    void        drawBitmap(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t * image);
    
    // Color
    void        setColorBpp(uint8_t colors);
    void        setForegroundColor(uint16_t color);
    void        setForegroundColor(uint8_t R,uint8_t G,uint8_t B);
    
    inline uint16_t Color565(uint8_t r,uint8_t g,uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3); }
    
    // PWM & Backlight
    void        PWMout(uint8_t pw,uint8_t p);
    void        brightness(uint8_t val);
    void        backlight(boolean on);
    void        PWMsetup(uint8_t pw,boolean on, uint8_t clock);
    
    // Touchscreen
    // void        touchBegin(void);
    // bool        touchReadAdc(uint32_t *x, uint32_t *y);
    // bool        touchReadPixel(uint32_t *x, uint32_t *y);
    
    // SPI & Low Level
    uint8_t     readStatus(void);
    void        writeCommand(const uint8_t d);
    
    
// #define InitTouch(o) touchBegin()
    //not compatible, no macro to be used here #define read()
// #define getX()
// #define getY()
// #define setPrecision(p)
// // #define dataAvailable() touched(false)
// #define calibrateRead()
    
    
protected:
    volatile bool _textMode;
    int16_t     LCD_WIDTH, LCD_HEIGHT;                                          //absolute
    int16_t     _width, _height;
    int16_t     _cursorX, _cursorY;

    
private:
    uint8_t     _lcdtype;
    uint8_t     _pixclk;
    uint8_t     _hsync_nondisp;
    uint8_t     _hsync_start;
    uint8_t     _hsync_pw;
    uint8_t     _hsync_finetune;
    uint8_t     _vsync_nondisp;
    uint8_t     _vsync_start;
    uint8_t     _vsync_pw;
    uint8_t     _pll_div;
    const tFont * _currentFont;
    // Touch Screen vars
    //uint32_t    _tc_debounce;
    uint16_t    _tsAdcMinX,_tsAdcMinY,_tsAdcMaxX,_tsAdcMaxY;
    // uint32_t    _readTouchADC(bool xy);
    // bool        readTouchADC(uint32_t *x, uint32_t *y);
    
    // system vars
    bool        _sleep;
    bool        _portrait;
    uint8_t     _rotation;
    int16_t     _activeWindowXL,_activeWindowXR,_activeWindowYT,_activeWindowYB;
    // color vars
    uint16_t    _foreColor;
    uint16_t    _backColor;
    uint8_t     _colorIndex;

    // color space
    uint8_t     _color_bpp;                                                     //8=256, 16=64K colors
    uint8_t     _brightness;

    // Register containers used to prevent slow readRegister from chip
    uint8_t     _DPCR_Reg;                                                      // Display Configuration [0x20]
    uint8_t     _FNCR0_Reg;                                                     // Font Control Register 0 [0x21]
    uint8_t     _FNCR1_Reg;                                                     // Font Control Register1 [0x22]
    uint8_t     _FWTSET_Reg;                                                    // Font Write Type Setting Register [0x2E]
    uint8_t     _SFRSET_Reg;                                                    // Serial Font ROM Setting [0x2F]
    uint8_t     _INTC1_Reg;                                                     // Interrupt Control Register1 [0xF0]
    volatile uint8_t _MWCR0_Reg;
    
    // Functions
    void        _setSysClock(uint8_t pll1,uint8_t pll2,uint8_t pixclk);
 
    void        _updateActiveWindow(bool full);
    void        _updateActiveWindow(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
    void        _scanDirection(boolean invertH,boolean invertV);
    void        _line_addressing(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
    // low level functions
    void        _writeRegister(const uint8_t reg, uint8_t val);
    uint8_t     _readRegister(const uint8_t reg);
    void        _writeData(uint8_t data);
    void        _writeData16(uint16_t data);
    uint8_t     _readData(bool stat=false);
    boolean     _waitPoll(uint8_t r, uint8_t f);
    void        _waitBusy(uint8_t res=0x80);                                    //0x80, 0x40(BTE busy), 0x01(DMA busy)


};

#endif
