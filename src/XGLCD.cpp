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

#include "XGLCD.h"

/******************************************************************************/
/*!
	Contructors
*/
/******************************************************************************/
XGLCD::XGLCD(void) {
}

/******************************************************************************/
/*!
	Initialize library, SPI, hardware and RA8875
*/
/******************************************************************************/
void XGLCD::begin(void) {
	
    // Init global variables
    _rotation = 0;
	_sleep = false;
    _portrait = false;
    _brightness = 255;
	_tsAdcMinX = TOUCSRCAL_XLOW; _tsAdcMinY = TOUCSRCAL_YLOW; _tsAdcMaxX = TOUCSRCAL_XHIGH; _tsAdcMaxY = TOUCSRCAL_YHIGH;
    _activeWindowXL = 0;
    _activeWindowYT = 0;
    _textMode = false;
    _color_bpp = 0;
    _lcdtype = 0;

	// Start SPI initialization
    pinMode(XG_PIN_LCD_CS, OUTPUT);
    digitalWrite(XG_PIN_LCD_CS, HIGH);
    _spibegin();
    setSPI();
    _spisetSpeed(SPI_SPEED_SLOW);
    
    // I/O lines initialization
    pinMode(XG_PIN_LCD_RESET, OUTPUT);
	digitalWrite(XG_PIN_LCD_RESET, HIGH);
	delay(10);
	digitalWrite(XG_PIN_LCD_RESET, LOW);
	delay(220);
	digitalWrite(XG_PIN_LCD_RESET, HIGH);
	delay(300);

    pinMode(XG_PIN_SD_CS, OUTPUT);
    digitalWrite(XG_PIN_SD_CS, HIGH);
    
    // RA8875 initialization
    
    // Fill shadow registers
    _DPCR_Reg = RA8875_DPCR_ONE_LAYER + RA8875_DPCR_HDIR_NORMAL + RA8875_DPCR_VDIR_NORMAL;
    _MWCR0_Reg = RA8875_MWCR0_GFXMODE + RA8875_MWCR0_NO_CURSOR + RA8875_MWCR0_CURSOR_NORMAL + RA8875_MWCR0_MEMWRDIR_LT + RA8875_MWCR0_MEMWR_CUR_INC + RA8875_MWCR0_MEMRD_NO_INC;
    _FNCR0_Reg = RA8875_FNCR0_CGROM + RA8875_FNCR0_INTERNAL_CGROM + RA8857_FNCR0_8859_1;
    _FNCR1_Reg = RA8875_FNCR1_ALIGNMENT_OFF + RA8875_FNCR1_TRANSPARENT_OFF + RA8875_FNCR1_NORMAL + RA8875_FNCR1_SCALE_HOR_1 + RA8875_FNCR1_SCALE_VER_1;
    _FWTSET_Reg = RA8875_FWTSET_16X16;
    _SFRSET_Reg = 0b00000000;
    _INTC1_Reg = 0b00000000;

    // Software Reset
    writeCommand(RA8875_PWRR);
    _writeData(RA8875_PWRR_SOFTRESET);
    delay(20);
    _writeData(RA8875_PWRR_NORMAL);
    delay(200);
    // Set slow clock speed with default pixclk
    _setSysClock(0x07, RA8875_PLLC2_DIV8, RA8875_PCSR_PDATL | RA8875_PCSR_2CLK);

    // Color space setup (RA8875_SYSR)
    setColorBpp(16);
    
    // Get lcd type and set timing values accordingly
    // PIN1-5 = GPI0-GPI4
    // PIN6-9 = GPO0-GPO3
    // PIN10 = PMW2
    #ifdef XG_LCD_AUTO
        _writeRegister(RA8875_KSCR1, 0x00);                                     // Disable keyboard scanning
        /*_writeRegister(RA8875_GPO, 0x00);
        delay(10);
        if (!(_readRegister(RA8875_GPI) & 0x01)) {
            _writeRegister(RA8875_GPO, 0x01);
            delay(10);
            if (_readRegister(RA8875_GPI) & 0x01) _lcdtype = 1;
        }
        _writeRegister(RA8875_GPO, 0x00);
        delay(10);
        if (!(_readRegister(RA8875_GPI) & 0x02)) {
            _writeRegister(RA8875_GPO, 0x02);
            delay(10);
            if (_readRegister(RA8875_GPI) & 0x02) _lcdtype += 2;
        }
        _writeRegister(RA8875_GPO, 0x00);
        delay(10);
        if (!(_readRegister(RA8875_GPI) & 0x04)) {
            _writeRegister(RA8875_GPO, 0x04);
            delay(10);
            if (_readRegister(RA8875_GPI) & 0x04) _lcdtype += 4;
        }*/
        //uint8_t i = _readRegister(RA8875_GPI);
        uint8_t i;
        writeCommand(RA8875_GPI);
        _spiCSLow;
#ifdef _spixread
        _spixread(RA8875_DATAREAD, i);
#else
        _spiwrite(RA8875_DATAREAD);
        delayMicroseconds(50);                                                  // Stabilize time, else first bit is read wrong
        _spiread(i);
#endif
        _spiCSHigh;

        if (i == 0x00) _lcdtype = 0;                                            // Default is lcdtype 0 = 5.0" 800x480
        if (i == 0x01) _lcdtype = 2;                                            // lcdtype 2 = 9.0" 800x480
    #else
        #if defined(XG_LCD_56)
            _lcdtype = 1;
        #elif defined(XG_LCD_90)
            _lcdtype = 2;
        #endif
    #endif

    switch (_lcdtype) {
        case 0:
        case 2:                                                                 // 800x480
            _activeWindowXR  = LCD_WIDTH = _width = 800;
            _activeWindowYB  = LCD_HEIGHT = _height = 480;
            _pixclk          = RA8875_PCSR_PDATL | RA8875_PCSR_2CLK;
            _hsync_finetune  = 0;
            _hsync_nondisp   = 26;
            _hsync_start     = 32;
            _hsync_pw        = 96;
            _vsync_nondisp   = 32;
            _vsync_start     = 23;
            _vsync_pw        = 2;
            _pll_div         = RA8875_PLLC2_DIV4;
            break;
        case 1:                                                                 // 640x480
            _activeWindowXR  = LCD_WIDTH = _width = 640;
            _activeWindowYB  = LCD_HEIGHT = _height = 480;
            _pixclk          = RA8875_PCSR_2CLK;
            _hsync_finetune  = 5;
            _hsync_nondisp   = 127;
            _hsync_start     = 16;
            _hsync_pw        = 8;
            _vsync_nondisp   = 11;
            _vsync_start     = 15;
            _vsync_pw        = 2;
            _pll_div         = RA8875_PLLC2_DIV4;
            break;
        /*
         case A:                                                                // 320240
            _pixclk          = RA8875_PCSR_8CLK;
            _hsync_finetune  = 0;
            _hsync_nondisp   = 42;
            _hsync_start     = 40;
            _hsync_pw        = 32;
            _vsync_nondisp   = 6;
            _vsync_start     = 15;
            _vsync_pw        = 3;
            _pll_div         = RA8875_PLLC2_DIV2;
            break;
         case B:                                                                // 480272
            _pixclk          = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
            _hsync_finetune  = 0;
            _hsync_nondisp   = 10;
            _hsync_start     = 8;
            _hsync_pw        = 48;
            _vsync_nondisp   = 3;
            _vsync_start     = 8;
            _vsync_pw        = 10;
            _pll_div         = RA8875_PLLC2_DIV4;
            break;
        */
    }
    
    _writeRegister(RA8875_HDWR, ((LCD_WIDTH)/8) - 1);		                    // LCD Horizontal Display Width Register
    _writeRegister(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + _hsync_finetune);     // Horizontal Non-Display Period Fine Tuning Option Register
    _writeRegister(RA8875_HNDR, (_hsync_nondisp-_hsync_finetune - 2) / 8);		// LCD Horizontal Non-Display Period Register
    _writeRegister(RA8875_HSTR, _hsync_start/8 - 1);		                    // HSYNC Start Position Register
    _writeRegister(RA8875_HPWR, RA8875_HPWR_LOW + (_hsync_pw/8 - 1));		    // HSYNC Pulse Width Register
    _writeRegister(RA8875_VDHR0, (uint16_t)(((LCD_HEIGHT) - 1) & 0xFF));         // LCD Vertical Display Height Register0
    _writeRegister(RA8875_VDHR0+1, (uint16_t)((LCD_HEIGHT) - 1) >> 8);	        // LCD Vertical Display Height Register1
    _writeRegister(RA8875_VNDR0, _vsync_nondisp-1);                             // LCD Vertical Non-Display Period Register 0
    _writeRegister(RA8875_VNDR0+1, _vsync_nondisp >> 8);	                    // LCD Vertical Non-Display Period Register 1
    _writeRegister(RA8875_VSTR0, _vsync_start-1);                               // VSYNC Start Position Register 0
    _writeRegister(RA8875_VSTR0+1, _vsync_start >> 8);	                        // VSYNC Start Position Register 1
    _writeRegister(RA8875_VPWR, RA8875_VPWR_LOW + _vsync_pw - 1);               // VSYNC Pulse Width Register
    
    
    // Set clock speed to normal
    _setSysClock(0x0b, _pll_div, _pixclk);
    _updateActiveWindow(true);									                // set the whole screen as active
    delay(10);

    _spisetSpeed(SPI_SPEED_WRITE);
    delay(1);
    
    // Clear the display and switch it on
    clearMemory();
    delay(1);
	displayOn(true);
    delay(1);
    backlight(true);
	setRotation(0);
    
    // Start touchscreen
    //touchBegin();
}

/******************************************************************************/
/*!
 This function set the sysClock accordingly datasheet
 Parameters:
 pll1: PLL Control Register 1
 pll2: PLL Control Register 2
 pixclk: Pixel Clock Setting Register
 [private]
 */
/******************************************************************************/
void XGLCD::_setSysClock(uint8_t pll1,uint8_t pll2,uint8_t pixclk)
{
    _writeRegister(RA8875_PLLC1,pll1);
    delay(1);
    _writeRegister(RA8875_PLLC1+1,pll2);
    delay(1);
    _writeRegister(RA8875_PCSR,pixclk);
    delay(1);
}

/******************************************************************************/
/*!
 turn display on/off
 */
/******************************************************************************/
void XGLCD::displayOn(boolean on)
{
    on == true ? _writeRegister(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON) : _writeRegister(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPOFF);
}


/******************************************************************************/
/*!
 Set SPI bus to defaults for LCD Driver
 */
/******************************************************************************/
// NO interrupts are disabled: Spark & MKR1000 do not allow this for a 'long' period which results in hang-ups
// do NOT allow interrupt routines to use the SPI bus as this will cause problems with the LCD & SDCard functions

// Every LCD SPI bus use MUST be preceded with an SPI bus configuration as other SPI device using the same bus
// might have reconfigured the SPI hardware
//
// Before calling a LCD library function, first call the setSPI() function to set the SPI bus in the correct mode.
// This is only required if other devices use the same SPI bus (for example the SDCard)

void XGLCD::setSPI(void) {
    _spisetDataMode(SPI_MODE_LCD);
    _spisetBitOrder(MSBFIRST);
    _spisetSpeed(SPI_SPEED_WRITE);
#ifdef _spisetBitLen
    _spisetBitLen;
#endif
}

/*
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+                                RA8875 INTERNAL                               +
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Stuff you will probably never need in your UI, but it is important for the
internals of the RA8875.
*/

/******************************************************************************/
/*!
	return true when register has done the job, otherwise false.
*/
/******************************************************************************/
boolean XGLCD::_waitPoll(uint8_t regname, uint8_t waitflag)
{
	uint8_t temp;
	unsigned long timeout = millis();
	
	while (1) {
		temp = _readRegister(regname);
		if (!(temp & waitflag)) return true;
		if ((millis() - timeout) > 20) return false;                            //emergency exit! Should never occur.
	}  
	return false;
}

/******************************************************************************/
/*!
	Just another specified wait routine until job it's done
	Parameters:
	res:
	0x80(for most operations),
	0x40(BTE wait), 
	0x01(DMA wait) (X-Graph:not allowed anymore)
 
Note:
Does a continuous RA8875_CMDREAD read (slow SPI bus, one byte read).
A CMDREAD is not possible, it always reads the 'Status Register'.
 - SR.7 = 1 = memory read/write busy (which includes font writes)
 - SR.6 = 1 = BTE busy
 - other bits are related to touch panel detection / sleep modus / external flash/rom busy

Used in the following library function calls:
 - clearMemory(): SR.7 memory clear is completed
 - _charWrite(): SR.7 -> internal font character is completely written
 - BTE_Move(): SR.6 is checked before AND after the BTE commands are given
 - BTE_Enable(): SR.6 -> wait for BTE done/idle
 - uselayers() when switching to 8-bit colors: _waitBusy() call = no parameters: ???? -> this results in a 100msec wait -> changed to SR.7 check
    (but why is this needed in this situation ?)
 - WritePattern(): SR.7 -> checks after each word written (needed for pattern ??)
 
 */
/******************************************************************************/
void XGLCD::_waitBusy(uint8_t res)
{
	uint8_t temp; 	
	unsigned long start = millis();
    
	do {
		temp = readStatus();
        if ((millis() - start) > 10) return;
    } while ((temp & res) == res);
}

/******************************************************************************/
/*!
 Performs a SW-based reset of the RA8875
 */
/******************************************************************************/
void XGLCD::softReset(void) {
    writeCommand(RA8875_PWRR);
    _writeData(RA8875_PWRR_SOFTRESET);
    _writeData(RA8875_PWRR_NORMAL);
    delay(1);
}

/******************************************************************************/
/*!
 Sleep mode on/off (complete sequence)
 The sleep on/off sequence it's quite tricky on RA8875 when in SPI mode!
 */
/******************************************************************************/
void XGLCD::sleep(boolean sleep)
{
    if (_sleep != sleep){                                                       // only when needed
        _sleep = sleep;
        if (_sleep == true){
            //1)turn off backlight
            //2)decelerate SPI clock
            _spisetSpeed(SPI_SPEED_SLOW);
            //3)set PLL to default
            _setSysClock(0x07,RA8875_PLLC2_DIV8,RA8875_PCSR_PDATR | RA8875_PCSR_4CLK);
            //4)display off & sleep
            _writeRegister(RA8875_PWRR, RA8875_PWRR_DISPOFF | RA8875_PWRR_SLEEP);
            delay(100);
        } else {
            //1)wake up with display off(100ms)
            _writeRegister(RA8875_PWRR, RA8875_PWRR_DISPOFF);
            delay(100);
            //2)bring back the pll
            _setSysClock(0x07, RA8875_PLLC2_DIV8, _pixclk);
            //_writeRegister(RA8875_PCSR,initStrings[_initIndex][2]);//Pixel Clock Setting Register
            delay(20);
            _writeRegister(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);//disp on
            delay(20);
            //4)resume SPI speed
            _spisetSpeed(SPI_SPEED_WRITE);
            //5)PLL afterburn!
            _setSysClock(0x0b, _pll_div, _pixclk);
            //_writeRegister(RA8875_PWRR, RA8875_PWRR_NORMAL);
        }
    }
}

/******************************************************************************/
/*!
 Sets or resets the GPIOX line
 Switches the power on/off of the LCD
 */
/******************************************************************************/
void XGLCD::GPIOX(boolean on) {
    if (on)
        _writeRegister(RA8875_GPIOX, 1);
    else
        _writeRegister(RA8875_GPIOX, 0);
}


/*
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+                                WINDOWS                                       +
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
The RA8875 has 768kByte internal memory.
For a 800x480 LCD (16-bit color) the full memory is needed.
By only using 8-bit color 384kByte is needed, and 2 layers are available.
 
Many RA8875 hardware accelerated drawing functions can limit the drawing area
to the 'active window'. If this is not defined the 'full screen' is used.
Depending on the number of layers 1 or 2 'full screens' are available. The
active window can be positioned anywhere in the fulls screen area.
 
Here you will also find functions to rotate the display.
*/

/******************************************************************************/
/*!
 Set the Active Window as FULL SCREEN
 */
/******************************************************************************/
void XGLCD::setActiveWindow(void)
{
    _activeWindowXL = 0; _activeWindowXR = LCD_WIDTH;
    _activeWindowYT = 0; _activeWindowYB = LCD_HEIGHT;
    if (_portrait){swapvals(_activeWindowXL,_activeWindowYT); swapvals(_activeWindowXR,_activeWindowYB);}
    _updateActiveWindow(true);
}

/******************************************************************************/
/*!
 Set the Active Window
 Parameters:
 XL: Horizontal Left
 XR: Horizontal Right
 YT: Vertical TOP
 YB: Vertical Bottom
 */
/******************************************************************************/
void XGLCD::setActiveWindow(int16_t XL,int16_t XR ,int16_t YT ,int16_t YB)
{
    if (_portrait) {swapvals(XL,YT); swapvals(XR,YB);}
    
    if (XR >= LCD_WIDTH) XR = LCD_WIDTH;
    if (YB >= LCD_HEIGHT) YB = LCD_HEIGHT;
    
    _activeWindowXL = XL; _activeWindowXR = XR;
    _activeWindowYT = YT; _activeWindowYB = YB;
    _updateActiveWindow(false);
}

/******************************************************************************/
/*!
 this updates the RA8875 Active Window registers
 [private]
 */
/******************************************************************************/
void XGLCD::_updateActiveWindow(bool full)
{
    if (full){
        // X
        _writeRegister(RA8875_HSAW0,    0x00);
        _writeRegister(RA8875_HSAW0 + 1,0x00);
        _writeRegister(RA8875_HEAW0,    (LCD_WIDTH) & 0xFF);
        _writeRegister(RA8875_HEAW0 + 1,(LCD_WIDTH) >> 8);
        // Y
        _writeRegister(RA8875_VSAW0,    0x00);
        _writeRegister(RA8875_VSAW0 + 1,0x00);
        _writeRegister(RA8875_VEAW0,    (LCD_HEIGHT) & 0xFF);
        _writeRegister(RA8875_VEAW0 + 1,(LCD_HEIGHT) >> 8);
    } else {
        // X
        _writeRegister(RA8875_HSAW0,    _activeWindowXL & 0xFF);
        _writeRegister(RA8875_HSAW0 + 1,_activeWindowXL >> 8);
        _writeRegister(RA8875_HEAW0,    _activeWindowXR & 0xFF);
        _writeRegister(RA8875_HEAW0 + 1,_activeWindowXR >> 8);
        // Y
        _writeRegister(RA8875_VSAW0,     _activeWindowYT & 0xFF);
        _writeRegister(RA8875_VSAW0 + 1,_activeWindowYT >> 8);
        _writeRegister(RA8875_VEAW0,    _activeWindowYB & 0xFF);
        _writeRegister(RA8875_VEAW0 + 1,_activeWindowYB >> 8);
    }
}

/******************************************************************************/
/*!
 Get the Active Window
 Parameters:
 XL: Horizontal Left
 XR: Horizontal Right
 YT: Vertical TOP
 YB: Vertical Bottom
 */
/******************************************************************************/
void XGLCD::getActiveWindow(int16_t &XL,int16_t &XR ,int16_t &YT ,int16_t &YB)//0.69b24
{
    XL = _activeWindowXL; XR = _activeWindowXR;
    YT = _activeWindowYT; YB = _activeWindowYB;
}

/******************************************************************************/
/*!		
		Clear memory = fill window with 0's
        (different from fillWindow/clearScreen which fill the window with a
        color)
 	    Parameters:
            1: start the clear memory operation
            0: stop (or interrupt) the clear memory operation
 
 This function will:
 1. clear the full window (1 layer) when bit 6 is '0' (clearActiveWindow())
 2. clear the current active window when bit 6 is '1'

 This function uses the MCLR register.
 By setting bit 7 '1' the memory clearing starts (clearMemory(false)).
 It can be stopped by writing a '0' to bit7 (clearMemory(true)).
 The RA8875 will auto-clear bit7 once the memory clearing is finished
 (hence the _waitBusy(0x80) call).
 The function works in co-operation with clearActiveWindow(). Based on 
 the contents of bit 6 of MCLR (set or reset with clearActiveWindow()),
 the clearMemory() function will clear all memory of the active windows
 memory only.
 
 This function is claimed to be slow, and it should be better to use
 fillWindow which is claimed to work faster and have the same result.
/******************************************************************************/
void XGLCD::clearMemory(bool stop)
{
    uint8_t temp;
    temp = _readRegister(RA8875_MCLR);
    stop == true ? temp &= ~RA8875_MCLR_START : temp |= RA8875_MCLR_START;
    _writeData(temp);
    if (!stop) _waitBusy(0x80);
}


/******************************************************************************/
/*!
 Clear the active window
 Parameters:
 full: false(clear current window), true clear full window
 */
/******************************************************************************/
void XGLCD::clearActiveWindow(bool full)
{
    uint8_t temp;
    temp = _readRegister(RA8875_MCLR);
    full == true ? temp &= ~RA8875_MCLR_ACTIVE : temp |= RA8875_MCLR_ACTIVE;
    _writeData(temp);
}

/******************************************************************************/
/*!		
		Return the max tft width.
		Parameters: 
		absolute:
        - true: physical width of LCD
        - false: visible width (depends on portrait or landscape mode)
*/
/******************************************************************************/
uint16_t XGLCD::width(bool absolute) const
{ 
	if (absolute) return LCD_WIDTH;
	return _width; 
}

/******************************************************************************/
/*!		
		Return the max tft height.
		Parameters: 
		absolute:
        - true: physical height of LCD
        - false: visible height (depends on portrait or landscape mode)
*/
/******************************************************************************/
uint16_t XGLCD::height(bool absolute) const
{ 
	if (absolute) return LCD_HEIGHT;
	return _height; 
}

/******************************************************************************/
/*!
      Change the beam scan direction on display
	  Parameters:
	  invertH: true(inverted),false(normal) horizontal
	  invertV: true(inverted),false(normal) vertical
*/
/******************************************************************************/
void XGLCD::_scanDirection(boolean invertH,boolean invertV)
{
	invertH == true ? _DPCR_Reg |= RA8875_DPCR_HDIR_REVERSE : _DPCR_Reg &= ~RA8875_DPCR_HDIR_REVERSE;
	invertV == true ? _DPCR_Reg |= RA8875_DPCR_VDIR_REVERSE : _DPCR_Reg &= ~RA8875_DPCR_VDIR_REVERSE;
	_writeRegister(RA8875_DPCR,_DPCR_Reg);
}

/******************************************************************************/
/*!
      Change the rotation of the screen
	  Parameters:
	  rotation:
	    0 = default
		1 = 90 clockwise
		2 = 180 = upside down
		3 = 270 clockwise
*/
/******************************************************************************/
void XGLCD::setRotation(uint8_t rotation)//0.69b32 - less code
{
	_rotation = rotation % 4;                                                   //limit to the range 0-3
	switch (_rotation) {
	case 0:
		//default
		_portrait = false;
		_scanDirection(0,0);
		_tsAdcMinX = TOUCSRCAL_XLOW; _tsAdcMinY = TOUCSRCAL_YLOW; _tsAdcMaxX = TOUCSRCAL_XHIGH; _tsAdcMaxY = TOUCSRCAL_YHIGH;
    break;
	case 1:
		//90
		_portrait = true;
		_scanDirection(1,0);
		_tsAdcMinX = TOUCSRCAL_XHIGH; _tsAdcMinY = TOUCSRCAL_YLOW; _tsAdcMaxX = TOUCSRCAL_XLOW; _tsAdcMaxY = TOUCSRCAL_YHIGH;
    break;
	case 2:
		//180
		_portrait = false;
		_scanDirection(1,1);
		_tsAdcMinX = TOUCSRCAL_XHIGH; _tsAdcMinY = TOUCSRCAL_YHIGH; _tsAdcMaxX = TOUCSRCAL_XLOW; _tsAdcMaxY = TOUCSRCAL_YLOW;
    break;
	case 3:
		//270
		_portrait = true;
		_scanDirection(0,1);
		_tsAdcMinX = TOUCSRCAL_XLOW; _tsAdcMinY = TOUCSRCAL_YHIGH; _tsAdcMaxX = TOUCSRCAL_XHIGH; _tsAdcMaxY = TOUCSRCAL_YLOW;
    break;
	}
    
	if (_portrait){
		_width = LCD_HEIGHT;
		_height = LCD_WIDTH;
	} else {
		_width = LCD_WIDTH;
		_height = LCD_HEIGHT;
	}
    
    if (_portrait) _FNCR1_Reg |= RA8875_FNCR1_90DEGREES; else _FNCR1_Reg &= ~RA8875_FNCR1_90DEGREES;
    _writeRegister(RA8875_FNCR1,_FNCR1_Reg);                                    // Rotates fonts 90 degrees (for build-in fonts that is)
    setActiveWindow();
}

/******************************************************************************/
/*!
      Get rotation setting
*/
/******************************************************************************/
uint8_t XGLCD::getRotation()
{
	return _rotation;
}

/******************************************************************************/
/*!
      true if rotation 1 or 3
*/
/******************************************************************************/
boolean XGLCD::isPortrait(void)
{
	return _portrait;
}



/******************************************************************************/
/*!
 Fill the ActiveWindow by using a specified RGB565 color
 Parameters:
 color: RGB565 color (default=BLACK)
 */
/******************************************************************************/
void XGLCD::fillWindow(uint16_t color)
{
    _line_addressing(0,0,LCD_WIDTH-1, LCD_HEIGHT-1);
    setForegroundColor(color);
    writeCommand(RA8875_DCR);
    _writeData(0xB0);
    _waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
    //_TXTrecoverColor = true;
}

/******************************************************************************/
/*!
 clearScreen it's different from fillWindow because it doesn't depends
 from the active window settings so it will clear all the screen.
 It should be used only when needed since it's slower than fillWindow.
 parameter:
 color: 16bit color (default=BLACK)
 */
/******************************************************************************/
void XGLCD::clearScreen(uint16_t color)
{
    setActiveWindow();
    fillWindow(color);
}


/*****************************************************************************
/*!
 Set the position for Graphic Write
 Parameters:
 x: horizontal position
 y: vertical position
 
 Note: X-GRAPH: these are actually internal functions. There is no
 purpose for normal drawing as X,Y are included in all drawing
 functions.
 */
/******************************************************************************/

void XGLCD::setXY(int16_t x, int16_t y)
{
    setX(x);
    setY(y);
}

/******************************************************************************/
/*!
 Set the x position for Graphic Write
 Parameters:
 x: horizontal position
 */
/******************************************************************************/
void XGLCD::setX(int16_t x)
{
    if (x < 0) x = 0;
    if (_portrait){
        if (x >= LCD_HEIGHT) x = LCD_HEIGHT-1;
        _writeRegister(RA8875_CURV0, x & 0xFF);
        _writeRegister(RA8875_CURV0+1, x >> 8);
    } else {
        if (x >= LCD_WIDTH) x = LCD_WIDTH-1;
        _writeRegister(RA8875_CURH0, x & 0xFF);
        _writeRegister(RA8875_CURH0+1, (x >> 8));
    }
}

/******************************************************************************/
/*!
 Set the y position for Graphic Write
 Parameters:
 y: vertical position
 */
/******************************************************************************/
void XGLCD::setY(int16_t y)
{
    if (y < 0) y = 0;
    if (_portrait){
        if (y >= LCD_WIDTH) y = LCD_WIDTH-1;
        _writeRegister(RA8875_CURH0, y & 0xFF);
        _writeRegister(RA8875_CURH0+1, (y >> 8));
    } else {
        if (y >= LCD_HEIGHT) y = LCD_HEIGHT-1;
        _writeRegister(RA8875_CURV0, y & 0xFF);
        _writeRegister(RA8875_CURV0+1, y >> 8);
    }
}


// /******************************************************************************/
// /*!
//  Graphic line addressing helper
//  [private]
//  */
// /******************************************************************************/
void XGLCD::_line_addressing(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
    //X0
    _writeRegister(RA8875_DLHSR0,    x0 & 0xFF);
    _writeRegister(RA8875_DLHSR0 + 1,x0 >> 8);
    //Y0
    _writeRegister(RA8875_DLVSR0,    y0 & 0xFF);
    _writeRegister(RA8875_DLVSR0 + 1,y0 >> 8);
    //X1
    _writeRegister(RA8875_DLHER0,    x1 & 0xFF);
    _writeRegister(RA8875_DLHER0 + 1,x1 >> 8);
    //Y1
    _writeRegister(RA8875_DLVER0,    y1 & 0xFF);
    _writeRegister(RA8875_DLVER0 + 1,y1 >> 8);
}



void XGLCD::drawBitmap(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t * image) {
    uint32_t count;

    uint32_t i;
    i = (uint32_t)image;
    
    setActiveWindow(x1,x2,y1,y2);
    setXY(x1,y1);
    count = (x2 - x1 + 1) * (y2 - y1 + 1);
    writeCommand(RA8875_MRWC);
    _spiCSLow;
#ifdef _spixwritedma
    _spixwritedma(RA8875_DATAWRITE, image, count);
#else
    _spiwrite(RA8875_DATAWRITE);
    _spiwritedma(image, count);
#endif
    _spiCSHigh;
}



// /******************************************************************************/
// /*!
//       Set the display 'Color Space'
// 	  Parameters:
// 	  Bit per Pixel color (colors): 8 or 16 bit
// 	  NOTE:
// 	  For display over 272*480 give the ability to use
// 	  Layers since at 16 bit it's not possible.
// */
// /******************************************************************************/
void XGLCD::setColorBpp(uint8_t colors)
{
	if (colors != _color_bpp){                                                  //only if necessary
		if (colors < 16) {
			_color_bpp = 8;
			_colorIndex = 3;
			_writeRegister(RA8875_SYSR, RA8875_SYSR_8BPP);
			//_maxLayers = 2;
		} else if (colors > 8) {                                                //65K
			_color_bpp = 16;
			_colorIndex = 0;
			_writeRegister(RA8875_SYSR, RA8875_SYSR_16BPP);
			//_maxLayers = 1;
			//_currentLayer = 0;
		}
	}
}

// /******************************************************************************/
// /*!
//       Return current Color Space (8 or 16)
// */
// /******************************************************************************/
// uint8_t XGLCD::getColorBpp(void)
// {
// 	return _color_bpp;
// }
                             
// /******************************************************************************/
// /*!
//       Sets set the foreground color using 16bit RGB565 color
// 	  It handles automatically color conversion when in 8 bit!
// 	  Parameters:
// 	  color: 16bit color RGB565
// */
// /******************************************************************************/
void XGLCD::setForegroundColor(uint16_t color)
{
	_foreColor = color;

    _writeRegister(RA8875_FGCR0,((color & 0xF800) >> _RA8875colorMask[_colorIndex]));
    _writeRegister(RA8875_FGCR0+1,((color & 0x07E0) >> _RA8875colorMask[_colorIndex+1]));
    _writeRegister(RA8875_FGCR0+2,((color & 0x001F) >> _RA8875colorMask[_colorIndex+2]));
}
    
// /******************************************************************************/
// /*!
//       Sets set the foreground color using 8bit R,G,B
// 	  Parameters:
// 	  R: 8bit RED
// 	  G: 8bit GREEN
// 	  B: 8bit BLUE
// */
// /******************************************************************************/
void XGLCD::setForegroundColor(uint8_t R,uint8_t G,uint8_t B)
{
	_foreColor = Color565(R,G,B);

    _writeRegister(RA8875_FGCR0,R);
    _writeRegister(RA8875_FGCR0+1,G);
    _writeRegister(RA8875_FGCR0+2,B);
}

/*
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+								PWM STUFF								       +
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/

/******************************************************************************/
/*!
		PWM out
		Parameters:
		pw: pwm selection (1,2)
		p: 0...255 rate
		
*/
/******************************************************************************/
void XGLCD::PWMout(uint8_t pw,uint8_t p)
{
	uint8_t reg;
	pw > 1 ? reg = RA8875_P2DCR : reg = RA8875_P1DCR;
	_writeRegister(reg, p);
}

/******************************************************************************/
/*!
		Set the brightness of the backlight (if connected to pwm)
		(basic controls pwm 1)
		Parameters:
		val: 0...255
*/
/******************************************************************************/
void XGLCD::brightness(uint8_t val)
{
	_brightness = val;
	PWMout(1,_brightness);
}

/******************************************************************************/
/*!
		controls the backligh by using PWM engine.
		Parameters:
		on: true(backlight on), false(backlight off)
*/
/******************************************************************************/
void XGLCD::backlight(boolean on)
{
	if (on == true){
        PWMsetup(1,true, RA8875_PWM_CLK_DIV1024);                               //setup PWM ch 1 for backlight
        PWMout(1,_brightness);                                                  //turn on PWM1
    } else {
        PWMsetup(1,false, RA8875_PWM_CLK_DIV1024);                              //setup PWM ch 1 for backlight
	}
}

/******************************************************************************/
/*!
		Setup PWM engine
		Parameters:
		pw: pwm selection (1,2)
		on: turn on/off
		clock: the clock setting
		[private]
*/
/******************************************************************************/
void XGLCD::PWMsetup(uint8_t pw,boolean on, uint8_t clock)
{
	uint8_t reg;
	uint8_t set;
	if (pw > 1){
		reg = RA8875_P2CR;
		on == true ? set = RA8875_PxCR_ENABLE : set = RA8875_PxCR_DISABLE;
	} else {
		reg = RA8875_P1CR;
		on == true ? set = RA8875_PxCR_ENABLE : set = RA8875_PxCR_DISABLE;
	}
	_writeRegister(reg,(set | (clock & 0xF)));
}



// /*
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +							     TOUCH SCREEN        					       +
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// */

// /******************************************************************************/
// /*!   
// 	  Initialize support for on-chip resistive Touch Screen controller

//  Touch Panel Control Register 0  TPCR0  [0x70]
//  7: 0(disable, 1:(enable)
//  6,5,4:TP Sample Time Adjusting (000...111)
//  3:Touch Panel Wakeup Enable 0(disable),1(enable)
//  2,1,0:ADC Clock Setting (000...111) set fixed to 010: (System CLK) / 4, 10Mhz Max
// !*/
// /******************************************************************************/
// #define RA8875_TPCR0_TOUCH 0x83 // enable touch panel / 512 system clocks / disable wake-up / system_clk/8
// #define RA8875_TPCR1_IDLE 0x40 // manual mode / Vref internal / debounce disabled / Idle mode
// #define RA8875_TPCR1_WAIT 0x41 // manual mode / Vref internal / debounce disabled / Wait for TP event
// #define RA8875_TPCR1_LATCH_X 0x42 // manual mode / Vref internal / debounce disabled / Latch X data
// #define RA8875_TPCR1_LATCH_Y 0x43 // manual mode / Vref internal / debounce disabled / Latch Y data

// void XGLCD::touchBegin(void)
// {
//     _writeRegister(RA8875_TPCR0, RA8875_TPCR0_TOUCH);
//     _writeRegister(RA8875_TPCR1, RA8875_TPCR1_WAIT);
// }

// /******************************************************************************/
// /*!   
// 	  Read 10bit internal ADC of RA8875 registers and perform corrections
// 	  It will return always RAW data
// 	  Parameters:
// 	  x: out 0...1023
// 	  Y: out 0...1023

// */
// /******************************************************************************/
// #define TC_AVERAGE 13                                                           // Number of samples for averaging (only 5 results are used)
// #define TC_DEBOUNCE 50                                                          // xx msec debouncing timeout

// uint32_t XGLCD::_readTouchADC(bool xy) {
//     uint32_t avg[TC_AVERAGE];
//     uint32_t i,j,n,t;
    
//     if (xy) _writeRegister(RA8875_TPCR1, RA8875_TPCR1_LATCH_Y); else _writeRegister(RA8875_TPCR1, RA8875_TPCR1_LATCH_X); // First do a dummy read to let the touchscreen voltage stabilize
//     delayMicroseconds(10);                                                      // Wait for the ADC to stabilize
//     _writeRegister(RA8875_TPCR1, RA8875_TPCR1_IDLE);                            // Read the ADC data but don't use this data
//     _readRegister(RA8875_TPXH);
//     _readRegister(RA8875_TPYH);
//     _readRegister(RA8875_TPXYL);

//     for (i = 0; i < TC_AVERAGE; i++) {                                          // Now get several readings for an averaging
//         if (xy) _writeRegister(RA8875_TPCR1, RA8875_TPCR1_LATCH_Y); else _writeRegister(RA8875_TPCR1, RA8875_TPCR1_LATCH_X);
//         delayMicroseconds(100);                                                 // Wait for the ADC to stabilize
//         _writeRegister(RA8875_TPCR1, RA8875_TPCR1_IDLE);                        // Read the ADC data and this time use the data
//         if (xy) j = _readRegister(RA8875_TPYH); else j = _readRegister(RA8875_TPXH);
//         avg[i] = j;
//         avg[i] <<= 2;
//         j = _readRegister(RA8875_TPXYL);
//         if (xy) avg[i] += (j >> 2) & 0x03; else avg[i] += j & 0x3;              // R8875 stores 2-bits of 10-bit result in 3th register (for both X and Y)
//     }
    
//     for (i = 0; i < TC_AVERAGE; i++) {                                          // Sort all read values
//         j = 0;
//         while (j < i) {
//             n = j++;
//             if (avg[n] > avg[j]) {
//                 t = avg[n];
//                 avg[n] = avg[j];
//                 avg[j] = t;
//             }
//         }
//     }

//     // Calculate the mean value of the middle measurements
//     return (avg[(TC_AVERAGE/2)-3]+avg[(TC_AVERAGE/2)-2]+avg[(TC_AVERAGE/2)-1]+avg[TC_AVERAGE/2]+avg[(TC_AVERAGE/2)+1]+avg[(TC_AVERAGE/2)+2]+avg[(TC_AVERAGE/2)+3])/7;
// }

// bool XGLCD::readTouchADC(uint32_t *x, uint32_t *y)
// {
//     if ((readStatus() & 0x20) == 0) {                                           // Read touchscreen touched bit in status register
//         _writeRegister(RA8875_TPCR1, RA8875_TPCR1_WAIT);                        // Not touched, put in touch detection mode again (maybe not needed this line)
//         _tc_debounce = TC_DEBOUNCE;                                             // Debouncing
//     } else {
//         delay(_tc_debounce);                                                    // Touched, first wait for debouncing timeout
//         _tc_debounce = TC_DEBOUNCE;                                             // Default debouncing time
//         if (readStatus() & 0x20) {                                              // If still touched get the data
//             *x = _readTouchADC(false);                                          // Read X coordinates
//             *y = _readTouchADC(true);                                           // Read Y coordinates
//             _writeRegister(RA8875_TPCR1, RA8875_TPCR1_WAIT);                    // Start waiting for a touch again
//             delayMicroseconds(1000);                                            // Wait a bit
//             if (readStatus() & 0x20) {                                          // Make sure the screen is still touched to prevent false coordinates during touch release
//                 _tc_debounce = 0;                                               // It's still touched, next touch detection does not require debouncing
//                 return true;
//             }
//         } else _writeRegister(RA8875_TPCR1, RA8875_TPCR1_WAIT);                 // Not touched, put in touch detection mode again
//     }
//     return false;                                                               // Not touched
// }

// /******************************************************************************/
// /*!   
// 	  Returns 10bit x,y data with TRUE scale (0...1023)
// 	  Parameters:
// 	  x: out 0...1023
// 	  Y: out 0...1023
// */
// /******************************************************************************/
// bool XGLCD::touchReadAdc(uint32_t *x, uint32_t *y)
// {
//     uint32_t tx,ty;
//     bool touched;
    
// 	touched = readTouchADC(&tx,&ty);
// 	*x = map(tx,_tsAdcMinX,_tsAdcMaxX,0,1024);
// 	*y = map(ty,_tsAdcMinY,_tsAdcMaxY,0,1024);
//     return touched;
// }

// /******************************************************************************/
// /*!   
// 	  Returns pixel x,y data with SCREEN scale (screen width, screen Height)
// 	  Parameters:
// 	  x: out 0...screen width  (pixels)
// 	  Y: out 0...screen Height (pixels)
// 	  Check for out-of-bounds here as touches near the edge of the screen
// 	  can be safely mapped to the nearest point of the screen.
// 	  If the screen is rotated, then the min and max will be modified elsewhere
// 	  so that this always corresponds to screen pixel coordinates.
// 	  /M.SANDERSCROCK added constrain
// */
// /******************************************************************************/
// bool XGLCD::touchReadPixel(uint32_t *x, uint32_t *y)
// {
// 	uint32_t tx,ty;
//     bool touched;
    
// 	touched = readTouchADC(&tx,&ty);
// 	*x = constrain(map(tx,_tsAdcMinX,_tsAdcMaxX,0,_width-1),0,_width-1);
// 	*y = constrain(map(ty,_tsAdcMinY,_tsAdcMaxY,0,_height-1),0,_height-1);
//     return touched;
// }

// /*
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +							SPI & LOW LEVEL STUFF						       +
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// */

// /******************************************************************************/
// /*! PRIVATE
// 		Write in a register
// 		Parameters:
// 		reg: the register
// 		val: the data
// */
// /******************************************************************************/
void XGLCD::_writeRegister(const uint8_t reg, uint8_t val)
{
    _spiCSLow;                                                                  //writeCommand(reg);
    _spiwrite16(RA8875_CMDWRITE+reg);
    _spiCSHigh;
    _spiCSHigh;
    _spiCSLow;
    _spiwrite16(val);                                                           // RA8875_DATAWRITE = 0x00 (so skip the addition to speed up things)
    _spiCSHigh;
}

// /******************************************************************************/
// /*! PRIVATE
// 		Returns the value inside register
// 		Parameters:
// 		reg: the register
// */
// /******************************************************************************/
uint8_t XGLCD::_readRegister(const uint8_t reg)
{
	writeCommand(reg);
	return _readData(false);
}

// /******************************************************************************/
// /*!
// 		Write data
// 		Parameters:
// 		d: the data
// */
// /******************************************************************************/
void XGLCD::_writeData(uint8_t data)
{
    _spiCSLow;
    //_spiwrite(RA8875_DATAWRITE);_spiwrite(data);
    _spiwrite16(data);                                                          // RA8875_DATAWRITE = 0x00 (so skip the addition to speed up things)
    _spiCSHigh;
}

// /******************************************************************************/
// /*! 
// 		Write 16 bit data
// 		Parameters:
// 		d: the data (16 bit)
// */
// /******************************************************************************/
void  XGLCD::_writeData16(uint16_t data)
{
    _spiCSLow;
#ifdef _spiwrite24
    _spiwrite24(RA8875_DATAWRITE, data);
#else
	_spiwrite(RA8875_DATAWRITE);
	_spiwrite16(data);
#endif
    _spiCSHigh;
}

// /******************************************************************************/
// /*!	PRIVATE

// */
// /******************************************************************************/
uint8_t XGLCD::_readData(bool stat)
{
    uint8_t x;
    
    _spisetSpeed(SPI_SPEED_READ);
    _spiCSLow;
#ifdef _spixread
    if (stat == true) {_spixread(RA8875_CMDREAD, x);} else {_spixread(RA8875_DATAREAD, x);}
#else
    if (stat == true) {_spiwrite(RA8875_CMDREAD);} else {_spiwrite(RA8875_DATAREAD);}
    delayMicroseconds(50);                                                      // Stabilize time, else first bit is read wrong
    _spiread(x);
#endif
    _spiCSHigh;
    _spisetSpeed(SPI_SPEED_WRITE);
    return x;

}

// /******************************************************************************/
// /*!

// */
// /******************************************************************************/
uint8_t	XGLCD::readStatus(void)
{
	return _readData(true);
}

// /******************************************************************************/
// /*! PRIVATE
// 		Write a command
// 		Parameters:
// 		d: the command
// */
// /******************************************************************************/
void XGLCD::writeCommand(const uint8_t d)
{
    _spiCSLow;
	//_spiwrite(RA8875_CMDWRITE);_spiwrite(d);
    _spiwrite16(RA8875_CMDWRITE+d);
    _spiCSHigh;
}

