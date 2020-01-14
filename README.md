# XGLCD-LVGL Library for RA8875

This library is designed to use a RA8875 LCD module with <a href="http://www.littlevgl.com" target="_blank"> LittleVGL </a>, especially using an ESP32 as MCU (but it will also works with MKR family - Pycom family - Particle family - Adafruit Feather - plain old Arduino).

It's not a full RA8875 library but a light weight version dedicated to LittleVGL (You need to install <a href="https://github.com/littlevgl/" target="_blank"> LittleVGL </a> to use this library).

XGLCD-LVGL can drive 5.0", 7.0" and 9.0" 800x480 TFT LCD with capacitive touchscreen controlled by FT5206 (The code for resistive touchscreen directly managed by the RA8875 is commented but available), so you need the <a href="https://github.com/sumotoy/FT5206" target="_blank">FT5206 library from Sumotoy</a>.

XGLCD-LVGL is based on <a href="https://github.com/xgraph/XGLCD" target="_blank">XGLCD from XGraph</a> witch is based on <a href="https://github.com/sumotoy/RA8875" target="_blank">RA8875 from Sumotoy</a>.

Power supply consideration : RA8875 usually control large displays witch needs lot of current, USB will not provide enough energy to feed an ESP32, the LCD and the touch panel, please use an external power supply to avoid weird behaviors.

Licence GNU GPL v3.0.

