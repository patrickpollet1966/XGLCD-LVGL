/*
 LittleVGL to XGLCD-LVGL library 
 Interface between LittleVGL C library and XGLCD-LVGL C++ library
 (compliant with LittleVGL V6)

 Based on LittleVGL to XGLCD library
 Copyright (c) 2019 DELCOMp bvba / UVee bvba
 2sd (a t) delcomp (d o t) com
 
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
 */

#include "lvgl.h"
#include "XGLCD.h"
#include <FT5206.h>

              
#define CTP_INT           15    // touch data ready for read from FT5206 touch controller
#define FT5206_RST        16

uint8_t registers[FT5206_REGISTERS];
uint16_t new_coordinates[5][2];
uint8_t current_touches = 0;


FT5206 cts = FT5206(CTP_INT);

XGLCD _tft = XGLCD();

static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  _tft.drawBitmap(area->x1, area->y1, area->x2, area->y2, (uint16_t *)color_p);                    // copy 'color_array' to the specifed coordinates
  lv_disp_flush_ready(disp);                                                           // Tell the flushing is ready
}


// bool my_tp_read_resistiveTS(lv_indev_drv_t * indev, lv_indev_data_t *data) {
  
//   bool tp_is_pressed;
//   uint32_t tx, ty;

//   tp_is_pressed = _tft.touchReadPixel(&tx, &ty);
    
//   data->point.x = (uint16_t)tx;
//   data->point.y = (uint16_t)ty;
//   data->state = tp_is_pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    
//   return false;                                                               // Return false because no more to be read
// }

bool my_tp_read(lv_indev_drv_t * indev, lv_indev_data_t *data) {
  if (cts.touched()){
     cts.getTSregisters(registers);
     current_touches = cts.getTScoordinates(new_coordinates, registers);
     if (current_touches < 1) {
      data->state = LV_INDEV_STATE_REL;
      return false;
     } 
     data->point.x = new_coordinates[0][0];
     data->point.y = new_coordinates[0][1];
     data->state = LV_INDEV_STATE_PR;
  }
  else
   data->state = LV_INDEV_STATE_REL;
  return false;
}  

void lv_xg_init() {

  lv_init();                                                                  // Init the LittleVGL library (not C++ library)
  
  _tft.begin();     
  delay(300);
#if defined(FT5206_RST)  
  pinMode(FT5206_RST, OUTPUT);
  digitalWrite(FT5206_RST, HIGH);
  delay(10);
  digitalWrite(FT5206_RST, LOW);
  delay(220);
  digitalWrite(FT5206_RST, HIGH);
  delay(300);
#endif  
  cts.begin();
  cts.setTouchLimit(1);

  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);                                                           // Start the LCD

  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 800;
  disp_drv.ver_res = 480;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);

  lv_indev_drv_t indev_drv;                                                   // Link the touchscreen to the LittleVGL library
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_tp_read;
  lv_indev_drv_register(&indev_drv);

}
