/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error "Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif

#ifndef BOARD_NAME
  #define BOARD_NAME "RAMPS 1.4"
#endif

#define IS_RAMPS_EFB

#define LARGE_FLASH true

#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2 

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          42
#define Y_MAX_PIN         -1

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          43

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  32
#endif
  
#define Y2_STEP_PIN        36
#define Y2_DIR_PIN         34
#define Y2_ENABLE_PIN      30

#define Z2_STEP_PIN        36
#define Z2_DIR_PIN         34
#define Z2_ENABLE_PIN      30

#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30


#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13

#define FAN_PIN            9
#define FAN2_PIN          -1
#define V5_COOLING_PIN     44
#define FIL_RUNOUT_PIN     33

#define CONTROLLER_FAN_PIN   7


#define POWER_LOSS_MONITOR_PIN  79
#define POWER_LOSS_CONTROL_PIN  58
#define POWER_LOSS_PIN           6

#define PS_ON_PIN         -1

    
#define BEEPER_PIN        31
#define SD_DETECT_PIN     49

#if defined(REPRAP_DISCOUNT_SMART_CONTROLLER) || defined(G3D_PANEL)
  #define KILL_PIN           41
#else
  #define KILL_PIN           -1
#endif


#define HEATER_0_PIN       10
#define HEATER_1_PIN      -1
#define HEATER_2_PIN      -1

#define TEMP_0_PIN         13
#define TEMP_1_PIN         14
 
#define TEMP_2_PIN         -1
#define HEATER_BED_PIN     45
#define TEMP_BED_PIN       14

#ifdef NUM_SERVOS
    #define SERVO0_PIN        -1
  #if NUM_SERVOS > 1
    #define SERVO1_PIN         6
  #endif

  #if NUM_SERVOS > 2
    #define SERVO2_PIN         5
  #endif

  #if NUM_SERVOS > 3
    #define SERVO3_PIN         4
  #endif
#endif
  #ifdef ULTRA_LCD
    #ifdef NEWPANEL
      #define LCD_PINS_RS     16
      #define LCD_PINS_ENABLE 17
      #define LCD_PINS_D4     23
      #define LCD_PINS_D5     25
      #define LCD_PINS_D6     27
      #define LCD_PINS_D7     29

      #ifdef REPRAP_DISCOUNT_SMART_CONTROLLER
        #define BEEPER_PIN    31
        #define BTN_EN1       -1
        #define BTN_EN2       -1
        #define BTN_ENC       -1

        #define SD_DETECT_PIN 49
       
   #elif defined(FULL_GRAPHIC_SMALL_PANEL)
   #define BEEPER_PIN   37
     // Pins for DOGM SPI LCD Support
     #define DOGLCD_A0    23
     #define DOGLCD_CS    27
     #define LCD_PIN_BL   25
     
     #define KILL_PIN   41
     //The encoder and click button
     #define BTN_EN1    -1
     #define BTN_EN2    -1
     #define BTN_ENC    35
     //not connected to a pin
     #define SD_DETECT_PIN 49
       #elif defined(MULTIPANEL)
         #define DOGLCD_A0    17
         #define DOGLCD_CS    16
         #define LCD_PIN_BL   23 
         #define SDSS   53
         #define KILL_PIN   64
         //not connected to a pin
         #define SD_DETECT_PIN    49
       #else
        //arduino pin which triggers an piezzo beeper
        #define BEEPER_PIN    31
      #endif
  #endif //ULTRA_LCD
#endif 
