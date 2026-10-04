/* =====================================================================
   debug.h   调试打印通道
   ---------------------------------------------------------------------
   这里这样写，是为了不依赖 Arduino IDE 里“USB CDC On Boot”那个设置：
   C3 的 USB 串口有两种模式，编译开关不同会走不同的类。三种情况都覆盖，
   所以不管 IDE 里怎么设，DBG 都能正常打印。
   ===================================================================== */

#ifndef CB_DEBUG_H
#define CB_DEBUG_H

#include <Arduino.h>

#ifndef ARDUINO_USB_CDC_ON_BOOT
#define ARDUINO_USB_CDC_ON_BOOT 0
#endif

#if ARDUINO_USB_CDC_ON_BOOT
  #define DBG Serial                       // 走框架里的 Serial
#elif defined(ARDUINO_USB_MODE) && ARDUINO_USB_MODE
  #include "HWCDC.h"
  extern HWCDC DBG;                        // 定义在 debug.cpp
#else
  #include "USBCDC.h"
  extern USBCDC DBG;                       // 定义在 debug.cpp
#endif

#endif
