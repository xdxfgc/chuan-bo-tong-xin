/* =====================================================================
   debug.cpp   调试串口对象的定义（全工程只有这一处）
   ===================================================================== */

#include "debug.h"

#if !ARDUINO_USB_CDC_ON_BOOT
  #if defined(ARDUINO_USB_MODE) && ARDUINO_USB_MODE
    HWCDC DBG;
  #else
    USBCDC DBG;
  #endif
#endif
