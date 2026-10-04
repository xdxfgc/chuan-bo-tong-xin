/* =====================================================================
   debug.h   调试打印通道
   ---------------------------------------------------------------------
   这里按「这块芯片实际有什么 USB 口」来选，不去猜 IDE 里的设置：

     1) 框架已经接管 USB（CDC On Boot = Enabled）→ 直接用框架的 Serial
     2) C3 / C6 / H2 / S3 这类有 USB Serial-JTAG  → 自己开一个 HWCDC
     3) S2 / S3 走 TinyUSB 且开着 CDC            → 自己开一个 USBCDC
     4) 以上都没有（普通 ESP32）                  → 退回 UART0，也就是 Serial

   以前的写法是拿 ARDUINO_USB_MODE 判断的。那个宏只有 C3 这类板子才定义，
   一旦开发板选成了普通 ESP32，就会走到 USBCDC 那条分支去 —— 而普通 ESP32
   根本没有 USBCDC 这个类，于是整个工程报 “'USBCDC' does not name a type”。
   现在第 4 条兜底，选错板子顶多打印不对，不会再编不过。
   ===================================================================== */

#ifndef CB_DEBUG_H
#define CB_DEBUG_H

#include <Arduino.h>
#include "soc/soc_caps.h"     // SOC_USB_SERIAL_JTAG_SUPPORTED
#include "sdkconfig.h"        // CONFIG_TINYUSB_CDC_ENABLED

// CB_DBG_OWN: 0 = 直接用框架的 Serial，1 = 本工程自己定义 HWCDC，2 = 自己定义 USBCDC
#define CB_DBG_OWN_NONE   0
#define CB_DBG_OWN_HWCDC  1
#define CB_DBG_OWN_USBCDC 2

#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  #define DBG Serial
  #define CB_DBG_OWN CB_DBG_OWN_NONE

#elif defined(SOC_USB_SERIAL_JTAG_SUPPORTED) && SOC_USB_SERIAL_JTAG_SUPPORTED
  #include "HWCDC.h"
  extern HWCDC DBG;                       // 定义在 debug.cpp
  #define CB_DBG_OWN CB_DBG_OWN_HWCDC

#elif defined(CONFIG_TINYUSB_CDC_ENABLED) && CONFIG_TINYUSB_CDC_ENABLED
  #include "USBCDC.h"
  extern USBCDC DBG;                      // 定义在 debug.cpp
  #define CB_DBG_OWN CB_DBG_OWN_USBCDC

#else
  #define DBG Serial                          // 普通 ESP32：调试口就是 UART0
  #define CB_DBG_OWN CB_DBG_OWN_NONE
#endif

#endif
