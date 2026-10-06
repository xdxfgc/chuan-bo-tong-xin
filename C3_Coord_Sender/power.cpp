/* =====================================================================
   power.cpp   深度休眠与水感唤醒实现
   ===================================================================== */

#include "power.h"
#include "water.h"
#include "radio.h"
#include "coord.h"

#include "esp_sleep.h"
#include "driver/gpio.h"

void powerBegin() {
  /* 醒来第一件事：把上次睡前"钉住"的引脚松开。
     不松开的话 GPIO_B 会一直是低电平，交替采样就做不了。 */
  gpio_hold_dis((gpio_num_t)WATER_PIN_B);
  gpio_deep_sleep_hold_dis();

#if GPS_PWR_PIN >= 0
  pinMode(GPS_PWR_PIN, OUTPUT);
  digitalWrite(GPS_PWR_PIN, LOW);       // 先断电，等确认要用了再上
#endif
}

bool powerWokeByWater() {
  return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO;
}

void powerGpsOn() {
#if GPS_PWR_PIN >= 0
  digitalWrite(GPS_PWR_PIN, HIGH);
  DBG.println("[电源] 北斗已上电");
#else
  /* 没接开关：北斗一直通着电，这里什么都不做 */
#endif
}

void powerGpsOff() {
#if GPS_PWR_PIN >= 0
  digitalWrite(GPS_PWR_PIN, LOW);
#endif
}

void powerGoSleep() {
  DBG.println("[电源] 准备进入深度休眠…");

  /* 1) 北斗：先发待机指令，再断电（有开关的话）。
        没开关也发待机能省一点。 */
  coordSleep();
  powerGpsOff();

  /* 2) 射频进睡眠（前提是已经初始化过） */
  if (radioIsReady()) radioSleep();

  /* 3) 姿态传感器不用管 —— 下一句 esp_deep_sleep_start() 会让它的供电
        跟着整块板子一起停（如果它是从 3V3 取的）。 */

  /* 4) 水感两个脚恢复高阻，然后把 B 脚钉低、A 脚配成低电平唤醒 */
  pinMode(WATER_PIN_A, INPUT);
  pinMode(WATER_PIN_B, INPUT);

  pinMode(WATER_PIN_B, OUTPUT);
  digitalWrite(WATER_PIN_B, LOW);              // 相当于把 B 片接到 GND
  gpio_hold_en((gpio_num_t)WATER_PIN_B);       // 深睡期间保持住
  gpio_deep_sleep_hold_en();

  /* A 脚：低电平唤醒。库会自动给这个脚配内部上拉，不用自己加电阻。
     （库文档原话："You don't need to worry about pull-up or pull-down
       resistors... It will automatically set them internally"） */
  esp_deep_sleep_enable_gpio_wakeup(1ULL << WATER_PIN_A,
                                    ESP_GPIO_WAKEUP_GPIO_LOW);

  DBG.printf("[电源] 睡了 —— 把 GPIO%d / GPIO%d 碰在一起就能叫醒\n",
             WATER_PIN_A, WATER_PIN_B);
  DBG.println("[电源]（休眠期间 USB 串口会断开，这是正常的）");
  DBG.flush();
  delay(100);                                  // 让上面的字打完

  esp_deep_sleep_start();
  /* 不会执行到这里 */
}
