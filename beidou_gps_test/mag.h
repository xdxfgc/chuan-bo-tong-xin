#pragma once
/* ============================================================
 *  mag.h —— GY-282（HMC5983）三轴磁力计模块（对外接口）
 *  实现全在 mag.cpp：I2C 读写、连续采样、磁偏角、标定、JSON。
 *  这一层不打印、不碰网络，只负责"测准"和"告诉别人测到了什么"。
 * ============================================================ */
#include <Arduino.h>

/* ---------- 一、生命周期 ---------- */
void magBegin();     // setup() 里调一次：拉 I2C、读 flash 配置、认芯片、开连续测量
void magUpdate();    // loop() 里每次都调：到点就读一帧、算角度、推进标定

/* ---------- 二、状态 ---------- */
bool   magPresent();        // I2C 上有没有应答
bool   magIdOk();           // 芯片 ID 是不是 "H43"
String magChipIdText();     // 形如 "H43 (0x48 0x34 0x33)"

/* ---------- 三、读数 ---------- */
float magX();               // 已做硬磁/软磁修正，单位 µT
float magY();
float magZ();
float magFieldUT();         // 合矢量大小，µT（地面一般 25~65）
float magHeadingDeg();      // 平滑后的航向角，0~360°，0=北，顺时针为正
float magHeadingRawDeg();   // 未平滑的瞬时航向角，对照用

/* ---------- 出发点（参考方向）----------
 *  把站定时的朝向记成 0°，之后对外输出"相对出发点的航向"和"偏差"。
 *  偏差为正 = 偏右（顺时针），为负 = 偏左（逆时针）。 */
float magHeadingRel();      // 相对出发点的航向，0~360°
float magDeviation();       // 相对出发点的偏差，-180~+180
bool  magZeroSet();         // 出发点设过没有
float magZeroDeg();         // 出发点对应的绝对航向
bool  magZeroHere(String& msg);    // 把当前朝向设为出发点（0°）
bool  magZeroClear(String& msg);   // 清除出发点，回到相对磁北
int   magRawX();            // 原始 16 位计数，排查"是不是饱和了"时看
int   magRawY();
int   magRawZ();
uint32_t magSamples();      // 上电以来成功读到的帧数
uint32_t magErrors();       // 读取失败的次数
uint32_t magStaleMs();      // 距最近一次成功读到数据过了多久（毫秒）
String   magDiagText();     // I2C 错误码统计，排查"为什么读失败"用
String   magTimingText();   // 读一帧耗时 / loop 间隔统计，排查"为什么变慢"用
String   magPullupText();   // 上电时两条线的空闲电平，判断模块有没有上拉
String   magBusLevelText(); // 此刻两条线的电平，判断总线是不是被拉住了
String   magRiseText();     // 上拉体检：上升时间 + 当前时钟 + 自动降速次数
uint32_t magBusHz();        // 当前实际使用的 I2C 时钟
const char* magBusName();   // 用的是哪条 I2C 总线（Wire / Wire1）
float  magTemperatureC(bool& ok);   // 片上温度（默认关闭，见 config.h）
int    magGain();           // 当前生效的量程档 0~7
int    magRate();           // 当前生效的输出速率档 0~7
int    magAverage();        // 当前生效的平均次数（1/2/4/8）
float  magDeclination();    // 当前磁偏角
String magModeText();       // "连续" / "单次" / "空闲"

/* ---------- 四、标定与参数 ----------
 *  统一约定：返回值表示成功与否，要给人看的文字全放在 msg 里，
 *  由调用方（串口 / 网页 / API）决定打印到哪儿。 */
bool magCalAutoStart(uint32_t seconds, String& msg);   // 开始"转 8 字"标定
bool magCalStop(String& msg);                          // 提前结束并计算
bool magCalCancel(String& msg);
bool magCalSetOffsets(float ox, float oy, float oz, String& msg);
bool magCalSetScale(float sx, float sy, float sz, String& msg);
bool magCalReset(String& msg);                         // 清掉 flash 里的标定
bool magCalibrating();
bool magCalibrated();                                  // 是否已有可用标定值
bool magCalJustFinished();                             // 读一次就清标志
String magCalLastMessage();
uint32_t magCalRemainingSec();                         // 标定还剩几秒，0=没在标定

bool magSetDeclination(float deg, String& msg);        // 磁偏角
bool magSetGain(int g, String& msg);                   // 0~7
bool magSetRate(int r, String& msg);                   // 0~7
bool magSetAverage(int a, String& msg);                // 0~3
bool magSaveConfig(String& msg);                       // 手动存 flash
bool magSelfTest(String& msg);                         // 认芯片 + 读几帧看是否合理
String magScanI2C(String& msg);                        // 扫总线，返回找到的地址文本

/* ---------- 五、数据打包 ---------- */
String magJson();           // 串口、网页、API 共用同一份 JSON
