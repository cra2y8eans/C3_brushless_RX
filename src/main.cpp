/***********************************************************************************************************************************************************

ESP32_C3 68手抛机接收机(无刷电机版)

        此版本为配合通用遥控器使用的接收机版本。
        接收数据改为：左右摇杆两个轴四个方向的数据、3个钮子开关的数据，以及差速转向系数共8项。
        采用余量油门动态计算转向增量的方式控制电机差速转向
        将数据回传跟飞机操控两个函数分别使用freertos任务运行。
        将舵机最大角度范围调整到100度
        将襟翼操作逻辑改为右拨摇杆

************************************************************************************************************************************************************/

#include "batteryReading.hpp"
#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*------------------------------------------------- ESP NOW -------------------------------------------------*/

// 创建ESP NOW通讯实例
esp_now_peer_info_t peerInfo;

// uint8_t padAddress[] = { 0x08, 0xa6, 0xf7, 0x17, 0x6d, 0x84 }; // ESP32_薄
uint8_t padAddress[] = { 0x2c, 0xbc, 0xbb, 0x00, 0x52, 0xd4 }; // ESP32_厚

// 存储接收到的数据
struct Pad {
  int   button_flag[3]     = {}; // 0、发送开关      1、襟翼开关     2、差速开关
  int   joystick_values[4] = {}; // 0、油门          1、差速         2、副翼         3、升降舵
  float diffrential_coe;
};
Pad pad;

// 发回遥控器数据
struct Aircraft {
  float batteryValue[2] = {}; // 0、电压           1、电量
};
Aircraft aircraft;

// 连接成功标志位
bool esp_connected;

/*------------------------------------------------- 舵机 -------------------------------------------------*/

#define HERTZ 50              // 频率
#define SERVO_AILERON_L 2     // 左副翼引脚
#define SERVO_AILERON_R 8     // 右副翼引脚
#define SERVO_ELEVATOR 3      // 升降舵引脚
#define SERVO_FREQ_MIN 500    // 舵机最小频率
#define SERVO_FREQ_MAX 2500   // 舵机最大频率
#define SERVO_ANGLE_RANGE 120 // 舵机最大角度

int pitch_servo_angle, roll_servo_angle;

// ESP32Servo库
Servo Aileron_L;
Servo Aileron_R;
Servo Elevator;

/*------------------------------------------------- 电机 -------------------------------------------------*/

#define MOTOR_PIN_L 5       // 左电机引脚
#define MOTOR_PIN_R 6       // 右电机引脚
#define MOTOR_FREQ_MIN 1000 // 电机频率
#define MOTOR_FREQ_MAX 2000
#define MOTOR_PWM_MIN 205
#define MOTOR_PWM_MAX 410
#define MOTOR_CHANNEL_L 4
#define MOTOR_CHANNEL_R 5
#define MOTOR_FREQUENCY 50
#define MOTOR_RESOLUTION 12
#define JOYSTICK_ADC_OUT_MAX 255  // 遥控器摇杆输出ADC最大值
#define JOYSTICK_ADC_OUT_MIN -255 // 遥控器摇杆输出ADC最小值

/*------------------------------------------------- 滤波 -------------------------------------------------*/

#define LIMIT_FILTER 10   // 限幅滤波阈值，建议取值范围3~10，值越小，操控越需要柔和
#define AVERAGE_FILTER 50 // 均值滤波，N次取样平均，建议取值范围20~80
#define ADC_MIN 0

/*----------------------------------------------- 电量读取 -------------------------------------------------*/

#define BATTERY_PIN 0         // 电量读取引脚
#define BATTERY_MAX_VALUE 8.4 // 电池最大电量
#define BATTERY_MIN_VALUE 7   // 电池最小电量
#define BATTERY_INTERVAL 3000 // 电量检测间隔时间，单位毫秒
#define R1 10000
#define R2 2000
#define ADC_RESOLUTION 12

int ADC_MAX = pow(2, ADC_RESOLUTION);

BatReading battery;

/*----------------------------------------------- 自定义函数 -------------------------------------------------*/

// 数据发出去之后的回调函数
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    esp_connected = true;
  } else {
    esp_connected = false;
  }
}

// 收到消息后的回调
void OnDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  memcpy(&pad, data, sizeof(pad)); // 将收到的消息进行存储
}

// // 平滑滤波
// int filter(int value) {
//   buffer[index] = value;
//   index         = (index + 1) % 10;
//   int sum       = 0;
//   for (int i = 0; i < 10; i++)
//     sum += buffer[i];
//   return sum / 10;
// }

// 限幅滤波，防止尖端突变
int limit_filter(int pin) {
  static int last_val = analogRead(pin); // 静态变量，只初始化一次，全程序中保存在内存
  int        val      = analogRead(pin);
  if (abs(val - last_val) > LIMIT_FILTER) {
    val = last_val;
  }
  last_val = analogRead(pin);
  return val;
}

// 均值滤波，抑制噪声
int avg_filter(int pin) {
  int val, sum = 0;
  for (int count = 0; count < AVERAGE_FILTER; count++) {
    sum += analogRead(pin);
  }
  val = sum / AVERAGE_FILTER;
  return val;
}

// 限幅滤波+均值滤波
int limit_avg_filter(int pin) {
  int val, sum = 0;
  for (int count = 0; count < AVERAGE_FILTER; count++) {
    sum += limit_filter(pin);
  }
  val = sum / AVERAGE_FILTER;
  return val;
}

//  数据回传
void dataSendBack(void* pt) {
  // 电量读取初始化
  battery.init(BATTERY_PIN, R1, R2, BATTERY_MAX_VALUE, BATTERY_MIN_VALUE);
  while (1) {
    BatReading::Bat batStatus = battery.read(AVERAGE_FILTER);
    aircraft.batteryValue[0]  = batStatus.voltage;
    aircraft.batteryValue[1]  = batStatus.voltsPercentage;
    esp_now_send(padAddress, (uint8_t*)&aircraft, sizeof(aircraft));
    vTaskDelay(BATTERY_INTERVAL);
  }
}

// 串口输出
void SerialDataPrint() {
  Serial.printf("收到的油门：%d\n", pad.joystick_values[0]);
  Serial.printf("收到的差速：%d\n", pad.joystick_values[1]);
  // Serial.printf("收到的升降舵：%d\n", pad.joystick_values[2]);
  // Serial.println("----分割----");
  // Serial.printf("是否收到消息：%d\n", esp_connected);
  delay(1000);
}

// 操控
void airCraftControl(void* pt) {

  // 舵机定时器
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  // 设定舵机频率
  Aileron_L.setPeriodHertz(HERTZ);
  Aileron_R.setPeriodHertz(HERTZ);
  Elevator.setPeriodHertz(HERTZ);
  // 绑定引脚和最大、最小频率
  Aileron_L.attach(SERVO_AILERON_L, SERVO_FREQ_MIN, SERVO_FREQ_MAX);
  Aileron_R.attach(SERVO_AILERON_R, SERVO_FREQ_MIN, SERVO_FREQ_MAX);
  Elevator.attach(SERVO_ELEVATOR, SERVO_FREQ_MIN, SERVO_FREQ_MAX);

  // 无刷电机
  ledcSetup(MOTOR_CHANNEL_L, MOTOR_FREQUENCY, MOTOR_RESOLUTION); // 通道、频率、精度
  ledcSetup(MOTOR_CHANNEL_R, MOTOR_FREQUENCY, MOTOR_RESOLUTION); // 通道、频率、精度
  ledcAttachPin(MOTOR_PIN_L, MOTOR_CHANNEL_L);                   // 引脚号、通道
  ledcAttachPin(MOTOR_PIN_R, MOTOR_CHANNEL_R);                   // 引脚号、通道
  ledcWrite(MOTOR_PIN_L, 0);                                     // 引脚号、PWM值
  ledcWrite(MOTOR_PIN_R, 0);

  while (1) {
    if (esp_connected == true) {
      /*
      int   button_status[3]    = {}; // 0、自稳开关    1、襟翼开关     2、微调开关
      int   joystick_cur_val[4] = {}; // 0、油门        1、差速         2、副翼         3、升降舵
      float diffrential_coe;
      */

      // 将摇杆ADC换算成舵机角度
      int roll_servo_angle  = map(pad.joystick_values[2], JOYSTICK_ADC_OUT_MIN, JOYSTICK_ADC_OUT_MAX, ADC_MIN, SERVO_ANGLE_RANGE);
      int pitch_servo_angle = map(pad.joystick_values[3], JOYSTICK_ADC_OUT_MIN, JOYSTICK_ADC_OUT_MAX, ADC_MIN, (SERVO_ANGLE_RANGE - 20));

      // 计算得出左右差速摇杆拨动的ADC值
      int throttle_base   = pad.joystick_values[0];               // 基础油门ADC
      int throttle_remain = JOYSTICK_ADC_OUT_MAX - throttle_base; // 油门余量
      int diffrential     = pad.joystick_values[1];               // 差速油门ADC
      int diffrential_l   = (diffrential >= 0) ? diffrential : 0;
      int diffrential_r   = (diffrential <= 0) ? abs(diffrential) : 0;

      // 将ADC值换算成对应的无刷电机高电平周期占空比
      int pwm_l = throttle_base + map(diffrential_l, 0, 255, 0, throttle_remain);
      int pwm_r = throttle_base + map(diffrential_r, 0, 255, 0, throttle_remain);
      pwm_l     = map(pwm_l, JOYSTICK_ADC_OUT_MIN, JOYSTICK_ADC_OUT_MAX, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
      pwm_r     = map(pwm_r, JOYSTICK_ADC_OUT_MIN, JOYSTICK_ADC_OUT_MAX, MOTOR_PWM_MIN, MOTOR_PWM_MAX);

      if (pad.button_flag[2] == 0) {
        throttle_base = map(pad.joystick_values[0], JOYSTICK_ADC_OUT_MIN, JOYSTICK_ADC_OUT_MAX, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
        ledcWrite(MOTOR_CHANNEL_L, throttle_base);
        ledcWrite(MOTOR_CHANNEL_R, throttle_base);
      } else {
        ledcWrite(MOTOR_CHANNEL_L, pwm_r);
        ledcWrite(MOTOR_CHANNEL_R, pwm_l);
      }

      Elevator.write(pitch_servo_angle);

      // 襟翼判断
      if (pad.button_flag[1] == 1) {
        Aileron_L.write(SERVO_ANGLE_RANGE - roll_servo_angle);
        Aileron_R.write(roll_servo_angle);
      } else {
        Aileron_L.write(roll_servo_angle);
        Aileron_R.write(roll_servo_angle);
      }
    } else {
      // 电机停转、飞机盘旋
      ledcWrite(MOTOR_CHANNEL_L, 205);
      ledcWrite(MOTOR_CHANNEL_R, 205);
      Elevator.write(20);
      Aileron_L.write(20);
      Aileron_R.write(80);
    }
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(ADC_RESOLUTION);

  // wifi及ESP NOW初始化
  WiFi.mode(WIFI_STA);                  // 设置wifi为STA模式
  esp_now_init();                       // 初始化ESP NOW
  esp_now_register_send_cb(OnDataSent); // 注册发送成功的回调函数
  esp_now_register_recv_cb(OnDataRecv); // 注册接受数据后的回调函数
  // 注册通信频道
  memcpy(peerInfo.peer_addr, padAddress, 6); // 设置配对设备的MAC地址并储存，参数为拷贝地址、拷贝对象、数据长度
  peerInfo.channel = 1;                      // 设置通信频道
  esp_now_add_peer(&peerInfo);               // 添加通信对象

  // 创建freertos任务
  xTaskCreate(dataSendBack, "dataSendBack", 1024 * 2, NULL, 1, NULL);
  xTaskCreate(airCraftControl, "airCraftControl", 1024 * 4, NULL, 3, NULL);
}

void loop() {
}
