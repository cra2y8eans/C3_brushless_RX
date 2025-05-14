#pragma once
#include <Arduino.h>
#include <math.h>

class BatReading {
  private:
  int   _pin, _r1, _r2, _avg;
  float _maxVoltage, _minVoltage;

  public:
  float _voltage, _voltsPercentage;

  typedef struct {
    float voltage, voltsPercentage;
  } Bat;

  BatReading();
  ~BatReading();
  void init(int pin, int r1, int r2, float maxVoltage, float minVoltage);
  void readMilliVolts(int avg);
  Bat  read(int avg);
};

BatReading::BatReading() { };

BatReading::~BatReading() { };

void BatReading::init(int pin, int r1, int r2, float maxVoltage, float minVoltage) {
  _pin        = pin;
  _r1         = r1;
  _r2         = r2;
  _maxVoltage = maxVoltage;
  _minVoltage = minVoltage;
}

void BatReading::readMilliVolts(int avg) {
  _avg    = avg;
  int sum = 0;
  for (int count = 0; count < _avg; count++) {
    sum += analogReadMilliVolts(_pin); // 累加毫伏值
  }

  // 使用浮点运算确保精度
  float avgMilliVolts = sum / (float)_avg;              // 平均毫伏值
  float adcVoltage    = avgMilliVolts / 1000.0f;        // 转换为伏特
  _voltage            = adcVoltage * (_r1 + _r2) / _r2; // 计算总电压

  // 百分比计算
  if (_voltage >= _maxVoltage) {
    _voltsPercentage = 100.0f;
  } else if (_voltage <= _minVoltage) {
    _voltsPercentage = 0.0f;
  } else {
    _voltsPercentage = (_voltage - _minVoltage) / (_maxVoltage - _minVoltage) * 100.0f;
  }
}

// void BatReading::readMilliVolts(int avg) {
//   _avg = avg;
//   int val, sum = 0;
//   for (int count = 0; count < _avg; count++) {
//     sum += analogReadMilliVolts(_pin);
//   }
//   // 电压计算
//   val      = sum / _avg;
//   _voltage = (val / 1000) * ((_r1 + _r2) / _r2);
//   // 百分比计算
//   if (_voltage >= _maxVoltage) {
//     _voltsPercentage = 100.0;
//   } else if (_voltage <= _minVoltage) {
//     _voltsPercentage = 0.0;
//   } else {
//     _voltsPercentage = (_voltage - _minVoltage) / (_maxVoltage - _minVoltage) * 100.0;
//   }
// }

/************************************************ 以下为使用结构体存储数据 *********************************************************/

/*------------------------------------------------ main.cpp中使用方式如下：---------------------------------------------

// 使用BatReading作用域内的结构体Bat创建一个叫batStatus的结构体实例，然后将battery.read函数返回的结构体复制给这个实例。
  BatReading::Bat batStatus = battery.read(avg);

  Serial.printf("struct'voltage: %f,struct'voltsPercentage: %f\n", batStatus.voltage, batStatus.voltsPercentage);
  float a = batStatus.voltage, b = batStatus.voltsPercentage;
  Serial.printf("电压: %f,电量: %f\n", a, b);

------------------------------------------------------------------------------------------------------------*/

// 注意结构体的作用域。使用 BatReading::Bat 来明确指定 Bat 的作用域。
// 而不能像声明其他函数那样 void BatReading::init(int pin, int resolution)，直接用Bat BatReading::read(int avg)来声明函数 read(int avg) 的类型。

BatReading::Bat BatReading::read(int avg) {
  // 取均值
  _avg = avg;
  Bat B;
  int sum = 0;
  for (int count = 0; count < _avg; count++) {
    sum += analogReadMilliVolts(_pin); // 累加毫伏值
  }

  // 强制转换为浮点运算确保精度
  float avgMilliVolts = sum / (float)_avg;       // 平均毫伏值
  float adcVoltage    = avgMilliVolts / 1000.0f; // 转换为伏特
  B.voltage           = adcVoltage * (_r1 + _r2) / _r2; // 计算总电压

  // 百分比计算
  if (B.voltage >= _maxVoltage) {
    B.voltsPercentage = 100.0;
  } else if (B.voltage <= _minVoltage) {
    B.voltsPercentage = 0.0;
  } else {
    B.voltsPercentage = (B.voltage - _minVoltage) / (_maxVoltage - _minVoltage) * 100.0;
  }
  return B;
}

// BatReading::Bat BatReading::read(int avg) {
//   // 取均值
//   _avg = avg;
//   Bat B;
//   int adcValue, sum = 0;
//   for (int count = 0; count < _avg; count++) {
//     sum += analogRead(_pin);
//   }
//   // 电压计算
//   adcValue    = sum / _avg;
//   B.voltage   = (adcValue / 1000) * ((_r1 + _r2) / _r2);
//   // 百分比计算
//   if (B.voltage >= _maxVoltage) {
//     B.voltsPercentage = 100.0;
//   } else if (B.voltage <= _minVoltage) {
//     B.voltsPercentage = 0.0;
//   } else {
//     B.voltsPercentage = (B.voltage - _minVoltage) / (_maxVoltage - _minVoltage) * 100.0;
//   }
//   return B;
// }

/*************************************************************************************************************/
