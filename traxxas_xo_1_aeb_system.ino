/**
 * Author: Ye Tian <ye.tian@makercollider.com>
 * Author: Zongyi Zhang <zongyi.zhang@makercollider.com>
 * Copyright (c) 2017 Maker Collider Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 */

// #define MEGA2560

#ifndef MEGA2560
#include <SoftwareSerial.h>
#endif

#define FULL_PERIOD 9925      // 控制信号周期
#define MIN_PERIOD 8240       // 最小高电平时间,代表最大油门
#define CENTER_PERIOD 8425    // 中位高电平时间,代表最低油门中位
#define MAX_PERIOD 8731       // 最大高电平时间,代表最大刹车力度
#define CENTER_OFFSET 25      // 油门中位判定范围

#ifndef MEGA2560
#define SENSOR_A_RX 12
#define SENSOR_A_TX 13
#define SENSOR_B_RX 7
#define SENSOR_B_TX 8
#endif

#ifndef MEGA2560
#define OUTPUT_SIGNAL 9      // 控制信号输出
#else
#define OUTPUT_SIGNAL 11
#endif
#define INPUT_SIGNAL 3        // 油门信号
#define SIGNAL_SWITCH 4       // 控制信号切换控制

#define AVOID_DISTANCE 700    // 避障距离

enum CarState {
  STOP,
  FORWARD,
  BACKWARD,
  UNCLEAR
};

CarState car_command = STOP;  // 控制信号状态

#ifndef MEGA2560
SoftwareSerial SensorA(SENSOR_A_RX, SENSOR_A_TX);
SoftwareSerial SensorB(SENSOR_B_RX, SENSOR_B_TX);
#endif

uint8_t data[4] = {0}, buffer_length = 0;  // 串口数据,传感器数据长度

union SensorData{
  uint16_t distance;
  uint8_t data[2];
};

SensorData sensor_data1, sensor_data2; // 传感器信息

unsigned long start_time=0, duration=0; // 运行周期计算

void setup() {
  Serial.begin(9600);
  
  #ifndef MEGA2560
  SensorA.begin(9600);
  SensorB.begin(9600);
  SensorA.setTimeout(15);
  SensorB.setTimeout(15);
  #else
  Serial1.begin(9600);
  Serial2.begin(9600);
  #endif

  pinMode(OUTPUT_SIGNAL, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1);
  TCCR1B = _BV(WGM13) | _BV(CS11);
  ICR1 = 10000;
  OCR1A = CENTER_PERIOD;

  pinMode(INPUT_SIGNAL, INPUT);
  pinMode(SIGNAL_SWITCH, OUTPUT);
  digitalWrite(SIGNAL_SWITCH, LOW);
}

void loop() {
  start_time = micros();

  duration = FULL_PERIOD - pulseIn(INPUT_SIGNAL, LOW);
  
  if(duration < MIN_PERIOD | duration > MAX_PERIOD) {
    // 超出范围,不稳定信号
    car_command = UNCLEAR;
  } else if(duration < CENTER_PERIOD-CENTER_OFFSET) {
    // 前进信号
    car_command = FORWARD;
  } else if(duration > CENTER_PERIOD+CENTER_OFFSET) {
    // 后退信号
    car_command = BACKWARD;
  } else {
    // 停车/中位信号
    car_command = STOP;
  }

  #ifndef MEGA2560
  SensorA.listen();
  SensorA.write(0x55);
  buffer_length = SensorA.readBytes(data, 4);
  #else
  Serial1.write(0x55);
  buffer_length = Serial1.readBytes(data, 4);
  #endif
  
  if(buffer_length != 0) {
    sensor_data1.data[0] = data[2];
    sensor_data1.data[1] = data[1]; 
  } else {
    sensor_data1.distance = 0;
  }

  #ifndef MEGA2560
  SensorB.listen();
  SensorB.write(0x55);
  buffer_length = SensorB.readBytes(data, 4);
  #else
  Serial2.write(0x55);
  buffer_length = Serial2.readBytes(data, 4);
  #endif
  
  if(buffer_length != 0) {
    sensor_data2.data[0] = data[2];
    sensor_data2.data[1] = data[1]; 
  } else {
    sensor_data2.distance = 0;
  }

  if(car_command == STOP) {
    // 只有在控制器在中位时,才将信号切换回控制器
    digitalWrite(SIGNAL_SWITCH, LOW);
  }

  if(car_command == FORWARD) {
    if(sensor_data1.distance < AVOID_DISTANCE || sensor_data2.distance < AVOID_DISTANCE) {
      // 如果任意一个传感器距离值小于阈值, 则将信号切换至本机,并给予刹车信号
      OCR1A = MAX_PERIOD-30;
      digitalWrite(SIGNAL_SWITCH, HIGH);
    }
  }

  if(car_command != UNCLEAR) {
    Serial.print("Time: ");
    Serial.print(micros()-start_time);
    Serial.print(", \tA: ");
    Serial.print(sensor_data1.distance);
    Serial.print(", \tB: ");
    Serial.print(sensor_data2.distance);
    Serial.print(", \tlength: ");
    Serial.print(duration);
    Serial.print(", \tstate: ");
    Serial.println(car_command);
  }
}

