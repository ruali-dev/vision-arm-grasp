#include <Arduino.h>
#include <Servo.h>
#include <mycontrol.h>
#include <ArduinoJson.h>

Servo servo1;  //底座，后面从下往上
Servo servo2;  
Servo servo3;  
Servo servo4;  
Servo servo5;  //A4关节
Servo servo6;  //夹爪

int s1Angle = 90;  // 默认角度
int s2Angle = 90;
int s3Angle = 90;
int s4Angle = 145;
int s5Angle = 90;
int s6Angle = 90;       

void setup(){
  Serial.begin(115200);
  servo1.attach(7,500,2500);  //90度朝前时，左边180方向，右边0方向
  servo2.attach(6,500,2500);  //180往前 90正上 0往后
  servo3.attach(5,500,2500);  //180往后 0往前
  servo4.attach(4,500,2500);  //145度使机械手臂夹爪旋转到中位， 属于是安装时的缺陷了，不过这个自由度不影响正常夹取
  servo5.attach(2,500,2500);  //90度竖直向上 180朝臂下 0朝上
  servo6.attach(3,500,2500);  //0闭合 180开启

  // 使用上次的角度初始化舵机
  servo1.write(s1Angle);
  servo2.write(s2Angle);
  servo3.write(s3Angle);
  servo4.write(s4Angle);
  servo5.write(s5Angle);
  servo6.write(s6Angle);
  delay(50);
}

void loop()
{

  // 检查串口是否有数据
  if (Serial.available() > 0)
  {
    // 读取串口数据
    String jsonData = Serial.readStringUntil('\n');
    Serial.println("Received: " + jsonData);  // 打印接收到的数据

    // 解析 JSON 数据
    JsonDocument doc;  // 200 是缓冲区大小
    DeserializationError error = deserializeJson(doc, jsonData);

    // 检查解析是否成功
    if (error)
    {
      Serial.print("JSON 解析失败: ");
      Serial.println(error.c_str());
      return;
    }

    s1Angle = doc["servo1"];
    s2Angle = doc["servo2"];
    s3Angle = doc["servo3"];
    s4Angle = doc["servo4"];
    s5Angle = doc["servo5"];
    s6Angle = doc["servo6"];
  }

  servo1.write(s1Angle);
  servo2.write(s2Angle);
  servo3.write(s3Angle);
  servo4.write(s4Angle);
  servo5.write(s5Angle);
  servo6.write(s6Angle);
  
}
