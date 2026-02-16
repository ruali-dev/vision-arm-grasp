#include "mycontrol.h"


/**
 * @brief 控制舵机平滑旋转到目标角度
 * 
 * 该函数通过比例、积分和微分控制（PID控制）实现舵机的平滑运动。
 * 误差越大，输出越大；误差越小，输出越小，从而实现柔和的运动效果。
 * 
 * @param servo 舵机对象，用于控制舵机角度
 * @param currentAngle 当前角度，作为输入和输出参数，函数会更新该值
 * @param targetAngle 目标角度，舵机最终需要达到的角度
 */
void ServoCtr(Servo &servo, int &currentAngle, int targetAngle) {
  int err = targetAngle - currentAngle;  // 计算误差，可能为正或负
  int output = 0;
  float integral = 0;  // 积分项
  float Ki = 0.005;    // 积分系数
  float Kd = 0.3;      // 微分系数
  float integralLimit = 100;  // 积分限幅值
  int lastErr = 0;     // 上一次的误差

  if (err != 0) {  // 如果有误差
    while (abs(err) >= 2) {  // 当误差绝对值大于等于2时
      // 动态比例控制
      if (abs(err) < 20) {  // 当误差小于20时，减小比例系数
        output = 0.05 * err;
      } else {
        output = 0.1 * err;  // 基础线性控制
        if (abs(err) > 60) {  // 如果误差较大，增加输出
          output = 0.5 * err;
        }
      }

      // 积分控制
      integral += err;  // 累积误差
      integral = constrain(integral, -integralLimit, integralLimit);  // 积分限幅
      output += Ki * integral;  // 加入积分项

      // 微分控制
      int derivative = err - lastErr;
      lastErr = err;
      output += Kd * derivative;  // 加入微分项

      // 动态输出范围限制
      if (abs(err) < 20) {  // 当误差小于20时，限制输出范围
        output = constrain(output, -1, 1);  // 进一步减小输出范围
      } else {
        output = constrain(output, -5, 5);  // 减小最大输出范围
      }

      // 死区控制
      if (abs(err) < 2) {  // 当误差小于2时，停止调整
        output = 0;
      }

      currentAngle += output;  // 更新当前角度
      currentAngle = constrain(currentAngle, 0, 180);  // 限制角度范围
      servo.write(currentAngle);  // 写入舵机
      err = targetAngle - currentAngle;  // 重新计算误差
      delay(30);  // 添加延迟，避免舵机响应过快

      // // 调试输出
      // Serial.print("Error: ");
      // Serial.print(err);
      // Serial.print(" | Integral: ");
      // Serial.print(integral);
      // Serial.print(" | Output: ");
      // Serial.println(output);
    }
    // 当误差小于2时，直接移动到目标角度
    currentAngle = targetAngle;
    servo.write(currentAngle);
  }
}