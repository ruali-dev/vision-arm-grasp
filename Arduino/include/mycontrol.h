#ifndef MYCONTROL_H
#define MYCONTROL_H

#include <Arduino.h>
#include <Servo.h>

void ServoCtr(Servo &servo, int &currentAngle, int targetAngle);

#endif