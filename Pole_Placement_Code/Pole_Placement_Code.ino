#include <Wire.h>
#include "MPU6050.h"
#include <Servo.h>
#include <math.h>

double x1 = 0.0, x2 = 0.0, x3 = 0.0, x4 = 0.0;
double K[4] = {0.1, 0.2, 0.5, 0.05};
Servo ESC1, ESC2;
MPU6050 mpu;
int16_t ax, ay, az;

const double accel_scale = 8192.0;
const double deadZone = 5.0;

const int filterSize = 10;
double angleHistory[filterSize] = {0.0};
int filterIndex = 0;
double filteredAngle = 0.0;

unsigned long lastSerialUpdate = 0;
const unsigned long serialInterval = 200;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    mpu.initialize();
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    ESC1.attach(9, 1000, 2000);
    ESC2.attach(10, 1000, 2000);
    ESC1.writeMicroseconds(1100);
    ESC2.writeMicroseconds(1100);
    delay(1000);
    Serial.println("Setup complete with ±4g range.");
}

void loop() {
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 disconnected!");
        return;
    }
    if (!readAccelOnly(x1, x2, x3, x4)) {
        return;
    }
    updateMovingAverage(x3);
    applyAngleBasedControl(filteredAngle);
    if (millis() - lastSerialUpdate >= serialInterval) {
        lastSerialUpdate = millis();
        Serial.print("Angle: ");
        Serial.print(filteredAngle * (180.0 / PI), 1);
        Serial.print(" U: ");
        Serial.println("N/A");
    }
}

bool readAccelOnly(double &x1, double &x2, double &x3, double &x4) {
    mpu.getAcceleration(&ax, &ay, &az);
    if (ay > 8192 || ay < -8192) {
        return false;
    }
    double ay_g = ay / accel_scale;
    x3 = asin(ay_g);
    x1 = 0.0;
    x2 = 0.0;
    x4 = 0.0;
    return true;
}

void updateMovingAverage(double newAngle) {
    angleHistory[filterIndex] = newAngle;
    filterIndex = (filterIndex + 1) % filterSize;
    double sum = 0.0;
    for (int i = 0; i < filterSize; i++) {
        sum += angleHistory[i];
    }
    filteredAngle = sum / filterSize;
}

void applyAngleBasedControl(double angle) {
    double pendulumAngleDeg = angle * (180.0 / PI);
    if (fabs(pendulumAngleDeg) <= deadZone) {
        ESC1.writeMicroseconds(1001);
        ESC2.writeMicroseconds(1001);
        return;
    }
    int primaryPwmSignal, opposingPwmSignal;
    if (fabs(pendulumAngleDeg) <= 15.0) {
        primaryPwmSignal = constrain(map(fabs(pendulumAngleDeg), 5, 10, 1450, 1700), 1450, 1900);
    } else if (fabs(pendulumAngleDeg) <= 30.0) {
        primaryPwmSignal = constrain(map(fabs(pendulumAngleDeg), 10, 30, 1900, 1980), 1900, 1980);
    } else {
        primaryPwmSignal = constrain(map(fabs(pendulumAngleDeg), 30, 45, 1980, 1980), 1980, 1980);
    }
    opposingPwmSignal = constrain(primaryPwmSignal * 0.1, 1001, primaryPwmSignal - 100);
    if (pendulumAngleDeg > 0) {
        ESC1.writeMicroseconds(opposingPwmSignal);
        ESC2.writeMicroseconds(primaryPwmSignal);
    } else {
        ESC1.writeMicroseconds(primaryPwmSignal);
        ESC2.writeMicroseconds(opposingPwmSignal);
    }
}
