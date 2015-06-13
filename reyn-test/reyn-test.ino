#include <Wire.h> // For accelerometer/gyro

// hc-sr04 ultrasonic 4-pin sensor
// http://www.instructables.com/id/Improve-Ultrasonic-Range-Sensor-Accuracy/
long temperature = 0; // Set temperature variable
boolean debug = true; // For serial communication set debug to true, for faster code set debug to false
long duration, cm; // Set time and cm for distance sensing
int trig = 6, 
    echo = 3; // Set pins for trig and echo

// MPU-6050 Accelerometer + Gyro
// http://playground.arduino.cc/Main/MPU-6050#short
const int MPU = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

void setup() {
  // initialize accel/gyro/temp module
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.begin(9600);
  Serial.println("Jastr Reyn Prototype Test");
}

void loop() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  temperature = Tmp / 340.00 + 36.53;  //equation for temperature in degrees C from datasheet
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(temperature);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);

  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  cm = microsecondsToCentimeters(duration, temperature);
  Serial.print("Distance: ");
  Serial.print(cm);
  Serial.println("cm");
  Serial.println("");

  delay(1000);
}

float getVoltage(int pin) {
  return (analogRead(pin) * .004882814); //Converting from 0 to 1024 to 0 to 5v 
}

long microsecondsToCentimeters(long microseconds, long temp) {
  return (microseconds * (331.3 + 0.606 * temp)) / 2; //Multiplying the speed of sound through a certain temperature of air by the 
                                                      //length of time it takes to reach the object and back, divided by two
}
