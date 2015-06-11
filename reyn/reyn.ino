#include <avr/sleep.h> // For sleep functionality
#include <Wire.h> // For accelerometer/gyro

// hc-sr04 ultrasonic 4-pin sensor
long temperature = 0; // Set temperature variable
boolean debug = true; // For serial communication set debug to true, for faster code set debug to false
long duration, cm; // Set time and cm for distance sensing
int trig = 10, 
    echo = 9; // Set pins for trig and echo

// accel/gyro
const int MPU = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
const int16_t gyroTolerance = 5;
const int16_t accelTolerance = 5;

bool notMeasured = true;
long lastStable = 0;
long startStable = 0;
const long stablizationTime = 2000;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}

void loop() {
  if(notMeasured) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
    AcX = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    temperature = Tmp / 340.00 + 36.53;  //equation for temperature in degrees C from datasheet
    
    if(GyX < gyroTolerance && GyY < gyroTolerance) {
      // and not accelerating
      while(AcX < accelTolerance && AcY < accelTolerance && AcZ < accelTolerance) {
        // wait for water to settle
        if(!startStable) {
          startStable = millis();
          lastStable = 0;
        } else {
          lastStable = millis();
          if((lastStable - startStable) > stablizationTime) {

            // take reading
            if (debug) {
              Serial.println("Temp: " + temperature);
            }
            // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
            pinMode(trig, OUTPUT);
            digitalWrite(trig, LOW);
            delayMicroseconds(2);
            digitalWrite(trig, HIGH);
            delayMicroseconds(5);
            digitalWrite(trig, LOW);
            duration = pulseIn(echo, HIGH);
            cm = microsecondsToCentimeters(duration, temperature);
            if (debug) {
              Serial.println("Distance: " + cm + "cm");
            }

            Serial.println("Current bottle content: " + reading + " cups");
            notMeasured = false;
            // startStable = 0;
            // lastStable = 0;
            break;
          }
        }
      }
    }
  } else { // Put to sleep, the level was recorded
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
    sleep_enable();          // enables the sleep bit in the mcucr register
    sleep_mode();            // here the device is actually put to sleep!!
  }
}


float getVoltage(int pin) {
  return (analogRead(pin) * .004882814); //Converting from 0 to 1024 to 0 to 5v 
}

long microsecondsToCentimeters(long microseconds, long temp) {
  return (microseconds * (331.3 + 0.606 * temp)) / 2; //Multiplying the speed of sound through a certain temperature of air by the 
                                                      //length of time it takes to reach the object and back, divided by two
}
