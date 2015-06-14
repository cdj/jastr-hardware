#include <avr/sleep.h> // For sleep functionality
#include <Wire.h> // For accelerometer/gyro

// nRF8001 Bluetooth Low Energy Breakout from Adafruit
//  Code based on Adafruit Bluefruit Low Energy nRF8001 Print echo demo
#include <SPI.h>
#include <Adafruit_BLE_UART.h>

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 8     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

// hc-sr04 ultrasonic 4-pin sensor
// http://www.instructables.com/id/Improve-Ultrasonic-Range-Sensor-Accuracy/
long temperature = 0; // Set temperature variable
boolean debug = true; // For serial communication set debug to true, for faster code set debug to false
long duration, cm; // Set time and cm for distance sensing
int trig = 5, 
    echo = 4; // Set pins for trig and echo

// MPU-6050 Accelerometer + Gyro
const int MPU = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
const int16_t gyroTolerance = 5;
const int16_t accelTolerance = 5;

bool notMeasured = true;
long reading = 0;
long lastStable = 0;
long startStable = 0;
const long stablizationTime = 2000;
const long crossSectionalArea = 75; // (cm^2) Set this for actual bottle
const long cubicCmInCup = 236.588236; // 1 US cup = 236.588236 cubic centimeters

void setup() {
  enablePinInterupt(ADAFRUITBLE_RDY); 
  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Jastr Reyn Prototype"));
  BTLEserial.setDeviceName("REYN"); /* 7 characters max! */
  BTLEserial.begin();

  // initialize accel/gyro/temp module
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

/**************************************************************************/
/*! Constantly checks for new events on the nRF8001 */
/**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop() {
  if(notMeasured) {
    // Tell the nRF8001 to do whatever it should be working on.
    BTLEserial.pollACI();
    // Ask what is our current status
    aci_evt_opcode_t status = BTLEserial.getState();
    // If the status changed....
    if (status != laststatus) {
      // print it out!
      if (status == ACI_EVT_DEVICE_STARTED) {
          Serial.println(F("* BTLE Advertising started"));
      }
      if (status == ACI_EVT_CONNECTED) {
          Serial.println(F("* BTLE Connected!"));
      }
      if (status == ACI_EVT_DISCONNECTED) {
          Serial.println(F("* BTLE Disconnected or advertising timed out"));
      }
      // OK set the last status change to this one
      laststatus = status;
    }
    
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
    
    // make sure bottle is standing up straight
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
              BTLEserial.print("Temp: ");
              BTLEserial.print(temperature);
              BTLEserial.println(" C");
            }
            // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
            pinMode(trig, OUTPUT);
            pinMode(echo,INPUT);
            digitalWrite(trig, LOW);
            delayMicroseconds(2);
            digitalWrite(trig, HIGH);
            delayMicroseconds(5);
            digitalWrite(trig, LOW);
            duration = pulseIn(echo, HIGH);
            cm = microsecondsToCentimeters(duration, temperature);
            if (debug) {
              BTLEserial.print("Dist: ");
              BTLEserial.print(cm);
              BTLEserial.println("cm");
            }
            
            reading = cm * crossSectionalArea / cubicCmInCup; // (cm^2) Set this for actual bottle

            BTLEserial.print("Volume: ");
            BTLEserial.print(reading);
            BTLEserial.println(" cups");
            notMeasured = false;

            break;
          }
        }
      }
    }
  } else {
    // Put to sleep becuase the level was recorded
    BTLEserial.pollACI();
    aci_evt_opcode_t status = BTLEserial.getState();
    if (status == ACI_EVT_CONNECTED) {
        BTLEserial.println("Sleeping...");
    }
   
    // http://playground.arduino.cc/Learning/ArduinoSleepCode
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
    sleep_enable();                      // enables the sleep bit in the mcucr register
    sleep_mode();                        // here the device is actually put to sleep!!
  }
}

float getVoltage(int pin) {
  return (analogRead(pin) * .004882814); //Converting from 0 to 1024 to 0 to 5v 
}

// Multiplying the speed of sound through a certain temperature of air by the
// length of time it takes to reach the object and back, divided by two
// Distance_CM = ((Duration of high level)*(Sonic :340m/s))/2 
// = ((Duration of high level)*(Sonic :0.034 cm/us))/2 
// = ((Duration of high level)/(Sonic :29.4 cm/us))/2 
long microsecondsToCentimeters(long microseconds, long temp) {
  return (microseconds * ((331.3 + 0.606 * temp) / 10000)) / 2;
}

// Need to change interrupt pin for BLE connectivity (Pro Trinket has no pin 2)
// http://forums.adafruit.com/viewtopic.php?f=25&t=59392
void enablePinInterupt(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}
