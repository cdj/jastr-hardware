#define numHistory  10; // number of most recent accel readings to keep

int currentHistory = 0; // current accel history entry
uint16_t impactScalar = 0;
uint16_t safetyPoint = 165; // predefined "danger" impact, presumable determined from research
AccelerationReading startAccel[numHistory]; // keep history of accel readings
AccelerationReading endAccel = {0, 0, 0};

void setup() {
    // initialize serial communication at 57600 bits per second:
    Serial.begin(57600);
    startAccel[0] = Bean.getAcceleration();
    for(int i = 1; i < numHistory; i++) {
      startAccel[i] = startAccel[0];
    }
    endAccel = {0, 0, 0};
    Bean.setLed(0, 255, 0);  // green
}

// the loop routine runs over and over again forever:
void loop() {
    endAccel = Bean.getAcceleration();
    
    for(int i = 0; i < numHistory; i++) {
      // end - start = impact
      impactScalar = GetImpact(startAccel[i], endAccel);
      if(impactScalar > safetyPoint) {
        Serial.println(";");
        Serial.print("Resulting accel: ");
        Serial.println(impactScalar);
        Bean.setLed(255, 0, 0);  // red
        Bean.sleep(1000);
        Bean.setLed(0, 255, 0);  // return to green

        // clear existing data to ensure to make sure we don't trigger the same impact
        endAccel = Bean.getAcceleration();
        for(int i = 0; i < numHistory; i++) {
          startAccel[i] = endAccel;
        }
        break;
      }
      else {
        Serial.print(impactScalar);
        Serial.print(", ");
        startAccel[currentHistory] = Bean.getAcceleration();
      }
    }

    // move to next entry in accel history
    currentHistory++;
    if(currentHistory >= numHistory) {
      currentHistory = 0;
    }
}

uint16_t GetImpact(AccelerationReading accStart, AccelerationReading accEnd) {
  // impact = end - start
  return sqrt(sq(accEnd.xAxis-accStart.xAxis) + 
              sq(accEnd.yAxis-accStart.yAxis) + 
              sq(accEnd.zAxis-accStart.zAxis));
}
