uint16_t impactScalar = 0;
AccelerationReading startAccel = {0, 0, 0};
uint16_t safetyPoint = 165; // predefined "danger" impact, presumable determined from research
AccelerationReading endAccel = {0, 0, 0};

void setup() {
    // initialize serial communication at 57600 bits per second:
    Serial.begin(57600);
    startAccel = Bean.getAcceleration();
    endAccel = {0, 0, 0};
    Bean.setLed(0,255,0);  // green
}

// the loop routine runs over and over again forever:
void loop() {
    endAccel = Bean.getAcceleration();
    
    // end - start = impact
    impactScalar = GetImpact(startAccel, endAccel);
    if(impactScalar > safetyPoint) {
      Serial.print("Resulting accel: ");
      Serial.println(impactScalar);
      Bean.setLed(255,0,0);  // red
      Bean.sleep(1000);
      Bean.setLed(0,255,0);  // return to green
    }

    Serial.println(impactScalar);

    startAccel = endAccel;
}

uint16_t GetImpact(AccelerationReading accStart, AccelerationReading accEnd) {
  // impact = end - start
  return sqrt(sq(accEnd.xAxis-accStart.xAxis) + 
              sq(accEnd.yAxis-accStart.yAxis) + 
              sq(accEnd.zAxis-accStart.zAxis));
}
