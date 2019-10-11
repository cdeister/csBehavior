#include <SPI.h>
#include <Wire.h>
#include <Trinamic_TMC2130.h>
#include <AccelStepper.h>


// stepper pins
#define stpPin_cs 10
#define stpPin_dir 4
#define stpPin_enable 5
#define stpPin_step 3

#define stpPin_rwdTrig 6
#define rotateState 2

// make stepper(s) and screen
Trinamic_TMC2130 spiStepper(stpPin_cs);


// we also make an accel stepper stepper object.
// we use the spi trinamic library configure the driver
// we use accelStepper to move it
AccelStepper accelStepper(AccelStepper::DRIVER, stpPin_step, stpPin_dir);


// **********************************
// **** Serial Variable Guide *******
// **********************************
// r/0: reward state (bool; 1 to trigger a reward)
// s/1: motor speed
// a/2: motor acceleration
// p/3: volume per microstep in pL
// v/4: reward volume in nl
// d/5: direction (0 for forward; 1 for backward)
// m/6: number of intrinsic steps per revolution the motor has (usually 200 or 400)
// u/7: microstepping resolution (powers of 2 til 256);


char knownHeaders[] = {'r', 's', 'a', 'p', 'v', 'd', 'm', 'u'};
int knownValues[] = {0, 25600, 10000, 14311, 1600, 0, 200, 256};
int lastValues[] = {0, 25600, 10000, 14311, 1600, 0, 200, 256};
bool varChanged[] = {0, 0, 0, 0, 0, 0, 0, 0};
int knownCount = 8;








void setup(void) {

  Serial.begin (115200);  // USB monitor
  delay(100);
  Serial2.begin(115200);  // HW UART1
  delay(100);
  initializeStepper();
  resetStepper();

  digitalWrite(stpPin_enable, HIGH);
  delay(10);

}



void loop() {


  // b) Look for usb serial changes.
  int nD = flagReceive(knownHeaders, knownValues);
  checkForVarChange();

  // c) Give a reward bolus if asked.
  bool giveReward = digitalRead(stpPin_rwdTrig);
  if ((knownValues[0] == 1) || (giveReward == 1)) {
    dispReward(knownValues[4], knownValues[5], knownValues[7], knownValues[1], knownValues[2]);
    knownValues[0] = 0;
  }

  
}






void stepSPI(int stepDelay) {
  digitalWrite(stpPin_enable, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(stpPin_step, LOW);
  delayMicroseconds(stepDelay);
}


void resetStepper() {
  pinMode(stpPin_enable, OUTPUT);
  pinMode(stpPin_dir, OUTPUT);
  pinMode(stpPin_step, OUTPUT);
  digitalWrite(stpPin_enable, HIGH); // disable driver
  digitalWrite(stpPin_dir, LOW); // chose direction
  digitalWrite(stpPin_step, LOW); // no step yet
}

void initializeStepper() {
  spiStepper.init();
  spiStepper.set_mres(knownValues[7]); // ({1,2,4,8,16,32,64,128,256}) number of microsteps
  spiStepper.set_IHOLD_IRUN(31, 31, 5); // ([0-31],[0-31],[0-5]) sets all currents to maximum
  spiStepper.set_I_scale_analog(1); // ({0,1}) 0: I_REF internal, 1: sets I_REF to AIN
  spiStepper.set_tbl(1); // ([0-3]) set comparator blank time to 16, 24, 36 or 54 clocks, 1 or 2 is recommended
  spiStepper.set_toff(8); // ([0-15]) 0: driver disable, 1: use only with TBL>2, 2-15: off time setting during slow decay phase
}


void dispReward(int numMuSteps, int cDir, int muRes, int curSpeed, int curAccel) {
  int dirMult;
  if (cDir == 0) {
    dirMult = 1;
  }
  else if (cDir == 1) {
    dirMult = -1;
  }
  accelStepper.setMaxSpeed(muRes * curSpeed);
  accelStepper.setSpeed(muRes * curSpeed);
  accelStepper.setAcceleration(muRes * curAccel);
  digitalWrite(stpPin_enable, LOW);
  long curPos = accelStepper.currentPosition();
  accelStepper.runToNewPosition(curPos + (dirMult * numMuSteps));
  digitalWrite(stpPin_enable, HIGH);
}





int flagReceive(char varAr[], int valAr[]) {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char endMarker = '>';
  char feedbackMarker = '<';
  char rc;
  int nVal;
  const byte numChars = 32;
  char writeChar[numChars];
  int selectedVar = 0;
  int newData = 0;

  while (Serial.available() > 0 && newData == 0) {
    rc = Serial.read();
    if (recvInProgress == false) {
      for ( int i = 0; i < knownCount; i++) {
        if (rc == varAr[i]) {
          selectedVar = i;
          recvInProgress = true;
        }
      }
    }

    else if (recvInProgress == true) {
      if (rc == endMarker ) {
        writeChar[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = 1;

        nVal = int(String(writeChar).toInt());
        valAr[selectedVar] = nVal;

      }
      else if (rc == feedbackMarker) {
        writeChar[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = 1;
        Serial.print("echo");
        Serial.print(',');
        Serial.print(varAr[selectedVar]);
        Serial.print(',');
        Serial.print(valAr[selectedVar]);
        Serial.print(',');
        Serial.println('~');
      }

      else if (rc != feedbackMarker || rc != endMarker) {
        writeChar[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
    }
  }
  return newData; // tells us if a valid variable arrived.
}






uint32_t estimateVolume(int stepVol, int uRes, int tMuSteps) {
  float actualSteps = tMuSteps / uRes;
  uint32_t volDispensed = actualSteps * stepVol * 1000000;
  return volDispensed; // in nL
}



void primeMode(int mult) {

}

void checkForVarChange() {
  for ( int i = 0; i < knownCount; i++) {
    if (knownValues[i] != lastValues[i]) {
      varChanged[i] = 1;
    }
    else {
      varChanged[i] = 0;
    }


    lastValues[i] = knownValues[i];
  }
}
