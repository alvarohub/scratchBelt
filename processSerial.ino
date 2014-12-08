// ATTENTION: when using bluetooth, and if attempting a connection in command mode, the module will ANSWER THINGS on the serial port. 
// We DON'T want to catch that as commands for the houlaled. 


void processSerial() {
  // read the incoming byte:
  char incomingByte = Serial.read();

  if (incomingByte=='&')  for (int i=0; i<NUM_BLOBS; i++) blobArray[i].randomizePositions();

  if (incomingByte=='$') interactionMode=false;
  if (incomingByte=='%') interactionMode=true;

  // test led matrix:
 // if (incomingByte=='t') testingLEDS=false;
 // if (incomingByte=='T') testingLEDS=true;

  // change topology:
//  if (incomingByte=='b')  for (int i=0; i<NUM_BLOBS; i++)  {
//    blobArray[i].topologyMode=BOUNDED; 
//    blobArray[i].randomizePositions();
//  }
//  if (incomingByte=='B')  for (int i=0; i<NUM_BLOBS; i++)  {
//    blobArray[i].topologyMode=CYCLIC;  
//    blobArray[i].randomizePositions();
//  }

  if (incomingByte=='(') {
    Serial.print("aX="); 
    Serial.print(ax);
    Serial.println(" m/s^2");
    Serial.print("aY="); 
    Serial.print(ay);
    Serial.println(" m/s^2");
    Serial.print("aZ="); 
    Serial.print(az);
    Serial.println(" m/s^2");
    Serial.print("Norm="); 
    Serial.print(normAcc);
    Serial.println(" m/s^2");
    Serial.print("inclination="); 
    Serial.print(inclination*180.0/PI); 
    Serial.print(" deg "); 
    Serial.print(" / phi="); 
    Serial.print(phi*180.0/PI);
    Serial.println(" deg");
    Serial.print("Omega="); 
    Serial.print(omega*180.0/PI);
    Serial.println(" deg/sec");
    // offsetTheta from potentiometer (to adjust offset horizontal rotation):
    Serial.print("offsetTheta ="); 
    Serial.print(offsetThetaDeg);
    Serial.println(" deg");
    // length set from potentiometer:
    Serial.print("length pendulum ="); 
    Serial.print(100*potLength); 
    Serial.println(" cm");
    Serial.println(" ");
    Serial.print("atheta="); 
    Serial.print(blobArray[0].atheta);
    Serial.println(" rad/sec^2");
    Serial.print("vtheta="); 
    Serial.print(blobArray[0].vtheta);
    Serial.println(" rad/sec");
  }

  // RAW values for tests and calibration: 
  if (incomingByte==')') {
    Serial.print("ax (raw) ="); 
    Serial.println(analogLocalValue[0][0]);
    Serial.print("ay (raw) ="); 
    Serial.println(analogLocalValue[1][0]);
    Serial.print("az (raw) ="); 
    Serial.println(analogLocalValue[2][0]);
    Serial.print("Gyro (raw) ="); 
    Serial.println(analogLocalValue[3][0]);
    // Potentiometer (to adjust offset horizontal rotation):
    Serial.print("Pot1 raw readout (offset theta) ="); 
    Serial.println(potThetaValue);
    // length set from potentiometer:
    Serial.print("Pot2 raw readout (length pendulum) ="); 
    Serial.println(potLengthValue);
    Serial.println(" ");
  }


  // For test of dynamics using the processing program "displayBelt": 
  if (incomingByte=='#') {
    if (sendingAngularSpeed) {
      Serial.print(blobArray[0].vtheta); 
      Serial.write(END_PACKET);
    } 
    else {
      for (int i=0; i<NUM_BLOBS; i++) {
        if (blobArray[i].topologyMode==CYCLIC) Serial.print( PIXEL_WIDTH*blobArray[i].blobX); 
        else Serial.print((PIXEL_WIDTH-1)*blobArray[i].blobX);
        Serial.print(","); 
        Serial.print( (PIXEL_HEIGHT-1)*blobArray[i].blobY);
        Serial.print(",");
      }
      // packet terminator:
      Serial.write(END_PACKET);
    }
  }

  if (incomingByte=='!') {
    continuousSending=true;
  }
  if (incomingByte=='"') {
    continuousSending=false;
  }
}


