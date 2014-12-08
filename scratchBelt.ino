/* 
 
 INERTIAL LIGHTS / HOULALED using LED STRIPS (adafruit) or "scratchBelt"
 Author: Alvaro Cassinelli
 

 
 VERSION: 
 14.4.2013: - this will use only one strip, width: 32, height:1 (in the future, we may put several strips)
            - the ledControl.h library (MAX7221) is used not for the blobs, but for counting the number of turns (using a 7 segment led display). 
            - 11.7.2013 added bluetooth. NOTE: The actual speed you set on the computer "serial port" corresponding to the bluetooth connection
              doesn’t change the physical speed at the RX/TX wires of the BlueSMiRF! For that we need to go into command mode and change some 
              of BlueSMiRF’s parameters: Open the terminal (or do it in the arduino program), enter “+++” and prss Return. It should respond with “OK”. 
              To verify, type “AT” and Return again, and it should again respond with “OK”. 
              Now type “ATSW20,236,0,0,1″. This changes the speed and other serial paraemters of the other ‘side’ of the BlueSMiRF (i.e. the RX/TX lines 
              plugged into the Arduino) to 57600 8N1. This needs to be done because by default, it comes up at something like 19200 bps.
            - NOTE: 1) if I connect the bluesmirf in the arduino talks directly to the integrated bluetooth in the computer, then we need to put it 
                       in "server" mode. The connection parameters will be handled using graphical interfaces in the computer;
                    2) if instead I connect both bluetooth "manually" (that is, one module is connected to a serial-to-usb converter, and the other directly to the Arduino)
                  then we need to call BTSetSERVER (wait for connection) or BTSetClient (attemps connection with the corresponding address of the server bluetooth).  
                    3) in order to be able to use both modes, the bluetooth module in the wearable device will be the SERVER, always, and will not
                  attemps a connection. 
 11.11.2014: connects with houlaGroove.maxHelp patch 
 */
 // ----> THE ADDRESS OF THE BLUETOOTH MODULE IN THE HOULALED is "0814080E1E2D"

// ==========================================================================================================================================================

// if seeking optimal speed, don't comment this line (averaging of sensor values won't be done)
#define USE_SERIAL // if not defined, then we may optimize the behaviour and speed

// if using potentiometers, do not comment the following line:
#define WITH_POTENTIOMETERS
#define CONTINUOUS_POT_READING

// if using a gyroscope, do not comment the following line:
#define WITH_GYRO

// ==========================================================================================================================================================
#include "classBlob.h"

#include "LedControl.h" // this is for controlling up to 64 LEDS using the MAX7221 (or even more by cascading them), with only 3 pins using SPI interface. 
// REM: the advantage of this library with respect to the "official" Matrix.h is that it provides naturally for cascaded devices. 
// I will use only ONE MAX7721, to control just TWO 7 segment leds.
// hardware SPI pins for the MAX7221:
#define LOADCSPIN 10
#define CLKPIN 11
#define DATAPIN 12
#define NBMATRICES 1 // only one MAX7221
LedControl ledSegment=LedControl(DATAPIN, CLKPIN, LOADCSPIN, NBMATRICES); 

#include "LPD8806.h"
#include "SPI.h"
#define dataPinStrip 9
#define clockPinStrip 8
// Set the first variable to the NUMBER of pixels. 32 = 32 pixels in a row
// The LED strips are 32 LEDs per meter but you can extend/cut the strip
LPD8806 strip = LPD8806(32, dataPinStrip, clockPinStrip);
// In the futre, we can create a martrix:
//LPD8806 matrixStrip[ROWS];
//... and instantiate each strip row with different pins

// MODE CHANGE SWITCH:
#define pinModeChange 2 // interrupts only on pins 2 and 3

#define BLOB_RADIUS 3 // blob "radius" in cm as measured on the belt
#define NUM_BLOBS 1
Blob blobArray[NUM_BLOBS];// call to constructor 
// Rem: in case of dynamic allocation: Blob *blobArray, we need to do then blobArray=new Blob[Nb], and then for each index on Nb: blobArray[i].init()

// The intermediate discrete display buffer (will map the position of the blobs to intensities, etc). 
#define PIXEL_WIDTH 32
#define PIXEL_HEIGHT 3
uint32_t beltMatrix[PIXEL_HEIGHT][PIXEL_WIDTH];

// buffer memory for the MAX72xx controller (normally 8x8):
#define MATRIX_WIDTH 8
#define MATRIX_HEIGHT 8
boolean matrixLed[MATRIX_HEIGHT][MATRIX_WIDTH][NBMATRICES]; // each 8x8 matrix is binary (on/off). Total brightness can be controlled though.
// REM: if one wants to use colors, we can use two interleaved matrices, or the sparkfun "backpack" matrix (with 8 predefined 'colors')
int cx, cy, indexMatrix; // coordinates of a led in this matrix (value: matrixLed[cy][cx][indexMatrix])

// Potentiometer readouts for adjusting parameters (no buffer averaging, but we could): 
#define potThetaPin 4 // potentiometer to adjust the theta offset
float potThetaValue;
#define potLengthPin 5 // potentiometer to adjust the length of the virtual pendulum
float potLengthValue;
float potLength; // this will be the length (in meters) of the virtual pendulum set by the potentiometer (if we want)

// Sensors that potentially use continuous averaging: 
#define NUM_SENSORS 4 // NOTE: first three sensors are the accelerometer axis X,Y,Z, and the last is the GYROSCOPE (axis Z). 
int analogInputPin[NUM_SENSORS] = {
  0,1,2,3};
int analogLocalValue[NUM_SENSORS][5];// values between 0 and 1024. [][0] is the current value, [1] is the max, [2] is the min, [3] is the "Zero" position, and [4] is the mean
#define MAX_AVGSAMPLES 10 // make it larger than one if we don't use wireless sensors (averaging is done in the wireless sensor side)
int numAvgSamples=3;// must be < MAX_AVGSAMPLES
int averagingBuffer[NUM_SENSORS][MAX_AVGSAMPLES];
int oldSum[NUM_SENSORS]; // needed because analogLocalValue contain only integers (we cannot retrieve the real sum by multiplying analogLocalValue[][4] by numAvgSamples)
int bufferIndex[NUM_SENSORS]; // circular buffer index for each sensor
float AREF_VOLTAGE=3.3; // should be 3.3V, but when I put 3.34Vcc, and for AREF_VOLTAGE=3.3, I measure 3.3 (printing on the serial port).
// This means that AREF_VOLTAGE should be changed to AREF_VOLTAGE=3.34V.


// This is a new mode for putting the module in "stand by" (for the display only):
#define DISPLAY_OFF 0
#define DISPLAY_ON  1
int displayMode=DISPLAY_ON;


// Conversion factors for sensors: 
float factorAccelerometer=9.81/122; // conversion to get values in m/s^2:
// (438 is raw measured norm when only gravity, and zero is 316, so when only gravity readout without offset, corresponding to g, this is: g/(438-316))
float factorGyro=0.05; // to get angular speed in rad/sec (this needs some experimenting...)

// this is for calibration of the gyroscope; we can comment this line after doing it once: 
float integratedGyro=0; 
float integratedTime=0;

// Accelerometer readout in cartesian and polar coordinates (in accelerometer referential frame). 
float ax, ay, az, ax_norm, ay_norm, az_norm, normAcc;
float inclination, phi, alpha; // alpha=atan2(az,ax)
float offsetThetaDeg; // (in degrees). This is to adjust the rotation of the accelerometer referential with respect to the belt.  

// Single-axis Gyroscope readout (note: we will also need to compute its derivative):
float omega, omegaOld; // omega is omega(t), and omegaOld is omega(t-dt)
float slopeOmega; // this is the approximation of [d(omega)/dt](t)

// Time integration step (to be passed to the update function): 
unsigned long oldTime;// = micros(); // attn: use long because we will measure in micro-seconds!
float dt; // in seconds (use real microseconds?)

// For tests: 
unsigned long tic_timer;//=millis();
#define PAUSE_TIC 50 // in milliseconds (will use milliseconds)
int tic_counter;

boolean continuousSending=false;
boolean sendingAngularSpeed=true; // when false, and in continuous mode, it will send the blob positions instead. 
#define END_PACKET 10 // for sending data to the Processing program for tests (for instance). 

// Interaction mode: can be set with command 'I' or reset with 'i'
boolean interactionMode=false; // PROBLEM!!! when I put this "true", particles are stuck to the borders sometimes!! WHY???

// Only one blob (the index 0) reacts to accelerations, the others follow like flies...
boolean mainBlobMode=false; // note: this makes sense only when inteactionMode==true

// test leds:
boolean testingLEDS=false; // can be activated/deactivated with ASCII code 't'/'T'

// in case of bluetooth:
 boolean isThisServer=true;
 
void setup() {

  Serial.begin(115200); //note: 57600 is a good speed to communicate with an XBee with default "fabric" parameters, and 9600 in the case of a bluesmirf modem bluetooth

  // Configure bluetooth module (in principle this could be done ONCE and for all, and save parameters to the module)
  //BTSetup();
  //delay(200);

  // Attempt a connection - unless this board is set as slave:
  // NOTE: we don't need to do this every time by the way (I think)
  if (isThisServer==true) BTSetSERVER(); else BTConnectCLIENT();  
  
  
  // Start up the LED strip
  strip.begin();
  // Update the strip, to start they are all 'off'
  strip.show();

  // Setup s x 7 segment leds:
  // NOTE:  The MAX72XX is in power-saving mode on startup, we have to do a wakeup call
  ledSegment.shutdown(0,false);
  /* Set the brightness to a medium values */
  ledSegment.setIntensity(0,8);
  /* and clear the display */
  ledSegment .clearDisplay(0);

  //Initialize avergaging buffer for sensors:
  numAvgSamples=MAX_AVGSAMPLES;
  for (int sensor=0; sensor<NUM_SENSORS; sensor++) {
    analogLocalValue[sensor][4]=0; // initialization of average
    oldSum[sensor]=0;
    bufferIndex[sensor]=numAvgSamples-1;       // initial value of the circular buffer index
    for (int buffIndex=0; buffIndex<numAvgSamples; buffIndex++) averagingBuffer[sensor][buffIndex]=0;
  }

  // Setting of the pins: 
  analogReference(DEFAULT); // DEFAULT means 3.3 volts in case of the arduino pro mini 3.3V. If we want to have 3.3 volts for any other board,
  // we may put EXTERNAL and connect the pin AREF to 3.3 V (through a 5KOhm resistor)

  // let's define the pin change mode as interruption:
  // NOTE: interrput  0 is on digital pin 2 (the pinModeChange)
  pinMode(pinModeChange, INPUT);
  digitalWrite(pinModeChange, HIGH); // this is to set the pullup resistors
  attachInterrupt(0, changeMode, CHANGE); // send an interrupt (and execute changeMode routine) when the pin changes state (we will use a SWITCH, not a button)


  // ATTN: normally there is no need to set the inputs as INPUTS, unless there were configured otherwise 
  //("The analogRead command will not work correctly if a pin has been previously set to an output,"
  // For security, we do it here (actually, wihout doing this, I had a problem!). 
  //BEWARE: we need to use A0, A1, A2, and A3. not 0, 1 , 2 and 3..
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  digitalWrite(A0, LOW);  
  digitalWrite(A1, LOW);  
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW); // reset pullup resistors 

  // Potentiometer readout, and setting of the offsetThetaDeg (only once!!): 
  pinMode(A4, INPUT);
  digitalWrite(A4, LOW); // reset pullup resistors
  int accum=0; // for averaging (ATTN!! int is signed 16 bits, meanining about 32000 to -32000!)
  for (int j=0; j<20; j++) { // this is for averaging the value on 100 readings (we could also measure order two or more statistics)
    accum+=analogRead(potThetaPin);
    delay(10);// delay for proper readout (is this really needed?)
  }
  potThetaValue=1.0*accum/20; // note: potThetaValue between 0 and 1024
  offsetThetaDeg=1.0*potThetaValue*180/1023; //offsetTheta in degrees

  // Same thing for the initial pendulum length:
  pinMode(A5, INPUT);
  digitalWrite(A5, LOW); // reset pullup resistors
  accum=0; // for averaging (ATTN!! int is signed 16 bits, meanining about 32000 to -32000!)
  for (int j=0; j<20; j++) { // this is for averaging the value on 100 readings (we could also measure order two or more statistics)
    accum+=analogRead(potLengthPin);
    delay(10);// delay for proper readout (is this really needed?)
  }
  potLengthValue=1.0*accum/20; // potLengthValue between 0 and 1024
  potLength=1.0*potLengthValue*1/1023; //potLength in meters. Maximum is 1 meter

  // CALIBRATE SENSORS: 
  calibrateSensors_Zero(); // ATTN: we need to start in the "zero" position. 
  // For the max and min, for the time being we will set the values by hand: 
  calibrateSensors_Max(); 
  calibrateSensors_Min();

  // Instantiation and initialization of light blobs: 
  //blob.init(); // not needed for only one blob, because the overloaded constructor calls the initializer already. 
  //  void init(float length, float mass, float damp, float rangeInteraction, float blobRadius, int identifier);
  for (int i=0; i<NUM_BLOBS; i++) {

    blobArray[i].init(0.3,0.7,3.5,3, i);
    //.init(float length, float mass, float damp, float rangeInteraction, int identifier);

    blobArray[i].setblobSize(1.0, 1.0); // elongated blob
    // blobArray[i].setblobSize(.3, 0.3);// same size blob
    // blobArray[i].setblobSize(2.00, 2.0); //sets the x and y elongation (radius) in meters (it can be square or elliptic)
    // REM: for a belt radius of .3 m (widthBelt=188cm), and an (cyclic) array of 8 leds, this means 188/8=23 cm / led

    blobArray[i].blobID=i;

    blobArray[i].blobShape=RECTANGULAR;//ELLIPTICAL;//;
    blobArray[i].rotatingBlob=false;

    blobArray[i].topologyMode=CYCLIC; // BOUNDED (this only affects X coordinate). Note that topology can be cyclic but the angular excursion can be limited!
    blobArray[i].setLimitsExcursionDegCm(-180,180,0,32); // (-50deg, 50deg, -4cm, 64cm); // rem: for instance, the long torso matrix is 64 cm in height, and the inter-led distance is 4 cm
    blobArray[i].bumpFactorTheta=0.9;
    blobArray[i].bumpFactorZ=0.9; // for KENTARO's performance, let's make a lot of speed absorption at the edges, unless this is working on the LEGS

    blobArray[i].dampFactorTheta=0.035; // NOTE: since I am now using Verlet method with "heuristic" damping, the damping factor is the proportion of the speed lost at each integration step - not a physical meaninful thing
    blobArray[i].dampFactorZ=0.006; // NOTE: since I am now using Verlet method with "heuristic" damping, the damping factor is the proportion of the speed lost at each integration step - not a physical meaninful thing

    // size and shift of actual displayed region:
    blobArray[i].setRegionDisplayDegCm(-180, 180, -0.02, 32+0.02); // if exactly equal to setLimitsExcursionDeg, then the display corresponds exactly to the computing area.  

    blobArray[i].interForceFactor=3;  
    blobArray[i].setRangeInteraction(0.15);// in meters  
    blobArray[i].rangeRepulsive=0.15; // in meters

    blobArray[i].sigmForce=0.1*(4*4);// 0.1*(blobArray[i].rangeInteractionLong)*(blobArray[i].rangeInteractionLong);

    if (mainBlobMode!=true) blobArray[i].setMass(0.15+i*0.05); // the main interest of slightly different masses here is to avoid having similar trajectories in certain cases
    else blobArray[i].setMass(0.15*(i==0)+0.01*(i!=0));

    // initialization of offsetThetaDeg and potLength from potentiometers or using hardcoded values:
    //NOTE: The offset on rotation could be different for each blob!  offsetTheta+180*i...
#ifdef WITH_POTENTIOMETERS  
    blobArray[i].setOffsetThetaDeg(offsetThetaDeg); // rem: value in degrees
    blobArray[i].setPendulumLength(potLength);
#else 
    blobArray[i].setOffsetThetaDeg(100);//180  // value input in degrees
    blobArray[i].setPendulumLength(0.05); // in meters
#endif

    // Noe: time step should be DECREASED when we have more spots (it will be slower, but accurate):
    blobArray[i].timeFactor=0.43/NUM_BLOBS;// this will modify the integration constant dt 
    // NOTE: verlet integration is stable for oscillations when the time step is smaller than the period of the oscillation; here, the pendulum oscillation is ok, 
    //but the particle interaction (modelled as a parabolic well near the equilibrium position) is much faster: it is equal to w=sqrt(k/m), where k is the  

    // default mode: 
    blobArray[i].blobMode=PATTERN0;

  }

  // SENCOND BLOB ADHOC changes:
  // blobArray[1].setblobSize(0.3, 0.3); 
  // blobArray[1].rotatingBlob=false;

  // Initialization of timer for tests: 
  tic_timer=millis();
  tic_counter=0;

  // Initialization of time counter for computing the real time integration step dt:
  oldTime=micros();

  // initialization of omega (angular speed) from sensor readout: 
  omegaOld=omega=factorGyro*(analogLocalValue[3][3]-336); // second index 3 means the value at "calibration"

  randomSeed(analogRead(7));
}

// Change mode (catch interrupt from pinModeChange, when CHANGING STATE):
void changeMode() {
  noInterrupts();
  // add a delay in case there is some bouncing BEFORE the switch is set to the proper new state:
  delayMicroseconds(500000); // half a second (note: delayMicroseconds does not disable interrupts anymore). SEEMS NOT TO WORK!!
  // then, read the pin: 
  int state=digitalRead(pinModeChange); 
  //Serial.print(state);
  // change the modes for all the blobs accordingly:
  if (state==LOW) {// this means CONNECTED = > STAND BY for the DISPLAY
    displayMode=DISPLAY_OFF;
  } 
  else { // change mode and put the display ON again:
    // but first, check there was no bouncing:
    if (displayMode!=DISPLAY_ON) { // otherwise, do nothing
      displayMode=DISPLAY_ON;
      for (int i=0; i<NUM_BLOBS; i++) blobArray[i].blobMode=(blobArray[i].blobMode+1)%4; 
    }
  }
  interrupts();
}

void loop()
{

  unsigned long newTime=micros();
  dt=0.000001*(newTime-oldTime); // Compute real dt in seconds
  //if (dt==0) dt=0.001;
  oldTime=newTime;

  // For tests on the integrity of the matrix mapping: (comment if unnecessary): 
  if (testingLEDS) {
    if ((millis()-tic_timer)>PAUSE_TIC) {
      tic_counter++;
      tic_timer=millis();
    }
  }

  // (1) Read all local sensor values (all the time), and store the raw data in analogLocalValue[NUM_SENSORS][]
  readSensors(); 

  // (2) Process raw data (will calculate ax, ay and az in m/s^2, as well as omega, omegaOld and slopeOmega in rad/sec and rad/sec^2)
  processRawData(false); // Mode can be AVERAGING (true) or NON AVERAGING (false). This could be set by a switch? 

  // (3) Compute interaction forces between all blobs (if needed) at time t (rem: this will only be used if blob mode is PATTERN0, but could be used in the other cases):
  // if (interactionMode==true) {for (int i=0; i<NUM_BLOBS; i++) blobArray[i].computeInteraction(blobArray, NUM_BLOBS);}
  if (interactionMode==true) {
    for (int i=0; i<NUM_BLOBS; i++) blobArray[i].computeInteractionSeparableCoord(blobArray, NUM_BLOBS);
  }

  // (b) update dynamics depending on the blob mode:
  // for tests:
  //ax=5; ay=0; az=-5; 
  //omega=0; slopeOmega=0;
  if (mainBlobMode==false) for (int i=0; i<NUM_BLOBS; i++) blobArray[i].update(ax, ay, az, inclination, omega, slopeOmega, dt);
  else { // only the first blob react to accelerations from sensors...
    blobArray[0].update(ax, ay, az, inclination, omega, slopeOmega, dt);
    // the others are insensitive, or less sensitive:
    // for (int i=1; i<NUM_BLOBS; i++) blobArray[i].updateDynamics(0,0,0,0,0, dt); // attn: there is interaction anyway
    for (int i=1; i<NUM_BLOBS; i++) blobArray[i].update(ax/70, ay/70, az/70, omega/70, inclination, slopeOmega/70,  dt); // attn: there is interaction anyway
  }

  // (3) display blob on the led matrix:  
  clearBeltMatrix();   

  if (displayMode==DISPLAY_ON) {
    // FILL BELT MATRIX WITH BLOBS of PROPER ELONGATION:
    for (int i=0; i<NUM_BLOBS; i++) fillBeltMatrix(blobArray[i]); 

    // for tests:
    // allBeltMatrixOn();
  } // otherwise, the matrix will be off

  // Finally, display on the LED matrix:
  displayBelt();

  // display counter (only for main blob):
  displayNumberTurns(int(1.0*blobArray[0].theta/2/PI));


  // for tests:
  //Serial.print("ax ="); Serial.print(analogLocalValue[0][4]); 
  //Serial.print(", ay ="); Serial.print(analogLocalValue[1][4]);
  //Serial.print(", az ="); Serial.println(analogLocalValue[2][4]);

  // (3) for tests and calibration or sending data to the computer (sound?):
#ifdef USE_SERIAL
  if (Serial.available() > 0) processSerial();
  
   // Continuous sending: 
  if (continuousSending==true) {
    // NOTE: sensing blog positions is for testing using the Processing sketch "displayBelt_HOULALED", 
    // sending the angular speed is for using the Processing sketch "houlaSound":
    if (sendingAngularSpeed) {
      Serial.print(blobArray[0].vtheta); 
      // packet terminator:
      Serial.write(END_PACKET);
    } 
    else {
      for (int i=0; i<NUM_BLOBS; i++) {
        Serial.print(blobArray[i].blobX); 
        Serial.print(","); 
        Serial.print(blobArray[i].blobY);
        Serial.print(","); 
      }
      // packet terminator:
      Serial.write(END_PACKET);
    }
  }
#endif

} 

void readSensors(void) { // read sensor data from on-board sensors: 
  for (int i=0; i<NUM_SENSORS; i++) {   
    // read the input pins
    int newValue=analogRead(analogInputPin[i]);
    updateSensorValue(i, newValue);
  }

  // FOR TEST ONLY? (better not to read all the time the angular offset and pendulum length from potentiometers, because these values are not averaged (maybe they should!)): 
#ifdef WITH_POTENTIOMETERS
#ifdef CONTINUOUS_POT_READING
  potThetaValue=analogRead(potThetaPin); // potThetaValue between 0 and 1024
  offsetThetaDeg=1.0*potThetaValue*180/1023; //offsetThetaDeg in degrees

  potLengthValue=analogRead(potLengthPin); // between 0 and 1024
  potLength=1.0*potLengthValue*1/1023; //potLength in meters. Maximum is 1 meter

  // modify only if values changes from a quantity larger than the noise variance?
  for (int i=0; i<NUM_BLOBS; i++) {
    blobArray[i].setOffsetThetaDeg(offsetThetaDeg); 
    blobArray[i].setPendulumLength(potLength);
  }
#endif
#endif
}

// update sensor value, and compute "statistics":
void updateSensorValue(int sensorNum, int newValue) {
  // store in data array:
  analogLocalValue[sensorNum][0]=newValue;

  /*
  // Update max and min values: 
   // THIS IS OPTIONAL, but interesting for an "online" calibration consisting on moving as much as one want at startup:
   if (newValue>analogLocalValue[sensorNum][1]) analogLocalValue[sensorNum][1]=newValue;
   if (newValue<analogLocalValue[sensorNum][2]) analogLocalValue[sensorNum][2]=newValue;
   
   // Averaging on numAvgSamples samples, and storing the average on analogLocalValue[i][4]:
   // REM: latest (old) sample is in averagingBuffer[i][bufferIndex], and older is in index (bufferIndex+1)%AVG_NUM
   //First, retrieve the PREVIOUS sum:
   int sum=oldSum[sensorNum];// we could do: analogLocalValue[i][4]*numAvgSamples, but analogLocalValue is an INT
   // then, substract the oldest sample that will be discarded in the buffer:
   bufferIndex[sensorNum]=(bufferIndex[sensorNum]+1)%numAvgSamples; // the index is incremented (this is the index of oldest value, as well as the place for the new sample in the circular buffer)
   sum-=averagingBuffer[sensorNum][bufferIndex[sensorNum]];
   // store the new sample on the buffer, and add to the current sum:
   averagingBuffer[sensorNum][bufferIndex[sensorNum]]=newValue;
   sum+=newValue;
   oldSum[sensorNum]=sum; //update old sum
   // finally, store the average on analogLocalValue (converted to integer):
   analogLocalValue[sensorNum][4]=int(1.0*sum/numAvgSamples);
   */
}

void processRawData(boolean avg_mode) {
  // Use averaging or instant data?
  int index;
  if (avg_mode==true) index=4; 
  else index=0;

  // Acceleration (the accelerometer referential frame is not inertial, the measured value is equal to a=aM/R-g (vectors)
  // Conversion in m/s^2 is not necessary, but it may be good for further exploration. 
  // To do the conversion (and assuming first we have properly find the offset for each axis - note, this is NOT the "zero" position), I have measured a NORM for the acceleration of 
  // about 203 AD units (equivalently, using the Z axis I found about 203 AD units from the zero position for g=9.81m/s^2 "vertical"). This means that the correction factor must be: 

  // NOTE: the neutral position must be calibrated for each module;
  // Also, we really assume the axis are parallel and perpendicular to the belt: we don't have rotation matrices here!
  ax=factorAccelerometer*(analogLocalValue[0][index]-320); // now, acceleration is computed in m/s^2
  ay=factorAccelerometer*(analogLocalValue[1][index]-339);
  az=factorAccelerometer*(analogLocalValue[2][index]-305);

  normAcc=sqrt(ax*ax+ay*ay+az*az);
  if (normAcc>0) { // else? for the time being, we don't change the previous values.

    //compute normalized acceleration vector:
    ax_norm=ax/normAcc;
    ay_norm=ay/normAcc;
    az_norm=az/normAcc;

    // polar coordinates (angular direction):
    inclination=acos(az_norm); 
    phi=atan2(ay,ax); // REM: atan2 gives values between -180 and 180

  }

  // Gyroscope reading converted to rad/sec: 
#ifdef WITH_GYRO
  omega=factorGyro*(analogLocalValue[3][index]-309);
#else
  omega=0; // this simulates a reading from the gyro equal to no rotational speed
  slopeOmega=omegaOld=0;
#endif
  slopeOmega=(omega-omegaOld)/dt; // attention! in the update blob dynamics, I am multiplying dt by a factor; this means that he factorGyro will be different from the value computed by 
  // simple experimentation (ie, matching the real speed of rotation). 
  omegaOld=omega;
}


void clearBeltMatrix() { // clear the matrix once before filling it for all the spots
  for (int by=0; by<PIXEL_HEIGHT; by++) 
    for (int bx=0; bx<PIXEL_WIDTH; bx++) beltMatrix[by][bx]=strip.Color(0,0,0); // off
}

void allBeltMatrixOn() { // activate all the leds in the matrix (for tests)
  for (int by=0; by<PIXEL_HEIGHT; by++) 
    for (int bx=0; bx<PIXEL_WIDTH; bx++) beltMatrix[by][bx]=strip.Color(127, 127, 127); // white
}

// The filling could be done using an OR or XOR function between the blobs
void fillBeltMatrix(Blob& blob) { 
  float blobDispX, blobDispY, elongXDisp, elongYDisp; // auxiliary coordinate of the blob and size, in pixel coordinates, [0-PIXEL_WIDTH] (or PIXEL_WITH+1 in case of cyclic topology) and [0-PIXEL_HEIGHT]
  // NOTES: -  blobX and blobY are in normalized display cordinates (but can be outside the range [0,1] - how this affect the display may depend on the blob dynamics topology)
  //        -  blobDispX coordinate is between 0 and PIXEL_WIDTH in the case of CYCLIC topology, or between 0 and PIXEL_WIDTH-1 in case of BOUNDED topology

  // normalized elongation in each axis:
  float elongXnorm= blob.elongX/blob.widthBelt;
  float elongYnorm= blob.elongY/blob.widthBelt;  

  blobDispY=(PIXEL_HEIGHT-1)*blob.blobY; // y display has never a cyclic topology
  elongYDisp=elongYnorm*(PIXEL_HEIGHT-1);

  if (blob.topologyMode==CYCLIC) {
    blobDispX= PIXEL_WIDTH*blob.blobX; 
    elongXDisp=elongXnorm*PIXEL_WIDTH;
  }
  else {
    blobDispX= (PIXEL_WIDTH-1)*blob.blobX;
    elongXDisp=elongXnorm*(PIXEL_WIDTH-1);
  }

  // Compute the activated leds on the "virtual" beltMatrix: 
  float ux, uy, uxx, uyy, distSq;
  if (blob.rotatingBlob) alpha=atan2(az,ay);

  for (int by=0; by<PIXEL_HEIGHT; by++) 
    for (int bx=0; bx<PIXEL_WIDTH; bx++) {
      ux=bx-blobDispX;
      uy=by-blobDispY;

      // Cyclic topology (only on x...). NOTE: display topology could be cyclic even thought the belt is NOT cyclic, and visceversa. But it is better to pass the cyclic parameter as the topology
      // of the blob itself:
      if (blob.topologyMode==CYCLIC) { 
        if (ux>0.5*PIXEL_WIDTH)  ux-=PIXEL_WIDTH; 
        else if (ux<-0.5*PIXEL_WIDTH) ux+=PIXEL_WIDTH; 
      }

      if (blob.rotatingBlob) { // this means that the acceration will affect the blob axis:
        // rotate and scale by the proper blob elongation:

        // uxx=(sin(alpha)*ux-cos(alpha)*uy)/elongXDisp;
        // uyy=(cos(alpha)*ux+sin(alpha)*uy)/elongYDisp;


        // HACK to avoid computation (rem: use alpha or inclination):
        if ((inclination>PI/3)&&(inclination<2*PI/3)) {
          uxx=-uy/elongXDisp;
          uyy=ux/elongYDisp;
        } 
        else {
          uxx=ux/elongXDisp;
          uyy=uy/elongYDisp;
        }

      } 
      else { // only "horizontal" blobs:
        uxx=ux/elongXDisp; 
        uyy=uy/elongYDisp;
      }  

      // Now, check if the LED is inside the blob (for a rectangular or elliptical blob): 
      // Change color as a function of somehting: height, speed, number of turns...
      int colorHeight=127-blob.blobY*127; // as a function of height
      int colorTurns=63+63*(1.0*int(1.0*blob.theta/2/PI)/99); // as a function of turns
      int colorSpeed=int(abs(blob.vtheta/2/PI/10*127)); // as a function of angular speed
      float factorIntensity=exp(-fabs(ux)*1.8);//0.5*(2-fabs(ux));
      switch (blob.blobShape) {
      case ELLIPTICAL: // using euclidian norm (elliptical blob):
        distSq=uxx*uxx+uyy*uyy;
        if (distSq<=1.0) beltMatrix[by][bx]=strip.Color(colorSpeed,127-colorSpeed,127);
        break;
      case RECTANGULAR: // (b) using independent norm in each axis (faster, but will give only squares): 
        //if ((fabs(uxx)<1.0)&&(fabs(uyy)<1.0)) beltMatrix[by][bx]=strip.Color(127,127,127);
        if (fabs(ux)<4.5) beltMatrix[by][bx]=strip.Color(factorIntensity*(127-colorHeight),factorIntensity*colorHeight,factorIntensity*127);
        // USING SPEED:
        //if (fabs(ux)<1.0) beltMatrix[by][bx]=strip.Color(colorSpeed, colorSpeed, colorSpeed+1);//auxcol,127-auxcol,127-blob.blobY*127);
        //if (fabs(ux)<2.0) beltMatrix[by][bx]=colorWheel(1.0*blob.atheta/200);
        //else strip.Color(auxcol,127-auxcol,127);
        break;
      case CROSS:
        if ((fabs(uxx)<1.0)||(fabs(uyy)<1.0)) beltMatrix[by][bx]=strip.Color(colorSpeed,127-colorSpeed,127);
        break;  
      }

    }
}

/*
 void fillBeltWaterLevel(float blobX, float blobY) {
 float A0, phi0;
 float alpha=atan2(az,ay);
 A0=
 
 }
 */


void displayBelt() {    
  // The beltMatrix is completed. We need to map it to the real matrix: 
  // (REM: this and previous step could be done at the same time, and we may completely forget about beltMatrix, but
  // I do this here for clarity (we may want to send beltMatrix data serially to check things)
  // for (int by=0; by<PIXEL_HEIGHT; by++)  // for the time being, this is in fact only one row...
  for (int bx=0; bx<PIXEL_WIDTH; bx++) //{
    strip.setPixelColor(PIXEL_WIDTH-1-bx, beltMatrix[1][bx]);
  // }
  strip.show();
}


void displayNumberTurns(int turns) {
  byte a0, a1;
  boolean sign=false;
  if (turns<0) {
    sign=true; 
    turns=-turns;
  }
  a0=turns%10; 
  a1=(turns-a0)/10;
  ledSegment.clearDisplay(0);
  ledSegment.setDigit(0,0,a0,false);
  ledSegment.setDigit(0,1,a1,sign); // the dot in the second digit will indicate negative values. 
  // delay(delaytime);
}

// This function updates the "zero" values for the LOCAL sensors. REM: one needs of course to first put the config in its "zero" position:
void calibrateSensors_Zero() {
  for (int i=0; i<NUM_SENSORS ; i++) {
    unsigned int accum=0; // for averaging (ATTN!! int is signed 16 bits, meanining about 32000 to -32000!)
    for (int j=0; j<20; j++) { // this is for averaging the value on 100 readings (we could also measure order two or more statistics)
      accum+=analogRead(analogInputPin[i]);
      delay(10);// delay for the readout of accelerometer 
    };
    // calculate the zero position: 
    analogLocalValue[i][3]=int(1.0*accum/20);
  }
}

// MAX values for LOCAL sensors: 
void calibrateSensors_Max() {
  // For the time being, we will just take the max POSSIBLE value, or some value set by hand: 
  for (int i=0; i<NUM_SENSORS ; i++) {
    analogLocalValue[i][1]=0;// 1023; // IF PUTTING 0, we will be able to compute the max as we sample date!
  }
}

// MIN values for LOCAL sensors:
void calibrateSensors_Min() {
  // For the time being, we will just take the min POSSIBLE value, or some value set by hand: 
  for (int i=0; i<NUM_SENSORS ; i++) {
    analogLocalValue[i][2]=1023;//0; if putting 1023, we will be able to compute the max as we sample date!
  }
}

//COLOR WHEEL:
//Input a float value 0 to 1  to  to get a color value for the led strip (uint32_t)
//The colours are a transition r - g -b - back to r
uint32_t colorWheel(float value)
{
  byte r, g, b;
  uint16_t WheelPos=value*384;
  switch(WheelPos / 128)
  {
  case 0:
    r = 127 - WheelPos % 128;   //Red down
    g = WheelPos % 128;      // Green up
    b = 0;                  //blue off
    break; 
  case 1:
    g = 127 - WheelPos % 128;  //green down
    b = WheelPos % 128;      //blue up
    r = 0;                  //red off
    break; 
  case 2:
    b = 127 - WheelPos % 128;  //blue down 
    r = WheelPos % 128;      //red up
    g = 0;                  //green off
    break; 
  }
  return(strip.Color(r,g,b));
}


