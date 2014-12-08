
#include "Arduino.h" // needs to be included if creating classes in standard C++ form (.cpp and .h) and still using most of Arduino core functions
#include "classBlob.h"

Blob::Blob() {
  init(0.3,1,1.0,.5,0); // or just call init() that will use the default parameters.
  //init();
}

Blob::~Blob() {
  // nothing to do (there was no dynamic allocation of memory)
}


void Blob::init(float length=0.3, float m=1.0, float damp=0.9, float rangeInter=1, int id=0) {
  // Blob modes:
  blobMode=PATTERN0;
  timeCounter=0;
  sign=1;
  bounceAxis=1;
  
  blobID=id;
 
 // Topology (for x axis, i.e., the theta variable):
 topologyMode=CYCLIC;
 
 // blob shape:
 blobShape=RECTANGULAR;
 
 // blob behaviour:
 rotatingBlob=false;
 
 // Physical model:
 lengthPendulum=length;
 mass=m;
 
 // damping: if isotropic, then dampFactorTheta=dampFactorZ
 // NOTE: in this version of the program, damping is done HEURISTICALLY, not computing the friction forces. 
 dampFactorTheta=damp;
 dampFactorZ=damp;
 
 // Vertical bumping on the limits of the belt:
thetaMin=-1.0*PI/3; // in radians 
thetaMax=1.0*PI/3; // in radians 
zMin=0; // limits for the vertical coordinate of the blob (in meters)
zMax=0.68; // in meters. Note: the inter-led distance is precisely 4 cm; the real length of the display is 64 cm
// bumping factors:
 bumpFactorZ=0.99;
 bumpFactorTheta=0.99; // in case of BOUNDED topology for theta
 
  // particle interaction:
 interForceTheta=0; interForceZ=0;
 rangeInteraction=rangeInter;
 rangeRepulsive=1;
 interForceFactor=1;
 sigmForce=0.5*rangeInteraction*rangeInteraction;
 
 // initialization of friction forces (because we still don't have the speed): 
 frictionForceTheta=0; frictionForceZ=0;
 
  // Init horizontal and vertical dynamics:
  randomizePositions();
  
 // Graphics:
 elongX=0.02; // in meters
 elongY=0.02;
  
 // offset of theta position for display only:
 offsetTheta=0;//wrapPhase(1.0*PI/180*ofstThetaDeg); // in RADIANS!!

  // Other things:  
  collision=false; 
  timeFactor=0.3;//0.35;// this will reduce the integration constant dt
  
  // total length of the belt in meters:
  widthBelt=2*PI*RADIUS_BELT; // this means that the belt forms a full circle 
 
}


void Blob::update(float pax, float pay, float paz, float inclination, float pomega, float pslopeOmega, float pdt) {
       float halfcone=1.0*PI/9;
       
  switch(blobMode) {
   
     case PATTERN0: 
     // HACK for Kentaro:
     if ((inclination>(PI/2-halfcone))&&(inclination<(PI/2+halfcone))) {  // this means the arm is horizonal
           setblobSize(0.3,2.0); // vertical blob with respect to the belt, meaning horizontal line
     } else {
          setblobSize(2.0,0.3); 
     }
       updateDynamicsPendulum(pax, pay, paz, pomega, pslopeOmega, pdt);
     break;
     
     case PATTERN1: // random lights
     setblobSize(0.3,0.3); // circular blob
     //setblobSize(random(0,100)*0.01,random(0,100)*0.01); // circular blob
      updateRandom(40.0, pdt); // first parameter is period in milliseconds
     break;
     
     case PATTERN2: // spirals going up or down
     setblobSize(0.25,0.25); 
     updateSpiral(pax, pay, paz, pomega, pslopeOmega, pdt);
     break;
     
     case PATTERN3: // autonomous motion (turning or up and down):
       updateAutoMove(4000.0, pdt); // the parameter is the period of axis change, in milliseconds (note: blob shape is changed in this function)
     break;
     
   }
}


void Blob::randomizePositions() {
 
  // reset particle interaction just in case:
 interForceTheta=0; interForceZ=0;
 
   // horizontal motion (theta is given in radians)
 theta=wrapPhase((thetaMax-thetaMin)*0.01*random(0,100)+thetaMin); // note: random generates a long int, and here theta min and max are given in radians
 thetaOld=theta;
 //Serial.println(theta);
 vtheta=0;
 atheta=0; 
 
 // vertical motion: 
  zeta=(zMax-zMin)*0.01*random(0,100)+zMin;
  zetaOld=zeta;
  vzeta=0;
  azeta=0;  
  
}


float Blob::wrapPhase(float phase) {// will wrap the phase between -PI and PI
   float auxphase;
   if (phase==PI) 
    auxphase=PI; 
  else {
    auxphase=phase+PI;
    auxphase=auxphase-2*PI*floor(1.0*auxphase/(2*PI))-PI;
  }
  //Serial.println(auxphase);
  return(auxphase);
} 

void Blob::computeInteraction(Blob *blobArray, int nbBlobs) {
  // pb with verlet: acceleration should be computed at the NEW position, but we don't have the new position of the particles when we compute the interaction force!!!
  // REM: do not forget to reset the interForces at the end of the updateDynamicsPendulum method!!
 collision=false; //initialize collision flag (rem: collision will take place when in REPULSIVE distance) 
 for (int i=blobID+1; i<nbBlobs; i++) { // ATTN: this will work if the identifiers are set in order
     float ux, uy, interSpotDist, interForce;
     
     ux=1.0*RADIUS_BELT*(wrapPhase(blobArray[i].theta)-wrapPhase(theta)); // the distance is calculated on the surface of the belt (cilinder!), measured in standard units (MKS)
     uy=(blobArray[i].zeta-zeta); // same here: distance is in standard (MKS) units
     
     // Toroidal topology (on x only): 
     if (topologyMode==CYCLIC) {
       if (ux>0.5*widthBelt)  
         ux-=widthBelt; 
       else if (ux<-0.5*widthBelt) 
         ux+=widthBelt; 
     }

     // Calculate inter-particule distance on cilinder: 
     interSpotDist=sqrt(ux*ux+uy*uy);
     if (interSpotDist<0.002) { //(in meters) - this is to avoid untractable singularities
        // we have two options: do nothing, or generate a ranfom force in a random direction 
        float angle=2*PI*0.01*random(0,100); 
        interForce=0;//-10; // here, the magnitude of the force is choosen constant (note: <0 means repulsive)
        ux=cos(angle); uy=sin(angle);
        collision=true;
     } else { // this means the "normal" case, with interSpotDist>0 
     
     // First, normalize ux and uy:
     ux/=interSpotDist;   
     uy/=interSpotDist; 
     
   // (a)  Sinusoidal well: (works well with rangeInteraction=2, interForceFactor=10
   /*
   float quartPer=rangeRepulsive;// rangeInteraction-rangeRepulsive;
   if (interSpotDist<3*quartPer) { 
       collision=true;
       interForce=-interForceFactor*cos(0.5*PI/quartPer*interSpotDist); // rem: interForce>0 means repulsive (see below)
     } else {
       interForce=0;
     }
   */
 
  // (b) pseudo-lenard-jones:
     /*
     if (interSpotDist<7) { // this particular value comes from my Mathematica simulation
     interForce=16;
     collision=true;
     } else {
       float r1=(interSpotDist/10+1.96); r1*=r1*r1*r1;
       float r2=(interSpotDist/10+0.44); r2*=r2*r2*r2;
       interForce=3*180.5/r1-3*3.2/r2; //(note: >0 means repulsive)
     }
     interForce/=10;
     */
     
  // (c) electric potential plus constant attractive force: [unstable!]
     /*
      if (interSpotDist<4) { // this particular value comes from my Mathematica simulation
     interForce=32.7;
     collision=true;
     } else {
       float r1=interSpotDist*interSpotDist/100;
       interForce=-6.3+6.24/r1; //(note: >0 means repulsive)
     }
      interForce/=10;
     */
     
  // (d) Spring: 
   if (interSpotDist<rangeInteraction) { 
       collision=true;
       interForce=interForceFactor*(interSpotDist-rangeRepulsive); // rem: interForce<0 means repulsive 
     } else {
       interForce=0;
     }
   
   
   //(e) ad-hoc, non continuous:
   /*
   if (interSpotDist<rangeInteraction) { 
       if (interSpotDist<rangeRepulsive) interForce=+interForceFactor/(interSpotDist+0.4); // rem: interForce>0 means repulsive (see below)
       else interForce=-interForceFactor/(interSpotDist+0.4); // rem: interForce>0 means repulsive (see below)
     } else {
       interForce=0;
     }
   */
     
    } // end case interSpotDist!=0
     
      // Now, compute the X and Y components and add to the forces on THIS particle, and on the other particle:
      interForceTheta+=interForce*ux;
      interForceZ+=interForce*uy;
      // Newton third law: 
      blobArray[i].interForceTheta-=interForce*ux;
      blobArray[i].interForceZ-=interForce*uy;
 }
}

void Blob::computeInteractionSeparableCoord(Blob *blobArray, int nbBlobs) {
  // this is using separable coordinate distances 
 collision=false; //initialize collision flag (rem: collision will take place when in REPULSIVE distance) 
 for (int i=blobID+1; i<nbBlobs; i++) { // ATTN: this will work if the identifiers are set in order
     float ux, uy, auxForceX, auxForceY;
     
     ux=1.0*RADIUS_BELT*(wrapPhase(blobArray[i].theta)-wrapPhase(theta)); // the distance is calculated on the surface of the belt (cilinder!), measured in standard (MKS) units
     uy=(blobArray[i].zeta-zeta); // same here: distance is in standard (MKS) units
     
      // Toroidal topology (on x only): 
     if (topologyMode==CYCLIC) {
       float widthBelt=2*PI*RADIUS_BELT; // this means that the belt forms a full circle
     if (ux>0.5*widthBelt)  ux-=widthBelt; 
     else if (ux<-0.5*widthBelt) ux+=widthBelt; 
     }
     
     // (c) ad-hoc spring force on theta (the "x" axis):
     float abx=fabs(ux);
     if ((abx<rangeInteraction)&&(abx>0.002)) { //otherwise no interaction, or do nothing
     // spring-like force (in this case, interForceFactor is equal to the spring constant - note: REALLY a spring on the distance, not the angle!!!)
     collision=true;
     auxForceX=50*interForceFactor*(abx-rangeRepulsive)*ux/abx; // negative means repulsive
     // NOTE: by multipying here by lengthPendulum, we have in fact ...
     } else {
       auxForceX=0;
     }
     
      float aby=fabs(uy);
      if ((aby<rangeInteraction)&&(aby>0.002)) { //otherwise no interaction or do nothing
     // spring-like force (in this case, interForceFactor is equal to the spring constant)
     collision=true;
     auxForceY=interForceFactor*(aby-rangeRepulsive)*uy/aby; // a negative value means repulsive
     } else {
       auxForceY=0;
     }
    
     
      // Now, compute the X and Y components and add to the forces on THIS particle, and on the other particle:
      interForceTheta+=auxForceX;
      interForceZ+=auxForceY;
      // Newton third law: 
      blobArray[i].interForceTheta-=auxForceX;
      blobArray[i].interForceZ-=auxForceY;
 }
}


void Blob::updateDynamicsPendulum(float ax, float ay, float az, float omega, float slopeOmega, float dt) { // parameter is the measured acceleration (or force)
  // Note1: the order of the call in the main loop is important. It should be: (a) calculateNextPositions() for all particles, (b) computeInteraction() for all particles, (c) updateDynamicsPendulum() for all particles
  // Note2: the integration step is no longer calculated here, but passed as a parameter to the updateDynamicsPendulum function. It is used both for angular and height calculation.
  // However, we scale it by a timeFactor that is proper to the instance (in fact, this corresponds to changing the mass!?):
  dt*=timeFactor;
  
  // HACK1: We may want to compensate for the gravity: this could be done by simply subtracting the average (meaning: use a differential acceleration for the
  // vertical acceleration!), or just adding +g (or just a part of it)
  //float aznew=az-azOld;
  //azOld=az; az=azNew;
  //az-=9.81/2;
  
  // HACK2: we can imagine that the accelerometer measures a FORCE, not an acceleration; this way, we will have different accelerations for the
  // different virtual masses. This is not completely unrealistic: the user don't feel any mass inertia, appart from that of the belt and his own body, which is constant, hence different
  // accelerations correspond to different FORCES. However, the problem is that the accelerometer mass does not depend on the virtual mass; if we divide the mesurement by the virtual mass, then it
  // would be actually as if the acceleration was different, not the force (this may be weird, in particular for the gravity that is also measured by the accelerometer...). But it can be nice anyway 
  // as a visual effect (slowly moving blobs and fast moving ones), hence the acceleration is ax=ax/mass... (force is equal to ax)
  // Comment the following lines if one wants real accelerations:
  ax=-ax/mass; // this means that we assumed the original ax was in reality fx (a force)
  ay=ay/mass;
  az=az/mass;
 
 //ax=-ax; // WHY??? PROBLEM: need to re-read the maths
  
  //HACK for Kentaro's legs:
 // if (az<0) az/=2.5;
 // if (az>0) az*=3.0;
  
  // (A) NUMERICAL INTEGRATION (for integrating the equation of motion (here these are two non-linear, 1st order DOEs)
  // EULER: bad precision
  // VERLET: - more stable, conserves the energy IF there is no damping. Damping is introduced HEURISTICALLY (the problem is that the acceleration is calculated assuming it does not depends on the speed...).
  //         - speed can be estimated in O(dt^2) by doing v(t)=(x(t+dt)-v(t-dt))/dt. 
  //         - needs initialization of position; a good approximation in O(dt^3) is done by initializing x0=x(dt). Anyway, not a problem is our case. 
  // VELOCITY VERLET: explicityly computes speed; however, the problem is that there is no way to introduce damping, even heuristically (true?). 

  // !!!!!! CONCLUSION (as for 25.11.2010): I will use normal Verlet, with heuristic damping - this means that I won't use the frictionForces!
 
 // (1) Compute the total force (we are only interested on the tangential and vertical components, the radial component is assumed to cancel with the reaction of the virtual "confining" cylinder):
 //   (a) Internal forces already calculated (computeInteraction() method): internalForceTheta, internalForceZ (from positions at time t):
 //   (b) "Inertial" forces (include fictive forces AND gravity): 
 //   NOTE: (ax, ay, az) is the notation for (a_ralpha, a_alpha, and az), which are the components of the acceleration measured at point A' (separated from a distance of RADIUS_BELT from the center of 
 //   rotation. These are exactly the values measured by the accelerometer, if we assume that the accelerometer axis are parallel respectively to u_ralpha and u_alpha - i.e., the radial and tangencial vectors to A').   

 inertialForceTheta=mass*((RADIUS_BELT*omega*omega+ax)*sin(theta+offsetTheta)+(RADIUS_BELT*slopeOmega-ay)*cos(theta+offsetTheta)-lengthPendulum*slopeOmega);
 //inertialForceTheta=-mass*ax;
 
 //inertialForceZ=... to do using ANGULAR elevation (... mass*((lengthPendulum*omega*omega+az)*sin(theta)+(lengthPendulum*slopeOmega-ay)*cos(theta)-L*slopeOmega);
 inertialForceZ=-mass*az;
 
  //  (c) friction forces (calculated at time t): ATTN!!! not used in VERLET with heuristic damping!!!
 //frictionForceTheta=-dampFactorTheta*L*vtheta;
 //frictionForceZ=-dampFactorZ*vzeta;
 //   Finally, the total force is equal to (a)+(b)+(c):
 totalForceTheta=interForceTheta+inertialForceTheta;//+frictionForceTheta;
 totalForceZ=interForceZ+inertialForceZ;//+frictionForceZ;
 
 // (2) Compute the acceleration at time t, using Newton's second law: 
 // Note: in Velocity Verlet method, we need to compute the acceleration a(t+dt) from the position at time t+dt...
  atheta=1.0/(mass*lengthPendulum)*totalForceTheta;
  azeta=1.0/mass*totalForceZ;
 
 // (3) Update the current "positions": 
 // NOTE: wrap the phase of theta here? (perhaps no need to do it here, but right after conversion in "x" coordinates, so we can add offsetTheta, 
 // and we can always send the "real" theta value (unwrapped) to the computer or other hardware (tours counter!!). 
 // (a) Without damping: 
 // float thetaNew=2*theta-thetaOld+atheta*dt*dt; 
 // float zetaNew=2*zeta-zetaOld+azeta*dt*dt;
 // or (b) WITH HEURISTIC DAMPING:
 float thetaNew=(2-dampFactorTheta)*theta-(1-dampFactorTheta)*thetaOld+atheta*dt*dt; 
 float zetaNew=(2-dampFactorZ)*zeta-(1-dampFactorZ)*zetaOld+azeta*dt*dt;
 
 // (4) Estimate the speed
 // Note 1: this may be not needed here; in fact, it is required only for damping at the reflexions in the borders, or 
 // in case we want the angular speed to control, say, the playback of a soundtrack (sending that to the computer by bluetooth) 
 // In this case, we need the "old" speed (time t), which can be computed whith O(dt^2) precision by doing:
 vtheta=0.5*(thetaNew-thetaOld)/dt*timeFactor; // note: we need to use the "real" dt, not the scaled one
 vzeta=0.5*(zetaNew-zetaOld)/dt*timeFactor; 
 
 // LIMIT SPEED (to avoid problems): 
 //if ((zetaNew-zeta)>(zMax-zMin)/100) zetaNew=zeta+(zMax-zMin)/100;
 //if ((zetaNew-zeta)<-(zMax-zMin)/100) zetaNew=zeta-(zMax-zMin)/100;
 
 // (4) update the old value and current value for zeta and theta:
 
 // BOUNCING on vertical axis (always):
 // NOTE: these sort of constraints could be solved differently, and Verlet method handles well constraints (such as infinite stiffness springs). 
   if (zetaNew>=zMax) { // note: zMaz=DY*BELT_HEIGHT
     // We need to limit the position, damp and invert the speed: this is tricky (it was easier in the Velocity Verlet algorithmby doing: vzeta=-vzeta1*bumpFactorZ;):
    // zetaOld=zMax;
    // zeta=zMax-bumpFactorZ*(zetaNew-zeta); // by doing this, next time the speed will be equal to -bumpFactorZ x old speed
    // or: 
     //zetaOld=zetaNew;
     //zeta=zetaNew-(zetaNew-zeta)*bumpFactorZ;  
    // or:
   // zetaOld=zetaNew-(zetaNew-zeta)*bumpFactorZ;// and zeta does not change value, or is set to the border:
    //zeta=zMax;
    // or:
  zetaOld=zMax+(zetaNew-zeta)*bumpFactorZ;// and zeta does not change value, or is set to the border:
  zeta=zMax;
    // or:
    //zetaOld=zMax+0.01; zeta=zMax; //speed is completely absorved...
 } else if (zetaNew<=zMin) {
    // zetaOld=zMin; 
    // zeta=zMin-bumpFactorZ*(zetaNew-zeta);
     //or: 
    //zetaOld=zetaNew;
    //zeta=zetaNew-(zetaNew-zeta)*bumpFactorZ;  
    //or:
    //zetaOld=zetaNew-(zetaNew-zeta)*bumpFactorZ;// and zeta does not change value, or is set to the border:
    //zeta=zMin;
     // or:
   zetaOld=zMin+(zetaNew-zeta)*bumpFactorZ;// and zeta does not change value, or is set to the border:
   zeta=zMin;
    // or:
   // zetaOld=zMin-0.01; zeta=zMin;
   } else {
     zetaOld=zeta;
     zeta=zetaNew; 
   }
   
 // BOUNCING on theta in case of non CYCLIC topology:
 if (topologyMode==BOUNDED) {
   if (thetaNew>=thetaMax) {
      thetaOld=thetaMax+(thetaNew-theta)*bumpFactorTheta;// and theta does not change value, or is set to the border:
      theta=thetaMax;
   } else if (thetaNew<=thetaMin) {
      thetaOld=thetaMin+(thetaNew-theta)*bumpFactorTheta;// and zeta does not change value, or is set to the border:
      theta=thetaMin;
   } else { 
    thetaOld=theta; 
    theta=thetaNew;
   }
 } else { // this is the case of CYCLIC topology:
    thetaOld=theta; 
    theta=thetaNew;
 }
 
 // (5) IMPORTANT: reset interforce for THIS particle (and also collision flag, because we may have reset interactionMode during a collision)
 interForceTheta=0; interForceZ=0; collision=false;

// (B) Update corresponding coordinates on the belt, DISPLAY PIXEL coordinates, taking into account the offsetTheta value (this is necessary to be able to activate the LEDs):
 // Map theta to the REARRANGED matrix of leds (16x4): 
 // map: [thetaMin, thetaMax]  -> [0, PIXEL_WIDTH]*DX + REAL_SHIFT_X  (note REAL_WIDTH, and not REAL_WIDTH-1) 
 //      theta |-> blobX
 // and  [zMin zMax] -> [0, PIXEL_HEIGHT]*DY + REAL_SHIFT_Y
 //      zeta  |-> blobY
 // (blobX and blobY is the center of the led "blob" in PIXEL coordinates)
 
 
 // (a) normalized horizontal coordinate on the blob in the DISPLAY REGION:
 // NOTE: in case of not bounded horizontal dynamics, then blobX may be beyond [0,1], but this means we just don't display anything
  blobX=(wrapPhase(theta)-thetaMinDisplay)/(thetaMaxDisplay-thetaMinDisplay);
 
 // (b) normalized vertical coordinate on the belt in PIXEL coordinates: 
   blobY=(zeta-zMinDisplay)/(zMaxDisplay-zMinDisplay); // a float value between 0 and 1
}  


void Blob::updateSpiral(float ax, float ay, float az, float omega, float slopeOmega, float dt) { //dt is in seconds
// here, the theta coordinate just turn in one or the other sense, depeding on the sign of az, and perhaps it's magnitude. 
// The same with z (constant speed, or intgrated through az):
float zetaNew;

//(1) constant angular speed and vertical speed:
//float sign=(az>0? 1.0 : -1.0);
//theta=wrapPhase(theta+sign*PI/180*10*dt); // dt is given in seconds, meaning that the turn is done at the angular speed of 10 degrees/sec
//zetaNew=zets+sign*.005*dt; // this means 5 mm / sec (units are in MKS).

// (2) or depending on the actual acceleration:
azeta=-az/6; // NOTE: to make it fall slowly, dampFactorZ must be relatively large.
float dampFactorZs=0.4;
zetaNew=(2-dampFactorZs)*zeta-(1-dampFactorZs)*zetaOld+azeta*dt*dt;
// theta turns as a screw (turns/cm on the vertical direction):
//float deltaz=100*(zetaNew-zeta); // in cm
//theta=wrapPhase(theta+1.0*PI*deltaz/3); // PI*deltaz would means one turn every 2 cm
theta=wrapPhase(theta+1.0*PI*azeta*dt*dt*90);

// test limits:
if (zetaNew>=zMax) {
  zetaOld=zMax+(zetaNew-zeta)*bumpFactorZ;// and zeta does not change value, or is set to the border:
  zeta=zMax;
 } else if (zetaNew<=zMin) {
   zetaOld=zMin+(zetaNew-zeta)*bumpFactorZ;// and zeta does not change value, or is set to the border:
   zeta=zMin;
  } else {
     zetaOld=zeta;
     zeta=zetaNew; 
 }

  // Normalized coordinates (to be sent to the display engine)
  blobX=(wrapPhase(theta)-thetaMinDisplay)/(thetaMaxDisplay-thetaMinDisplay); // a float value between 0 and 1
  blobY=(zeta-zMinDisplay)/(zMaxDisplay-zMinDisplay); // a float value between 0 and 1

}

void Blob::updateRandom(float period, float dt) { // period in milliseconds
 timeCounter+=dt; // this is given in seconds
 if (1000*timeCounter>period) {timeCounter=0; randomizePositions();};
  
   // normalize coorinates into display:
  blobX=(wrapPhase(theta)-thetaMinDisplay)/(thetaMaxDisplay-thetaMinDisplay); // a float value between 0 and 1
  blobY=(zeta-zMinDisplay)/(zMaxDisplay-zMinDisplay); // a float value between 0 and 1
  
}

void Blob::rotateHorizontal(float angularSpeed, float dt) { // angularSpeed in degrees/sec
  theta=wrapPhase(theta+1.0*PI/180*angularSpeed*dt); 
  zeta=0.5*(zMax-zMin); // just in case we make a CROSS instead of a vertically elongated blob
  
   // normalize coorinates into display:
  blobX=(wrapPhase(theta)-thetaMinDisplay)/(thetaMaxDisplay-thetaMinDisplay); // a float value between 0 and 1
  blobY=(zeta-zMinDisplay)/(zMaxDisplay-zMinDisplay); // a float value between 0 and 1
  
}

void Blob::bounceVertical(float bouncesPerSecond, float dt) { 
  theta=offsetTheta; // jut in case we have a CROSS blob and not an horizontally elongated blob
  zeta+=1.0*sign*dt*bouncesPerSecond*2*(zMax-zMin);
  
  // check bouncing conditions:
 if (zeta>=zMax) {
   zeta=zMax; sign=-1;
 } else if (zeta<=zMin) {
   zeta=zMin; sign=1;
 }
 
   // normalize coorinates into display:
  blobX=(wrapPhase(theta)-thetaMinDisplay)/(thetaMaxDisplay-thetaMinDisplay); // a float value between 0 and 1
  blobY=(zeta-zMinDisplay)/(zMaxDisplay-zMinDisplay); // a float value between 0 and 1
 
}

void Blob::updateAutoMove(float periodChangeAxis, float pdt) { // period of axis change in milliseconds
 timeCounter+=pdt; // this is given in seconds
 if (1000*timeCounter>periodChangeAxis) {
   timeCounter=0; 
   bounceAxis*=-1;
 }
 if (bounceAxis==1) {
   setblobSize(2.0,0.25); //  blob filling all the horizontal direction
   bounceVertical(2.0, pdt); // parameter is bounces/sec 
 } else {
   setblobSize(0.23,2.0); // a vertical elongated blob
   rotateHorizontal(360*3, pdt);
 }
 
}

void Blob::setblobSize(float paramElongX, float paramElongY) { // in cm (as measured in the belt)
elongX=paramElongX; 
elongY=paramElongY;
}
  
void Blob::setPendulumLength(float length) {lengthPendulum=length;}
void Blob::setMass(float m) {mass=m;}
void Blob::setOffsetThetaDeg(float ofstThetaDeg) {offsetTheta=wrapPhase(1.0*PI/180*ofstThetaDeg);}
void Blob::setLimitsExcursionDegCm(float thminDeg, float thmaxDeg, float zminCm, float zmaxCm) {
  thetaMax=thmaxDeg/180.0*PI; 
  thetaMin=thminDeg/180.0*PI;
  zMin=0.01*zminCm;
  zMax=0.01*zmaxCm;
}

void Blob::setRegionDisplayDegCm(float thetaMinDisplayDeg, float thetaMaxDisplayDeg, float zMinDisplayCm, float zMaxDisplayCm) {
  thetaMinDisplay=thetaMinDisplayDeg/180.0*PI;
  thetaMaxDisplay=thetaMaxDisplayDeg/180.0*PI;
  zMinDisplayCm=0.01*zMinDisplayCm; zMaxDisplay=0.01*zMaxDisplayCm;
}

void Blob::setRangeInteraction(float rangeInt) {rangeInteraction=rangeInt;}
void Blob::setInterForceFactor(float interFactor) {interForceFactor=interFactor;}
