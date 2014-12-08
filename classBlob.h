#include "Arduino.h" // needs to be included if creating classes in standard C++ form (.cpp and .h) and still using most of Arduino core functions

// blob mode:
// this is the main mode of operation, that can be changed by pressing a button. 
#define PATTERN0 0
#define PATTERN1 1
#define PATTERN2 2
#define PATTERN3 3

// TOPOLOGY (topologyMode state):
#define CYCLIC 0
#define BOUNDED 1

//blob shape (blobShape) : rectangular, elliptical or cross blob shape:
#define RECTANGULAR 0
#define ELLIPTICAL  1
#define CROSS       2

// SIZE OF THE BELT in standard units (the led display matrix can cover a partial zone - be smaller/larger, or shifted, rotated, etc...):
#define RADIUS_BELT 0.3  // this is the RADIUS (in meters) of the belt. The accelerometer is attached at this distance from the center of rotation (center of the body). 

class Blob{
  
  public:
  
  Blob();
  ~Blob();

  void init(float length, float mass, float damp, float rangeInteraction, int identifier);
  // this is a generic update function, that will update the motion depending on the operation mode. 
  void update(float ax, float ay, float az, float inclination, float omega, float slopeOmega, float dt); // 
  
  // different update methods for each mode:
  // (a) pendulum:
  void updateDynamicsPendulum(float ax, float ay, float az, float omega, float slopeOmega, float dt);
  void computeInteraction(Blob *blobArray, int nbBlobs);
  void computeInteractionSeparableCoord(Blob *blobArray, int nbBlobs);
  // (b) spiral: 
  void updateSpiral(float ax, float ay, float az, float omega, float slopeOmega, float dt);
  // (c) random:
  void updateRandom(float period,float dt);
  // (d) up-down or circular motion (without interaction):
  void rotateHorizontal(float angularSpeed, float dt);
  void bounceVertical(float bouncesPerSecond, float dt);
  void updateAutoMove(float periodChangeAxis, float dt);
  
  
  // others:
  void randomizePositions(void);
  
  //settings:
  void setMass(float mass);
  void setPendulumLength(float length);
  void setblobSize(float paramElongX, float paramElongY); // in cm (as measured in the belt)
  void setOffsetThetaDeg(float ofstThetaDeg); // in degrees, but the affected variable offsetTheta is in radians
  void setLimitsExcursionDegCm(float thmaxDeg, float thminDeg, float zminCm, float zmaxCm); 
  void setRegionDisplayDegCm(float thetaMinDisplaydeg, float thetaMaxDisplaydeg, float zMinDisplaycm, float zMaxDisplaycm);
  void setRangeInteraction(float rangeInt);
  void setInterForceFactor(float interFactor);
  float wrapPhase(float phase);
  
  // Variables:
  
  // blob mode of operation:
   int blobMode;
  
 // Blob identifier (will be useful to compute inter-blob interaction):  
 int blobID;
  
  // Topology of the dynamics (for the horizontal axis only): 
 int topologyMode; // CYCLIC or BOUNDED
 int blobShape;//=RECTANGULAR;
 boolean rotatingBlob;//=true;
  
 // Physical model parameters:
// The pendulum on a non-inertial frame, with a point mass is attached to the center of the moving frame by masseless arm of fixed length L:
float lengthPendulum; //this is the length of the virtual pendulum, in meters. It does not need to be equal to the RADIUS_BELT, the radius of the belt! ; actually, this could be controlled by a potentiometer or wirelessly. 
float mass; // note: the mass will not affect the "pendular" horizontal acceleration, but will affect the inter-particle acceleration (resulting from interacting forces!). 
float dampFactorTheta;//=1.5;
float dampFactorZ;
float widthBelt; // length of the belt in meters (=2*PI*RADIUS_BELT)

// Interaction force model and parameters:
float rangeInteraction;
float rangeRepulsive;
float interForceFactor, sigmForce;
boolean collision; 

 // Forced limits of the pendulum excursion:
float thetaMin; //in radians
float thetaMax;
float zMin; // limits for the vertical coordinate of the blob (in meters)
float zMax; // in meters. Note: the inter-led distance is precisely 4 cm; the real length of the display is 64 cm

// Elastic or inelastic collision at the borders (rem: it won't be treated as a force, but by just by inverting the speed)*
float bumpFactorZ; 
float bumpFactorTheta;

// REGION DISPLAYED in the DISPLAY matrix (blobX and blobY will give values between 0 and 1 for each axis on this range): 
float thetaMinDisplay; // in radians (ex: -45 in deg)
float thetaMaxDisplay; // in radians (ex: 45 in deg)
float zMinDisplay;// in m (ex: 0.04m). Note: the inter-led distance is precisely 4 cm
float zMaxDisplay; // in m (ex: 0.68m). The real length of the display is 64 cm


// Forces (we only need the tangencial component of the horizontal force, and the vertical force):
float interForceTheta, interForceZ;
float frictionForceTheta, frictionForceZ;
float inertialForceTheta, inertialForceZ; // note: this is the force measured by the accelerometer, and compensated by the gyro since the accelerometer is eccentric (ATTN: the accelerometer
// measurement also includes the real gravitational force! this means that this "inertial" force includes "fictitious" inertial forces, and gravitational forces). 
float totalForceTheta, totalForceZ; // the sum of all the terms above. 

// Generalized coordinates. Motion is not explicitly constrained (circular motion)
// (1) horizontal motion (given by the rotation angle in RADIANS). 
// NOTE: I will use VERLET integration method, so I need two variables for the "position": 
float theta, thetaOld; // angle at time t and t-dt respectively (in radians)
float vtheta; // (approximation) angular speed (rad/sec) at time t
float atheta; // angular acceleration (rad/sec^2) at time t 
//(2) vertical motion (translation), in METERS (motion is constrained to zMin and zMax, and bounces at these borders):
float zeta, zetaOld; // height at time t and t-dt respectively
float vzeta; // (approximation) of z-speed at time t 
float azeta; // z-acceleration at time t

// Integration of the differential equation using Verlet method: dt is a parameter given to the update function, but we have a timeFactor that can have different values for different instances of the class
float timeFactor;// this will reduce the integration constant dt

// a counter for random lights or autonomous continuous rotating or up/down motion:
float timeCounter;
int sign; // an auxiliary variable to make bouncing
int bounceAxis; // auxiliary variable to change axis of bouncing

// Graphic display: 
float offsetTheta; // (in RADIANS), offset on horizontal rotation from belt to accelerometer referential. 
float blobX, blobY; // NORMALIZED position of the light blob on the belt matrix (i.e., a proportion of its circumferece, and a proportion of its height). 
// Note: it is a float, because we may want to generate a blob by calculating the distance to the discrete points in the matrix.

// Elongation of the blob in x and y directions (in cm):
float elongX, elongY; 
};


