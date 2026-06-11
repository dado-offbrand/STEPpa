// Libraries
#include <ODriveFlexCAN.hpp>                  // ODrive library (version 0.10.6)
#include <Ramp.h>                             // Modified Ramp library by Sylvain Garnavault (version 0.6.3)
#include <SparkFun_BNO08x_Arduino_Library.h>  // SparkFun BNO08x Library by Sparkfun (version 1.0.6)

// Constants
#define OFF 1000  // This means don't move the foot (X, Y, and Z coords for a leg must be set OFF in order to prevent the foot from moving)

// Objects
rampFloat rampCoord[12];  // Ramp objects for the XYZ effector coordinates for each leg
rampFloat rampTilt[3];    // Ramp objects for the roll, pitch, and yaw angles of the robot
rampFloat rampT;          // Ramp object for the parameter t which represents time
BNO08x myIMU;             // BNO086 IMU object

// ODrive setup for Teensy 4.1
struct ODriveStatus;  // Hack to prevent compile error
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;
bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(1000000);  // CAN bus baudrate (must match ODrive settings)
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}
// ODrive CAN setup
const ODriveCanIntfWrapper q = wrap_can_intf(can_intf);  // Define a constant q to be used below
// ODrive objects (add all ODrives here and set CAN Node IDs)
ODriveCAN odrv0(q, 0), odrv1(q, 1), odrv2(q, 2), odrv3(q, 3), odrv4(q, 4), odrv5(q, 5),
  odrv6(q, 6), odrv7(q, 7), odrv8(q, 8), odrv9(q, 9), odrv10(q, 10), odrv11(q, 11);
// Array that points to the ODrive objects (add all ODrives here)
ODriveCAN* odrive[] = { &odrv0, &odrv1, &odrv2, &odrv3, &odrv4, &odrv5, &odrv6, &odrv7, &odrv8, &odrv9, &odrv10, &odrv11 };
struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};
// User Data (UD) objects (add all ODrives here)
ODriveUserData odrv0UD, odrv1UD, odrv2UD, odrv3UD, odrv4UD, odrv5UD, odrv6UD, odrv7UD, odrv8UD, odrv9UD, odrv10UD, odrv11UD;
// Array that points to the User Data objects (add all ODrives here)
ODriveUserData* userData[] = { &odrv0UD, &odrv1UD, &odrv2UD, &odrv3UD, &odrv4UD, &odrv5UD, &odrv6UD, &odrv7UD, &odrv8UD, &odrv9UD, &odrv10UD, &odrv11UD };

// Constant variables
const float gr = 9.7;                                                                                                                 // Gear reduction
const float homingIq = 3;                                                                                                             // Positive Iq limit for homing (no units, but proportional to current in A)
const int incDir[12] = { -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1 };                                                                  // The direction the motors spin when the joints increase in position (1 = CW and -1 = CCW)
const int homeDir[12] = { 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1 };                                                                 // The direction to spin the motors when homing (1 = CW and -1 = CCW)
const float homeAbsPos[3] = { 8.183482, 8.183482, 60.0 };                                                                             // The absolute homed position of the A, B, and ABAD actuators [deg]
const float c = 100, d = 150, e = 47.125, h = 43.23193;                                                                               // User defined parameters (UDFs) for single leg kinematics calculations [mm]
const float m = 45, phi = 150;                                                                                                        // More UDFs for single leg kinematics calculations [m = mm, phi = deg]
const float w = 188, v = 359;                                                                                                         // UDFs for IKR calculations [mm]
const float xStart = 0, yStart = e, zStart = 160;                                                                                     // X and Y coordinates for robot startup [mm]
const int chnPin[8] = { 7, 6, 5, 4, 3, 2, 1, 0 };                                                                                     // Digital pins for channels 1-8
const int mapRange[8][2] = { { -10, 10 }, { 10, -10 }, { -10, 10 }, { -10, 10 }, { 1, -1 }, { 0, 1 }, { 1, -1 }, { 0, 100 } };        // Range to map the channels
const int constrainRange[8][2] = { { -10, 10 }, { -10, 10 }, { -10, 10 }, { -10, 10 }, { -1, 1 }, { 0, 1 }, { -1, 1 }, { 0, 100 } };  // Range to constrain the channels
const float sensitivity = 0.9;                                                                                                        // A value from 0 to 1 to reduce the mapping range of the joystick channels (lower value = less sensative)

// ODrive variables
float theta[12];                // Measured joint positions [revs]
float thetaOffset[12] = { 0 };  // The difference between the relative 0 position of the joint and the absolute 0 position of the joint [revs]
float iq[12];                   // Measured torque producing current (no units, but proportional to current in A)

// Kinematics variables
float coordFK[12];    // End effector coordinates calcuated by the FK (forward kinematics) equations [mm]
float thetaIK[12];    // The actuator angles calculated by the inverse kinematics equations [revs]
float coordsIKR[12];  // X, Y, and Z coordinates calculated by the IKR equations

// RC variables
int chnVal[8], chn5Prev, chn6Prev;             // Current RC channel values, previous channel 5 value, and previous channel 6 value
volatile float start[8], width[8];             // Pulse start time [μs] and pulse width [μs] of each channel
unsigned long int btnTime = 0, loopTimeI = 0;  // Total time [ms] that channel 6 (push button) has been pressed and total loop time

// Other variables
int state = 0;                                                                                                              // The robot states (0 = non calibrated, 1 = starting, 2 = idle, 3 = standing)
float zDefault = 280, zLow = 200;                                                                                           // The default Z height and the low Z height
float xNom = 0, yNom = e, zNom = zDefault;                                                                             // Nominal effector coordinates [mm]
float xHome[4] = { xNom, xNom, xNom, xNom }, yHome[4] = { yNom, yNom, yNom, yNom }, zHome[4] = { zNom, zNom, zNom, zNom };  // Home coordinates for each leg [mm] (initialize all as nom coords)
float rawRoll, rawPitch, filtRoll, filtPitch;                                                                               // Raw and filtered IMU angles

void setup() {
  Serial.begin(115200);  // Begin Serial Monitor
  odriveSetup();         // ODrive setup

  // IMU setup
  Wire.begin();
  myIMU.begin(0x4B, Wire, A7, A6);  // INT and RST pins can be changed
  myIMU.enableRotationVector();

  // RC receiver setup
  for (int i = 0; i < 8; i++) {
    pinMode(chnPin[i], INPUT);                                              // Set channel pin to INPUT
    attachInterrupt(digitalPinToInterrupt(chnPin[i]), isrHandler, CHANGE);  // Add pin to interrupt
  }
}

void loop() {
  /*
  // Stabalize the robot when it's not walking
  if (state == 3) {  // If state is 3 (standing)
    getIMU();        // Read the IMU angles
    getShifts();     // Read the IMU and calculate the new home positions
    moveFeetInst(xHome[0], yHome[0], zHome[0], xHome[1], yHome[1], zHome[1], xHome[2], yHome[2], zHome[2], xHome[3], yHome[3], zHome[3]);
  }
  */
  
  getReceiver();  // Get user inputs from the RC receiver (comment out if using getTestingInput())
  //getTestingInput();  // Get user input from the serial monitor (comment out if using getReceiver())
  delay(5);
}
