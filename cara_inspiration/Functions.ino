//--------------------***Filtering***--------------------
// Filter made by Curio Res (https://www.youtube.com/watch?v=eM4VHtettGg)
template<int order>  // order is 1 or 2
class LowPass {
private:
  float a[order];
  float b[order + 1];
  float omega0;
  float dt;
  bool adapt;
  float tn1 = 0;
  float x[order + 1];  // Raw values
  float y[order + 1];  // Filtered values

public:
  LowPass(float f0, float fs, bool adaptive) {
    // f0: cutoff frequency (Hz)
    // fs: sample frequency (Hz)
    // adaptive: boolean flag, if set to 1, the code will automatically set
    // the sample frequency based on the time history.

    omega0 = 6.28318530718 * f0;
    dt = 1.0 / fs;
    adapt = adaptive;
    tn1 = -dt;
    for (int k = 0; k < order + 1; k++) {
      x[k] = 0;
      y[k] = 0;
    }
    setCoef();
  }

  void setCoef() {
    if (adapt) {
      float t = micros() / 1.0e6;
      dt = t - tn1;
      tn1 = t;
    }

    float alpha = omega0 * dt;
    if (order == 1) {
      a[0] = -(alpha - 2.0) / (alpha + 2.0);
      b[0] = alpha / (alpha + 2.0);
      b[1] = alpha / (alpha + 2.0);
    }
    if (order == 2) {
      float alphaSq = alpha * alpha;
      float beta[] = { 1, sqrt(2), 1 };
      float D = alphaSq * beta[0] + 2 * alpha * beta[1] + 4 * beta[2];
      b[0] = alphaSq / D;
      b[1] = 2 * b[0];
      b[2] = b[0];
      a[0] = -(2 * alphaSq * beta[0] - 8 * beta[2]) / D;
      a[1] = -(beta[0] * alphaSq - 2 * beta[1] * alpha + 4 * beta[2]) / D;
    }
  }

  float filt(float xn) {
    // Provide me with the current raw value: x
    // I will give you the current filtered value: y
    if (adapt) {
      setCoef();  // Update coefficients if necessary
    }
    y[0] = 0;
    x[0] = xn;
    // Compute the filtered values
    for (int k = 0; k < order; k++) {
      y[0] += a[k] * y[k + 1] + b[k] * x[k];
    }
    y[0] += b[order] * x[order];

    // Save the historical values
    for (int k = order; k > 0; k--) {
      y[k] = y[k - 1];
      x[k] = x[k - 1];
    }

    // Return the filtered value
    return y[0];
  }
};

LowPass<1> lpRoll(0.1, 1e3, true);   // Filter instance for the roll angle (cutoff frequency [Hz], sampling frequency [Hz], adaptive boolean flag)
LowPass<1> lpPitch(0.1, 1e3, true);  // Filter instance for the pitch angle (cutoff frequency [Hz], sampling frequency [Hz], adaptive boolean flag)

// --------------------***CAN Functions***--------------------
// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}
// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}
// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrives : odrive) {
    onReceive(msg, *odrives);
  }
}
// CAN Setup for the ODrives
void odriveSetup() {
  // Register callbacks for the heartbeat and feedback messages
  for (int i = 0; i < 12; i++) {
    odrive[i]->onFeedback(onFeedback, userData[i]);
    odrive[i]->onStatus(onHeartbeat, userData[i]);
  }
  // Configure and initialize the CAN bus interface
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true)
      ;  // Spin indefinitely
  }
}

// --------------------***ODrive Functions***--------------------
// Set the state of an individual ODrive (0 = IDLE, 1 = CLOSED LOOP CONTROL)
void setState(int num, int state) {
  // Set state for 5ms to ensure it get's set
  long timeI = millis();
  while (millis() - timeI < 5) {
    odrive[num]->clearErrors();
    odrive[num]->setState(state ? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE);
    pumpEvents(can_intf);
    delay(1);
  }
}
// Read each motor's position and calculate their joint's position
void getJointPos() {
  // Read joint positions for 5ms to ensure a good reading
  long timeI = millis();
  while (millis() - timeI < 5) {
    pumpEvents(can_intf);  // Helps handle incoming feedback
    for (int i = 0; i < 12; i++) {
      if (userData[i]->received_feedback) {
        Get_Encoder_Estimates_msg_t feedback = userData[i]->last_feedback;
        userData[i]->received_feedback = false;
        theta[i] = feedback.Pos_Estimate * incDir[i] / gr - thetaOffset[i];  // revs
      }
    }
  }
}
// Get torque producing "current" from each joint
void getJointIq() {
  for (int i = 0; i < 12; i++) {
    Get_Iq_msg_t msg;             // Create objects that will be populated
    odrive[i]->getCurrents(msg);  // Request the message from the ODrive and populate the object
    iq[i] = msg.Iq_Measured;      // Set the measured "current"
  }
}

//--------------------***Kinematics Functions***--------------------
// Compute the target joint positions of a single leg using the inverse kinematics (IK) equations
void getIK(int leg, float x, float y, float z) {
  // If  all the coordinates are marked OFF mark the joint angles as OFF
  if (x == OFF && y == OFF && z == OFF) {
    thetaIK[leg * 3] = OFF;
    thetaIK[leg * 3 + 1] = OFF;
    thetaIK[leg * 3 + 2] = OFF;
  }
  // Calculate the joint angles using IK if all coordinates are not marked as OFF
  else {
    // Calculate the intermediate variables
    float n = sqrt(sq(d) + sq(m) - 2 * d * m * cos(phi * PI / 180));  // mm
    float l = sqrt(sq(y) + sq(z) - sq(e)) - h;                        // mm
    float f = sqrt(sq(x) + sq(l));                                    // mm
    float theta2 = acos((sq(c) + sq(f) - sq(n)) / (2 * c * f));       // rads

    // Calculate joint ABAD and B's target positions
    thetaIK[leg * 3 + 2] = acos(e / sqrt(sq(y) + sq(z))) + atan(y / z) - PI / 2;  // thetaABAD
    thetaIK[leg * 3 + 1] = theta2 - atan(x / l);                                  // thetaB

    // Calculate more intermediate variables
    float theta8 = PI - thetaIK[leg * 3 + 1] + acos((sq(d) + sq(n) - sq(m)) / (2 * d * n)) - acos((sq(c) + sq(n) - sq(f)) / (2 * c * n));  // rads
    float v = sqrt(sq(d * sin(theta8) - c * sin(thetaIK[leg * 3 + 1])) + sq(d * cos(theta8) + c * cos(thetaIK[leg * 3 + 1])));             // mm

    // Calculate joint A's target position
    thetaIK[leg * 3] = 2 * acos((sq(v) + sq(c) - sq(d)) / (2 * v * c)) - thetaIK[leg * 3 + 1];  // thetaA

    // Convert the joint positions from rads to revs
    thetaIK[leg * 3] /= 2 * PI;
    thetaIK[leg * 3 + 1] /= 2 * PI;
    thetaIK[leg * 3 + 2] /= 2 * PI;
  }
}
// Compute the current XYZ position of the feet using the forward kinematics (FK) equations
void getFK() {
  getJointPos();                 // Read joint positions
  for (int i = 0; i < 4; i++) {  // parse through each leg
    // Convert joint positions from revs to rads
    float thetaA = theta[i * 3] * 2 * PI;
    float thetaB = theta[i * 3 + 1] * 2 * PI;
    float thetaABAD = theta[i * 3 + 2] * 2 * PI;

    // Calculate the intermediate variables
    float n = sqrt(sq(d) + sq(m) - 2 * d * m * cos(phi * PI / 180));                                                                      // mm
    float theta11 = PI - asin(c * sin((thetaA + thetaB) / 2) / d) - (thetaA + thetaB) / 2 + acos((sq(d) + sq(n) - sq(m)) / (2 * d * n));  // rads
    float f = sqrt(sq(c) + sq(n) - 2 * c * n * cos(theta11));                                                                             // mm
    float theta1 = acos((sq(c) + sq(f) - sq(n)) / (2 * c * f)) - thetaB;                                                                  // rads
    float theta4 = thetaABAD + PI / 2 - atan((f * cos(theta1) + h) / e);                                                                  // rads
    float g = sqrt(sq(f * cos(theta1) + h) + sq(e));

    // Calculate effector coordinates
    coordFK[i * 3] = f * sin(theta1);      // X
    coordFK[i * 3 + 1] = g * sin(theta4);  // Y
    coordFK[i * 3 + 2] = g * cos(theta4);  // Z
  }
}
// Compute the target foot positions for rotating the robot using the inverse kinematics rotations (IKR) equations
void getIKR(float roll, float pitch, float yaw) {
  // Convert angles to radians
  roll *= PI / 180;
  pitch *= PI / 180;
  yaw *= PI / 180;
  // Compute roll axis IK
  float b = zNom - w / 2 * sin(roll);
  float c = yNom + w / 2 * (1 - cos(roll));
  float zR = sqrt(b * b + c * c) * sin(PI / 2 - roll - atan(c / b));
  float yR = sqrt(b * b + c * c) * cos(PI / 2 - roll - atan(c / b));
  b = zNom + w / 2 * sin(roll);
  float zL = sqrt(b * b + c * c) * sin(PI / 2 + roll - atan(c / b));
  float yL = sqrt(b * b + c * c) * cos(PI / 2 + roll - atan(c / b));

  // Compute pitch axis IK using the Z values (ZL and ZR) from the roll axis
  float z[2] = { zR, zL };       // Calculated roll axis Z values
  float zOutF[2], zOutB[2];      // Output Z values for the front and back legs
  float xF[2], xB[2];            // X values for roll and pitch for the front and back legs
  for (int i = 0; i < 2; i++) {  // Loop twice for each side of the robot
    b = z[i] - v / 2 * sin(pitch);
    c = xNom + v / 2 * (1 - cos(pitch));
    zOutF[i] = sqrt(b * b + c * c) * sin(PI / 2 - pitch - atan(c / b));
    xF[i] = sqrt(b * b + c * c) * cos(PI / 2 - pitch - atan(c / b));
    b = z[i] + v / 2 * sin(pitch);
    zOutB[i] = sqrt(b * b + c * c) * sin(PI / 2 + pitch - atan(c / b));
    xB[i] = sqrt(b * b + c * c) * cos(PI / 2 + pitch - atan(c / b));
  }

  // Compute yaw axis IK
  float y[2] = { yR, yL };                       // Calculated roll axis Z values
  float x0[2] = { xF[0], xB[1] };                // X values from the pitch and roll axis for the FR and BL legs
  float x1[2] = { xB[0], xF[1] };                // X values from the pitch and roll axis for the FL and BR legs
  float xOut0[2], yOut0[2], xOut1[2], yOut1[2];  // Outuput x and y values for the diagonal leg pairs
  for (int i = 0; i < 2; i++) {
    float e = w / 2 + y[i] - 0.5 * sqrt(sq(v) + sq(w)) * cos(atan(v / w) + yaw);
    float f = v / 2 + x0[i] - 0.5 * sqrt(sq(v) + sq(w)) * sin(atan(v / w) + yaw);
    float g = f * tan(yaw);
    yOut0[i] = (e + g) * cos(yaw);
    xOut0[i] = f / cos(yaw) - (e + g) * sin(yaw);
    e = w / 2 + y[i] - 0.5 * sqrt(sq(v) + sq(w)) * cos(atan(v / w) - yaw);
    f = v / 2 + x1[i] - 0.5 * sqrt(sq(v) + sq(w)) * sin(atan(v / w) - yaw);
    g = -f * tan(yaw);
    yOut1[i] = (e + g) * cos(yaw);
    xOut1[i] = f / cos(yaw) + (e + g) * sin(yaw);
  }
  // Transfer output coordinates to an array
  float coords[12] = { xOut1[1], yOut1[1], zOutF[1], xOut0[0], yOut0[0], zOutF[0], xOut0[1], yOut0[1], zOutB[1], xOut1[0], yOut1[0], zOutB[0] };
  // Transfer array to the coordsIKR array
  for (int i = 0; i < 12; i++) {
    coordsIKR[i] = coords[i];
  }
}

// --------------------***Movement Functions***--------------------
// Set the position of the joints
void setJointPos() {
  getIMU();  // Read the IMU here since this is the only frequenty polled function when the robot is walking
  for (int i = 0; i < 12; i++) {
    if (thetaIK[i] != OFF) {                                                   // Only set the joint position if it's not marked as OFF
      odrive[i]->setPosition((thetaIK[i] + thetaOffset[i]) * incDir[i] * gr);  // Convert theta from joint coords to motor coords and set position
    }
  }
}
// Home the actuators by moving them to the physical stop
void home() {
  bool check[12] = { 0 };  // Array to check whether each joint has been homed (initialize all array elements as 0)
  int sum = 0;             // The sum of the number of controllers that have been homed

  // Start spinning the joints
  for (int i = 0; i < 12; i++) {
    setState(i, 1);                                                                       // Enter all controllers into CLOSED LOOP CONTROL mode
    thetaOffset[i] = 0;                                                                   // Set theta offset to 0
    odrive[i]->setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);  // Set the control mode of the ODrives to velocity control
    odrive[i]->setVelocity(1.5 * homeDir[i]);                                             // Set the motor velocity to 1.5 rev/s (90 RPM)
  }

  // Wait for the joints to hit their physical endstops
  while (sum != 12) {  // While the sum is not equal to the total number of joints (12)
    getJointIq();      // Get torque producing "current"
    for (int i = 0; i < 12; i++) {
      if (!check[i] && (abs(iq[i]) > homingIq)) {                                            // Stop the joint if check is 0 and the measured "current" is greater than homingIq
        check[i] = 1;                                                                        // Set check equal to 1
        getJointPos();                                                                       // Read joint positions
        thetaOffset[i] = theta[i] - homeAbsPos[i % 3] / 360;                                 // Set theta offset
        odrive[i]->setVelocity(0);                                                           // Set motor velocity to 0 rev/s
        setState(i, 0);                                                                      // Put the controller in IDLE mode so that it resets it's closed loop control position
        setState(i, 1);                                                                      // Put the controller back in closed loop control
        odrive[i]->setControllerMode(CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_POS_FILTER);  // Set the controllers back to filtered position control
        sum++;                                                                               // Add 1 to sum
      }
    }
  }
  moveFeet(xStart, yStart, zStart, xStart, yStart, zStart, xStart, yStart, zStart, xStart, yStart, zStart, LINEAR, 500);  // Move  the back legs to their start position
}
// Move the feet to their target XYZ positions
void moveFeet(float x0, float y0, float z0, float x1, float y1, float z1,
              float x2, float y2, float z2, float x3, float y3, float z3, int mode, int time) {
  float coord[12] = { x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3 };  // Define coord array using the input coords
  getFK();                                                               // Calculate the current XYZ feet positions

  // Define the ramps
  for (int i = 0; i < 12; i++) {
    if (coord[i] == OFF) {   // Define the ramp if the coord is marked as OFF
      rampCoord[i].go(OFF);  // Set the ramp to OFF if the coord is OFF
    } else {
      rampCoord[i].go(coordFK[i]);            // Set the ramps to the current XYZ foot position
      rampCoord[i].go(coord[i], time, mode);  // Set the target XYZ effector positions [mm], duration [ms], and interpolation mode
    }
  }

  // Set rampT to track time
  rampT.go(0);
  rampT.go(1, time, LINEAR);

  // Move the feet
  while (rampT.isRunning()) {
    for (int i = 0; i < 4; i++) getIK(i, rampCoord[i * 3].update(), rampCoord[i * 3 + 1].update(), rampCoord[i * 3 + 2].update());  // Update ramps and calculate joint positions for each leg using IK
    rampT.update();                                                                                                                 // Update rampT
    setJointPos();                                                                                                                  // Set the position of the joints
    delay(5);                                                                                                                       // Short delay to prevent errors
  }
}

// Move the feet to their target XYZ positions instantaneously (must be looped)
void moveFeetInst(float x0, float y0, float z0, float x1, float y1, float z1,
                  float x2, float y2, float z2, float x3, float y3, float z3) {
  float coord[12] = { x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3 };                    // Define coord array using the input coords
  for (int i = 0; i < 4; i++) getIK(i, coord[i * 3], coord[i * 3 + 1], coord[i * 3 + 2]);  // Calculate the joint positions using IK
  setJointPos();                                                                           // Set the position of the joints
}
// Move the robot's center to an XYZ position [mm]
void moveRobot(float x, float y, float z, int mode, int time) {
  moveFeet(xNom - x, yNom + y, zNom + z, xNom - x, yNom - y, zNom + z, xNom + x, yNom + y, zNom + z, xNom + x, yNom - y, zNom + z, mode, time);
}
// Tilt the robot to a roll, pitch, and yaw angle about it's center [deg]
void tiltRobot(float roll, float pitch, float yaw, int mode, float time) {
  // Define the angle ramps
  rampTilt[0].go(roll, time, mode);
  rampTilt[1].go(pitch, time, mode);
  rampTilt[2].go(yaw, time, mode);

  // Set rampT to track time
  rampT.go(0);
  rampT.go(1, time, LINEAR);

  // Move the joints
  while (rampT.isRunning()) {                                                                            // Track time with rampT
    getIKR(rampTilt[0].update(), rampTilt[1].update(), rampTilt[2].update());                            // Update ramps and calculate coordsIKR[] using the IKR equations
    for (int i = 0; i < 4; i++) getIK(i, coordsIKR[i * 3], coordsIKR[i * 3 + 1], coordsIKR[i * 3 + 2]);  // Calculate the joint positions using IK
    rampT.update();                                                                                      // Update rampT
    setJointPos();                                                                                       // Set the position of the joints
    delay(5);                                                                                            // Short delay to prevent errors
  }
}
// Calculate joint positions to move a single leg along the swing phase (0), stance phase (1), or stance' phase (2) (this function must be looped)
void stepPhase(int leg, int phase, float h, float l, float alpha[], float t) {
  float x, y, z;  // Coordinates
  // Calculate swing phase coords
  if (phase == 0) {
    float aT = l * (t - sin(2 * PI * t) / 2 / PI);  // Parametric value
    x = xHome[leg] - l / 2 * cos(alpha[leg]) + aT * cos(alpha[leg]);
    y = yHome[leg] - l / 2 * sin(alpha[leg]) + aT * sin(alpha[leg]);
    z = zHome[leg] - h / 2 * (1 - cos(2 * PI * t));
  }
  // Calculate stance phase coords
  if (phase == 1) {
    x = xHome[leg] + l * cos(alpha[leg]) * (0.5 - 2 * t);
    y = yHome[leg] + l * sin(alpha[leg]) * (0.5 - 2 * t);
    z = zHome[leg];
  }
  // Calculate stance'1 phase coords
  if (phase == 2) {
    x = xHome[leg] + l * cos(alpha[leg]) * (0.5 - t);
    y = yHome[leg] + l * sin(alpha[leg]) * (0.5 - t);
    z = zHome[leg];
  }

  getIK(leg, x, y, z);  // Calculate joints angles using the IK equations
}
// Make the robot step in a cycloidal trajectory
void step(float h, float l, float alpha0, float alpha1, float alpha2, float alpha3, int time, int num) {
  if (criteria(num)) {
    float alpha[4] = { alpha0 * PI / 180, alpha1 * PI / 180, alpha2 * PI / 180, alpha3 * PI / 180 };  // Pass phi angles to an array and convert to rads

    // **Pre loop sequences**
    // FL and BR = up, FR and BL = back
    moveFeet(xHome[0], yHome[0], zHome[0] - h, xHome[1] - h / 2 * cos(alpha[1]), yHome[1] - h / 2 * sin(alpha[1]), zHome[1], xHome[2] - h / 2 * cos(alpha[2]), yHome[2] - h / 2 * sin(alpha[2]), zHome[2], xHome[3], yHome[3], zHome[3] - h, LINEAR, time);
    // FL and BR = swing 2, FR and BL = nothing
    rampT.go(0.5);                                   // Set ramp to 0.5
    rampT.go(1, time, LINEAR);                       // Set ramp to go to 1
    while (rampT.isRunning()) {                      // Run the loop while rampT is running
      stepPhase(0, 0, h, l, alpha, rampT.update());  // FL
      stepPhase(3, 0, h, l, alpha, rampT.update());  // BR
      setJointPos();                                 // Set the position of the joints
      delay(5);                                      // Short delay to prevent errors
    }

    // Take steps
    while (1) {     // Infinite loop, the exiting criteria is within
      //getShifts();  // Get the X and Y shifts since all 4 legs are on the ground and the robot is stable

      // FL and BR = stance, FR and BL = swing 1
      rampT.go(0);                                     // Set ramp to 0
      rampT.go(0.5, time, LINEAR);                     // Set ramp to go to 0.5
      while (rampT.isRunning()) {                      // Run the loop while rampT is running
        stepPhase(0, 1, h, l, alpha, rampT.update());  // FL
        stepPhase(1, 0, h, l, alpha, rampT.update());  // FR
        stepPhase(2, 0, h, l, alpha, rampT.update());  // BL
        stepPhase(3, 1, h, l, alpha, rampT.update());  // BR
        setJointPos();                                 // Set the position of the joints
        delay(5);                                      // Short delay to prevent errors
      }

      //  FL and BR = nothing, FR and BL = swing 2
      rampT.go(0.5);                                   // Set ramp to 0.5
      rampT.go(1, time, LINEAR);                       // Set ramp to go to 1
      while (rampT.isRunning()) {                      // Run the loop while rampT is running
        stepPhase(1, 0, h, l, alpha, rampT.update());  // FR
        stepPhase(2, 0, h, l, alpha, rampT.update());  // BL
        setJointPos();                                 // Set the position of the joints
        delay(5);                                      // Short delay to prevent errors
      }

      // **Continuous loop sequences**
      if (criteria(num)) {  // If this isn't the last loop (criteria is still true)
        // FL and BR = swing 1, FR and BL = stance
        rampT.go(0);                                     // Set ramp to 0
        rampT.go(0.5, time, LINEAR);                     // Set ramp to go to 0.5
        while (rampT.isRunning()) {                      // Run the loop while rampT is running
          stepPhase(0, 0, h, l, alpha, rampT.update());  // FL
          stepPhase(1, 1, h, l, alpha, rampT.update());  // FR
          stepPhase(2, 1, h, l, alpha, rampT.update());  // BL
          stepPhase(3, 0, h, l, alpha, rampT.update());  // BR
          setJointPos();                                 // Set the position of the joints
          delay(5);                                      // Short delay to prevent errors
        }
        // FL and BR = swing 2, FR and BL = nothing
        rampT.go(0.5);                                   // Set ramp to 0.5
        rampT.go(1, time, LINEAR);                       // Set ramp to go to 1
        while (rampT.isRunning()) {                      // Run the loop while rampT is running
          stepPhase(0, 0, h, l, alpha, rampT.update());  // FL
          stepPhase(3, 0, h, l, alpha, rampT.update());  // BR
          setJointPos();                                 // Set the position of the joints
          delay(5);                                      // Short delay to prevent errors
        }
      }

      // **Post loop sequences**
      else {
        // FL and BR = swing 1, FR and BL = stance'
        rampT.go(0);                                     // Set ramp to 0
        rampT.go(0.5, time, LINEAR);                     // Set ramp to go to 0.5
        while (rampT.isRunning()) {                      // Run the loop while rampT is running
          stepPhase(0, 0, h, l, alpha, rampT.update());  // FL
          stepPhase(1, 2, h, l, alpha, rampT.update());  // FR
          stepPhase(2, 2, h, l, alpha, rampT.update());  // BL
          stepPhase(3, 0, h, l, alpha, rampT.update());  // BR
          setJointPos();                                 // Set the position of the joints
          delay(5);                                      // Short delay to prevent errors
        }

        // FL and BR = move down, FR and BL = nothing
        moveFeet(xHome[0], yHome[0], zHome[0], OFF, OFF, OFF, OFF, OFF, OFF, xHome[3], yHome[3], zHome[3], LINEAR, time);
        break;  // Exit the loop
      }
    }
  }
}
// Make the robot step in place without moving
void stepInPlace(float h, float time) {
  //getShifts();  // Get the X and Y shifts since all 4 legs are on the ground and the robot is stable

  // FL and BR = move up
  moveFeet(xHome[0], yHome[0], zHome[0] - h, xHome[1], yHome[1], zHome[1], xHome[2], yHome[2], zHome[2], xHome[3], yHome[3], zHome[3] - h, QUADRATIC_OUT, time);

  // FL and BR = move down
  moveFeet(xHome[0], yHome[0], zHome[0], xHome[1], yHome[1], zHome[1], xHome[2], yHome[2], zHome[2], xHome[3], yHome[3], zHome[3], QUADRATIC_OUT, time);

  // FR and BL = move up
  moveFeet(xHome[0], yHome[0], zHome[0], xHome[1], yHome[1], zHome[1] - h, xHome[2], yHome[2], zHome[2] - h, xHome[3], yHome[3], zHome[3], QUADRATIC_OUT, time);

  // FL and BR = move down
  moveFeet(xHome[0], yHome[0], zHome[0], xHome[1], yHome[1], zHome[1], xHome[2], yHome[2], zHome[2], xHome[3], yHome[3], zHome[3], QUADRATIC_OUT, time);
}
// Demonstrate the IK equations for both translation and rotation
void kinematicsDemo() {
  int mode = CUBIC_OUT;
  int time = 500;

  // **Translation kinematics**
  // Move in Z (up and up)
  moveRobot(0, 0, 40, mode, time);
  moveRobot(0, 0, -100, mode, time * 3);
  moveRobot(0, 0, 0, mode, time * 2);
  // Move in X (forward and backwards)
  moveRobot(50, 0, 0, mode, time);
  moveRobot(-50, 0, 0, mode, time * 2);
  moveRobot(0, 0, 0, mode, time);
  // Move in Y (right and left)
  moveRobot(0, 25, 0, mode, time);
  moveRobot(0, -25, 0, mode, time * 2);
  moveRobot(0, 0, 0, mode, time);

  delay(100);

  // **Rotation Kinematics**
  time = 500;  
  // Tilt in roll
  tiltRobot(15, 0, 0, mode, time);
  tiltRobot(-15, 0, 0, mode, time * 2);
  tiltRobot(0, 0, 0, mode, time);
  // Tilt in pitch
  tiltRobot(0, 15, 0, mode, time);
  tiltRobot(0, -15, 0, mode, time * 2);
  tiltRobot(0, 0, 0, mode, time);
  // Tilt in yaw
  tiltRobot(0, 0, 15, mode, time);
  tiltRobot(0, 0, -15, mode, time * 2);
  tiltRobot(0, 0, 0, mode, time);

  delay(100);

  time = 500;
  // Tilt in pitch and roll
  tiltRobot(0, 10, 0, mode, time);
  tiltRobot(-10, 10, 0, mode, time);
  tiltRobot(10, 10, 0, mode, time * 2);
  tiltRobot(0, 0, 0, mode, time * 2);
  tiltRobot(0, -10, 0, mode, time);
  tiltRobot(-10, -10, 0, mode, time);
  tiltRobot(10, -10, 0, mode, time * 2);
  // Tilt in yaw, pitch, and roll
  tiltRobot(0, 0, 15, mode, time * 2);
  tiltRobot(0, 10, 15, mode, time);
  tiltRobot(0, -10, 15, mode, time * 2);
  tiltRobot(0, 0, 15, mode, time);
  tiltRobot(10, 0, 15, mode, time);
  tiltRobot(-10, 0, 15, mode, time * 2);
  tiltRobot(0, 0, 0, mode, time * 1.5);
  tiltRobot(0, 0, -15, mode, time * 2);
  tiltRobot(0, 10, -15, mode, time);
  tiltRobot(0, -10, -15, mode, time * 2);
  tiltRobot(0, 0, -15, mode, time);
  tiltRobot(10, 0, -15, mode, time);
  tiltRobot(-10, 0, -15, mode, time * 2);
  tiltRobot(0, 0, 0, mode, time * 1.5);

  delay(100);

  // Circulate in roll and pitch
  time = 500;
  tiltRobot(0, 15, 0, mode, time);
  tiltRobot(-15, 0, 0, QUADRATIC_IN, time);
  tiltRobot(0, -15, 0, QUADRATIC_OUT, time);
  tiltRobot(15, 0, 0, QUADRATIC_IN, time);
  tiltRobot(0, 15, 0, QUADRATIC_OUT, time);
  tiltRobot(0, 0, 0, mode, time);
  tiltRobot(0, -15, 0, mode, time);
  tiltRobot(-15, 0, 0, QUADRATIC_IN, time);
  tiltRobot(0, 15, 0, QUADRATIC_OUT, time);
  tiltRobot(15, 0, 0, QUADRATIC_IN, time);
  tiltRobot(0, -15, 0, QUADRATIC_OUT, time);
  tiltRobot(0, 0, 0, mode, time);

  delay(100);

  jump(3);  // Jump
}
// Make the robot jump
void jump(int num) {
  for (int i = 0; i < num; i++) {
    moveRobot(0, 0, -120, LINEAR, 300);            // Down to starting height (160 mm)
    moveRobot(0, 0, 40, EXPONENTIAL_OUT, 100);     // Up
    moveRobot(0, 0, 0, EXPONENTIAL_OUT, 100 / 4);  // Back home
    delay(500);                                    // Delay to restabalize
  }
}

// --------------------***Control Functions***--------------------
// Get IMU angles
void getIMU() {
  if (myIMU.wasReset()) {
    myIMU.enableRotationVector();
  }

  // Get roll and pitch angles [rads], reverse them, and constrain them between -20 and 20 degrees
  if (myIMU.getSensorEvent() == true && myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
    rawRoll = constrain(-myIMU.getRoll(), -20 * PI / 180, 20 * PI / 180);
    rawPitch = constrain(-myIMU.getPitch(), -20 * PI / 180, 20 * PI / 180);

    // Filter roll and pitch
    filtRoll = lpRoll.filt(rawRoll);
    filtPitch = lpPitch.filt(rawPitch);
  }

  //Serial.println((String) "roll: " + rawRoll * 180 / PI + "   pitch: " + rawPitch * 180 / PI);  // Print raw IMU angles [deg]
}
// Calculate X and Y shifts and the new home positions to keep the center of mass of the robot directly under the legs
void getShifts() {
  // Calculate X and Y shifts
  float xShift = zNom * tan(filtPitch);
  float yShift = zNom * tan(filtRoll);

  // Calculate new X home and Y home variables
  xHome[0] = xNom + xShift;
  xHome[1] = xNom + xShift;
  xHome[2] = xNom - xShift;
  xHome[3] = xNom - xShift;

  yHome[0] = yNom - yShift;
  yHome[1] = yNom + yShift;
  yHome[2] = yNom - yShift;
  yHome[3] = yNom + yShift;
}
// Determines if the criteria is met by the RC controller to continue walking or turning in a certain direction
int criteria(int num) {
  int criteria;
  switch (num) {
    case 0:                                              // Walking forward
      criteria = abs(chnVal[0]) < 6 && chnVal[1] > 7;  // Right joystick pushed forward
      break;
    case 1:                                               // Walking backwards=
      criteria = abs(chnVal[0]) < 6 && chnVal[1] < -7;  // Right joystick pushed backwards
      break;
    case 2:                                              // Walking right
      criteria = chnVal[0] > 7 && abs(chnVal[1]) < 6;  // Right joystick pushed backwards
      break;
    case 3:                                               // Walking left
      criteria = chnVal[0] < -7 && abs(chnVal[1]) < 6;  // Right joystick pushed backwards
      break;
    case 4:                                       // Walking forward right
      criteria = chnVal[0] > 7 && chnVal[1] > 7;  // Right joystick pushed to the top right corner
      break;
    case 5:                                        // Walking back left
      criteria = chnVal[0] > 7 && chnVal[1] < -7;  // Right joystick pushed to bottom right corner
      break;
    case 6:                                        // Walking forward left
      criteria = chnVal[0] < -7 && chnVal[1] > 7;  // Right joystick pushed to top left corner
      break;
    case 7:                                         // Walking back left
      criteria = chnVal[0] < -7 && chnVal[1] < -7;  // Right joystick pushed to bottom left corner
      break;
    case 8:                        // Turn CW
      criteria = chnVal[3] > 7;  // Left joystick pushed right
      break;
    case 9:                         // Turn CCW
      criteria = chnVal[3] < -7;  // Left joystick pushed left
      break;
  }
  return criteria;
}

//--------------------***User Input Functions***--------------------
// Get RC channel values and delegate tasks
void getReceiver() {
  // Is the controller on? The pulse width of channel 3 happens to be less than 1000 when the controller is disconnected
  if (width[2] > 1000) {
    // Add to the total button time if channel 6 (push button) is being pressed, otherwise set button time to 0 by subtracting its value from itself
    btnTime += chnVal[5] ? millis() - loopTimeI : -btnTime;
    loopTimeI = millis();  // Set initial loop time for the next loop

    // If channel 6 (push button) has been pressed for at least 1 seconds
    if (btnTime >= 1000) {
      btnTime = 0;  // Set button time to 0
      switch (state) {
        case 0:  // State = not homed
          home();
          state = 1;  // State is now 1 (starting)
          break;
        case 1:  // State = starting
          // Move the robot to standing
          moveFeet(xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, LINEAR, 500);
          state = 3;  // State is now 3 (standing)
          break;
        case 3:  // State = standing
          // Move the robot the starting
          moveFeet(xStart, yStart, zStart, xStart, yStart, zStart, xStart, yStart, zStart, xStart, yStart, zStart, LINEAR, 500);
          state = 1;  // State is now 1 (starting)
          break;
      }
    }
    // If state is 1 (starting) and channel 5 (right switch) is pulled back
    if (state == 1 && chnVal[4] == -1) {
      for (int i = 0; i < 12; i++) setState(i, 0);  // Enter IDLE Mode
      state = 2;                                    // State is now 2 (IDLE)
    }
    // If state is 2 (IDLE) and channel 5 (right switch) is pulled forward
    else if (state == 2 && chnVal[4] == 1) {
      for (int i = 0; i < 12; i++) setState(i, 1);                                                                             // Enter CLOSED LOOP CONTROL
      moveFeet(xStart, yStart, zStart, xStart, yStart, zStart, xStart, yStart, zStart, xStart, yStart, zStart, LINEAR, 1000);  // Move the robot the starting position
      state = 1;                                                                                                               // State is now 1 (start position)
    }
    // If state is 3 (standing)
    if (state == 3) {
      if (chnVal[4] == 0 && zNom == zLow) {  // Channel 5 (right switch) is in the middle
        zNom = zDefault;                     // Make Z height zDefault again

        // Not needed when using getShifts()
        zHome[0] = zNom;
        zHome[1] = zNom;
        zHome[2] = zNom;
        zHome[3] = zNom;

        moveFeet(xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, LINEAR, 500);  // Move feet to the new Z height
      }
      if (zNom == zDefault) {  // If Z height is zDefault
        if (chnVal[4] == 1) {  // Channel 5 (right switch) is pulled forward
          stepInPlace(100, 50);
        }
        if (chnVal[4] == -1) {  // Channel 5 (right switch) is pulled backwards
          zNom = zLow;          // Reduce Z height

          // Not needed when using getShifts()
          zHome[0] = zNom;
          zHome[1] = zNom;
          zHome[2] = zNom;
          zHome[3] = zNom;

          moveFeet(xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, LINEAR, 500);
        }
        if (chnVal[6] == 1 && chnVal[5] == 1) {  // Channel 7 (left switch is pulled forward) and  Channel 6 (button) is pressed
          kinematicsDemo();
        }
        if (chnVal[6] == -1 && chnVal[5] == 1) {  // Channel 7 (left switch is pushed back) and  Channel 6 (button) is pressed
          jump(1);
        }
        if (chnVal[2] == 10) {  // If channel 3 (left joystick) is pushed forward tilt in roll
          tiltRobot(15, 0, 0, CUBIC_OUT, 500);
          tiltRobot(-15, 0, 0, CUBIC_OUT, 1000);
          tiltRobot(0, 0, 0, CUBIC_OUT, 500);
        }
        if (chnVal[2] == -10) {  // If channel 3 (left joystick) is pushed back tilt in pitch
          tiltRobot(0, 15, 0, CUBIC_OUT, 500);
          tiltRobot(0, -15, 0, CUBIC_OUT, 1000);
          tiltRobot(0, 0, 0, CUBIC_OUT, 500);
        }
      }
      if (zNom == zDefault) {                     // If Z height is default
        step(70, 90, 0, 0, 180, 180, 50, 0);      // Step forward
        step(70, 90, 180, 180, 0, 0, 50, 1);      // Step backwards
        step(50, 70, -90, 90, -90, 90, 50, 2);    // Step right
        step(50, 70, 90, -90, 90, -90, 50, 3);    // Step left
        step(50, 70, -45, 45, -135, 135, 50, 4);  // Step front right
        step(50, 70, -135, 135, -45, 45, 50, 5);  // Step back left
        step(50, 70, 45, -45, 135, -135, 50, 6);  // Step front left
        step(50, 70, 135, -135, 45, -45, 50, 7);  // Step back left
        step(70, 90, -45, 135, 135, -45, 75, 8);  // Spin CW
        step(70, 90, 135, -45, -45, 135, 75, 9);  // Spin CCW
      } else if (zNom == zLow) {                  // If Z height is low
        step(50, 70, 0, 0, 180, 180, 50, 0);      // Step forward
        step(50, 70, 180, 180, 0, 0, 50, 1);      // Step backwards
        step(50, 70, -90, 90, -90, 90, 50, 2);    // Step right
        step(50, 70, 90, -90, 90, -90, 50, 3);    // Step left
        step(50, 70, -45, 45, -135, 135, 50, 4);  // Step front right
        step(50, 70, -135, 135, -45, 45, 50, 5);  // Step back left
        step(50, 70, 45, -45, 135, -135, 50, 6);  // Step front left
        step(50, 70, 135, -135, 45, -45, 50, 7);  // Step back left
        step(50, 70, -45, 135, 135, -45, 75, 8);  // Spin CW
        step(50, 70, 135, -45, -45, 135, 75, 9);  // Spin CCW
      }
    }
  }
  // If the controller is disconnected
  else {
    Serial.println("DISCONNECTED");
  }
}
// (isr = interrupt service routine) Handles interrupts for all channels
void isrHandler() {
  // Get pulse width
  for (int i = 0; i < 8; i++) {
    // Get the start time if the pin is HIGH
    if (digitalRead(chnPin[i]) == HIGH && !start[i]) {
      start[i] = micros();
    }
    // Calculate pulse width if the pin is LOW
    else if (digitalRead(chnPin[i]) == LOW && start[i]) {
      width[i] = micros() - start[i];  // Calculate pulse width
      start[i] = 0;                    // Reset start time
    }
  }
  // Map and constrain the pulse widths to channel values
  for (int i = 0; i < 8; i++) {
    chnVal[i] = constrain(map(width[i], 1000 / sensitivity, 2000 * sensitivity, mapRange[i][0], mapRange[i][1]), constrainRange[i][0], constrainRange[i][1]);
  }
}
// Gets user input from the serial monitor when testing without the RC receiver
void getTestingInput() {
  int time, mode;
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'h':  // Home joints
        home();
        delay(500);
        moveFeet(xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, LINEAR, 500);  // Move the joints to their nominal positions
        state = 3;                                                                                      // Set state to standing
        break;
      case 'c':  // Enter CLOSED LOOP CONTROL Mode
        for (int i = 0; i < 12; i++) setState(i, 1);
        state = 3;  // Set state to standing
        break;
      case 'n':  // Move legs to nominal position
        moveFeet(xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, xNom, yNom, zNom, LINEAR, 1000);
        break;
      case 'i':  // Enter IDLE Mode
        for (int i = 0; i < 12; i++) setState(i, 0);
        state = 2;  // Set state to IDLE
        break;
      case 'q':  // Rotate while speaking
        for (int i = 0; i < 5; i++) {
          mode = CUBIC_OUT;
          time = 700;
          // Tilt in roll
          tiltRobot(15, 0, 0, mode, time);
          tiltRobot(-15, 0, 0, mode, time * 2);
          tiltRobot(0, 0, 0, mode, time);
          // Tilt in pitch
          tiltRobot(0, 15, 0, mode, time);
          tiltRobot(0, -15, 0, mode, time * 2);
          tiltRobot(0, 0, 0, mode, time);
          // Tilt in yaw
          tiltRobot(0, 0, 15, mode, time);
          tiltRobot(0, 0, -15, mode, time * 2);
          tiltRobot(0, 0, 0, mode, time);
          time = 250;
          // Tilt in pitch and roll
          tiltRobot(0, 10, 0, mode, time);
          tiltRobot(-10, 10, 0, mode, time);
          tiltRobot(10, 10, 0, mode, time * 2);
          tiltRobot(0, 0, 0, mode, time * 2);
          tiltRobot(0, -10, 0, mode, time);
          tiltRobot(-10, -10, 0, mode, time);
          tiltRobot(10, -10, 0, mode, time * 2);
          // Tilt in yaw, pitch, and roll
          tiltRobot(0, 0, 15, mode, time * 2);
          tiltRobot(0, 10, 15, mode, time);
          tiltRobot(0, -10, 15, mode, time * 2);
          tiltRobot(0, 0, 15, mode, time);
          tiltRobot(10, 0, 15, mode, time);
          tiltRobot(-10, 0, 15, mode, time * 2);
          tiltRobot(0, 0, 0, mode, time * 1.5);
          tiltRobot(0, 0, -15, mode, time * 2);
          tiltRobot(0, 10, -15, mode, time);
          tiltRobot(0, -10, -15, mode, time * 2);
          tiltRobot(0, 0, -15, mode, time);
          tiltRobot(10, 0, -15, mode, time);
          tiltRobot(-10, 0, -15, mode, time * 2);
          tiltRobot(0, 0, 0, mode, time * 1.5);
          // Circulate in roll and pitch
          time = 500;
          tiltRobot(0, 15, 0, mode, time);
          tiltRobot(-15, 0, 0, QUADRATIC_IN, time);
          tiltRobot(0, -15, 0, LINEAR, time);
          tiltRobot(15, 0, 0, QUADRATIC_IN, time);
          tiltRobot(0, 15, 0, LINEAR, time);
          tiltRobot(0, 0, 0, mode, time);
          tiltRobot(0, -15, 0, mode, time);
          tiltRobot(-15, 0, 0, QUADRATIC_IN, time);
          tiltRobot(0, 15, 0, LINEAR, time);
          tiltRobot(15, 0, 0, QUADRATIC_IN, time);
          tiltRobot(0, -15, 0, LINEAR, time);
          tiltRobot(0, 0, 0, mode, time);
        }
        break;
      case 'w':  // Thumbnail Pose 1
        tiltRobot(-10, 10, 15, LINEAR, 1000);
        break;
      case 'e':  // Thumbnail Pose 2
        tiltRobot(-10, -10, 15, LINEAR, 1000);
        break;
        jump(1);
        break;
    }
  }
}
