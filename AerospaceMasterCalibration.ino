#include "Arduino_BMI270_BMM150.h"
#include "SensorFusion.h"
#include <Servo.h>
#include <math.h>

// ----------------------
// Sensor Fusion & IMU Setup
// ----------------------
SF fusion;
float ax, ay, az, gx, gy, gz;
float mx, my, mz;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
float deltat;

// Expected magnetic field strength (uT)
float M_scale = 27.4584;
// Hard‑iron offset
float M_B[3] = {-27.53679457, -0.2111695583, -27.52377034};
// Soft‑iron scale correction
float M_Ainv[3][3] = {
  {  1.0080,  -0.0186, 0.0695},
  {  -0.0186,  0.9447, 0.0119},
  { 0.0695, 0.0119, 1.0555}
};

// ----------------------
// PID Controller Parameters
// ----------------------

// For pitch and roll, we want the rocket to hold 0° attitude.
float Kp_pitch = 1.0, Ki_pitch = 0.0, Kd_pitch = 0.2;
float Kp_roll  = 1.0, Ki_roll  = 0.0, Kd_roll  = 0.2;

// For yaw, we aim to drive the yaw angular velocity (computed as the derivative
// of the sensor fusion’s yaw) to 0.
float Kp_yaw = 1.0, Ki_yaw = 0.0, Kd_yaw = 0.2;

// PID state variables
float pitch_integral = 0, pitch_prev_error = 0;
float roll_integral = 0, roll_prev_error = 0;
float yaw_integral = 0, yaw_prev_error = 0;

// Setpoints: 
// - Pitch and roll angles should be 0° (pointing straight up)
// - Yaw rate should be 0 (i.e. no angular velocity around yaw)
float desired_pitch = 0.0;
float desired_roll = 0.0;
float desired_yaw_rate = 0.0;

// ----------------------
// Attitude Offsets (to zero the sensor fusion at startup)
// ----------------------
float pitch_offset = 0, roll_offset = 0, yaw_offset = 0;

// ----------------------
// Additional variable for yaw derivative calculation
// ----------------------
float yaw_prev = 0;

// ----------------------
// Servo Definitions for Canards
// ----------------------
// Pitch control surfaces: digital pins 2 and 4
// Roll control surfaces:  digital pins 3 and 5
// All surfaces also contribute to yaw correction.
Servo servo_pitch1; // Pin 2 (e.g., left side)
Servo servo_roll1;  // Pin 3 (e.g., left side)
Servo servo_pitch2; // Pin 4 (e.g., right side)
Servo servo_roll2;  // Pin 5 (e.g., right side)

void setup() {
  Serial.begin(9600);
  // Do not wait for a USB connection so that the code runs when powered directly

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
//gyroSetup
  calibrateGyro();
  Serial.println("Started");

  // Attach servos
  servo_pitch1.attach(2);
  servo_roll1.attach(3);
  servo_pitch2.attach(4);
  servo_roll2.attach(5);
  
  // First, set all servos to angle 0 for 4 seconds
  servo_pitch1.write(0);
  servo_roll1.write(0);
  servo_pitch2.write(0);
  servo_roll2.write(0);
  delay(4000);
  
  // Now set servos to neutral (90°) before starting the PID control loop
  servo_pitch1.write(90);
  servo_roll1.write(90);
  servo_pitch2.write(90);
  servo_roll2.write(90);

  // ----------------------
  // Zero-out sensor fusion offsets by averaging a number of samples
  // ----------------------
  float sum_pitch = 0, sum_roll = 0, sum_yaw = 0;
  const int samples = 100;
  for (int i = 0; i < samples; i++) {
    if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable() && IMU.magneticFieldAvailable()){
      IMU.readGyroscope(gx, gy, gz);
      IMU.readAcceleration(ax, ay, az);
      IMU.readMagneticField(mx, my, mz);
      gx -= gyroXOffset;
      gy -= gyroYOffset;
      gz -= gyroZOffset;
      updateMag(mx, my, mz);
      applyScalingFactor(mx, my, mz, M_scale);
      applyMagneticDeclination(mx, my, -0.19);
      ax *= 9.80665; ay *= 9.80665; az *= 9.80665;
      gx *= PI / 180.0; gy *= PI / 180.0; gz *= PI / 180.0;
      deltat = fusion.deltatUpdate();
      fusion.MadgwickUpdate(-gx, gy, gz, -ax, ay, az, mx, my, mz, deltat);
      sum_pitch += fusion.getPitch();
      sum_roll += fusion.getRoll();
      sum_yaw += fusion.getYaw();
      delay(10);
    }
  }
  pitch_offset = sum_pitch / samples;
  roll_offset = sum_roll / samples;
  yaw_offset = sum_yaw / samples;  // Although yaw isn't used directly for attitude control, we capture it

  // Initialize yaw_prev using the first fused yaw value (offset corrected)
  yaw_prev = fusion.getYaw() - yaw_offset;
//end of gyro setup

  fullCalibration();
}

void loop() {

  //gyro Loop
  // ----------------------
  // Sensor Acquisition
  // ----------------------
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);  // deg/s
  } else {
    return;
  }
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);  // g
  } else {
    return;
  }
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);  // uT
  } else {
    return;
  }

  // Calibrate gyroscope values
  gx -= gyroXOffset;
  gy -= gyroYOffset;
  gz -= gyroZOffset;  // Now gz is in deg/s

  // Process magnetometer data
  updateMag(mx, my, mz);
  applyScalingFactor(mx, my, mz, M_scale);
  applyMagneticDeclination(mx, my, -0.19);  // Magnetic declination for Kuala Lumpur

  // Convert acceleration to m/s² and gyroscope values to rad/s
  ax *= 9.80665;  ay *= 9.80665;  az *= 9.80665;
  gx *= PI / 180.0;  gy *= PI / 180.0;  gz *= PI / 180.0;

  // ----------------------
  // Sensor Fusion for Pitch, Roll & Yaw
  // ----------------------
  deltat = fusion.deltatUpdate();
  fusion.MadgwickUpdate(-gx, gy, gz, -ax, ay, az, mx, my, mz, deltat);
  
  // Subtract the initial offsets to "zero" the attitude
  float current_pitch = fusion.getPitch() - pitch_offset;
  float current_roll  = fusion.getRoll() - roll_offset;
  float current_yaw   = fusion.getYaw() - yaw_offset;  // Using sensor fusion yaw

  // ----------------------
  // PID for Pitch Control
  // ----------------------
  float error_pitch = desired_pitch - current_pitch;
  pitch_integral += error_pitch * deltat;
  float d_pitch = (error_pitch - pitch_prev_error) / deltat;
  float pitch_output = Kp_pitch * error_pitch + Ki_pitch * pitch_integral + Kd_pitch * d_pitch;
  pitch_prev_error = error_pitch;

  // ----------------------
  // PID for Roll Control
  // ----------------------
  float error_roll = desired_roll - current_roll;
  roll_integral += error_roll * deltat;
  float d_roll = (error_roll - roll_prev_error) / deltat;
  float roll_output = Kp_roll * error_roll + Ki_roll * roll_integral + Kd_roll * d_roll;
  roll_prev_error = error_roll;

  // ----------------------
  // Yaw Control Using the Derivative of Fused Yaw
  // ----------------------
  // Compute the yaw angular velocity from the derivative of the fused yaw value.
  float yaw_rate = (current_yaw - yaw_prev) / deltat;
  float error_yaw = desired_yaw_rate - yaw_rate; // desired yaw rate is 0
  yaw_integral += error_yaw * deltat;
  float d_yaw = (error_yaw - yaw_prev_error) / deltat;
  float yaw_output = Kp_yaw * error_yaw + Ki_yaw * yaw_integral + Kd_yaw * d_yaw;
  yaw_prev_error = error_yaw;
  yaw_prev = current_yaw;

  // ----------------------
  // Scale Down the Servo Slew (25% of computed deflection)
  // ----------------------
  float servo_scale = 0.25;

  // For pitch, apply corrections in opposite directions on the two sides:
  float servo_pitch_command_left  = 90 - (pitch_output * servo_scale) + (yaw_output * servo_scale);
  float servo_pitch_command_right = 90 + (pitch_output * servo_scale) + (yaw_output * servo_scale);
  
  // For roll, apply corrections in opposite directions on the two sides:
  float servo_roll_command_left  = 90 + (roll_output * servo_scale) + (yaw_output * servo_scale);
  float servo_roll_command_right = 90 - (roll_output * servo_scale) + (yaw_output * servo_scale);

  // Constrain commands to servo range (0° to 180°)
  servo_pitch_command_left  = constrain(servo_pitch_command_left, 0, 180);
  servo_pitch_command_right = constrain(servo_pitch_command_right, 0, 180);
  servo_roll_command_left   = constrain(servo_roll_command_left, 0, 180);
  servo_roll_command_right  = constrain(servo_roll_command_right, 0, 180);

  // Write commands to servos
  servo_pitch1.write((int)servo_pitch_command_left);
  servo_pitch2.write((int)servo_pitch_command_right);
  servo_roll1.write((int)servo_roll_command_left);
  servo_roll2.write((int)servo_roll_command_right);

  // Optional: output debugging information to Serial
  Serial.print("Pitch: "); Serial.print(current_pitch);
  Serial.print(" | Roll: "); Serial.print(current_roll);
  Serial.print(" | Yaw Rate: "); Serial.print(yaw_rate);
  Serial.print(" || Servo Pitch L: "); Serial.print(servo_pitch_command_left);
  Serial.print(" R: "); Serial.print(servo_pitch_command_right);
  Serial.print(" | Servo Roll L: "); Serial.print(servo_roll_command_left);
  Serial.print(" R: "); Serial.println(servo_roll_command_right);

  delay(10); // Adjust loop timing as needed
  //end of gyro loop


}