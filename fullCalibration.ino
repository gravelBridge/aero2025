void fullCalibration(){
  //check gyro
  checkGyro();

  //confirm GPS is within range of current coordinates
  checkGPS();

  //velocity in all directions should be 0
  checkVelocity();

  //move each servo to max and min rotations
  checkServos();

  //battery level over a certain value
  checkBattery();

  //set barometer

}