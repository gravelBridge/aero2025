void updateMag(float &mx, float &my, float &mz) {
  float temp[3];
  temp[0] = (mx - M_B[0]);
  temp[1] = (my - M_B[1]);
  temp[2] = (mz - M_B[2]);
  mx = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  my = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  mz = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2]; 
}

void applyScalingFactor(float &mx, float &my, float &mz, float expectedFieldStrength) {
  float currentMagnitude = sqrt(mx * mx + my * my + mz * mz);
  float scaleFactor = expectedFieldStrength / currentMagnitude;
  mx *= scaleFactor;
  my *= scaleFactor;
  mz *= scaleFactor;
}

void applyMagneticDeclination(float &mx, float &my, float declination_deg) {
  float declinationRad = declination_deg * (PI / 180.0);
  float mx_corrected = mx * cos(declinationRad) - my * sin(declinationRad);
  float my_corrected = mx * sin(declinationRad) + my * cos(declinationRad);
  mx = mx_corrected;
  my = my_corrected;
}

void calibrateGyro(){
  Serial.print("Calibrating gyroscope...");
  const int numSamples = 3000;
  float sumX = 0, sumY = 0, sumZ = 0;
  int actualSample = 0;
  for (int i = 0; i < numSamples; i++) {
    if (IMU.gyroscopeAvailable()) {  
      IMU.readGyroscope(gx, gy, gz);  // deg/s
    } else {
      continue;
    }
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(3);
    actualSample++;
  }
  gyroXOffset = sumX / actualSample;
  gyroYOffset = sumY / actualSample;
  gyroZOffset = sumZ / actualSample;
  Serial.println(" Finished");
  Serial.println(actualSample);
  Serial.println(gyroXOffset);
  Serial.println(gyroYOffset);
  Serial.println(gyroZOffset);
}
