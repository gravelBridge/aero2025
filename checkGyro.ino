void checkGyro(){
  bool isGyroReady(int numSamples = 100, float maxDrift = 0.1) {
  Serial.println("Checking gyroscope stability...");

  float sumX = 0, sumY = 0, sumZ = 0;
  float avgX, avgY, avgZ;

  // Collect multiple samples
  for (int i = 0; i < numSamples; i++) {
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      
    } else {
      continue;
    }
    delay(5);  // Small delay between samples
  }

  // Calculate average angular velocity
  avgX = sumX / numSamples;
  avgY = sumY / numSamples;
  avgZ = sumZ / numSamples;

  // Check if the gyro is stable
  if (abs(avgX) < maxDrift && abs(avgY) < maxDrift && abs(avgZ) < maxDrift) {
    Serial.println("✅ Gyro is stable and ready for launch.");
    Serial.print("Avg X: "); Serial.print(avgX);
    Serial.print(" | Avg Y: "); Serial.print(avgY);
    Serial.print(" | Avg Z: "); Serial.println(avgZ);
    return true;
  } else {
    Serial.println("❌ Gyro is NOT stable. Wait for stability...");
    Serial.print("Avg X: "); Serial.print(avgX);
    Serial.print(" | Avg Y: "); Serial.print(avgY);
    Serial.print(" | Avg Z: "); Serial.println(avgZ);
    return false;
  }
}

}