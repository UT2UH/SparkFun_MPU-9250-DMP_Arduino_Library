/************************************************************
ICM20689_FIFO_AHRS
https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

This example sketch demonstrates how to use the ICM20689's
1K first-in, first-out (FIFO) buffer. The FIFO can be
set to store either accelerometer and/or gyroscope (not the
magnetometer, though :( ).

Development environment specifics:
Arduino IDE 1.6.12


Supported Platforms:

*************************************************************/
#include <SparkFunMPU9250-DMP.h>


MPU9250_DMP imu;

void setup() 
{
  Serial.begin(230400);

  // Call imu.begin() to verify communication with and
  // initialize the MPU-9250 to it's default values.
  // Most functions return an error code - INV_SUCCESS (0)
  // indicates the IMU was present and successfully set up
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }

    // Use setSensors to turn on or off MPU-9250 sensors.
  // Any of the following defines can be combined:
  // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
  // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
  // Enable all sensors:
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at 
  // +/- 4912 uT (micro-tesla's)

  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu.setSampleRate(4); // Set sample rate to 4Hz

  // setLPF() can be used to set the digital low-pass filter
  // of the gyroscope only!
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setGyroLPF(5); // Set LPF corner frequency to 5Hz

  // Use configureFifo to set which sensors should be stored
  // in the buffer.  
  // Parameter to this function can be: INV_XYZ_GYRO, 
  // INV_XYZ_ACCEL, INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
  imu.configureFifo(INV_XYZ_GYRO |INV_XYZ_ACCEL);
}

void loop() 
{
  // fifoAvailable returns the number of bytes in the FIFO
  // The FIFO is 512 bytes max. We'll read when it reaches
  // half of that.
  if ( imu.fifoAvailable() >= 256)
  {
    // Then read while there is data in the FIFO
    while ( imu.fifoAvailable() > 0)
    {
    // Call updateFifo to update ax, ay, az, gx, gy, and/or gz
      if ( imu.updateFifo() == INV_SUCCESS)
      {
        // computeEulerAngles can be used -- after updating the
        // quaternion values -- to estimate roll, pitch, and yaw
        imu.computeEulerAngles();
        printIMUData();
      }
    }
  }
}

void printIMUData(void)
{  
  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);

  float gyroX = imu.calcGyro(imu.gx);
  float gyroY = imu.calcGyro(imu.gy);
  float gyroZ = imu.calcGyro(imu.gz);
  
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);

  Serial.println("Accel: " + String(accelX) + ", " + String(accelY) + ", " + String(accelZ) + " g");
  Serial.println("Gyro: " + String(gyroX) + ", " + String(gyroY) + ", " + String(gyroZ) + " dps");
  Serial.println("Q: " + String(q0, 4) + ", " + String(q1, 4) + ", " + String(q2, 4) + ", " + String(q3, 4));
  Serial.println("R/P/Y: " + String(imu.roll) + ", " + String(imu.pitch) + ", " + String(imu.yaw));
  Serial.println("Time: " + String(imu.time) + " ms");
  Serial.println();
/*
Serial.printf("FLOAT:\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",
			  imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az),
			  imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz));
Serial.printf("Roll: %.2f\tPitch: %.2f\tYaw: %.2f\n",
			  imu.roll,
			  imu.pitch,
			  imu.yaw);
*/
}