#include <M5Stack.h>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"


#define SerialDebug true  // Set to true to get Serial output for debugging

MPU9250 IMU;
// Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances

void MPU9250setup() {
  Wire.begin();
  delay(1);
  IMU.initMPU9250();
  delay(1);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  Serial.println("MPU9250 is online...");

  // Start by performing self test and reporting values
  IMU.MPU9250SelfTest(IMU.SelfTest);
  Serial.print("x-axis self test: acceleration trim within : ");
  Serial.print(IMU.SelfTest[0],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: acceleration trim within : ");
  Serial.print(IMU.SelfTest[1],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: acceleration trim within : ");
  Serial.print(IMU.SelfTest[2],1); Serial.println("% of factory value");
  Serial.print("x-axis self test: gyration trim within : ");
  Serial.print(IMU.SelfTest[3],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: gyration trim within : ");
  Serial.print(IMU.SelfTest[4],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: gyration trim within : ");
  Serial.print(IMU.SelfTest[5],1); Serial.println("% of factory value");

  // Calibrate gyro and accelerometers, load biases in bias registers
  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);

  // Initialize device for active mode read of acclerometer, gyroscope, and
  // temperature
  Serial.println("MPU9250 initialized for active data mode....");
}

void MPU9250loop(void *arg)
{
  for(;;) {
    // If intPin goes high, all data registers have new data
    // On interrupt, check if data ready interrupt
    if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {  
      IMU.readAccelData(IMU.accelCount);  // Read the x/y/z adc values
      IMU.getAres();

      // Now we'll calculate the accleration value into actual g's
      // This depends on scale being set
      IMU.ax = (float)IMU.accelCount[0]*IMU.aRes; // - accelBias[0];
      IMU.ay = (float)IMU.accelCount[1]*IMU.aRes; // - accelBias[1];
      IMU.az = (float)IMU.accelCount[2]*IMU.aRes; // - accelBias[2];

      IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
      IMU.getGres();

      // Calculate the gyro value into actual degrees per second
      // This depends on scale being set
      IMU.gx = (float)IMU.gyroCount[0]*IMU.gRes;
      IMU.gy = (float)IMU.gyroCount[1]*IMU.gRes;
      IMU.gz = (float)IMU.gyroCount[2]*IMU.gRes;

    } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

    // Must be called before updating quaternions!
    IMU.updateTime();

    IMU.delt_t = millis() - IMU.count;
    if (IMU.delt_t > 5000)
    {
      if(SerialDebug)
      {
        // Print acceleration values in milligs!
        Serial.print("X-acceleration: "); Serial.print(1000*IMU.ax);
        Serial.print(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(1000*IMU.ay);
        Serial.print(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(1000*IMU.az);
        Serial.println(" mg ");

        // Print gyro values in degree/sec
        Serial.print("X-gyro rate: "); Serial.print(IMU.gx, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Y-gyro rate: "); Serial.print(IMU.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(IMU.gz, 3);
        Serial.println(" degrees/sec");
      }

      IMU.count = millis();
    } // if (IMU.delt_t > 500)
  }
}

void vTimerCallback( TimerHandle_t pxTimer ) {
  Serial.println("timer expired");
}

void setup() {
  M5.begin();
  delay(1);
 
  MPU9250setup();
  Serial.begin(115200);
  xTaskCreatePinnedToCore(MPU9250loop, "MPU9250", 0x10000, NULL, 1, NULL, 0);
  TimerHandle_t x = xTimerCreate("Timer", ( 1000 / portTICK_PERIOD_MS ), pdTRUE, NULL, vTimerCallback);
  xTimerStart( x, 0 );
  delay(100);
}
 
void loop() {

  M5.update();
}