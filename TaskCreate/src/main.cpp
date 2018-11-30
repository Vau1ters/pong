#include <M5Stack.h>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

#include <WiFi.h> 
const char* ssid     = "<script>alert(0);</script>"; /*ここを書き換える*/
const char* password = "' -- OR 1"; /*ここを書き換える*/
 
WiFiServer server(80);
 
void WifiServersetup()
{
    //WiFi.begin(ssid, password);
    WiFi.softAP(ssid, password);
    //while (WiFi.status() != WL_CONNECTED) {
    //    delay(500);
    //    Serial.println(".");
    //}
    Serial.println("connected");
    Serial.println(WiFi.softAPIP());
    delay(100);
    M5.Lcd.println(WiFi.softAPIP());
    delay(100);
    server.begin();
    delay(100);
    Serial.println("server begin");
    delay(100);
}
 
void WifiServerloop(void *arg) {

for(;;) {
  Serial.println("webserverloop");
  delay(10);
  WiFiClient client = server.available();   // listen for incoming clients
    if (client) {                             // if you get a client,
    String currentLine = "";                // make a String to hold incoming data from the client
      while (client.connected()) {            // loop while the client's connected
        if (client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          if (c == '\n') {                    // if the byte is a newline character
            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            if (currentLine.length() == 0) {
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println();
              // the content of the HTTP response follows the header:
              client.print("Click <a href=\"/H\">here</a> to turn the LCD GREEN.<br>");
              client.print("Click <a href=\"/L\">here</a> to turn the LCD RED.<br>");
              // The HTTP response ends with another blank line:
              client.println();
              // break out of the while loop:
              break;
            } else {    // if you got a newline, then clear currentLine:
              currentLine = "";
            }
          } else if (c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }
          // Check to see if the client request was "GET /H" or "GET /L":
          if (currentLine.endsWith("GET /H")) {
            M5.Lcd.fillScreen(GREEN);              // GET /H turns the LCD GREEN
          }
          if (currentLine.endsWith("GET /L")) {
            M5.Lcd.fillScreen(RED);                // GET /L turns the LCD RED
          }
        }
      }
      client.stop();
    }
    M5.update();
  }
}

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
    if (IMU.delt_t > 500)
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

void Mainloop(void *arg) {
  for(;;) {
    static int cnt = 0;
    Serial.printf("Maintask thread_cnt=%ld\n", cnt++);
    delay(1200);
  }
}


void setup() {
  M5.begin();
  delay(1);
 
  MPU9250setup();
  WifiServersetup();

  Serial.begin(115200);
  xTaskCreatePinnedToCore(MPU9250loop, "MPU9250", 0x10000, NULL, 1, NULL, 0);
  delay(100);
}
 
void loop() {

WiFiClient client = server.available();   // listen for incoming clients
  if (client) {                             // if you get a client,
  String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:application/json");
            client.println();
            // the content of the HTTP response follows the header:
            client.printf("{\"ax\":%f, \"ay\":%f, \"az\":%f, \"gx\":%f, \"gy\":%f, \"gz\":%f}\n", IMU.ax,IMU.ay,IMU.az,IMU.gx,IMU.gy,IMU.gz);
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    client.stop();
  }
  M5.update();
}