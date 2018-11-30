#include <M5Stack.h>
#include <WiFi.h>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

const char* ssid     = "<script>alert(0);</script>"; /*ここを書き換える*/
const char* password = "' -- OR 1"; /*ここを書き換える*/
const int port = 1234;
boolean isServer;
WiFiClient conn;

#if 1
#define ssid "nya"
#define password "nyannyan"
#endif
 
static void WiFiSetup() {
  M5.Lcd.println("press btn A or B");
  while (true) {
    if (M5.BtnA.wasPressed()) {
      isServer = true;
      Serial.println("btnA pressed; starting WiFi AP");
      WiFi.softAP(ssid, password);
      Serial.print("local IP address: ");
      Serial.println(WiFi.softAPIP());

      Serial.println("waiting for client...");
      isServer = true;
      WiFiServer server(port);
      server.begin();
      while (!(conn = server.available())) {
        Serial.print(".");
        delay(100);
      }
      server.end();
      Serial.println("connected");
      break;
    }
    if (M5.BtnB.wasPressed()) {
      Serial.println("btnB pressed; connecting to WiFi AP");
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("connected");
      Serial.print("local IP address: ");
      Serial.println(WiFi.localIP());
      Serial.print("gateway IP address: ");
      Serial.println(WiFi.gatewayIP());

      Serial.println("connecting to server...");
      isServer = false;
      // Assuming the gateway is the other device
      while (!conn.connect("192.168.43.22", port)) {
      // while (!conn.connect(WiFi.gatewayIP(), port)) {
        Serial.print(".");
        delay(100);
      }
      Serial.println("connected");
      break;
    }
    delay(100);
    M5.update();
  }
  M5.Lcd.clear();
  M5.update();
}

// An event from the other device would be one of:
//  0. The other device started the game
//     * (none)
//  0. The other device hit the ball
//     * x, dx, dy
//  1. The other device failed to hit the ball
//     * (none)
//  2. The other device moved the paddle
//     * x
struct StartEvent { uint8_t type; };
struct HitEvent { uint8_t type; uint32_t x; float dx, dy; };
struct MissHitEvent { uint8_t type; };
struct MoveEvent { uint8_t type; uint32_t x; };
#define EventMaxSize 16

static void notifyStartGame() {
  assert(isServer);
  union { uint8_t buf[EventMaxSize]; struct StartEvent e; } buf;
  memset(&buf, 0, sizeof(buf));
  buf.e.type = 0;
  conn.write(buf.buf, sizeof(buf.buf));
}

static void notifyPaddleHit(uint32_t x, float dx, float dy) {
  union { uint8_t buf[EventMaxSize]; struct HitEvent e; } buf;
  memset(&buf, 0, sizeof(buf));
  buf.e.type = 1;
  buf.e.x = x;
  buf.e.dx = dx;
  buf.e.dy = dy;
  conn.write(buf.buf, sizeof(buf.buf));
}

static void notifyPaddleMissHit() {
  union { uint8_t buf[EventMaxSize]; struct MissHitEvent e; } buf;
  memset(&buf, 0, sizeof(buf));
  buf.e.type = 2;
  conn.write(buf.buf, sizeof(buf.buf));
}

static void notifyPaddleMove(uint32_t x) {
  union { uint8_t buf[EventMaxSize]; struct MoveEvent e; } buf;
  memset(&buf, 0, sizeof(buf));
  buf.e.type = 3;
  buf.e.x = x;
  conn.write(buf.buf, sizeof(buf.buf));
}

static void WiFiLoop(void *arg) {
  size_t len = 0;
  union {
    uint8_t buf[EventMaxSize];
    struct { uint8_t type; } x;
    struct HitEvent hit;
    struct MissHitEvent missHit;
    struct MoveEvent move;
  } buf;
  _Static_assert(sizeof(buf) == sizeof(buf.buf), "x");

  while (true) {
    int ret = conn.read(&buf.buf[len], sizeof(buf.buf) - len);
    if (ret < 0) {
      Serial.println("read error; aborting");
      break;
    }
    len += ret;
    if (len == sizeof buf) {
      len = 0;
      switch (buf.x.type) {
        case 0: // Start game
          assert(!isServer);
          // FIXME: The ball starts moving on the screen (towards the opponent's side)
          // Reset the ball location to (50%, 50%), ...
          break;
        case 1: // Hit
          // FIXME: Reset the ball location information to (x, 0)
          // buf.hit.{x,dx,dy}
          break;
        case 2: // MissHit
          // FIXME: Show the "WON" screen
          break;
        case 3: // Move
          // FIXME: opponentPaddlePosition = buf.move.x;
          break;
        default:
          Serial.println("unexpected event");
      }
    }
  }
  conn.stop();
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

        notifyPaddleHit(1, IMU.ax, IMU.ay);
      }

      IMU.count = millis();
    } // if (IMU.delt_t > 500)
  }
}

void setup() {
  M5.begin();
  Serial.begin(115200);
  delay(1);
 
  MPU9250setup();
  // Become a WiFi AP or client, and then connect to the other device
  WiFiSetup();

  xTaskCreatePinnedToCore(MPU9250loop, "MPU9250", 0x10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(WiFiLoop, "WiFiLoop", 0x10000, NULL, 1, NULL, 0);
  delay(100);
}

void loop() { }
