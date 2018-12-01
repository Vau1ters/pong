#include <M5Stack.h>
#include <WiFi.h>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

#define error(...) (Serial.printf("%s:%d:%s:", __FILE__, __LINE__, __func__), Serial.printf(__VA_ARGS__), Serial.println(""))

const int port = 1234;
const char* ssid     = "<script>alert(0);</script>"; /*ここを書き換える*/
const char* password = "' -- OR 1"; /*ここを書き換える*/

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
      //while (!conn.connect("192.168.43.22", port)) {
      while (!conn.connect(WiFi.gatewayIP(), port)) {
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
typedef enum {
  START,
  HIT,
  MISSHIT,
  MOVE,
  EVENT_NUM
} evtype_t;

struct StartEvent { evtype_t type; };
struct HitEvent { evtype_t type; uint32_t x; float dx, dy; };
struct MissHitEvent { evtype_t type; };
struct MoveEvent { evtype_t type; uint32_t x; };
#define EventMaxSize 16

static void notifyStartGame() {
  assert(isServer);
  union { uint8_t buf[EventMaxSize]; struct StartEvent e; } buf;
  memset(&buf, 0, sizeof(buf));
  buf.e.type = START;
  conn.write(buf.buf, sizeof(buf.buf));
}

static void notifyPaddleHit(uint32_t x, float dx, float dy) {
  union { uint8_t buf[EventMaxSize]; struct HitEvent e; } buf;
  memset(&buf, 0, sizeof(buf));
  buf.e.type = HIT;
  buf.e.x = x;
  buf.e.dx = dx;
  buf.e.dy = dy;
  conn.write(buf.buf, sizeof(buf.buf));
}

static void notifyPaddleMissHit() {
  union { uint8_t buf[EventMaxSize]; struct MissHitEvent e; } buf;
  memset(&buf, 0, sizeof(buf));
  buf.e.type = MISSHIT;
  conn.write(buf.buf, sizeof(buf.buf));
}

static void notifyPaddleMove(uint32_t x) {
  union { uint8_t buf[EventMaxSize]; struct MoveEvent e; } buf;
  memset(&buf, 0, sizeof(buf));
  buf.e.type = MOVE;
  buf.e.x = x;
  conn.write(buf.buf, sizeof(buf.buf));
}

static int readbuf(uint8_t buf[EventMaxSize]) {
  int ret = 0;
  for(size_t len = 0; len < EventMaxSize; len += ret) {
    ret = conn.read(&buf[len], sizeof(uint8_t[EventMaxSize]) - len);
    if (ret < 0) {
      //error("read error; aborting\n");
      return -1;
    }
  }
  return EventMaxSize;
}

static void WiFiLoop(void *arg) {
  union {
    uint8_t buf[EventMaxSize];
    struct { evtype_t type; } x;
    struct StartEvent start;
    struct HitEvent hit;
    struct MissHitEvent missHit;
    struct MoveEvent move;
  } buf;
  _Static_assert(sizeof(buf) == sizeof(buf.buf), "x");

  while (true) {
    if(readbuf(buf.buf) == -1) continue;
    switch (buf.x.type) {
      case START: // Start game
        assert(!isServer);
        // FIXME: The ball starts moving on the screen (towards the opponent's side)
        // Reset the ball location to (50%, 50%), ...
        Serial.println("start");
        break;
      case HIT: // Hit
        // FIXME: Reset the ball location information to (x, 0)
        // buf.hit.{x,dx,dy}
        Serial.printf("hit: %d, %f, %f\n", buf.hit.x, buf.hit.dx, buf.hit.dy);
        break;
      case MISSHIT: // MissHit
        // FIXME: Show the "WON" screen
        Serial.println("misshit");
        break;
      case MOVE: // Move
        // FIXME: opponentPaddlePosition = buf.move.x;
        Serial.printf("move: %d\n", buf.move.x);
        break;
      default:
        Serial.println("unexpected event");
    }
  }
  conn.stop();
}

//#define SerialDebug

MPU9250 IMU;

void MPU9250setup() {
  Wire.begin();
  IMU.initMPU9250();

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

  Serial.println("MPU9250 initialized for active data mode....");
}

void MPU9250loop(void *arg) {
// If intPin goes high, all data registers have new data
// On interrupt, check if data ready interrupt
  if (!(IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)) return; 
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

#ifdef SerialDebug
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
#endif
  //notifyStartGame();
  notifyPaddleHit(1, IMU.ax, IMU.ay);
  notifyPaddleMissHit();
  notifyPaddleMove(2);
}

void setup() {
  M5.begin();
  Serial.begin(115200);
  delay(1);
 
  MPU9250setup();
  // Become a WiFi AP or client, and then connect to the other device
  WiFiSetup();

  xTaskCreate(WiFiLoop, "WiFiLoop", 0x10000, NULL, 1, NULL);
  TimerHandle_t x = xTimerCreate("Timer", ( 1000 / portTICK_PERIOD_MS ), pdTRUE, NULL, MPU9250loop);
  xTimerStart( x, 0 );
  delay(100);
}

void loop() { }
