#include <M5Stack.h>
#include <WiFi.h>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

//prototype
void setOtherPaddleState(int);
void setBallState(int, int, int, int);
void winTheGame();


boolean isServer;
boolean isStartPlayer;
WiFiClient conn;

#if 1
#define port 1234
#define ssid "nya"
#define password "nyannyan"
#endif

static void startAP() {
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
}

static void connectAP() {
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
}

static void WiFiSetup() {
  M5.Lcd.println("press btn A or B");
  while (true) {
    if (M5.BtnA.wasPressed()) {
      startAP();
      break;
    }
    if (M5.BtnB.wasPressed()) {
      connectAP();
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
struct HitEvent { evtype_t type; uint16_t x, y; float dx, dy; };
struct MissHitEvent { evtype_t type; };
struct MoveEvent { evtype_t type; uint32_t x; };
#define EventMaxSize 16

static void notifyStartGame() {
  union { uint8_t buf[EventMaxSize]; struct StartEvent e; } buf;
  memset(&buf, 0, sizeof(buf));
  buf.e.type = START;
  conn.write(buf.buf, sizeof(buf.buf));
  isStartPlayer = true;
}

static void notifyPaddleHit(uint16_t x, uint16_t y, float dx, float dy) {
  union { uint8_t buf[EventMaxSize]; struct HitEvent e; } buf;
  memset(&buf, 0, sizeof(buf));
  buf.e.type = HIT;
  buf.e.x = x;
  buf.e.y = y;
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
        // FIXME: The ball starts moving on the screen (towards the opponent's side)
        // Reset the ball location to (50%, 50%), ...
        Serial.println("start");
        isStartPlayer = false;
        break;
      case HIT: // Hit
        // FIXME: Reset the ball location information to (x, 0)
        // buf.hit.{x,dx,dy}
        Serial.printf("hit: %d, %d, %f, %f\n", buf.hit.x, buf.hit.y, buf.hit.dx, buf.hit.dy);
        setBallState(buf.hit.x, buf.hit.y, buf.hit.dx, buf.hit.dy);
        break;
      case MISSHIT: // MissHit
        // FIXME: Show the "WON" screen
        Serial.println("misshit");
        winTheGame();
        break;
      case MOVE: // Move
        // FIXME: opponentPaddlePosition = buf.move.x;
        Serial.printf("move: %d\n", buf.move.x);
        setOtherPaddleState(buf.move.x);
        break;
      default:
        Serial.println("unexpected event");
    }
  }
  conn.stop();
}


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

void getSensorValue(void *arg) {
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
}

// macro for drawing picture
#define get(b, i) !!((b) & (1 << (i))) 
#define dotget(b, x, y) (get(graphicBuffer[(b)][((y) * SCREEN_HEIGHT + (x)) / 8], (x) % 8))
#define set(b, i, s) ((b) = ((b) & ~(1 << (i))) | ((s) << (i)))
#define dotset(b, x, y, s) if((x) >= 0 && (y) >= 0 && (x) < SCREEN_HEIGHT && (y) < SCREEN_WIDTH ){(set(graphicBuffer[(b)][((y) * SCREEN_HEIGHT + (x)) / 8], (x) % 8, (s)));}

// struct
struct GameObject{
    int x;
    int y;
    int vx;
    int vy;
    int width;
    int height;
};

// constant
const int SCREEN_HEIGHT = 320;
const int SCREEN_WIDTH = 240;
const int BALL_RADIUS = 5;
const int PADDLE_WIDTH = 50;
const int PADDLE_HEIGHT = 5;


// graphic buffer
uint8_t graphicBuffer[2][SCREEN_WIDTH * SCREEN_HEIGHT / 8];
bool bufFlag = false;

// game object
GameObject ball, paddle[2];
int otherPaddleX;

int score[2];

// prototype declaration
void clean(bool buf);
void drawBall(int x, int y);
void drawPaddle(int x, int y);
void flush();
void gameInit();
void gameLoop(void *arg);
void swapBuffer();
GameObject collideWall(GameObject object, bool reflect);
bool aabb(GameObject o1, GameObject o2);
void resetGame();

void setup() {
    M5.begin();
    Serial.begin(115200);

    MPU9250setup();
    // Become a WiFi AP or client, and then connect to the other device
    WiFiSetup();

    if(isServer) isStartPlayer = true;

    gameInit();

    xTaskCreate(WiFiLoop, "WiFiLoop", 0x10000, NULL, 1, NULL);
    TimerHandle_t x = xTimerCreate("Timer", ( 20 / portTICK_PERIOD_MS ), pdTRUE, NULL, gameLoop);
    xTimerStart( x, 0 );

}

void loop() {
    //gameLoop();
    //M5.update();
}

void gameInit(){
    for(int i = 0; i < 2; i++){
        clean((bool)i);
    }
    paddle[0].width = paddle[1].width = PADDLE_WIDTH;
    paddle[0].height = paddle[1].height = PADDLE_HEIGHT;

    ball.width = ball.height = BALL_RADIUS * 2;
    resetGame();

    score[0] = score[1] = 0;
}

// 相手が跳ね返したボールの状態を設定するときに呼ぶ
void setBallState(int x, int y, int vx, int vy){
    ball.x = SCREEN_WIDTH - BALL_RADIUS * 2 - x;
    ball.y = SCREEN_HEIGHT - BALL_RADIUS * 2 - y;
    ball.vx = -vx;
    ball.vy = -vy;
}

// 相手のパドルの位置を指定するときにこれを呼ぶ
void setOtherPaddleState(int x){
    int new_x = SCREEN_WIDTH - PADDLE_WIDTH - x;
    int dir = new_x - otherPaddleX;

    if(dir < 0){
        paddle[1].vx = -2;
    }else if(dir > 0){
        paddle[1].vx = 2;
    }else{
        paddle[1].vx = 0;
    }
    paddle[1].x = new_x;
    otherPaddleX = new_x;
}

// 相手のゴールに入ったときにこれを呼ぶ
void winTheGame(){
    score[0]++;
    resetGame();
}

// 加速度センサーの値を取得
void setAccelX(int x){
    paddle[0].x = x;
}

void resetGame(){
    ball.x = 110;
    ball.y = 150;
    otherPaddleX = paddle[0].x = paddle[1].x = 95;
    paddle[0].y = 310;
    paddle[1].y = 5;
    ball.vx = ball.vy = (isStartPlayer?1:-1) * 2;
}

int cnt = 0;
void gameLoop(void *arg){
    M5.update();
    // update
    // move paddle0
    if(M5.BtnC.isPressed())paddle[0].x-=2;
    if(M5.BtnA.isPressed())paddle[0].x+=2;
    if((cnt++ % 5) == 0)  notifyPaddleMove(paddle[0].x);

    // move paddle1
    paddle[1].x += paddle[1].vx;
    paddle[1].y += paddle[1].vy;

    // move ball
    ball.x += ball.vx;
    ball.y += ball.vy;

    // collision
    // paddle and ball
    for(int i = 0; i < 2; i++){
        if (aabb(paddle[i], ball)){
            ball.vy = 2 * ((i == 0) ? -1 : 1);
            ball.vx = -((paddle[i].x + paddle[i].width / 2) - (ball.x + ball.width / 2)) / 10;
            if(i == 0) notifyPaddleHit(ball.x, ball.y, ball.vx, ball.vy);
        }
    }

    // wall
    ball = collideWall(ball, true);
    for(int i = 0; i < 2; i++){
        paddle[i] = collideWall(paddle[i], false);
    }

    // goal judge
    if(ball.y > 305){
        score[1]++;
        notifyPaddleMissHit();
        resetGame();
    }

    // draw
    clean(bufFlag);
    drawBall(ball.x, ball.y);
    for(int i = 0; i < 2; i++){
        drawPaddle(paddle[i].x, paddle[i].y);
    }
    flush();
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("%d, %d", score[0], score[1]);
    swapBuffer();

}

bool aabb(GameObject o1, GameObject o2){
    return o1.x + o1.width > o2.x &&
        o2.x + o2.width > o1.x &&
        o1.y + o1.height > o2.y &&
        o2.y + o2.height > o1.y;
}
GameObject collideWall(GameObject object, bool reflect){
    if (object.x < 0){
        object.x = 0;
        if (reflect) object.vx = -object.vx;
    }
    if (object.x > SCREEN_WIDTH - object.width){
        object.x = SCREEN_WIDTH - object.width;
        if (reflect) object.vx = -object.vx;
    }
    if (object.y < 0){
        object.y = 0;
        if (reflect) object.vy = -object.vy;
    }
    if (object.y > SCREEN_HEIGHT - object.height){
        object.y = SCREEN_HEIGHT - object.height;
        if (reflect) object.vy = -object.vy;
    }
    return object;
}

void clean(bool buf){
    for (int j = 0; j < SCREEN_WIDTH * SCREEN_HEIGHT / 8; j++){
        graphicBuffer[buf][j] = 0;
    }
}
void drawBall(int x, int y){
    for(int i = 0; i < 20; i++){
        dotset(bufFlag, (int)(y + BALL_RADIUS + cos(PI / 10 * i) * BALL_RADIUS + 0.5), (int)(x + BALL_RADIUS + sin(PI / 10 * i) * BALL_RADIUS + 0.5), 1);
    }
}

void drawPaddle(int x, int y){
    for(int i = 0; i < PADDLE_WIDTH; i++){
        for (int j = 0; j < PADDLE_HEIGHT; j++){
            dotset(bufFlag, y + j, x + i, 1);
        }
    }
}

void flush(){
    for(int y = 0; y < SCREEN_WIDTH; y++){
        for (int x = 0; x < SCREEN_HEIGHT; x++){
            if(dotget(0, x, y) ^ dotget(1, x, y)){
                M5.Lcd.drawPixel(x, y, 0xFFFF * (dotget(0, x, y) ^ bufFlag));
            }
        }
    }
}

void swapBuffer(){
    bufFlag = !bufFlag;
}