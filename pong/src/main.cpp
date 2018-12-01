#include <M5Stack.h>

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

// prototype declaration
void clean(bool buf);
void drawBall(int x, int y);
void drawPaddle(int x, int y);
void flush();
void gameInit();
void gameLoop();
void swapBuffer();
GameObject collideWall(GameObject object);
bool aabb(GameObject o1, GameObject o2);

void setup() {
    M5.begin();
    gameInit();
}

void loop() {
    gameLoop();
    M5.update();
}

void gameInit(){
    for(int i = 0; i < 2; i++){
        clean((bool)i);
    }
    paddle[0].x = paddle[1].x = 95;
    paddle[0].y = 5;
    paddle[1].y = 310;
    paddle[0].width = paddle[1].width = PADDLE_WIDTH;
    paddle[0].height = paddle[1].height = PADDLE_HEIGHT;

    ball.x = ball.y = 50;
    ball.vx = ball.vy = 1;
    ball.width = ball.height = BALL_RADIUS * 2;
}

void gameLoop(){
    // update
    // move paddle0
    if(M5.BtnC.isPressed())paddle[1].x--;
    if(M5.BtnA.isPressed())paddle[1].x++;
    // move paddle1
    // move ball
    // collision
    // paddle and ball
    if(aabb(paddle[1], ball)){
        ball.vy = -1;
    }
    // wall
    ball = collideWall(ball);
    for(int i = 0; i < 2; i++){
        paddle[i] = collideWall(paddle[i]);
    }
    ball.x += ball.vx;
    ball.y += ball.vy;

    // draw
    clean(bufFlag);
    drawBall(ball.x, ball.y);
    for(int i = 0; i < 2; i++){
        drawPaddle(paddle[i].x, paddle[i].y);
    }
    flush();
    swapBuffer();
}

bool aabb(GameObject o1, GameObject o2){
    return o1.x + o1.width > o2.x &&
        o2.x + o2.width > o1.x &&
        o1.y + o1.height > o2.y &&
        o2.y + o2.height > o1.y;
}
GameObject collideWall(GameObject object){
    if (object.x < 0){
        object.x = 0;
        object.vx = -object.vx;
    }
    if (object.x > SCREEN_WIDTH - object.width){
        object.x = SCREEN_WIDTH - object.width;
        object.vx = -object.vx;
    }
    if (object.y < 0){
        object.y = 0;
        object.vy = -object.vy;
    }
    if (object.y > SCREEN_HEIGHT - object.height){
        object.y = SCREEN_HEIGHT - object.height;
        object.vy = -object.vy;
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