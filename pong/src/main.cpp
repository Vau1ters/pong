#include <M5Stack.h>

// macro for drawing picture
#define get(b, i) !!((b) & (1 << (i))) 
#define dotget(b, x, y) (get(graphicBuffer[(b)][((y) * SCREEN_WIDTH + (x)) / 8], (x) % 8))
#define set(b, i, s) ((b) = ((b) & ~(1 << (i))) | ((s) << (i)))
#define dotset(b, x, y, s) (set(graphicBuffer[(b)][((y) * SCREEN_WIDTH + (x)) / 8], (x) % 8, (s)))

// struct
struct Point{
    int x;
    int y;
    int vx;
    int vy;
};

// constant
const int SCREEN_WIDTH = 320;
const int SCREEN_HEIGHT = 240;
const int BALL_RADIUS = 5;
const int PADDLE_WIDTH = 50;
const int PADDLE_HEIGHT = 5;


// graphic buffer
uint8_t graphicBuffer[2][SCREEN_WIDTH * SCREEN_HEIGHT / 8];
bool bufFlag = false;

// game object
Point ball, paddle[0];

// prototype declaration
void clean(bool buf);
void drawBall(int x, int y);
void drawPaddle(int x, int y);
void flush();

void setup() {
    M5.begin();
    for(int i = 0; i < 2; i++){
        clean((bool)i);
    }
}

void loop() {
    bufFlag = !bufFlag;
    clean(bufFlag);
    drawBall(ball.x, ball.y);
    for(int i = 0; i < 2; i++){
        drawPaddle(paddle[i].x, paddle[i].y);
    }
    flush();
    M5.update();
}

void clean(bool buf){
    for (int j = 0; j < SCREEN_WIDTH * SCREEN_HEIGHT / 8; j++){
        graphicBuffer[buf][j] = 0;
    }
}
void drawBall(int x, int y){
    for(int i = 0; i < 20; i++){
        dotset(bufFlag, (int)(x + cos(PI / 10 * i) * BALL_RADIUS + 0.5), (int)(y + sin(PI / 10 * i) * BALL_RADIUS + 0.5), 1);
    }
}

void drawPaddle(int x, int y){
    for(int i = 0; i < PADDLE_WIDTH; i++){
        for (int j = 0; j < PADDLE_HEIGHT; j++){
            dotset(bufFlag, x + j, y + i, 1);
        }
    }
}

void flush(){
    for(int y = 0; y < SCREEN_HEIGHT; y++){
        for (int x = 0; x < SCREEN_WIDTH; x++){
            if(dotget(0, x, y) ^ dotget(1, x, y)){
                M5.Lcd.drawPixel(x, y, 0xFFFF * (dotget(0, x, y) ^ bufFlag));
            }
        }
    }
}

