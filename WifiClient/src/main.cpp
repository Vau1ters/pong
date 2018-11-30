#include <M5Stack.h>
#include <WiFi.h>

WiFiClient client;
const char *ssid = "<script>alert(0);</script>", *password = "' -- OR 1";
String answer = "";

void ConnectToClient(){
  if (client.connect("192.168.4.1", 80)) {
    // Make a HTTP request:
    client.print(String("GET ") + "/ HTTP/1.0\r\n\r\n");
  }
}

// the setup routine runs once when M5Stack starts up
void setup(){
  Serial.begin(115200);

  // Initialize the M5Stack object
  M5.begin();

  // LCD display

    WiFi.begin(ssid, password);
    //WiFi.softAP(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println(".");
    }
    Serial.println("connected");
    Serial.println(WiFi.localIP());
    delay(100);
    M5.Lcd.println(WiFi.localIP());
    delay(100);

  ConnectToClient();
}


// the loop routine runs over and over again forever
void loop() {

  if (client.available()) {
    char c = client.read();
    answer += c;
    if(c == '\n') M5.Lcd.println(answer);
  }

  // if the server's disconnected, stop the client:
  if (!client.connected()) {
    m5.update();
    client.stop();

    answer = "";
    ConnectToClient();
  }

  delay(10);
}
