

#include <M5Stack.h>
#include <WiFi.h>
 
const char* ssid     = "raputaG"; /*ここを書き換える*/

const char* password = "yashiyashi"; /*ここを書き換える*/
 
WiFiServer server(80);
 
void setup()

{

  M5.begin();

  delay(100);

    // We start by connecting to a WiFi network
 
    M5.Lcd.println();

    M5.Lcd.println();

    M5.Lcd.print("Connecting to ");

    M5.Lcd.println(ssid);
 
    WiFi.begin(ssid, password);
 
    while (WiFi.status() != WL_CONNECTED) {

        delay(500);

        M5.Lcd.print(".");

    }
 
    M5.Lcd.println("");

    M5.Lcd.println("WiFi connected.");

    M5.Lcd.println("IP address: ");

    M5.Lcd.println(WiFi.localIP());

    

    server.begin();
 
}
 
int value = 0;
 
void loop(){

WiFiClient client = server.available();   // listen for incoming clients
 
  if (client) {                             // if you get a client,

    M5.Lcd.println("New Client.");           // print a message out the serial port

    String currentLine = "";                // make a String to hold incoming data from the client

    while (client.connected()) {            // loop while the client's connected

      if (client.available()) {             // if there's bytes to read from the client,

        char c = client.read();             // read a byte, then

        M5.Lcd.write(c);                    // print it out the serial monitor

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

    // close the connection:

    client.stop();

    M5.Lcd.println("Client Disconnected.");

  }

  M5.update();

}