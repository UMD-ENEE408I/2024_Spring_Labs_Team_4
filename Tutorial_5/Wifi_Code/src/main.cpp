#include <Arduino.h>
#include <Encoder.h>

/*
  WiFiAccessPoint.ino creates a WiFi access point and provides a web server on it.

  Steps:
  1. Connect to the access point "yourAp"
  2. Point your web browser to http://192.168.4.1/H to turn the LED on or http://192.168.4.1/L to turn it off
     OR
     Run raw TCP "GET /H" and "GET /L" on PuTTY terminal with 192.168.4.1 as IP address and 80 as port

  Created for arduino-esp32 on 04 July, 2018
  by Elochukwu Ifediora (fedy0)
*/

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include "move.h"

#define LED_BUILTIN 2   // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED

// Set these to your desired credentials.
const char *ssid = "PatConnect";
const char *password = "YourPassword";

WiFiServer server(80);

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

const unsigned int M1_I_SENSE = 35;
const unsigned int M2_I_SENSE = 34;

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 10;

const int M_PWM_FREQ = 5000;
const int M_PWM_BITS = 8;
//const unsigned int MAX_PWM_VALUE = 512; // Max PWM given 8 bit resolution

float METERS_PER_TICK = (3.14159 * 0.032) / 360.0; // The diameter of the wheel is 0.032 not 0.031
float TURNING_RADIUS_METERS = 4.3 / 100.0; // Wheels are about 4.3 cm from pivot point



void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);


  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuring access point...");

  // You can remove the password parameter if you want the AP to be open.
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();

  Serial.println("Server started");


  //motor stuff
    // Stop the right motor by setting pin 14 low
  // this pin floats high or is pulled
  // high during the bootloader phase for some reason


  // configure motor pins
  ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);

  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);


}

void loop() {
  WiFiClient client = server.available();   // listen for incoming clients

  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);
  

  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
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
            //client.print("Click <a href=\"/RIGHT\">here</a> to turn Right.<br>");
            //client.print("Click <a href=\"/LEFT\">here</a> to turn Left.<br>");
            //client.print("Hello World <br>");
            client.print("Hello World");

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
        if (currentLine.endsWith("GET /RIGHT")) {
          //digitalWrite(LED_BUILTIN, HIGH);// GET /H turns the LED on


          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
          // the content of the HTTP response follows the header:
          client.print("RIGHT<br>");
          // The HTTP response ends with another blank line:
          client.println();


          //spin(90, 400, false,  enc1, enc2); //turn right

          //currentLine = "";

        } else if (currentLine.endsWith("GET /LEFT")) {
          //digitalWrite(LED_BUILTIN, LOW);                // GET /L turns the LED off

          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
          // the content of the HTTP response follows the header:
          client.print("LEFT<br>");
          // The HTTP response ends with another blank line:
          client.println();   


          //spin(90, -400, true,  enc1, enc2);
     
        }
      }
    }

    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
}


// const unsigned int M1_ENC_A = 39;
// const unsigned int M1_ENC_B = 38;
// const unsigned int M2_ENC_A = 37;
// const unsigned int M2_ENC_B = 36;


// void setup() {
//   // Stop the right motor by setting pin 14 low
//   // this pin floats high or is pulled
//   // high during the bootloader phase for some reason
//   pinMode(14, OUTPUT);
//   digitalWrite(14, LOW);
//   delay(100);


//   Serial.begin(115200);
// }

// void loop() {
//   // Create the encoder objects after the motor has
//   // stopped, else some sort exception is triggered
//   Encoder enc1(M1_ENC_A, M1_ENC_B);
//   Encoder enc2(M2_ENC_A, M2_ENC_B);

//   while(true) {
//     long enc1_value = enc1.read();
//     long enc2_value = enc2.read();
    
//     Serial.print(enc1_value);
//     Serial.print("\t");
//     Serial.print(enc2_value);
//     Serial.println();
//     delay(100); // Delay works now that interrupts are fixed
//   }
// }
