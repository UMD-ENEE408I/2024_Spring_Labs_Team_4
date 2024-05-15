#include <Arduino.h>
#include <Encoder.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;

// main external reference: https://techtutorialsx.com/2018/05/17/esp32-arduino-sending-data-with-socket-client/

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
//#include "move.h"
#include "lineFollow.h"

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;
#define LED_BUILTIN 2   // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED

//const unsigned int ADC_1_CS = 2;
//const unsigned int ADC_2_CS = 17;

const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

const unsigned int M1_I_SENSE = 35;
const unsigned int M2_I_SENSE = 34;

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 10;

const int M_PWM_FREQ = 2500;
const int M_PWM_BITS = 8;
//const unsigned int MAX_PWM_VALUE = 512; // Max PWM given 8 bit resolution

float METERS_PER_TICK = (3.14159 * 0.032) / 360.0; // The diameter of the wheel is 0.032 not 0.031
float TURNING_RADIUS_METERS = 4.3 / 100.0; // Wheels are about 4.3 cm from pivot point


/*THIS IS DONE OVER PATS HOTSPOT AND LAPTOP*/
/*VALUES MUST BE CHANGED FOR THE NVIDIA, RUN ifconfig while on the hotstop to get the ip*/
/*On windows found in settings -> wifi -> advanced -> hardware*/
/*Seems that python script must be running for this to work*/
int status = WL_IDLE_STATUS;
const char *ssid = "PatsIphone";
const char *password = "patshotspot408";
WiFiClient client;
const uint16_t port = 8090;
const char * host = "172.20.10.3";



void setup() {
  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);


  Serial.begin(115200);


  //connect to the hotspot
 
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(WiFi.status());
    Serial.println("...");
  }
 
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());

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

  
  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);


  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);


  //wait 3000 before starting
  delay(3000);

}



int sound_localization(Encoder& enc1, Encoder& enc2){

  int sound_state = 0;
  char buffer[100] = "";


  while (sound_state < 4){
    //if the state is 0, then we need to turn left
    if (sound_state == 0){
      //attempt to connect to the server, if not stay in this state
      if (!client.connect(host, port)) {
          Serial.println("Connection to host failed");
          delay(1000);
          continue;
          //return;
        }
    
        //connected to the server, so turn right
        Serial.println("Connected to server successful!");
        //spin 90 degrees to the left
        spin(75, 420, false, enc1, enc2);
        //send the command to the server to record to the left
        sprintf(buffer, "SOUND1");
        client.print(buffer);
        //sound record for 2 seconds so wait 2100
        delay(2100);
        //disconnect from the server
        Serial.println("Disconnecting...");
        client.stop();
  
        sound_state++;

        delay(1000);

    //move to the next state, turn 180 degrees and read the sound again
    } else if (sound_state == 1) {

      //attempt to connect to the server, if not stay in this state
      if (!client.connect(host, port)) {
    
          Serial.println("Connection to host failed");
          delay(1000);
          continue;
        }
    
        //connected to the server, so turn right
        Serial.println("Connected to server successful!");
    
        //spin 180 degrees to the left
        spin(155, 410, true, enc1, enc2);

        //send the command to the server to record to the right
        sprintf(buffer, "SOUND2");
        client.print(buffer);

        //sound record for 2 seconds so wait 2100
        delay(2100);
    
        //disconnect from the server
        Serial.println("Disconnecting...");
        client.stop();
  
        sound_state++;

        delay(1000);


    //do the final step, send the command to the server to run the power calculation, and wait for a recieve, based on the signal do something
    } else if (sound_state == 2) {

      //try to connect
      if (!client.connect(host, port)) {
          Serial.println("Connection to host failed");
          delay(1000);
          continue;
        }

      //connected to the server, so turn right
      Serial.println("Connected to server successful!");
      
      //send the command to the server to record to the left
      sprintf(buffer, "SOUND3");
      client.print(buffer);
      delay(2500);   
      char c = client.read(); 
      while (c != 'L' && c!='R'){
        delay(100);
        Serial.write(c);
        if(c == 'L'){
        //turn 180 to the left then move
          sprintf(buffer, "ack");
          client.print(buffer);
          spin(170, 410, false, enc1, enc2);
          straight(5, 0, 30, 100, 380, enc1, enc2);
          break;
        } else if (c == 'R'){
          sprintf(buffer, "ack");
          client.print(buffer);
          straight(5, 0, 30, 100, 380, enc1, enc2);
          break;
        }
        sprintf(buffer, "nan");
        client.print(buffer);
        delay(100);
        c = client.read();
      }
      sound_state++;
      sprintf(buffer, "COMPLETE");
      client.print(buffer);
      delay(100);
      client.stop();
      brake();

    

    } else if (sound_state == 3){
      Serial.println("Sound Over");
      delay(100);
      sound_state = 4;
    }
  } 

  return 0;
}


//infro needed for the maze portion of the code
//define 1 as visited but nothing
//define 2 as red
//define 3 as blue
//define 4 as green
int maze[6][6] = {
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
};

//location of the robot
int loc[2] = {0,0};

//solve the maze looking for blocks
int maze_solver(Encoder& enc1, Encoder& enc2){
  char buffer[100];
  //lawnmower search, look for the colored boxes
  
  //go through each row
  char camera_reading = 'N';
  for(int row = 0; row < 6; row++){
    //go through each column
    for(int col = 0; col < 5; col++){
      
      //TAKE A PICTURE AND LOOK FOR A BLOCK
      //delay(100);

      //try to connect
      if (!client.connect(host, port)) {
          Serial.println("Connection to host failed");
          delay(1000);
          continue;
        }

      //connected to the server, so turn right
      Serial.println("Connected to server successful!");
      
      //send the command to the server to record to the left
      sprintf(buffer, "MAZEBLOCK");
      client.print(buffer);

      //now wait until we get a message from the server
      //f (client.available()) {  
          //read the message, itll be either an L or an R   
      delay(2500);   

      char camera_reading = client.read(); 
      while (camera_reading != 'N' && camera_reading != 'R'&& camera_reading != 'G' && camera_reading != 'B'){
        sprintf(buffer, "nan");
        client.print(buffer);
        delay(100);
        camera_reading = client.read();
        Serial.println(camera_reading);
      }
        sprintf(buffer, "ack");
        client.print(buffer);


      //do a wifi call here instead
      //camera_reading = 0;

      //UPDATE THE SPACE IN FRONT OF LOC WITH INFO
      int updateVal = 0;
      if(camera_reading == 'R'){
        updateVal = 1;
      } else if (camera_reading == 'G'){
        updateVal = 2;
      } else if (camera_reading == 'B'){
        updateVal = 3;
      }

      if(row % 2 == 0){
        maze[loc[0]][loc[1]+1] = updateVal;
      } else {
        maze[loc[0]][loc[1]-1] = updateVal;
      }

      //IF NO BLOCK, MOVE FORWARD
      if(camera_reading == 'N'){
        followLine_distance(30,0,300,350,enc1,enc2,235);
        delay(100);
        if(row % 2 == 0){
          loc[1]++;
        } else {
          loc[1]--;
        }

      } else {
        //go aroud the block
        //right
        spin(74, 360, true, enc1, enc2);
        followLine_distance(30,0,300,350,enc1,enc2,100);
        //left
        spin(68, 360, false, enc1, enc2);
        //past the block
        straight(5, 0, 30, 350, 380, enc1, enc2);
        //left
        spin(68, 360, false, enc1, enc2);
        straight(5, 0, 30, 100, 380, enc1, enc2);
        //right
        spin(74, 360, true, enc1, enc2);
      }

    }

    //depending on the row number either turn left or right
    //even rows turn right
    //odd rows turn left

    if(row % 2 == 0){
      spin(74, 360, true, enc1, enc2);
    } else {
      spin(68, 360, false, enc1, enc2);
    }
  

    //try to connect
    if (!client.connect(host, port)) {
          Serial.println("Connection to host failed");
          delay(1000);
          continue;
        }

      //connected to the server, so turn right
      Serial.println("Connected to server successful!");
      
      //send the command to the server to record to the left
      sprintf(buffer, "MAZEBLOCK");
      client.print(buffer);

      //now wait until we get a message from the server
      //f (client.available()) {  
          //read the message, itll be either an L or an R   
      delay(2500);   

      char camera_reading = client.read(); 
      while (camera_reading != 'N' && camera_reading != 'R' && camera_reading != 'G' && camera_reading != 'B'){
        sprintf(buffer, "nan");
        client.print(buffer);
        delay(100);
        camera_reading = client.read();
      }
        sprintf(buffer, "ack");
        client.print(buffer);

    //do a reading
    //do a wifi call here instead
    //camera_reading = 0;


    //UPDATE THE SPACE IN FRONT OF LOC WITH INFO
    int updateVal = 0;
    if(camera_reading == 'R'){
      updateVal = 1;
    } else if (camera_reading == 'G'){
      updateVal = 2;
    } else if (camera_reading == 'B'){
      updateVal = 3;
    }
    maze[loc[0]+1][loc[1]] = updateVal;

      //IF NO BLOCK, MOVE FORWARD
    if(camera_reading == 'N'){
      //move forward one space
      //straight(5, 0, 30, 235, 370, enc1, enc2);
      followLine_distance(30,0,300,350,enc1,enc2,234);
      loc[0]++;
    } else {
      //go aroud the block
        //right
        spin(74, 360, true, enc1, enc2);
        followLine_distance(30,0,300,350,enc1,enc2,100);
        //left
        spin(68, 360, false, enc1, enc2);
        //past the block
        straight(5, 0, 30, 350, 380, enc1, enc2);
        //left
        spin(68, 360, false, enc1, enc2);
        straight(5, 0, 30, 100, 380, enc1, enc2);
        //right
        spin(74, 360, true, enc1, enc2);
    }

    //turn left or right onto the row, except for the last row, then just go stright
    if (row != 5){
      if(row % 2 == 0){
        spin(74, 360, true, enc1, enc2);
      } else {
        spin(68, 360, false, enc1, enc2);
      }
  }

  }
  delay(1000);
  //line follow to the box
  followLine(30,0,300,350,0,enc1,enc2,0);
  delay(1000);

  //post maze processing, counting colors and such
  //get the counts of the blocks and their colors
  int colorArr[3] = {0,0,0};

  for(int i = 0; i > 6; i++){
    for (int j = 0; j>6; j++){
      
      switch(maze[i][j]){
        //case 1 is no block
        case 1:
          break;
        //red
        case 2:
          colorArr[0]++;  
          break;
        //blue
        case 3: 
          colorArr[1]++;
          break;
        //green
        case 4:
          colorArr[2]++;
          break;
        //anythiing else should never happen
        default:
          break;
      }
    }
  }


  //transmit the results of the counted blocks over wifi, to the computer


  return 0;

};

/*Idea: Transmit the current state of the robot to the webserver, either maze or sound localization
Then have the server respond with just a command to use on how to move properly.*/
int count = 0;
char buffer[100] = "";

//sound processing has 3 distinct steps
int sound_state = 0;


//this value represents where the robot is on the track
int robot_state = 0;

void loop() {

  //for wifi
  WiFiClient client;
  //for the encoders
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);

  Serial.print("robotState: ");
  Serial.println(robot_state);

  //runs the code for the sound porition of the code
  /*SOUND LOCALIZATION SEEMS TO BE WORKING, THESE ARE THE STATES NESSASSARY FOR IT TO TAKE PLACE
    I CLIPPED THE MICROPHONE ON THE CORNER OF THE BLUE IMU SO THE SENSORS DID NOT GET COVORED
    FOLLOW LINE HAD TO BE UPDATED TO ALLOW FOR TURNING AND STOPPING AFTER A TURN*/

  /* UNCOMMENT THE IF TREE TO RUN THE SOUND LOCALIZATION CODE
    TESTING STARTED RIGHT AFTER THE WHITE BOX BEFOREHAND
  if(robot_state == 0){
    Serial.println("LINE FOLLOW 1, Before SOUND");
    robot_state = 1;
    //break after a turn
    followLine(30,0,300,350,0,enc1,enc2,true);
    //straight(5, 0, 30, 20, 380, enc1, enc2);
    delay(200);
  } else if (robot_state == 1){
    robot_state = 2;
    Serial.println("RUN THE SOUND CODE!!!!!");
    
    //move up to the junction
    //consider replacing this with lineFollow_distance(30,0,300,500,enc1,enc2,380);
    straight(5, 0, 30, 500, 380, enc1, enc2);
    
    //run the sound code
    sound_localization(enc1,enc2);
    delay(100);
  } else if (robot_state == 2){
    Serial.println("LINE FOLLOW 2, AFTER SOUND");
    robot_state = 0;
    //follow the line out of the box
    followLine(30,0,300,350,0,enc1,enc2, false);
    //enter center of box
    straight(5, 0, 30, 30, 380, enc1, enc2);
    delay(10000);
  }  */

  //runs the maze portion of the code, testing movement
    if(robot_state == 0){
      //get from the box to the first intersection
      robot_state++;
      followLine_distance(30,0,300,350,enc1,enc2,440);
      delay(1000);
    }else if (robot_state == 1){
      robot_state++;
      maze_solver(enc1,enc2);
    } else if (robot_state == 2){
      robot_state = 0;
      delay(10000);
    }
    
   
 
  }












  /*WiFiClient client = server.available();   // listen for incoming clients

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


          spin(90, 450, false,  enc1, enc2); //turn right

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


          spin(90, -450, true,  enc1, enc2);
     
        }
      }
    }

    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
  */
//}


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
