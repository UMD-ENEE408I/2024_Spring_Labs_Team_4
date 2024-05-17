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
  //disable by commenting out...
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
        if(c == 'L' ){
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
  followLine(30,0,300,350,0,enc1,enc2,0,false,false);
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
  /*
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
    */
   


    //see if we can get this to work by demo time
    //look for a battery in the 6v range maybe?
    //if not then retune asap

    //BATTERY AT 8V
    //task 1: Line of republic
    //30/300 was origional
    //works well for stright line

    //WORKED UP TO SOUND AT 7.95V
    
    straight(5, 0, 30, 130, 420, enc1, enc2);
    delay(10000);
    followLine(40,0,500,380,0,enc1,enc2,false,false,false);
    delay(100);
    straight(5, 0, 30, 90, 420, enc1, enc2);
    delay(100);
    arc(0, 160, 420, 0, enc1, enc2);
    delay(100);
    straight(5, 0, 30, 130, 420, enc1, enc2);
    //should be out of the second box by now

    //task 2: figureout how to skip the maze
    
    //try line follow and turn break
    followLine(40,0,500,390,0,enc1,enc2,true,true,false); //hopefully get to the corner and breaks
    delay(100);
    for(int i = 0; i< 6; i++){
      followLine_distance(40,0,500,350,enc1,enc2,250);
      delay(100);
    }
    followLine(40,0,500,380,0,enc1,enc2,false,false,false);

    //enter the box
    straight(5, 0, 30, 105, 420, enc1, enc2);
    delay(100);
    arc(0, 160, 420, 1, enc1, enc2);
    delay(100);
    straight(5, 0, 30,   100, 420, enc1, enc2);
    //at the end of the box before dotted line

    //task 3: Dotted Line
    //this worked  V8 battery
    
    followLine(40,0,550,340,0,enc1,enc2,false,false,false);
    delay(500);

    //task 4: dodge
    straight(5, 0, 30, 130, 420, enc1, enc2); // move up to get out of box
    brake();
    delay(500);
    arc(0, 150, 420, 1, enc1, enc2); // turn out
    brake();
    delay(500);
    straight(5, 0, 30, 100, 420, enc1, enc2);
    brake();
    delay(500);
    //followLine(40, 0, 300, 420, 2);
    //brake();
    //delay(500);
    arc(0, 150, 420, 0, enc1, enc2);

    brake();
    delay(500);
    straight(5, 0, 30, 100, 420, enc1, enc2);
    brake();
    
    delay(500);
    //the long stright
    arc(0, 160, 420, 1, enc1, enc2); // turn to prepare to go striaght
    
    brake();
    delay(500);
    //straight(5, 0, 30, 1450, 420, enc1, enc2);
    straight(5, 0, 30, 0, 420, enc1, enc2); // go until line detected before sound section
    brake();
    delay(500);
    arc(0, 170, 420, 0, enc1, enc2); // set 
    

    
    //task 5: sound code (copy from above...)
    //break after a turn
    
    delay(100);
    followLine_distance(40,0,500,350,enc1,enc2,100);
    delay(100);
    followLine(40,0,500,380,0,enc1,enc2,true,true,false); //get to the right turn
    delay(100);
    followLine(40,0,500,380,0,enc1,enc2,false,false,true); //run until black reading
    //run the sound function 
    sound_localization(enc1,enc2);
    delay(100);
    followLine(40,0,500,360,0,enc1,enc2,true,true,false); //turn 1
    delay(100);
    followLine(40,0,500,360,0,enc1,enc2,true,true,false); //turn 2
    delay(100);
    followLine(40,0,500,360,0,enc1,enc2,true,true,false); //turn 3
    delay(100);
    followLine(40,0,550,340,0,enc1,enc2,false,false,false);//stop at the box
    
   
    //from box to the next box
    straight(5, 0, 30, 105, 420, enc1, enc2);
    delay(100);
    arc(0, 160, 420, 1, enc1, enc2);
    delay(100);
    straight(5, 0, 30, 130, 420, enc1, enc2);//entering next box
    delay(100);

    //line follow to next box
    followLine(40,0,500,320,0,enc1,enc2,false,false,false);

    delay(100);
    straight(5, 0, 30, 105, 380, enc1, enc2);
    delay(100);
    arc(0, 170, 420, 1, enc1, enc2); //turn to face last obsticle


    //task 6: endor dash
    //this seems to work well 
    //battery 7.98 V
    straight(5, 0, 3, 110, 420, enc1, enc2); // move up to get out of box
    delay(100);
    followLine(40, 0, 550, 360,0, enc1,enc2,false,false,true); // follow until black detected
    delay(100);
    //prev 400
    straight_detect(2, 0, 0, 0, 360, enc1, enc2); // go straight until box


    delay(3000);

  }
