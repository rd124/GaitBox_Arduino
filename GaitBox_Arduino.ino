// GaitBox LIDAR stable code 
// Created Spring 2017
// Edited 4/29/19
// Final 1-10-2020
// by Leighanne Jarvis & Kevin Caves modified from previous 
// work by Jon Usher
// Duke University
// Code to measure walking speed by walking towards the device
// Modified June 2021 
// by Riddhi Ranjithkumar
// for integration with mobile app 

#include <Firebase_Arduino_WiFiNINA.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <LiquidCrystal_I2C.h> // make sure programmer option is set to "Onboard Atmel"
#include <SPI.h>
LIDARLite lidarLite; 
int cal_cnt = 0;

// Variables for measurement 
double distance;
double far_thold = 200;  // distance (cm) from sensor you will take first measurement
double close_thold = 100;  // distance (cm) from sensor you will take stop measurement
double far_distance;  
double close_distance;
double far_time = 0;
double close_time = 0;
double total_time;
double velocity = 0;
double velTime = 0;
int flag = 0;
double temptime = 0;
int var = 0;
int green = 7;

String fbdisval;
String cbdisval;
String startover;
int resettest = 0 ;

//Firebase settings
#define FIREBASE_HOST "gaitbox-12dd9-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "HaeQrmyCd8s4ANRlDyLKMmU2xPMwbD1ncsnGuPzq"

//Wi-Fi settings
#define WIFI_SSID "Network_name"
#define WIFI_PASSWORD "Password"

FirebaseData firebaseData;
 
String path = "/GB";
String jsonStr;

//----------buttons------------------------//
int buttonPin = 8;
int buttonState = 0;

// LCD
//LiquidCrystal_I2C lcd(0x3F,20,4);
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7);  // need to make sure you identify the correct address.  do this with an i2c scanning sketch

void setup()
{
  Serial.begin(115200); // Initialize serial connection to display distance readings
  int configuration = 3;
  char lidarliteAddress = 0x0f;
  lidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  lidarLite.configure(0); // Change this number to try out alternate configurations
  pinMode(green, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); //high or low doesnt always work

// setup the LCD and display start up message
// need to check to see what display address is:  use an i2c scanner or try 0x3F, 0x27

  lcd.begin(16,2);  // cannot be blank (), must have (chars, lines)
//  lcd.init();  // cannot be blank (), must have (chars, lines)
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.clear();
  lcd.setCursor(1,1);
  lcd.print("Hello!");
  lcd.setCursor(0,1);
  lcd.print("Starting up...");

//SETTING UP WIFI
  Serial.print("Connecting to WiFi…");
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
  status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print(".");
  delay(300);
 }
  Serial.print(" IP: ");
  Serial.println(WiFi.localIP());
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Connected");
  lcd.setCursor(0,1);
  lcd.print("to WIFI!");
  delay(2000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Open GB app or ");
  lcd.setCursor(0,1);
  lcd.print("press reset");
  
 Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, WIFI_SSID, WIFI_PASSWORD);
 Firebase.reconnectWiFi(true);

 // flash LED
 var = 0;
  while(var < 11){
    digitalWrite(green, HIGH); // blink green when turning on.
    delay(100);
    digitalWrite(green, LOW);  
    delay(100);
    var++;
    }
}

void loop(){

//Checking whether to start a new test
if(resettest==0){
//Physically checking button
  buttonState = digitalRead(buttonPin);
     while(buttonState == HIGH && resettest == 0) {  // check if the pushbutton is pressed, start over
          resettest = 1;
          Serial.println(resettest);
          delay(1000);
          Serial.println("Wait for LED to start");
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Wait for LED");
          lcd.setCursor(0,1);
          lcd.print("to start...");
        }

// Reading reset button from app
 if (Firebase.getString(firebaseData, path + "/Reset/resettest/")) {
    if (firebaseData.dataType() == "string") {
      startover = firebaseData.stringData();
    }

  } else {
    Serial.println(firebaseData.errorReason());
  }
if(startover == "reset"){
  resettest = 1;
}
 
  }

// Begin Testing
if (resettest == 1){

  //Reading for far threshold
 if (Firebase.getString(firebaseData, path + "/Far/fardistance")) {
    if (firebaseData.dataType() == "string") {
      fbdisval = firebaseData.stringData();
    }
} 
 else {
    Serial.println(firebaseData.errorReason());
  }
 if(fbdisval.toInt()!=0){
  far_thold = fbdisval.toInt()*100;
}

//Reading for close threshold
if (Firebase.getString(firebaseData, path + "/Close/closedistance")) {
    if (firebaseData.dataType() == "string") {
      fbdisval = firebaseData.stringData();
    }

  } else {
    Serial.println(firebaseData.errorReason());
  }
if(cbdisval.toInt()!=0){
  close_thold = cbdisval.toInt()*100;
}

  distance = lidarLite.distance(false); // distance from LIDAR Sensor
  temptime = millis();
  buttonState = digitalRead(buttonPin);

  //wait for sensor to stabilize and wait for path to be clear
  if (distance >= far_thold && flag == 0) {  // check that path is clear (can see past the far_thold and that you're not in a reading (flag == 0))
    Serial.println("The path is clear");
    flag = 1; // set flag to 1 and start looking for far_distance
    digitalWrite(green, HIGH);
  }

    //look for far distance
    if (distance <= far_thold && distance > 50 && flag == 1) {  // if you reach far_thold, record the starting distance and time, distance > 50 prevents false triggers
    
      far_distance = distance; //far_distance = distance*10; 
      far_time = temptime;
      digitalWrite(green, LOW); //turn off  LED
      Serial.print("Far = ");
      Serial.println(far_distance);
      flag = 2;  //set flag to 2 and look for close distance
  
    }
  
    //look for close distance
    if (distance <= close_thold && distance > 50 && flag == 2) { // when you reach close_thold, record ending distance and time, distance > 50 prevents false triggers
      close_distance = distance; //close_distance = distance*10; // set the ending distance
      close_time = temptime;
      velocity = (((far_distance - close_distance)/100)/((close_time - far_time)/1000)); //calculate velocity
      velTime = (close_time - far_time)/1000;


      Serial.print("Close = ");
      Serial.println(close_distance);
      delay(2500);
      flag = 0;  // set flag back to new reading state
    
  
      //Changing units
      far_distance = far_distance/100;
      close_distance = close_distance/100;
      total_time = (close_time - far_time)/1000;


      // Print distance to Serial
      Serial.print(" far_distance: ");
      Serial.println(far_distance);
      Serial.print(" far_time: ");
      Serial.println(far_time);
      Serial.print(" close_distance: ");
      Serial.println(close_distance);
      Serial.print(" close_time: ");
      Serial.println(close_time);
      Serial.print(" velocity: ");
      Serial.println(velocity);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Velocity = ");
      lcd.setCursor(11,0);
      lcd.print(velocity);
      lcd.setCursor(0,1);
      lcd.print("Reset when ready");

 

 //Sending data to Firebase
  if (Firebase.setFloat(firebaseData, path + "/resultvals/far_d", far_distance)) {
 Serial.println(firebaseData.dataPath() + " = " + far_distance);
 }
 if (Firebase.setFloat(firebaseData, path + "/resultvals/close_d", close_distance)) {
 Serial.println(firebaseData.dataPath() + " = " + close_distance);
 }
 if (Firebase.setFloat(firebaseData, path + "/resultvals/t", total_time)) {
 Serial.println(firebaseData.dataPath() + " = " + total_time);
 }
 if (Firebase.setFloat(firebaseData, path + "/resultvals/v", velocity)) {
 Serial.println(firebaseData.dataPath() + " = " + velocity);
 }

 //Changing reset back to no 
Firebase.setString(firebaseData, path + "/Reset/resettest", "no");
 
 resettest = 0;
    }
  }

}
