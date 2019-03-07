// ------------------------------------------------------------
//
//    MMRC client for controlling two single turnouts
//    Copyright (C) 2019 Peter Kindström
//
//    This program is free software: you can redistribute
//    it and/or modify it under the terms of the GNU General
//    Public License as published by the Free Software 
//    Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
// -----------------------------------------------------------
// #include <ESP8266WiFi.h>
// #include <PubSubClient.h>
#include <Servo.h>
#include "MMRCsettings.h"

// WiFiClient wifiClient;
// PubSubClient client(wifiClient);

// Servo initialisation
Servo turnoutOneServo;       // Create servo object to control turnout 1
Servo turnoutTwoServo;       // Create servo object to control turnout 2

// -----------------------------------------------------
// Convert settings in MMRCsettings.h to constants and variables
const char* ssid = SSID;
const char* password = PASSWORD;
const char* mqttBrokerIP = IP;
String cccCategory = CATEGORY;
String cccModule = MODULE;
String cccObject = OBJECT;
String cccCLEES = CLEES;

// -----------------------------------------------------
// Various variable definitions

// Variable for topics to subscribe to
const int nbrTopics = 1;
String subTopic[nbrTopics];

// Variable for topics to publish to
String pubTopic[1];

// Variables for client info
String clientID;      // Id/name for this specific client, shown i MQTT and router

// Detect power on
int startOne = 1;
int startTwo = 1;

// Define turnout states
int NORMAL = 0;
int REVERSE = 1;

// Temporary
int tmpCnt = 0;
int tempCnt = 0;
int btnState = 0;     // To get two states from a momentary button

// -----------------------------------------------------
// Turnout ONE variables
int actionOne = 0;                  // To 'remember' that an action is in progress

int ledOneState = 0;
int ledOneInterval = 750;
unsigned long ledOneMillis = 0;

int turnoutOneState = NORMAL;
int turnoutOneInterval = 150;
int turnoutOneStep = 1;                  // Variable to step servo position.
int turnoutOneNormal = 75;
int turnoutOneReverse = 59;
int turnoutOnePosition = turnoutOneNormal;
unsigned long turnoutOneMillis = 0;

// -----------------------------------------------------
// Turonut TWO variables
int actionTwo = 0;                  // To 'remember' that an action is in progress

int ledTwoState = 0;
int ledTwoInterval = 750;
unsigned long ledTwoMillis = 0;

int turnoutTwoState = NORMAL;
int turnoutTwoInterval = 175;
int turnoutTwoStep = 1;                  // Variable to step servo position.
int turnoutTwoNormal = 85;
int turnoutTwoReverse = 64;
int turnoutTwoPosition = turnoutTwoNormal;
unsigned long turnoutTwoMillis = 0;

// -----------------------------------------------------
// Define which pins to use for different actions - Wemos D1 R2 mini clone
int btnOnePin = 1;    // Pin for first button
int ledOneUpPin = 2;
int ledOneDnPin = 3;
int turnoutOnePin = 4;
int btnTwoPin = 5;    // Pin for second button
int ledTwoUpPin = 6;
int ledTwoDnPin = 7;
int turnoutTwoPin = 8;

// Uncomment next line to use built in LED on NodeMCU (which is pin D4)
// #define LED_BUILTIN D4


/*
 * Standard setup function
 */
void setup() {
  // Setup Arduino IDE serial monitor for "debugging"
  Serial.begin(115200);

  // Define build-in LED pin as output
  pinMode(LED_BUILTIN, OUTPUT);   // For Arduino, Wemos D1 mini
  pinMode(btnOnePin, INPUT);
  pinMode(btnTwoPin, INPUT);
  pinMode(ledOneUpPin, OUTPUT);
  pinMode(ledOneDnPin, OUTPUT);
  pinMode(ledTwoUpPin, OUTPUT);
  pinMode(ledTwoDnPin, OUTPUT);

  // Attach servos to PWM pins
  turnoutOneServo.attach(turnoutOnePin); 
  turnoutTwoServo.attach(turnoutTwoPin); 

  // Set initial state
  digitalWrite(ledOneUpPin, LOW);
  digitalWrite(ledOneDnPin, LOW);
  digitalWrite(ledTwoUpPin, LOW);
  digitalWrite(ledTwoDnPin, LOW);

  // Assemble topics to subscribe and publish to
  if (cccCLEES == "1") {
    cccCategory = "clees";
    subTopic[0] = cccCategory+"/"+cccModule+"/cmd/turnout/"+cccObject;
    pubTopic[0] = cccCategory+"/"+cccModule+"/rpt/turnout"+cccObject;
  } else {
    subTopic[0] = "mmrc/"+cccModule+"/"+cccObject+"/"+cccCategory+"/turnout/cmd";
    subTopic[1] = "mmrc/"+cccModule+"/"+cccObject+"/"+cccCategory+"/button/cmd";
    pubTopic[0] = "mmrc/"+cccModule+"/"+cccObject+"/"+cccCategory+"/turnout/rpt";
 }

  // Unique name for this client
  if (cccCLEES == "1") {
    clientID = "CLEES "+cccModule;
  } else {
    clientID = "MMRC "+cccModule;
  }

  // Connect to wifi network
//  wifiConnect();
//  delay(1000);

  // Connect to MQTT broker and define function to handle callbacks
//  client.setServer(mqttBrokerIP, 1883);
//  client.setCallback(mqttCallback);

}


/**
 * Connects to WiFi and prints out connection information
 */
void wifiConnect() {
  char tmpID[clientID.length()];
  delay(200);

   // Convert String to char* for the client.subribe() function to work
  clientID.toCharArray(tmpID, clientID.length()+1);

  // Connect to WiFi
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(ssid);
//  WiFi.hostname(tmpID);
//  WiFi.begin(ssid, password);
 
  // Wait for connection
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
  Serial.println("");
  Serial.println("WiFi connected");
 
  // Print connection information
  Serial.print("Client hostname: ");
//  Serial.println(WiFi.hostname());
  Serial.print("IP address: ");
//  Serial.println(WiFi.localIP());
  Serial.print("MAC address: ");
//  Serial.println(WiFi.macAddress());
  Serial.println("---");
}


/**
 * (Re)connects to MQTT broker and subscribes to one or more topics
 */
void mqttConnect() {
  char tmpTopic[254];
  char tmpID[clientID.length()];
  
  // Convert String to char* for the client.subribe() function to work
  clientID.toCharArray(tmpID, clientID.length()+1);

  // Loop until we're reconnected
//  while (!client.connected()) {
  Serial.print("MQTT connection...");
  // Attempt to connect
//  if (client.connect(tmpID)) {
    Serial.println("connected");
    Serial.print("MQTT client id: ");
    Serial.println(tmpID);
    Serial.println("Subscribing to:");

    // Subscribe to all topics
    for (int i=0; i < nbrTopics; i++){
      // Convert String to char* for the client.subribe() function to work
      subTopic[i].toCharArray(tmpTopic, subTopic[i].length()+1);

      // ... print topic
      Serial.print(" - ");
      Serial.println(tmpTopic);

      // ... and subscribe to topic
//      client.subscribe(tmpTopic);
    }
//  } else {
    Serial.print("failed, rc=");
//    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
    // Wait 5 seconds before retrying
    delay(5000);
//    }
//  }
  Serial.println("---");

}

/**
 * Function to handle MQTT messages sent to this device
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Don't know why this have to be done :-(
  payload[length] = '\0';

  // Print the topic
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  // Make strings and print payload
  String msg = String((char*)payload);
  String tpc = String((char*)topic);
  Serial.println(msg);

  // Send topic and payload to be interpreted
  mqttResolver(tpc, msg);
}


/**
 * Resolves messages from MQTT broker 
 */
void mqttResolver(String sbTopic, String sbPayload) {

  // Check first topic
  if (sbTopic == subTopic[0]){

    // Check message
    if (sbPayload == "0") {
      // Turn LED on and report back (via MQTT)
      digitalWrite(LED_BUILTIN, HIGH);
      mqttPublish(pubTopic[0], "0");
    }
    if (sbPayload == "1") {
      // Turn LED off and don't report back (via MQTT)
      digitalWrite(LED_BUILTIN, LOW);
      mqttPublish(pubTopic[0], "1");
    }
  }

}


/**
 * Publish messages to MQTT broker 
 */
void mqttPublish(String pbTopic, String pbPayload) {

  // Convert String to char* for the client.publish() function to work
  char msg[pbPayload.length()+1];
  pbPayload.toCharArray(msg, pbPayload.length()+1);
  char tpc[pbTopic.length()+1];
  pbTopic.toCharArray(tpc, pbTopic.length()+1);

  // Report back to pubTopic[]
//  int check = client.publish(tpc, msg);

  // TODO check "check" integer to see if all went ok

  // Print information
  Serial.print("Message sent    [");
  Serial.print(pbTopic);
  Serial.print("] ");
  Serial.println(pbPayload);

  // Action 1 is executed, ready for new action
  actionOne = 0;

}

/**
 * Initialize turnout ONE moving
 */
void turnoutOneStart() {

  // Turn off both LED
  digitalWrite(ledOneUpPin, LOW);
  digitalWrite(ledOneDnPin, LOW);

  // Publish button press
  if (turnoutOneState == NORMAL) {
      // mqttPublish(subTopic[0], "1");
      turnoutOneState = REVERSE;
  } else {
      // mqttPublish(subTopic[0], "0");
      turnoutOneState = NORMAL;
    }
    
  // Action ONE is executing, new actions forbidden
  actionOne = 1;

  // Turn off turnout TWO Leds start blink
  startOne = 0;

  // Set millis counter
  ledOneMillis = millis();
  turnoutOneMillis = millis();

  // Debug
  Serial.print(" - Turnout state = ");
  Serial.print(turnoutOneState);
  Serial.print(" - Action = ");
  Serial.print(actionOne);
  Serial.print(" - Led millis = ");
  Serial.print(ledOneMillis);
  Serial.print(" - Turnout millis = ");
  Serial.println(turnoutTwoMillis);

}

/**
 * Initialize turnout TWO moving
 */
void turnoutTwoStart()
{
  // Turn off bith LED
  digitalWrite(ledTwoUpPin, LOW);
  digitalWrite(ledTwoDnPin, LOW);

  // Publish button press
  if (turnoutTwoState == NORMAL) {
      // mqttPublish(subTopic[0], "1");
      turnoutTwoState = REVERSE;
  } else {
      // mqttPublish(subTopic[0], "0");
      turnoutTwoState = NORMAL;
  }
    
  // Action TWO is executing, new actions forbidden
  actionTwo = 1;

  // Turn off turnout TWO Leds start blink
  startTwo = 0;

  // Set millis counter
  ledTwoMillis = millis();
  turnoutTwoMillis = millis();

  // Debug
  Serial.print(" - Turnout state = ");
  Serial.print(turnoutTwoState);
  Serial.print(" - Action = ");
  Serial.print(actionTwo);
  Serial.print(" - Led millis = ");
  Serial.print(ledTwoMillis);
  Serial.print(" - Turnout millis = ");
  Serial.println(turnoutTwoMillis);
}

/**
 * Move servo for turnout ONE
 */
void turnoutOneMove()
{

  if (turnoutOneState == NORMAL) {

    // Count down the position by [turnoutOneStep] steps
    turnoutOnePosition = turnoutOnePosition + turnoutOneStep;

    // Debug
    Serial.print("Move up");

    // Check if we reached the right position
    if (turnoutOnePosition >= turnoutOneNormal) {

      // Set max position (useful if we take big steps...)
      turnoutOnePosition = turnoutOneNormal;

      // End this action and allow for the next
      actionOne = 0;

      // Set led status
      digitalWrite(ledOneUpPin, HIGH);
      digitalWrite(ledOneDnPin, LOW);

    // Debug
      Serial.print(" - Stopped");
    }

 } else {

    // Count upn the position by [turnoutOneStep] steps
    turnoutOnePosition = turnoutOnePosition - turnoutOneStep;

    // Debug
    Serial.print("Move down");

    // Check if we reached the right position
    if (turnoutOnePosition <= turnoutOneReverse) {

      // Set min position (useful if we take big steps...)
      turnoutOnePosition = turnoutOneReverse;

      // End this action and allow for the next
      actionOne = 0;

      // Set led status
      digitalWrite(ledOneUpPin, LOW);
      digitalWrite(ledOneDnPin, HIGH);

    // Debug
      Serial.print(" - Stopped");
    }

  }
    // Move servo
    turnoutOneServo.write(turnoutOnePosition);

    // TODO
    // Maybe we need a small servo adjustment here in the future 
    // to prevent the servo to be left in "tension"?
    
    // Debug
    Serial.print("- Step = ");
    Serial.print(turnoutOneStep);
    Serial.print(" - Position = ");
    Serial.println(turnoutOnePosition);
}

/**
 * Move servo for turnout TWO
 */
void turnoutTwoMove()
{

  if (turnoutTwoState == NORMAL) {

    // Count down the position by [turnoutTwoStep] steps
    turnoutTwoPosition = turnoutTwoPosition + turnoutTwoStep;

    // Debug
    Serial.print("Move up");

    // Check if we reached the right position
    if (turnoutTwoPosition >= turnoutTwoNormal) {

      // Set max position (useful if we take big steps...)
      turnoutTwoPosition = turnoutTwoNormal;

      // End this action and allow for the next
      actionTwo = 0;

      // Set led status
      digitalWrite(ledTwoUpPin, HIGH);
      digitalWrite(ledTwoDnPin, LOW);

    // Debug
      Serial.print(" - Stopped");
    }

 } else {

    // Count upn the position by [turnoutTwoStep] steps
    turnoutTwoPosition = turnoutTwoPosition - turnoutTwoStep;

    // Debug
    Serial.print("Move down");

    // Check if we reached the right position
    if (turnoutTwoPosition <= turnoutTwoReverse) {

      // Set min position (useful if we take big steps...)
      turnoutTwoPosition = turnoutTwoReverse;

      // End this action and allow for the next
      actionTwo = 0;

      // Set led status
      digitalWrite(ledTwoUpPin, LOW);
      digitalWrite(ledTwoDnPin, HIGH);

    // Debug
      Serial.print(" - Stopped");
    }

  }
    // Move servo
    turnoutTwoServo.write(turnoutTwoPosition);

    // TODO
    // Maybe we need a small servo adjustment here in the future 
    // to prevent the servo to be left in "tension"?
    
    // Debug
    Serial.print("- Step = ");
    Serial.print(turnoutTwoStep);
    Serial.print(" - Position = ");
    Serial.println(turnoutTwoPosition);
}

/**
 * Main loop
 */
void loop()
{

  unsigned long currentMillis = millis();
  
  // -----------------------------------------------------
  // -- Waiting for actions
  // -----------------------------------------------------

  // -----------------------------------------------------
// Check connection to the MQTT broker. If no connection, try to reconnect
//  if (!client.connected()) {
//    mqttConnect();
//    }

  // -----------------------------------------------------
  // Wait for incoming messages
//  client.loop();

  // -----------------------------------------------------
  // Check for button ONE press
  int btnOnePress = digitalRead(btnOnePin);
  if (btnOnePress == LOW && actionOne == 0) {
    Serial.print("Button ONE pressed");
    delay(400);

    // Start moving turnout ONE
    turnoutOneStart();
  }

  // -----------------------------------------------------
  // Check for button TWO press
  int btnTwoPress = digitalRead(btnTwoPin);
  if (btnTwoPress == LOW && actionTwo == 0) {
    Serial.print("Button TWO pressed");
    delay(300);

    // Start moving turnout TWO
    turnoutTwoStart();
  }

  // -----------------------------------------------------
  // -- Multitasking actions
  // -----------------------------------------------------

  // -----------------------------------------------------
  // Check if it is time to blink turnout ONE Leds
  if(currentMillis - ledOneMillis > ledOneInterval && actionOne == 1) {
    // LED on or off state
    ledOneState = 1-ledOneState;

    if (turnoutOneState == NORMAL) {
      digitalWrite(ledOneUpPin, ledOneState);
    } else {
      digitalWrite(ledOneDnPin, ledOneState);
    }

    // Reset LED One millis counter
    ledOneMillis = currentMillis;

    // Debug
    Serial.print("Led ONE blink - ");
    Serial.println(ledOneState);
  }

  // -----------------------------------------------------
  // Check if it is time to blink turnout TWO Leds
  if(currentMillis - ledTwoMillis > ledTwoInterval && actionTwo == 1) {
    // LED on or off state
    ledTwoState = 1-ledTwoState;

    if (turnoutTwoState == NORMAL) {
      digitalWrite(ledTwoUpPin, ledTwoState);
    } else {
      digitalWrite(ledTwoDnPin, ledTwoState);
    }

    // Reset LED One millis counter
    ledTwoMillis = currentMillis;

    // Debug
    Serial.print("Led TWO blink - ");
    Serial.println(ledTwoState);
  }

  // -----------------------------------------------------
  // Check if it is time to move turnout ONE servo
  if(currentMillis - turnoutOneMillis > turnoutOneInterval && actionOne == 1) {

    // Debug
    Serial.print("Servo ONE move - ");

    // Move servo one step
    turnoutOneMove();

    // Reset LED One millis counter
    turnoutOneMillis = currentMillis;
  }

  // -----------------------------------------------------
  // Check if it is time to move turnout TWO servo
  if(currentMillis - turnoutTwoMillis > turnoutTwoInterval && actionTwo == 1) {

    // Debug
    Serial.print("Servo TWO move - ");

    // Move servo one step
    turnoutTwoMove();

    // Reset LED Two millis counter
    turnoutTwoMillis = currentMillis;
/*
    if (turnoutTwoState == NORMAL) {
      Serial.println("UP");
      digitalWrite(ledTwoUpPin, ledTwoState);
    } else {
      Serial.println("DOWN");
      digitalWrite(ledTwoDnPin, ledTwoState);
    }
*/
    // Temporary code to turn off blink
/*    
     tmpCnt = tmpCnt+1;
    if (tmpCnt == 10) {
      tmpCnt = 0;
      actionTwo = 0;
      if (turnoutTwoState == NORMAL) {
        digitalWrite(ledTwoUpPin, HIGH);
        digitalWrite(ledTwoDnPin, LOW);
      } else {
        digitalWrite(ledTwoUpPin, LOW);
        digitalWrite(ledTwoDnPin, HIGH);
      }
    }
*/
  }

  // -----------------------------------------------------
  // Check if newly started and blink both turnout ONE Leds
  if(currentMillis - ledOneMillis > ledOneInterval && startOne == 1) {

    // LED on or off state
    ledOneState = 1-ledOneState;
    digitalWrite(ledOneUpPin, ledOneState);
    digitalWrite(ledOneDnPin, ledOneState);

    // Reset LED One millis counter
    ledOneMillis = currentMillis;

    // Debug
    Serial.print("Start Led ONE - ");
    Serial.println(ledOneState);

  }

  // -----------------------------------------------------
  // Check if newly started and blink both turnout TWO Leds
  if(currentMillis - ledTwoMillis > ledTwoInterval && startTwo == 1) {

    // LED on or off state
    ledTwoState = 1-ledTwoState;
    digitalWrite(ledTwoUpPin, ledTwoState);
    digitalWrite(ledTwoDnPin, ledTwoState);

    // Reset LED Two millis counter
    ledTwoMillis = currentMillis;

    // Debug
    Serial.print("Start Led TWO - ");
    Serial.println(ledTwoState);

  }
}
