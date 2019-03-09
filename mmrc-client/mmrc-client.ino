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
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>

#include "MMRCsettings.h"

WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Servo initialisation
Servo turnoutOneServo;       // Create servo object to control turnout 1
Servo turnoutTwoServo;       // Create servo object to control turnout 2

// -----------------------------------------------------
// Convert settings in MMRCsettings.h to constants and variables
const char* mqttBrokerIP = IP;
String deviceID = DEVICEID;
String nodeID01 = NODEID01;
String nodeID02 = NODEID02;
String cccCLEES = CLEES;

// -----------------------------------------------------
// Various variable definitions

// Variable for topics to subscribe to
const int nbrSubTopics = 2;
String subTopic[nbrSubTopics];

// Variable for topics to publish to
const int nbrPubTopics = 13;
String pubTopic[nbrPubTopics];
String pubTopicContent[nbrPubTopics];

String pubTopicTurnoutOne;
String pubTopicTurnoutTwo;
String pubTopicDeviceState;

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
// Turnout TWO variables
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
int btnOnePin = D1;    // Pin for first turnout
int ledOneUpPin = D2;
int ledOneDnPin = D3;
int turnoutOnePin = D4;
int btnTwoPin = D5;    // Pin for second turnout
int ledTwoUpPin = D6;
int ledTwoDnPin = D7;
int turnoutTwoPin = D8;

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
    deviceID = "clees";
//    subTopic[0] = deviceID+"/"+cccModule+"/cmd/turnout/"+cccObject;
//    pubTopic[0] = deviceID+"/"+cccModule+"/rpt/turnout"+cccObject;
  } else {
    // Subscribe
    subTopic[0] = "mmrc/"+deviceID+"/"+nodeID01+"/turnout/set";
    subTopic[1] = "mmrc/"+deviceID+"/"+nodeID02+"/turnout/set";

    // Publish
    pubTopic[0] = "mmrc/"+deviceID+"/$name";
    pubTopicContent[0] = "SJ06 växelkort";

    pubTopic[1] = "mmrc/"+deviceID+"/$state";
    pubTopicContent[1] = "lost";

    pubTopic[2] = "mmrc/"+deviceID+"/$nodes";
    pubTopicContent[2] = nodeID01+","+nodeID02;
    
    pubTopic[3] = "mmrc/"+deviceID+"/"+nodeID01+"/$name";
    pubTopicContent[3] = "Växelnod 1";

    pubTopic[4] = "mmrc/"+deviceID+"/"+nodeID01+"/$type";
    pubTopicContent[4] = "Turnout control";

    pubTopic[5] = "mmrc/"+deviceID+"/"+nodeID01+"/$properties";
    pubTopicContent[5] = "turnout";

    pubTopic[6] = "mmrc/"+deviceID+"/"+nodeID01+"/turnout/$name";
    pubTopicContent[6] = "Växel 1";

    pubTopic[7] = "mmrc/"+deviceID+"/"+nodeID01+"/turnout/$datatype";
    pubTopicContent[7] = "string";

    pubTopic[8] = "mmrc/"+deviceID+"/"+nodeID02+"/$name";
    pubTopicContent[8] = "Växelnod 2";

    pubTopic[9] = "mmrc/"+deviceID+"/"+nodeID02+"/$type";
    pubTopicContent[9] = "Turnout control";

    pubTopic[10] = "mmrc/"+deviceID+"/"+nodeID02+"/$properties";
    pubTopicContent[10] = "turnout";

    pubTopic[11] = "mmrc/"+deviceID+"/"+nodeID02+"/turnout/$name";
    pubTopicContent[11] = "Växel 2";

    pubTopic[12] = "mmrc/"+deviceID+"/"+nodeID02+"/turnout/$datatype";
    pubTopicContent[12] = "string";

    // Often used publish topics
    pubTopicTurnoutOne = "mmrc/"+deviceID+"/"+nodeID01+"/turnout";
    pubTopicTurnoutTwo = "mmrc/"+deviceID+"/"+nodeID02+"/turnout";
    pubTopicDeviceState = pubTopic[1];
    
 }

  // Unique name for this client
  if (cccCLEES == "1") {
    clientID = "CLEES "+deviceID;
  } else {
    clientID = "MMRC "+deviceID;
  }

  // Connect to wifi network
  WiFiManager wifiManager;

  // Start the WifiManager for connection to network
  // First parameter is name of access point, second is the password
  wifiManager.autoConnect("MMRC 2-2 Turnout", "1234");

  // Connect to MQTT broker and define function to handle callbacks
  client.setServer(mqttBrokerIP, 1883);
  client.setCallback(mqttCallback);

}


/**
 * (Re)connects to MQTT broker and subscribes/publishes to one or more topics
 */
void mqttConnect() {
  char tmpTopic[254];
  char tmpContent[254];
  char tmpID[clientID.length()];
  char* tmpMessage = "lost";
  
  // Convert String to char* for the client.subribe() function to work
  clientID.toCharArray(tmpID, clientID.length()+1);
  pubTopicDeviceState.toCharArray(tmpTopic, pubTopicDeviceState.length()+1);

  // Loop until we're reconnected
  while (!client.connected()) {
  Serial.print("MQTT connection...");
  // Attempt to connect
  // boolean connect (tmpID, pubTopicDeviceState, willQoS, willRetain, willMessage)
  if (client.connect(tmpID,tmpTopic,0,true,tmpMessage)) {
//  if (client.connect(tmpID)) {
    Serial.println("connected");
    Serial.print("MQTT client id: ");
    Serial.println(tmpID);
    Serial.println("Subscribing to:");

    // Subscribe to all topics
    for (int i=0; i < nbrSubTopics; i++){
      // Convert String to char* for the client.subribe() function to work
      subTopic[i].toCharArray(tmpTopic, subTopic[i].length()+1);

      // ... print topic
      Serial.print(" - ");
      Serial.println(tmpTopic);

      // ... and subscribe to topic
      client.subscribe(tmpTopic);
    }

    // Publish to all topics
    Serial.println("Publishing to:");
    for (int i=0; i < nbrPubTopics; i++){
      // Convert String to char* for the client.publish() function to work
      pubTopic[i].toCharArray(tmpTopic, pubTopic[i].length()+1);
      pubTopicContent[i].toCharArray(tmpContent, pubTopicContent[i].length()+1);

      // ... print topic
      Serial.print(" - ");
      Serial.print(tmpTopic);
      Serial.print(" = ");
      Serial.println(tmpContent);

      // ... and subscribe to topic
      client.publish(tmpTopic, tmpContent);

    }    

  } else {

    // Show why the connection failed
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");

    // Wait 5 seconds before retrying
    delay(5000);

    }
  }

  // Publish new device state = active
  mqttPublish(pubTopicDeviceState, "ready", 1);
  Serial.println("---");

}

/**
 * Function to handle MQTT messages sent to this device
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Don't know why this have to be done :-(
  payload[length] = '\0';

  // Print the topic
  Serial.print("Incoming: ");
  Serial.print(topic);
  Serial.print(" = ");

  // Make strings and print payload
  String msg = String((char*)payload);
  String tpc = String((char*)topic);
  Serial.println(msg);

  // Check set command for turnout ONE
  if (tpc == subTopic[0] && actionOne == 0){

    // Check message
    if (msg == "normal" && turnoutOneState == REVERSE) {
      turnoutOneStart();
    }
    if (msg == "reverse" && turnoutOneState == NORMAL) {
      turnoutOneStart();
    }
  }

  // Check set command for turnout TWO
  if (tpc == subTopic[1] && actionTwo == 0){

    // Check message
    if (msg == "normal" && turnoutTwoState == REVERSE) {
      turnoutTwoStart();
    }
    if (msg == "reverse" && turnoutTwoState == NORMAL) {
      turnoutTwoStart();
    }
  }

}


/**
 * Publish messages to MQTT broker 
 */
void mqttPublish(String pbTopic, String pbPayload, boolean retained) {

  // Convert String to char* for the client.publish() function to work
  char msg[pbPayload.length()+1];
  pbPayload.toCharArray(msg, pbPayload.length()+1);
  char tpc[pbTopic.length()+1];
  pbTopic.toCharArray(tpc, pbTopic.length()+1);

  // Report back to pubTopic[]
  client.publish(tpc, msg, retained);

  // TODO check "check" integer to see if all went ok

  // Print information
  Serial.print("Publish: ");
  Serial.print(pbTopic);
  Serial.print(" = ");
  Serial.println(pbPayload);

  // Action 1 is executed, ready for new action
//  actionOne = 0;

}

/**
 * Initialize turnout ONE moving
 */
void turnoutOneStart() {

  // Turn off both LED
  digitalWrite(ledOneUpPin, LOW);
  digitalWrite(ledOneDnPin, LOW);

  // Publish moving message
  mqttPublish(pubTopicTurnoutOne, "moving", 1);

  // Publish button press
  if (turnoutOneState == NORMAL) {
      turnoutOneState = REVERSE;
  } else {
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
  // Turn off both LED
  digitalWrite(ledTwoUpPin, LOW);
  digitalWrite(ledTwoDnPin, LOW);

  // Publish moving message
  mqttPublish(pubTopicTurnoutTwo, "moving", 1);

  // Publish button press
  if (turnoutTwoState == NORMAL) {
      turnoutTwoState = REVERSE;
  } else {
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

      // Debug
      Serial.print(" - Stopped");

      // Set max position (useful if we take big steps...)
      turnoutOnePosition = turnoutOneNormal;

      // End this action and allow for the next
      actionOne = 0;

      // Publish turnout final state
      mqttPublish(pubTopicTurnoutOne, "normal", 1);

      // Set led status
      digitalWrite(ledOneUpPin, HIGH);
      digitalWrite(ledOneDnPin, LOW);
    }

 } else {

    // Count up the position by [turnoutOneStep] steps
    turnoutOnePosition = turnoutOnePosition - turnoutOneStep;

    // Debug
    Serial.print("Move down");

    // Check if we reached the right position
    if (turnoutOnePosition <= turnoutOneReverse) {

      // Debug
      Serial.print(" - Stopped");

      // Set min position (useful if we take big steps...)
      turnoutOnePosition = turnoutOneReverse;

      // End this action and allow for the next
      actionOne = 0;

      // Publish turnout final state
      mqttPublish(pubTopicTurnoutOne, "reverse", 1);

      // Set led status
      digitalWrite(ledOneUpPin, LOW);
      digitalWrite(ledOneDnPin, HIGH);
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

      // Debug
      Serial.print(" - Stopped");

      // Set max position (useful if we take big steps...)
      turnoutTwoPosition = turnoutTwoNormal;

      // End this action and allow for the next
      actionTwo = 0;

      // Publish turnout final state
      mqttPublish(pubTopicTurnoutTwo, "normal", 1);

      // Set led status
      digitalWrite(ledTwoUpPin, HIGH);
      digitalWrite(ledTwoDnPin, LOW);

    }

 } else {

    // Count up the position by [turnoutTwoStep] steps
    turnoutTwoPosition = turnoutTwoPosition - turnoutTwoStep;

    // Debug
    Serial.print("Move down");

    // Check if we reached the right position
    if (turnoutTwoPosition <= turnoutTwoReverse) {

      // Debug
      Serial.print(" - Stopped");

      // Set min position (useful if we take big steps...)
      turnoutTwoPosition = turnoutTwoReverse;

      // End this action and allow for the next
      actionTwo = 0;

      // Publish turnout final state
      mqttPublish(pubTopicTurnoutTwo, "reverse", 1);
      
      // Set led status
      digitalWrite(ledTwoUpPin, LOW);
      digitalWrite(ledTwoDnPin, HIGH);

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
  if (!client.connected()) {
    mqttConnect();
  }

  // -----------------------------------------------------
  // Wait for incoming MQTT messages
  client.loop();

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
//    Serial.print("Start Led ONE - ");
//    Serial.println(ledOneState);

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
//    Serial.print("Start Led TWO - ");
//    Serial.println(ledTwoState);

  }
}
