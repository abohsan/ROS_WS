#include <WiFi.h>
#include "ros.h"
#include "tasks.h"
#include "Encoder.h"

const char* ssid     = "Jaguar";
const char* password = "Crawlerx01";
//const char* ssid     = "arc-demeter-streaming";
//const char* password = "eml-4840";

// run the following command to start the TCP connection
// rosrun rosserial_python serial_node.py tcp


IPAddress server(10, 0, 0, 203);
//IPAddress server(192, 168, 1, 3);

// OTHER pins

int frontLight = 33;

// motor pins

void setup() {
 pinMode(led, OUTPUT);
 digitalWrite(led, LOW);
  
//  Serial.begin(115200);
//  Serial.print("Connecting to ");
//  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
//    Serial.print(".");
  }
  // Print local IP address and start web server
//  Serial.println("");
//  Serial.println("WiFi connected.");
//  Serial.println("IP address: ");
//  Serial.println(WiFi.localIP());


//  encoder_setUp();
  ros_setup(server);
}

void loop()
{
  if (!rosConnected()) {
    stopRobot();
    blinkLed(200);
  } else {
    blinkLed(0);
  }
  node.spinOnce();
}
