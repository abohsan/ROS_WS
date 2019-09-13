#include <WiFi.h>
#include "ros.h"
#include "tasks.h"

const char* ssid     = "Jaguar";
const char* password = "Crawlerx01";

IPAddress server(10, 0, 0, 203);

// OTHER pins

int frontLight = 33;

// motor pins

void setup() {
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  pinMode(led, OUTPUT);


  rosSetting(server);
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
