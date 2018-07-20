#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

char auth[] = "xxxxxxxxxxxxxxx"; //(Din motive de securitate nu am vrut sa afisez tokenul.

char ssid[] = "Orange-tFpc"; //SSID
char pass[] = "RYQxcUkB"; //PASS
//Este necesara o retea locala (hotspot)

BLYNK_WRITE(V1) {
  int x = param[0].asInt();
  int y = param[1].asInt();

  //---DEBUGGING---
  Serial.print("Xcam = ");
  Serial.print(x);
  Serial.print("; Ycam = ");
  Serial.println(y);
  Serial.println();
}

BLYNK_WRITE(V2){
  int speedM = param[0].asInt();
  int dirServo = param[1].asInt();

  //---DEBUGGING---
  Serial.print("speedM = ");
  Serial.print(x);
  Serial.print("; driServo = ");
  Serial.println(y);
  Serial.println();
}

void setup()
{
  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass);
}

void loop()
{
  Blynk.run();
}

