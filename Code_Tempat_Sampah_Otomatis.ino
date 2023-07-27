#include <ESP32Servo.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#define BLYNK_PRINT Serial
#define AUTH "OJ2AQD8UzKI0pPhPMpo2pwrk427D7G_B"
#define SSID "Memed"
#define PASSWORD "juni1576"

#define TRIG_PIN  14  // ESP32 pin GIOP23 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN  12  // ESP32 pin GIOP22 connected to Ultrasonic Sensor's ECHO pin
#define SERVO_PIN1 33   //ESP32 pin GIOP26 connected to Servo Motor's pin
#define DISTANCE_THRESHOLD  50 // centimeters
#define DHTTYPE DHT11
#define DHTPIN 32

DHT dht(DHTPIN, DHTTYPE);
int humidity, temp;                
unsigned long previousMillis = 0;
const long interval = 15000;
Servo servo1; // create servo object to control a servo
int button_manual = 0;

// variables will change:
float duration_us, distance_cm;

BLYNK_WRITE(V2) {
  int button_manual = param.asInt();
  servo1.write(button_manual ? 125 : 0);
  delay(2000);
}

void setup() {
  Serial.begin (921600);       // initialize serial port
  WiFi.begin(SSID, PASSWORD);
  pinMode(TRIG_PIN, OUTPUT); // set ESP32 pin to output mode
  pinMode(ECHO_PIN, INPUT);  // set ESP32 pin to input mode
  servo1.attach(SERVO_PIN1);   // attaches the servo on pin 9 to the servo object
  servo1.write(0);
   Blynk.begin("OJ2AQD8UzKI0pPhPMpo2pwrk427D7G_B", SSID, PASSWORD);
   dht.begin();
   while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Menghubungkan ke WiFi...");
  }
}
 
  bool isFirstConnect = true;

 BLYNK_CONNECTED() {
  if (isFirstConnect)
  {
    Blynk.syncAll();
    isFirstConnect = false;
  }
}


void loop() {
  // generate 10-microsecond pulse to TRIG pin
  Blynk.run();
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(ECHO_PIN, HIGH);
  // calculate the distance
  distance_cm = 0.017 * duration_us;
  Blynk.virtualWrite(V1, distance_cm) ;

  if (distance_cm < DISTANCE_THRESHOLD)
    servo1.write(125); // rotate servo motor to 90 degree
  else if (!button_manual)
    servo1.write(0);  // rotate servo motor to 0 degree

    


 
  // print the value to Serial Monitor
  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  delay(2000);

   gettemperature();
}
void gettemperature() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
   
    humidity = dht.readHumidity();        
    temp = dht.readTemperature(); 
    Blynk.virtualWrite(V4, temp);
    Blynk.virtualWrite(V3, humidity);
    
    if (isnan(humidity) || isnan(temp)) {
      Serial.println("Sensor Tidak Terbaca");
      return;
    }
  }
}