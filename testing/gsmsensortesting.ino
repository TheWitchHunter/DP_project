#include <SoftwareSerial.h>

// Define ultrasonic sensor pins
#define TRIGGER_PIN_1 22
#define ECHO_PIN_1 23
#define TRIGGER_PIN_2 24
#define ECHO_PIN_2 25

// Define GPS pins
#define GPS_RX_PIN 8
#define GPS_TX_PIN 7

// Define GSM pins
#define GSM_RX_PIN 6
#define GSM_TX_PIN 5

// Define phone number for notification
String phoneNumber = "+1234567890";

// Create software serial objects for GPS and GSM modules
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
SoftwareSerial gsmSerial(GSM_RX_PIN, GSM_TX_PIN);

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  gpsSerial.begin(9600);
  gsmSerial.begin(9600);
  
  // Wait for the GPS and GSM modules to start up
  delay(3000);
  
  // Turn on echo mode for the GSM module
  gsmSerial.println("ATE1");
  delay(1000);
  
  // Set text mode for the GSM module
  gsmSerial.println("AT+CMGF=1");
  delay(1000);
  
  // Set up ultrasonic sensors
  pinMode(TRIGGER_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIGGER_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
}

void loop() {
  // Read GPS data
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    Serial.write(c);
  }
  
  // Check for cracks using ultrasonic sensors
  long duration_1, duration_2, distance_1, distance_2;
  digitalWrite(TRIGGER_PIN_1, LOW);
  digitalWrite(TRIGGER_PIN_2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN_1, HIGH);
  digitalWrite(TRIGGER_PIN_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_1, LOW);
  digitalWrite(TRIGGER_PIN_2, LOW);
  duration_1 = pulseIn(ECHO_PIN_1, HIGH);
  duration_2 = pulseIn(ECHO_PIN_2, HIGH);
  distance_1 = duration_1 * 0.034 / 2;
  distance_2 = duration_2 * 0.034 / 2;
  if (distance_1 < 10 || distance_2 < 10) {
    // If a crack is detected, send a notification via SMS
    String message = "Crack detected at latitude " + String(gps.location.lat()) + ", longitude " + String(gps.location.lng());
    gsmSerial.println("AT+CMGS=\"" + phoneNumber + "\"");
    delay(1000);
    gsmSerial.println(message);
    delay(1000);
    gsmSerial.write((char)26);
    delay(10000);
  }
}

