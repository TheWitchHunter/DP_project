
// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

// Define ultrasonic sensor pins
#define TRIGGER_PIN_1 22
#define ECHO_PIN_1 23

// Define GPS pins
#define GPS_RX_PIN 8
#define GPS_TX_PIN 7

#include <SoftwareSerial.h>
#include <TinyGPS.h>

long lat,lon; // create variable for latitude and longitude object
 
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); // create gps sensor connection
TinyGPS gps; // create gps object

#define buzzer A2


void setup()
{
  Serial.begin(9600);  ///baudrate bps
  gpsSerial.begin(4800); // connect gps sensor
  // Wait for the GPS and GSM modules to start up
  delay(3000);
  // Set up ultrasonic sensors
  pinMode(TRIGGER_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);

  // Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);



  pinMode(buzzer, OUTPUT);
//   pinMode(m1, OUTPUT);
//   pinMode(m2, OUTPUT);
  digitalWrite(buzzer,HIGH);
  delay(10000);
  digitalWrite(buzzer,HIGH);
  delay(1000);
 digitalWrite(buzzer,LOW);
 delay(1000);
 tracking();
} 
  
void loop()
{

    directionControl();
    delay(1000);

    while(gpsSerial.available()){ // check for gps data
    if(gps.encode(gpsSerial.read())){ // encode gps data
    gps.get_position(&lat,&lon); // get latitude and longitude
    // display position
    // Serial.print("Position: ");
    // Serial.print("lat: ");Serial.print(lat);Serial.print(" ");// print latitude
    // Serial.print("lon: ");Serial.println(lon); // print longitude
    }
    }

   // Check for cracks using ultrasonic sensors
    long duration_1, duration_2, distance_1, distance_2;
    digitalWrite(TRIGGER_PIN_1, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN_1, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN_1, LOW);
    duration_1 = pulseIn(ECHO_PIN_1, HIGH);
    distance_1 = duration_1 * 0.034 / 2;

    if (distance_1 < 10 || distance_2 < 10) {
      // If a crack is detected, send a notification via SMS
      digitalWrite(buzzer, HIGH);
      Stopmotor()
      tracking();
    }
// using another ultrasonic to detect obstacle

//   else if ((sensor1<20) &&( sensor1>=2))
//   {
//     digitalWrite(buzzer, HIGH);
//    digitalWrite(m1, LOW); 
//    digitalWrite(m2, LOW); 
//    tracking1();
//   }
 
  else 
  {
   digitalWrite(buzzer,LOW);
  //  digitalWrite(m1, HIGH);
  //  digitalWrite(m2, HIGH);
      directionControl() ;
  }
 delay(100);
}

 void init_sms()
 {
  Serial.println("AT+CMGF=1");
  delay(400);
  Serial.println("AT+CMGS=\"+918742960961\"");   // use your 10 digit cell no. here
  delay(400);
 }
//   void init_sms1()
//  {
//   Serial.println("AT+CMGF=1");
//   delay(400);
//   Serial.println("AT+CMGS=\"+919368389316\"");   // use your 10 digit cell no. here
//   delay(400);
//  }
  
 void send_data(String message)
 {
  Serial.println(message);
  delay(200);
 }
 
 void send_sms()
 {
  Serial.write(26);
 }
 
 void tracking()
 {
  init_sms();
  send_data("Track fault");
  send_data(latitude);
   send_data("Longitude:");
   send_data(longitude);
  send_sms();
  delay(5000);
  init_sms1();
  send_data("Track fault");
  send_data(latitude);
   send_data("Longitude:");
   send_data(longitude);
  send_sms();
  delay(5000);
 }
 
//  for obstacle detection
//  void tracking1()
//  {
//   init_sms();
//   send_data("Obstraction Ahead");
  
//   send_sms();
//   delay(5000);
//    init_sms1();
//   send_data("Obstraction Ahead");
//   send_sms();
//   delay(5000);
//  }
 
//  void ultrasensor(int trigPin,int echoPin)
//  { 
//   digitalWrite(trigPin, LOW);  // Added this line
//   delayMicroseconds(2); // Added this line
//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(10); // Added this line
//   digitalWrite(trigPin, LOW);
//   duration = pulseIn(echoPin, HIGH);
//   distance = (duration/2) / 29.1;
//  }

void directionControl() {
	// Set motors to maximum speed
	// For PWM maximum possible values are 0 to 255
	analogWrite(enA, 255);
	analogWrite(enB, 255);

	// Turn on motor A & B in forward direction
	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);
}

void Stopmotor(){
  // Now turn off motors
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}
