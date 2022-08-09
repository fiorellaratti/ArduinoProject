
// Definition of variables and constants related to the left motor
const int IN1 = 13;  // Digital pin 13 to control left motor rotation direction
const int IN2 = 12;  // Digital pin 12 to control left motor rotation direction

// Definition of variables and constants related to the right motor
const int IN3 = 11;  // Digital pin 11 to control right motor rotation direction
const int IN4 = 10;  // Digital pin 10 to control right motor rotation direction
 
/* using ultrasonic sensor to measure the distance of the different obstacles 
in centimeters and display it through the serial port.
*/

const int triggerEmisor = 3;
const int echoreceiver = 2;
const int thresholdvalue = 20;
long timeinput;  // Stores the response time of the input sensor
float distanceinput;  // Stores the distance in cm that the object is
 
// Function that is executed only once when loading the program
void setup()
{
  // All pins are declared as outputs
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
 
  pinMode(triggerEmisor,OUTPUT); // The emitter emits so it is configured as output
  pinMode(echoreceiver,INPUT);   // The receiver receives so it is configured as input
  Serial.begin(9600); // Starts the serial communications port
}
// Function that repeats periodically
void loop()
{
  Ultrasonicsensor();
  // If the distance value is less than  cm, stop and go straight otherwise
  if(distanceinput>thresholdvalue)
  {
 
    forwardrobot();
  }
  else
  {
    rightrobot ();
  }
}
/*
Sensor function Ultrasound: to measure the length of the incoming pulse.
  Measures the time elapsed between sending the ultrasonic pulse
  and when the sensor receives the bounce,
  that is: since the echo pin begins to receive the bounce, HIGH,
  until it stops, LOW, the length of the incoming pulse.
*/
void Ultrasonicsensor()
{
    // The infrasound sensor is initialized
    digitalWrite(triggerEmisor,LOW);  // To stabilize
    delayMicroseconds(10);
 
    // We start the measurements
    // A signal is sent activating the trigger output for 10 microseconds
    digitalWrite(triggerEmisor, HIGH);  // ultrasonic pulse sending
    delayMicroseconds(10);
    timeinput=pulseIn(echoreceiver, HIGH); 
    distanceinput= int(0.017*timeinput); // Formula to calculate the distance in cm
    Serial.println("The value of the distance is");
    Serial.println(distanceinput);
    delay(200);
}
/*
  Robot Advance function: this function will cause both motors to activate at full power
   so the robot will move forward
*/
void forwardrobot()
{
  // Left motor
  // Holding one pin HIGH and the other LOW rotates the motor in one direction
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  // Right motor
  // Holding one pin HIGH and the other LOW rotates the motor in one direction
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
}
/*
  Robot function Reverse: this function will cause both motors to activate at full power
   in the opposite direction to the previous one so the robot will move backwards
*/
void backwardsrobot()
{
  // Left motor
  // By keeping one pin LOW and the other HIGH, the motor rotates in the opposite direction to the previous one
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  // Motor derecho
  // By keeping one pin LOW and the other HIGH, the motor rotates in the opposite direction to the previous one
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
}
/*
  Right robot function: this function will drive the left motor and stop the right
   so the car will turn to the right (clockwise)
*/
void rightrobot()
{
  //  Left motor
  // Left motor is activated
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  // Right motor
  // Right motor stops
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, LOW);
}
/*
  Robot Left function: this function will drive the right motor and stop the left
   so the car will turn to the left (counterclockwise)
*/
void leftrobot ()
{
   //  Left motor
  // Left motor stops
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);
  // Right motor
  // Right motor is activated
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
}
/*
  Robot Stop function: this function will stop both motors
   so the robot will stop.
*/
void stoprobot()
{
  // Left motor
  // Left motor stops
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);
  // Right motor
  // Right motor stops
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, LOW);
}