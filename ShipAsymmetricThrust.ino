/* Read the throttle and rudder of a ship and give asymmetric thrust to the motors. */

#include <Servo.h>

Servo motor[2];

float MIN_POWER = 1000.0f; // 1000 PWM is for zero power/angle
float MAX_POWER = 2000.0f; // 2000 PWM is for full power/angle
int PIN_RUDDER = 9;
int PIN_THROTTLE = 10;

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(PIN_RUDDER, INPUT); // Attach to rudder input
  pinMode(PIN_THROTTLE, INPUT); // Attach to throttle input

  Serial.begin(9600);
  Serial.println("Activating motor");
  motor[0].attach(3);
  motor[0].write(30);
  //delay(5000);
}

// the loop function runs over and over again forever
void loop() {
  long rudderIn = pulseIn(PIN_RUDDER, HIGH);
  long throttleIn = pulseIn(PIN_THROTTLE, HIGH);

  float throttlePercent = (throttleIn - MIN_POWER) / MIN_POWER; // From 0 to 1
  float rudderPercent = (((rudderIn - MIN_POWER) / MIN_POWER) * 2.0f) - 1.0f; // From -1 to 1

  float leftMultiplier = constrain(throttlePercent - rudderPercent, 0.0f, 1.0f);
  float rightMultiplier = constrain(throttlePercent + rudderPercent, 0.0f, 1.0f);
  Serial.print("Left: ");
  Serial.print(leftMultiplier);
  Serial.print(" Right: ");
  Serial.println(rightMultiplier);
  
}
