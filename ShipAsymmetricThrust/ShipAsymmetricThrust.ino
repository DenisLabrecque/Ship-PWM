/* Read the throttle and rudder channels of a radio-controlled vehicle from the receiver's PWM signals. */
/* Mix the throttle and rudder to drive the ship with asymmetric thrust using two motors. */
/* Detect when the ship is active/inactive and change the navigation lights accordingly. */

#include <Servo.h>

Servo motor[2];
unsigned long last_input_time = 0;  // Milliseconds when the last throttle/rudder input occurred
unsigned long time_since_input = 0; // Milliseconds since the last throttle/rudder input

float throttle_percent = 0.0f;
float rudder_percent = 0.0f;
float left_power = 0.0f;
float right_power = 0.0f;
int light_mode;

volatile int throttle_pwm_value = 0;
volatile int rudder_pwm_value = 0;
volatile unsigned long throttle_previous_time = 0;
volatile unsigned long rudder_previous_time = 0;

const float MIN_POWER = 1000.0f; // 1000 PWM is for zero power/angle
const float MAX_POWER = 2000.0f; // 2000 PWM is for full power/angle
const byte PIN_INTERRUPT_THROTTLE = 2;
const byte PIN_INTERRUPT_RUDDER = 3;
const byte PIN_PWM_MOTOR1 = 11;
const byte PIN_PWM_MOTOR2 = 5;
const byte PIN_WHITE_LIGHTS = 6;          // Bow, mast, and stern (masthead lights)
const byte PIN_SIDELIGHTS = 7;            // Red to the left (port) and green to the right (starboard)
const byte PIN_STOPPED_LIGHTS = 8;        // Red "not under command" lights
const int ANCHOR_LIGHT_INTERVAL = 5000;   // Milliseconds before navigation lights are turned off
const int STOPPED_LIGHT_INTERVAL = 20000; // Milliseconds before the ship is "not under command"
const float TOLERANCE = 0.305f;


// the setup function runs once when you press reset or power the board
void setup() {
  // Throttle interrupt
  pinMode(PIN_INTERRUPT_THROTTLE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT_THROTTLE), rising_throttle, RISING);

  // Rudder interrupt
  pinMode(PIN_INTERRUPT_RUDDER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT_RUDDER), rising_rudder, RISING);

  pinMode(PIN_WHITE_LIGHTS, OUTPUT);
  pinMode(PIN_SIDELIGHTS, OUTPUT);
  pinMode(PIN_STOPPED_LIGHTS, OUTPUT);

  Serial.begin(9600);
  Serial.println("Activating motor");
  motor[0].attach(PIN_PWM_MOTOR1);
  motor[1].attach(PIN_PWM_MOTOR2);
  motor[0].write(30);
  motor[1].write(30);
  delay(5000);
}


// the loop function runs over and over again forever
void loop() {
  mix_motor_torque();
  control_lights();
}


// Use the throttle and rudder channels; find their percent and match the power
// of each motor to the rudder's turn.
void mix_motor_torque() {
  // Mix motors to the rudder
  left_power = constrain(throttle_percent - rudder_percent, 0.0f, 1.0f);
  right_power = constrain(throttle_percent + rudder_percent, 0.0f, 1.0f);
  
  Serial.print("Left: ");
  Serial.print(left_power);
  Serial.print(" Right: ");
  Serial.println(right_power);

  // Send signal to the motors
  motor[0].write(180 * left_power);
  motor[1].write(180 * right_power);
}


// Based on current speed and time since the last input, determine whether the ship is running,
// at anchor, or completely stopped. Set the lights accordingly.
void control_lights() {
  if(left_power > TOLERANCE || right_power > TOLERANCE || rudder_percent > TOLERANCE) {
    last_input_time = millis();
    time_since_input = 0;
  }
  else {
    time_since_input = millis() - last_input_time;
  }
  
  // There is relatively recent input
  if(time_since_input < ANCHOR_LIGHT_INTERVAL) {
    // Running lights
    //Serial.println("Running lights");
    digitalWrite(PIN_WHITE_LIGHTS, HIGH);
    digitalWrite(PIN_SIDELIGHTS, HIGH);
    digitalWrite(PIN_STOPPED_LIGHTS, LOW);
  }
  // There has been no input for a little while
  else if(time_since_input < STOPPED_LIGHT_INTERVAL) {
    // Anchor lights
    //Serial.println("Anchor lights");
    digitalWrite(PIN_WHITE_LIGHTS, HIGH);
    digitalWrite(PIN_SIDELIGHTS, LOW);
    digitalWrite(PIN_STOPPED_LIGHTS, LOW);
  }
  // There has been no input for a long time
  else {
    // Stopped "not under command" lights
    //Serial.println("Stopped lights");
    digitalWrite(PIN_WHITE_LIGHTS, LOW);
    digitalWrite(PIN_SIDELIGHTS, LOW);
    digitalWrite(PIN_STOPPED_LIGHTS, HIGH);
  }
}


// Detect the minimum/maximum times of PWM waves to find the signal value.
void rising_throttle() {
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT_THROTTLE), falling_throttle, FALLING);
  throttle_previous_time = micros();
}

void falling_throttle() {
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT_THROTTLE), rising_throttle, RISING);
  throttle_pwm_value = micros() - throttle_previous_time;
  throttle_percent = (throttle_pwm_value - MIN_POWER) / MIN_POWER; // From 0 to 1
}


// Detect the minimum/maximum times of PWM waves to find the signal value.
void rising_rudder() {
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT_RUDDER), falling_rudder, FALLING);
  rudder_previous_time = micros();
}

void falling_rudder() {
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT_RUDDER), rising_rudder, RISING);
  rudder_pwm_value = micros() - rudder_previous_time;
  rudder_percent = (((rudder_pwm_value - MIN_POWER) / MIN_POWER) * 2.0f) - 1.0f; // From -1 to 1
}
