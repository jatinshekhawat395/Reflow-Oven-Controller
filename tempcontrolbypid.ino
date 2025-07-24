#include <math.h>
#include <PID_v1.h>

// Thermistor setup
const int thermistorPin = A0;
const float SERIES_RESISTOR = 10130.0;
const float NOMINAL_RESISTANCE = 10000.0;
const float NOMINAL_TEMPERATURE = 25.0;
const float B_COEFFICIENT = 3950.0;
const float SUPPLY_VOLTAGE = 5.0;

// Peltier control pins
const int dirPin = 8;
const int pwmPin = 9;
const int brkPin = 10;

// PID setup
double Setpoint = 50.0;   // Target temperature (change as needed)
double Input;             // Current temp
double Output;            // PWM output

// PID Tuning values (can adjust for smoother behavior)
double Kp = 2.0;
double Ki = 5.0;
double Kd = 1.0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);

  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(brkPin, OUTPUT);

  digitalWrite(dirPin, HIGH);  // Set direction to heating (can invert if needed)
  digitalWrite(brkPin, LOW);   // Enable motor driver

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // PWM range for analogWrite
}

void loop() {
  Input = readTemperature();     // Read current temperature
  myPID.Compute();               // Update PID output (PWM)

  analogWrite(pwmPin, Output);   // Apply PWM to Peltier

  // Debug output
  Serial.print("Temp: ");
  Serial.print(Input);
  Serial.print(" °C | PWM: ");
  Serial.println(Output);

  delay(500);
}

// Function to convert thermistor analog reading to temperature
float readTemperature() {
  int analogValue = analogRead(thermistorPin);
  float voltage = analogValue * SUPPLY_VOLTAGE / 1023.0;
  float resistance = (SUPPLY_VOLTAGE - voltage) * SERIES_RESISTOR / voltage;

  float steinhart;
  steinhart = resistance / NOMINAL_RESISTANCE;
  steinhart = log(steinhart);
  steinhart /= B_COEFFICIENT;
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;

  return steinhart;
}
