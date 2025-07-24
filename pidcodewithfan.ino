#include <math.h>
#include <PID_v1.h>

// ---------------- Thermistor Setup ----------------
const int thermistorPin = A0;
const float SERIES_RESISTOR = 10130.0;
const float NOMINAL_RESISTANCE = 10000.0;
const float NOMINAL_TEMPERATURE = 25.0;
const float B_COEFFICIENT = 3950.0;
const float SUPPLY_VOLTAGE = 5.0;

// ---------------- Motor Driver Pins ----------------
const int dirPin = 8;     // DIR pin
const int pwmPin = 9;     // PWM pin
const int brkPin = 10;    // BRK pin

// ---------------- Fan Control (NPN Transistor) ----------------
const int fanPin = 7;     // Connects to base of NPN transistor

// Fan Thresholds
const float FAN_ON_TEMP = 45.0;   // Fan ON if above this
const float FAN_OFF_TEMP = 40.0;  // Fan OFF if below this

// ---------------- PID Variables ----------------
double Setpoint = 45.0;    // Desired temperature (°C)
double Input;              // Measured temperature
double Output;             // PWM output for Peltier

double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);

  // Set pin modes
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(brkPin, OUTPUT);
  pinMode(fanPin, OUTPUT);

  // Set initial states
  digitalWrite(dirPin, HIGH);   // Peltier in heating mode
  digitalWrite(brkPin, LOW);    // Enable motor driver
  digitalWrite(fanPin, HIGH);   // HIGH = Fan OFF for NPN

  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
}

void loop() {
  Input = readTemperature();
  myPID.Compute();

  // Apply PWM to Peltier
  analogWrite(pwmPin, Output);

  // Control fan using inverse logic for NPN transistor
  if (Input >= FAN_ON_TEMP) {
    digitalWrite(fanPin, LOW);   // LOW = fan ON
  } else if (Input <= FAN_OFF_TEMP) {
    digitalWrite(fanPin, HIGH);  // HIGH = fan OFF
  }

  // Debugging output
  Serial.print("Temp: ");
  Serial.print(Input);
  Serial.print(" °C | PWM: ");
  Serial.print(Output);
  Serial.print(" | Fan: ");
  Serial.print(digitalRead(fanPin) == LOW ? "ON" : "OFF");
  Serial.print(" | DIR: ");
  Serial.println(digitalRead(dirPin) ? "Heating" : "Cooling");

  delay(500);
}

// ---------------- Read Temperature from NTC ----------------
float readTemperature() {
  int analogValue = analogRead(thermistorPin);
  float voltage = analogValue * SUPPLY_VOLTAGE / 1023.0;
  float resistance = (SUPPLY_VOLTAGE - voltage) * SERIES_RESISTOR / voltage;

  float steinhart = resistance / NOMINAL_RESISTANCE;
  steinhart = log(steinhart);
  steinhart /= B_COEFFICIENT;
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;

  return steinhart;
}
