#include <math.h>
#include <AutoPID.h>

// Thermistor Parameters
const int thermistorPin = A0;
const float SERIES_RESISTOR = 10130.0;
const float NOMINAL_RESISTANCE = 10000.0;
const float NOMINAL_TEMPERATURE = 25.0;
const float B_COEFFICIENT = 3950.0;
const float SUPPLY_VOLTAGE = 5.0;

// Motor Driver Pins
const int dirPin = 8;
const int pwmPin = 9;
const int brkPin = 10;

// Temperature & Control Variables
double currentTemp = 0.0;
double setTemp = 45.0;
double outputPWM = 0;

// PID Configuration
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
AutoPID myPID(&currentTemp, &setTemp, &outputPWM, OUTPUT_MIN, OUTPUT_MAX, 2.0, 1.0, 1.5);

// Reflow Stage Parameters
const int NUM_STAGES = 4;
float stageTemps[NUM_STAGES] = {45.0, 50.0, 55.0, 40.0};   // Preheat, Soak, Reflow, Cool
int stageTimes[NUM_STAGES] = {90, 45, 50, 190};         // Seconds
int currentStage = 0;

unsigned long stageStartTime = 0;
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(brkPin, OUTPUT);
  digitalWrite(brkPin, LOW);  // Disable braking

  myPID.setBangBang(0.5);     // Deadband to prevent rapid direction flipping
  myPID.setTimeStep(500);     // PID update every 500ms

  setTemp = stageTemps[currentStage];
  stageStartTime = millis();
  
  Serial.println("Stage,Time(s),Current_Temp,Set_Temp,Error,PWM");
}

void loop() {
  // Read temperature
  currentTemp = readTemperature();
  double error = setTemp - currentTemp;

  // Run PID
  myPID.run();

  // Direction control
  if (error > 0.5) {
    digitalWrite(dirPin, HIGH); // Heat
  } else if (error < -0.5) {
    digitalWrite(dirPin, LOW);  // Cool
  }

  // Apply PWM output
  analogWrite(pwmPin, (int)outputPWM);

  // Time tracking
  unsigned long currentTime = millis();
  unsigned long elapsedTime = (currentTime - stageStartTime) / 1000;

  // Check if stage is completed
  if (elapsedTime >= stageTimes[currentStage]) {
    currentStage++;
    if (currentStage < NUM_STAGES) {
      setTemp = stageTemps[currentStage];
      stageStartTime = millis();
    } else {
      // All stages done, stop output
      analogWrite(pwmPin, 0);
      Serial.println("Reflow complete.");
      while (1); // Stop loop
    }
  }

  // Print status every 1 sec
  if (currentTime - lastPrintTime >= 1000) {
    Serial.print(currentStage + 1);
    Serial.print(",");
    Serial.print(elapsedTime);
    Serial.print(",");
    Serial.print(currentTemp);
    Serial.print(",");
    Serial.print(setTemp);
    Serial.print(",");
    Serial.print(error);
    Serial.print(",");
    Serial.println(outputPWM);
    lastPrintTime = currentTime;
  }
}

// Read temperature from thermistor using Steinhart-Hart equation
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