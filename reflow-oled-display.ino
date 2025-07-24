#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AutoPID.h>

// OLED config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
float stageTemps[NUM_STAGES] = {45.0, 55.0, 65.0, 35.0}; // Default
int stageTimes[NUM_STAGES] = {100, 120, 150, 150};       // Default
int currentStage = 0;

unsigned long stageStartTime = 0;
unsigned long lastPrintTime = 0;
bool profileReceived = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(brkPin, OUTPUT);
  digitalWrite(brkPin, LOW);  // Disable braking

  myPID.setBangBang(0.5);
  myPID.setTimeStep(500);

  // Prompt for Serial Input
  displayMessage("Enter profile:");
  Serial.println("Enter reflow profile: format -> 45,90;50,50;55,50;40,100");

  unsigned long waitStart = millis();
  while (!profileReceived && millis() - waitStart < 20000) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      profileReceived = parseProfile(input);
      if (!profileReceived) Serial.println("Invalid format. Try again.");
    }
  }

  setTemp = stageTemps[currentStage];
  stageStartTime = millis();

  Serial.println("Stage,Time(s),Current_Temp,Set_Temp,Error,PWM");
}

void loop() {
  currentTemp = readTemperature();
  double error = setTemp - currentTemp;

  myPID.run();

  // Direction control
  if (error > 0.5) digitalWrite(dirPin, HIGH);  // Heat
  else if (error < -0.5) digitalWrite(dirPin, LOW);  // Cool

  analogWrite(pwmPin, (int)outputPWM);

  unsigned long currentTime = millis();
  unsigned long elapsedTime = (currentTime - stageStartTime) / 1000;

  if (elapsedTime >= stageTimes[currentStage]) {
    currentStage++;
    if (currentStage < NUM_STAGES) {
      setTemp = stageTemps[currentStage];
      stageStartTime = millis();
    } else {
      analogWrite(pwmPin, 0);
      displayMessage("Reflow Complete");
      Serial.println("Reflow complete.");
      while (1);
    }
  }

  // OLED & Serial Print every 1 sec
  if (currentTime - lastPrintTime >= 1000) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Stage: "); display.println(currentStage + 1);
    display.print("Temp: "); display.print(currentTemp, 1); display.println(" C");
    display.print("Set: "); display.print(setTemp, 1); display.println(" C");
    display.print("PWM: "); display.println((int)outputPWM);
    display.display();

    Serial.print(currentStage + 1); Serial.print(",");
    Serial.print(elapsedTime); Serial.print(",");
    Serial.print(currentTemp); Serial.print(",");
    Serial.print(setTemp); Serial.print(",");
    Serial.print(error); Serial.print(",");
    Serial.println(outputPWM);

    lastPrintTime = currentTime;
  }
}

// Parse Serial Profile Input
bool parseProfile(String input) {
  input.trim();
  input.replace(" ", "");

  int stage = 0;
  while (input.length() > 0 && stage < NUM_STAGES) {
    int semiIndex = input.indexOf(';');
    String segment = (semiIndex == -1) ? input : input.substring(0, semiIndex);

    int commaIndex = segment.indexOf(',');
    if (commaIndex == -1) return false;

    float temp = segment.substring(0, commaIndex).toFloat();
    int time = segment.substring(commaIndex + 1).toInt();
    if (temp < 0 || time <= 0) return false;

    stageTemps[stage] = temp;
    stageTimes[stage] = time;
    stage++;

    if (semiIndex == -1) break;
    input = input.substring(semiIndex + 1);
  }

  return (stage == NUM_STAGES);
}

// Display message on OLED
void displayMessage(String msg) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(msg);
  display.display();
}

// Read temp using Steinhart-Hart
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