
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
const int MAX_STAGES = 10;
float stageTemps[MAX_STAGES];
int stageTimes[MAX_STAGES];
int numStages = 0;
int currentStage = 0;

unsigned long stageStartTime = 0;
unsigned long lastPrintTime = 0;

void parseProfile(String input) {
  numStages = 0;
  int start = input.indexOf(':') + 1;
  input = input.substring(start);
  
  while (input.length() > 0 && numStages < MAX_STAGES) {
    int commaIndex = input.indexOf(',');
    int semicolonIndex = input.indexOf(';');

    if (commaIndex == -1) break;
    float temp = input.substring(0, commaIndex).toFloat();

    int timeEnd = (semicolonIndex != -1) ? semicolonIndex : input.length();
    int time = input.substring(commaIndex + 1, timeEnd).toInt();

    stageTemps[numStages] = temp;
    stageTimes[numStages] = time;
    numStages++;

    if (semicolonIndex == -1) break;
    input = input.substring(semicolonIndex + 1);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(brkPin, OUTPUT);
  digitalWrite(brkPin, LOW);

  myPID.setBangBang(0.5);
  myPID.setTimeStep(500);

  Serial.println("Send PROFILE:<temp,time;...> (e.g., PROFILE:35,60;40,50;45,40;30,60)");
  Serial.println("Stage,Time(s),Current_Temp,Set_Temp,Error,PWM");
}

void loop() {
  static String input = "";
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      input.trim();
      if (input.startsWith("PROFILE:")) {
        parseProfile(input);
        currentStage = 0;
        setTemp = stageTemps[currentStage];
        stageStartTime = millis();
        Serial.println("Profile loaded. Starting...");
      }
      input = "";
    } else {
      input += ch;
    }
  }

  if (numStages == 0) return;

  currentTemp = readTemperature();
  double error = setTemp - currentTemp;

  myPID.run();

  if (error > 0.5) digitalWrite(dirPin, HIGH);
  else if (error < -0.5) digitalWrite(dirPin, LOW);

  analogWrite(pwmPin, (int)outputPWM);

  unsigned long currentTime = millis();
  unsigned long elapsedTime = (currentTime - stageStartTime) / 1000;

  if (elapsedTime >= stageTimes[currentStage]) {
    currentStage++;
    if (currentStage < numStages) {
      setTemp = stageTemps[currentStage];
      stageStartTime = millis();
    } else {
      analogWrite(pwmPin, 0);
      Serial.println("Reflow complete.");
      numStages = 0;
      return;
    }
  }

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
