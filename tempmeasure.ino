int thermistorPin = A0;        // analog input
const float SERIES_RESISTOR = 10130.0;   // 10kΩ resistor
const float NOMINAL_RESISTANCE = 10000.0; // Thermistor resistance at 25°C
const float NOMINAL_TEMPERATURE = 25.0;  // in Celsius
const float B_COEFFICIENT = 3950.0;       // Beta coefficient of thermistor

void setup() {
  Serial.begin(9600);
}

void loop() {
  int analogValue = analogRead(thermistorPin);
  float voltage = analogValue * 5.0 / 1023.0;

  // Calculate thermistor resistance
  float resistance = (5.0 - voltage) * SERIES_RESISTOR / voltage;

  // Steinhart-Hart Equation
  float steinhart;
  steinhart = resistance / NOMINAL_RESISTANCE; // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= B_COEFFICIENT;                  // 1/B * ln(R/Ro)
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // Convert to Celsius

  Serial.print("Temperature: ");
  Serial.print(steinhart);
  Serial.println(" °C");
  delay(1000);
}