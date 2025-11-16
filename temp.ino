#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Pins
#define MQ7_PIN   A0
#define MQ135_PIN A1
#define PM25_PIN  A2
#define PM_LED_PIN 9  // GP2Y1010AU0F IR LED pin (usually active-LOW to turn ON)

Adafruit_BME280 bme; // I2C BME280 (default addr 0x76 used below)

// --- PM2.5 pulsed-LED timing (microseconds) ---
const unsigned int samplingTime = 280;   // microseconds
const unsigned int deltaTime    = 40;    // microseconds
const unsigned int sleepTime    = 9680;  // microseconds

// Averaging for PM reading
const int PM_AVG_SAMPLES = 5;

// ---------- Helper: AQI Category ----------
String getAQICategory(int aqi) {
  if (aqi <= 50) return "GOOD";
  else if (aqi <= 100) return "SATISFACTORY";
  else if (aqi <= 200) return "MODERATE";
  else if (aqi <= 300) return "POOR";
  else if (aqi <= 400) return "VERY POOR";
  else return "SEVERE";
}

// ---------------- PM2.5 Reading (GP2Y1010AU0F style) ----------------
float readPM25() {
  // Many GP2Y modules are active-LOW for LED (LOW => LED ON). If your module uses HIGH to turn ON, invert the writes.
  long sumCentiMicro = 0; // we'll store centi-ug (ug * 100) to reduce float accumulation issues

  for (int s = 0; s < PM_AVG_SAMPLES; ++s) {
    digitalWrite(PM_LED_PIN, LOW);          // LED ON (active-LOW) - pulse
    delayMicroseconds(samplingTime);        // wait for sensor to respond

    int voMeasured = analogRead(PM25_PIN);  // ADC reading (0-1023)

    delayMicroseconds(deltaTime);
    digitalWrite(PM_LED_PIN, HIGH);         // LED OFF
    delayMicroseconds(sleepTime);           // pause between pulses

    // Convert ADC reading to voltage (assumes 5.0V Vref and 10-bit ADC)
    float voltage = voMeasured * (5.0 / 1024.0);

    // Calibration from image: dustDensity (ug/m3) = 170 * Vout - 0.1
    float dustDensity = (170.0 * voltage) - 0.1;
    if (dustDensity < 0.0) dustDensity = 0.0;

    sumCentiMicro += (long)(dustDensity * 100.0); // keep centi-ug
  }

  float avgCenti = (float)sumCentiMicro / PM_AVG_SAMPLES;
  float avgDust = avgCenti / 100.0; // back to ug/m3
  return avgDust;
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 2000); // small wait for Serial on some boards

  pinMode(PM_LED_PIN, OUTPUT);
  digitalWrite(PM_LED_PIN, HIGH); // LED OFF initial (assuming active-LOW)

  // initialize BME280
  if (!bme.begin(0x76)) {
    Serial.println("BME280 ERROR - Not detected at 0x76!");
    // If BME not present, we continue but will skip readings (handled in loop)
  } else {
    Serial.println("BME280 OK");
  }

  Serial.println("Air Quality System Started...");
}

void loop() {
  // -------- Sensor raw reads ----------
  int mq7_raw = analogRead(MQ7_PIN);     // CO sensor raw
  int mq135_raw = analogRead(MQ135_PIN); // General gas sensor raw
  float pm25 = readPM25();               // ug/m3

  // BME readings (if available)
  float temp = NAN, hum = NAN, pressure = NAN;
  if ((uint32_t)bme.readTemperature() == (uint32_t)0 && (uint32_t)bme.readHumidity() == (uint32_t)0 && (uint32_t)bme.readPressure() == (uint32_t)0) {
    // this check is weak; if bme.begin() failed earlier you already printed an error.
  } 
  // Safer: use the object anyway; if not present values may be zero or nonsense
  temp = bme.readTemperature();                    // Celsius
  hum  = bme.readHumidity();                       // %
  pressure = bme.readPressure() / 100.0;           // hPa

  // ---------- Convert raw values to simple AQI-like scales ----------
  // These are simple linear maps for quick display; replace with proper calibrations if available.
  int COppm = map(mq7_raw, 0, 1023, 0, 1000);       // crude mapping to 0-1000 ppm
  int gasAQI = map(mq135_raw, 0, 1023, 0, 500);     // crude gas index 0-500
  int pmAQI = constrain((int)round(pm25), 0, 500);  // use PM2.5 ug/m3 ~ index (simple)

  // Combined AQI (weighted) - weights can be tuned
  int AQI = (int)round((0.4 * pmAQI) + (0.3 * gasAQI) + (0.3 * COppm));
  AQI = constrain(AQI, 0, 9999); // limit just in case

  String category = getAQICategory(AQI);

  // ---------- Serial Output ----------
  Serial.println(F("================================="));
  Serial.print(F("PM2.5 (ug/m3): ")); Serial.println(pm25);
  Serial.print(F("CO (MQ7 raw -> ppm): ")); Serial.print(mq7_raw); Serial.print(F(" -> ")); Serial.println(COppm);
  Serial.print(F("MQ135 raw -> gas AQI: ")); Serial.print(mq135_raw); Serial.print(F(" -> ")); Serial.println(gasAQI);
  Serial.print(F("Temperature: ")); Serial.print(temp); Serial.println(F(" C"));
  Serial.print(F("Humidity: ")); Serial.print(hum); Serial.println(F(" %"));
  Serial.print(F("Pressure: ")); Serial.print(pressure); Serial.println(F(" hPa"));
  Serial.print(F("Final AQI: ")); Serial.println(AQI);
  Serial.print(F("Category: ")); Serial.println(category);
  Serial.println(F("=================================\n"));

  delay(1200); // ~1.2s between prints
}