#include <DHT.h>
#include <SoftwareSerial.h>

/*
 * COMBINED ARDUINO UNO SENSOR CODE
 * Includes:
 *   - 3 Ultrasonic sensors
 *   - DHT22 temperature and humidity
 *   - EC sensor
 *
 * Sends packet format:
 *   <wl1,wl2,wl3,temp,hum,0,ec>
 *
 * NOTES:
 * - wl1, wl2, wl3 are computed water levels in cm
 * - TDS is currently set to 0 as placeholder
 * - Replace dummy tank height values below with your real tank heights later
 */

// ================= PIN DEFINITIONS =================
#define DHTPIN      2
#define DHTTYPE     DHT22
#define EcSensorPin A0

// Ultrasonic sensor pins
#define TRIG1 8
#define ECHO1 9

#define TRIG2 10
#define ECHO2 11

#define TRIG3 4
#define ECHO3 5

// SoftwareSerial: RX, TX
// RX is unused, TX goes to ESP32
SoftwareSerial espSerial(7, 6);

DHT dht(DHTPIN, DHTTYPE);

// ================= TANK HEIGHT CALIBRATION (cm) =================
float TANK1_HEIGHT_CM = 38.0;  //main reservior
float TANK2_HEIGHT_CM = 30.0;  //nutrient tank
float TANK3_HEIGHT_CM = 18.0;  //mixing tank

// ================= EC VARIABLES =================
#define VREF   5.0
#define SCOUNT 10

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;

float ecValue_uScm = 0.0;
float ecValue_dSm  = 0.0;

// ================= DHT VARIABLES =================
float currentTemp = 0.0;
float currentHum  = 0.0;

// ================= ULTRASONIC VARIABLES =================
float waterLevel1 = 0.0;
float waterLevel2 = 0.0;
float waterLevel3 = 0.0;

// ================= TIMING =================
unsigned long lastECsampleMs   = 0;
unsigned long lastDHTreadMs    = 0;
unsigned long lastUltraReadMs  = 0;
unsigned long lastSendPacketMs = 0;

const unsigned long EC_SAMPLE_INTERVAL_MS = 200;
const unsigned long DHT_INTERVAL_MS       = 2000;
const unsigned long ULTRA_INTERVAL_MS     = 2000;
const unsigned long SEND_INTERVAL_MS      = 2000;

// ================= ULTRASONIC FUNCTION =================
float readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // timeout 30 ms

  if (duration == 0) {
    return -1.0; // invalid reading
  }

  float distance = duration * 0.0343 / 2.0;
  return distance;
}

// ================= UPDATE ULTRASONIC =================
void updateUltrasonicSensors() {
  float dist1 = readDistanceCM(TRIG1, ECHO1);
  delay(60);

  float dist2 = readDistanceCM(TRIG2, ECHO2);
  delay(60);

  float dist3 = readDistanceCM(TRIG3, ECHO3);
  delay(60);

  // Convert measured distance to water level
  // Water level = tank height - distance from sensor to water
  // If sensor mounting adds offset, adjust these formulas later
  if (dist1 >= 0) {
    waterLevel1 = TANK1_HEIGHT_CM - dist1;
    if (waterLevel1 < 0) waterLevel1 = 0;
  }

  if (dist2 >= 0) {
    waterLevel2 = TANK2_HEIGHT_CM - dist2;
    if (waterLevel2 < 0) waterLevel2 = 0;
  }

  if (dist3 >= 0) {
    waterLevel3 = TANK3_HEIGHT_CM - dist3;
    if (waterLevel3 < 0) waterLevel3 = 0;
  }
}

// ================= DHT READING =================
void updateDHTSensor() {
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();

  if (!isnan(temp) && !isnan(hum)) {
    currentTemp = temp;
    currentHum  = hum;
  }
}

// ================= EC READING =================
void updateECSensor() {
  analogBuffer[analogBufferIndex] = analogRead(EcSensorPin);
  analogBufferIndex++;

  if (analogBufferIndex >= SCOUNT) {
    analogBufferIndex = 0;

    for (int i = 0; i < SCOUNT; i++) {
      analogBufferTemp[i] = analogBuffer[i];
    }

    // Median filter sort
    for (int i = 0; i < SCOUNT - 1; i++) {
      for (int j = i + 1; j < SCOUNT; j++) {
        if (analogBufferTemp[i] > analogBufferTemp[j]) {
          int temp = analogBufferTemp[i];
          analogBufferTemp[i] = analogBufferTemp[j];
          analogBufferTemp[j] = temp;
        } 
      }
    }

    int medianValue = analogBufferTemp[SCOUNT / 2];
    float voltage = medianValue * VREF / 1023.0;

    ecValue_uScm = (133.42 * voltage * voltage * voltage)
                 - (255.86 * voltage * voltage)
                 + (857.39 * voltage);

    if (ecValue_uScm < 0) ecValue_uScm = 0;

    ecValue_dSm = ecValue_uScm / 1000.0;
  }
}

// ================= SEND PACKET =================
void sendSensorPacket() {
  espSerial.print("<");
  espSerial.print(waterLevel1, 2);
  espSerial.print(",");
  espSerial.print(waterLevel2, 2);
  espSerial.print(",");
  espSerial.print(waterLevel3, 2);
  espSerial.print(",");
  espSerial.print(currentTemp, 2);
  espSerial.print(",");
  espSerial.print(currentHum, 2);
  espSerial.print(",");
  espSerial.print(0); // TDS placeholder
  espSerial.print(",");
  espSerial.print(ecValue_dSm, 3);
  espSerial.println(">");
}

// ================= DEBUG PRINT =================
void printDebug() {
  Serial.print("WL1: ");
  Serial.print(waterLevel1, 2);
  Serial.print(" cm | WL2: ");
  Serial.print(waterLevel2, 2);
  Serial.print(" cm | WL3: ");
  Serial.print(waterLevel3, 2);
  Serial.print(" cm | Temp: ");
  Serial.print(currentTemp, 2);
  Serial.print(" C | Hum: ");
  Serial.print(currentHum, 2);
  Serial.print(" % | EC: ");
  Serial.print(ecValue_dSm, 3);
  Serial.println(" dS/m");
}

// ================= SETUP =================
void setup() {
  Serial.begin(9600);      // Serial Monitor
  espSerial.begin(9600);   // ESP32 communication

  dht.begin();
  pinMode(EcSensorPin, INPUT);

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);

  Serial.println("Arduino Uno Combined Ultrasonic + DHT22 + EC Sender Started");
}

// ================= LOOP =================
void loop() {
  unsigned long now = millis();

  if (now - lastECsampleMs >= EC_SAMPLE_INTERVAL_MS) {
    lastECsampleMs = now;
    updateECSensor();
  }

  if (now - lastDHTreadMs >= DHT_INTERVAL_MS) {
    lastDHTreadMs = now;
    updateDHTSensor();
  }

  if (now - lastUltraReadMs >= ULTRA_INTERVAL_MS) {
    lastUltraReadMs = now;
    updateUltrasonicSensors();
  }

  if (now - lastSendPacketMs >= SEND_INTERVAL_MS) {
    lastSendPacketMs = now;
    sendSensorPacket();
    printDebug();
  }
}