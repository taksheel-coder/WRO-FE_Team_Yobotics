#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

#define NUM_MEDIAN_SAMPLES 5

MPU6050 mpu;
Servo myServo;

const int trigFront = 6;
const int echoFront = 7;
const int servoPin = 9;
const int startButtonPin = A0;
const int motorIn1 = 4;
const int motorIn2 = 5;
const int motorEnable = 10;

bool started = false;
bool turning = false;
bool completedLaps = false;

float totalRotation = 0;
float cumulativeRotation = 0;   // 💡 NEW: Total rotation tracker for lap logic
float angleThreshold = 70.0;
float GYRO_DEADBAND = 1.5;

unsigned long lastTime = 0;

long readRawDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long dur = pulseIn(echoPin, HIGH, 30000);
  if (!dur) return -1;
  return dur * 0.034 / 2.0;
}

long readMedianDistance(int trigPin, int echoPin) {
  long samples[NUM_MEDIAN_SAMPLES];
  for (int i = 0; i < NUM_MEDIAN_SAMPLES; i++) {
    samples[i] = readRawDistance(trigPin, echoPin);
    delay(10);
  }
  for (int i = 1; i < NUM_MEDIAN_SAMPLES; i++) {
    long key = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }
  return samples[NUM_MEDIAN_SAMPLES / 2];
}

void setup() {
  Serial.begin(9600);

  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(startButtonPin, INPUT_PULLUP);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorEnable, OUTPUT);

  myServo.attach(servoPin);
  myServo.write(87);  // Center

  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("❌ MPU6050 not connected");
    while (true);
  }

  Serial.println("🔧 System ready, waiting for button...");
  while (digitalRead(startButtonPin) == HIGH) {
    delay(10);
  }

  delay(500);
  Serial.println("🚦 Button pressed! Starting...");
  started = true;

  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  analogWrite(motorEnable, 200);

  lastTime = millis();
}

void loop() {
  if (!started || completedLaps) return;

  float dt = (millis() - lastTime) / 1000.0;
  lastTime = millis();

  // 🌀 Get gyro data
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  float degPerSec = gz / 131.0;

  // Update both turning logic and lap tracking
  if (abs(degPerSec) > GYRO_DEADBAND) {
    float angleThisFrame = abs(degPerSec) * dt;
    if (turning) {
      totalRotation += angleThisFrame;
    }
    cumulativeRotation += angleThisFrame;
  }

  // 🧱 Front detection
  long frontDist = readMedianDistance(trigFront, echoFront);
  Serial.print("Front: ");
  Serial.print(frontDist);
  Serial.print(" cm | Turned: ");
  Serial.print(totalRotation);
  Serial.print("° | TotalRot: ");
  Serial.println(cumulativeRotation);

  // Start turning
  if (!turning && frontDist > 0 && frontDist <= 60) {
    Serial.println("🚧 Wall detected! Turning left...");
    myServo.write(45);
    turning = true;
    totalRotation = 0;
  }

  // Finish turning
  if (turning && totalRotation >= angleThreshold) {
    Serial.println("✅ Turn done. Resetting to center.");
    myServo.write(90);
    turning = false;
  }

  // 🏁 3 Laps = 1080 degrees
  if (cumulativeRotation >= 1180.0) {
    Serial.println("🎉 3 laps completed (1260°)! Stopping bot.");
    delay(4500);

    // STOP EVERYTHING
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW);
    analogWrite(motorEnable, 0);
    myServo.write(90);
    completedLaps = true;
  }

  delay(50);
}

