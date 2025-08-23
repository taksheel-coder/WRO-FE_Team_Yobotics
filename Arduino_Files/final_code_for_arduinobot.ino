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

#define S0 10
#define S1 11
#define S2 12
#define S3 13
#define sensorOut 8

struct RGB {
  int r;
  int g;
  int b;
};

RGB orangeRef = {61, 196, 152};
RGB blueRef  = {170, 134, 72};
RGB whiteRef = {28, 30, 24};

RGB currentColor;
String turnDirection = "Left";  // Default turn direction
bool colorDetected = false;

bool started = false;
bool turning = false;
bool completedLaps = false;

float totalRotation = 0;
float cumulativeRotation = 0;
float angleThresholdLeft = 70.0;
float angleThresholdRight = 50.0;
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

RGB readColor() {
  RGB color;
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  delay(50);
  color.r = pulseIn(sensorOut, LOW);

  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  delay(50);
  color.g = pulseIn(sensorOut, LOW);

  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  delay(50);
  color.b = pulseIn(sensorOut, LOW);

  return color;
}

long colorDistance(RGB c1, RGB c2) {
  return sqrt(
    pow((long)c1.r - c2.r, 2) +
    pow((long)c1.g - c2.g, 2) +
    pow((long)c1.b - c2.b, 2)
  );
}

String detectColor(RGB input) {
  long dOrange = colorDistance(input, orangeRef);
  long dBlue   = colorDistance(input, blueRef);
  long dWhite  = colorDistance(input, whiteRef);

  Serial.print("🔍 HSV Debug — R: ");
  Serial.print(input.r);
  Serial.print(", G: ");
  Serial.print(input.g);
  Serial.print(", B: ");
  Serial.println(input.b);

  if (dOrange < dBlue && dOrange < dWhite) return "Orange";
  if (dBlue < dOrange && dBlue < dWhite) return "Blue";
  return "Unknown";
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
  myServo.write(87);

  Wire.begin();
  mpu.initialize();

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

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

  while (!colorDetected) {
    currentColor = readColor();
    String detected = detectColor(currentColor);
    Serial.print("🎨 Detected Color: ");
    Serial.println(detected);

    if (detected == "Orange") {
      turnDirection = "Right";
      colorDetected = true;
    } else if (detected == "Blue") {
      turnDirection = "Left";
      colorDetected = true;
    } else {
      Serial.println("❓ Ignoring unrecognized color. Retrying...");
    }
    delay(200);
  }

  Serial.print("🔁 Turn direction set to: ");
  Serial.println(turnDirection);

  lastTime = millis();
}

void loop() {
  if (!started || completedLaps) return;

  float dt = (millis() - lastTime) / 1000.0;
  lastTime = millis();

  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  float degPerSec = gz / 131.0;

  if (abs(degPerSec) > GYRO_DEADBAND) {
    float angleThisFrame = abs(degPerSec) * dt;
    if (turning) {
      totalRotation += angleThisFrame;
    }
    cumulativeRotation += angleThisFrame;
  }

  long frontDist = readMedianDistance(trigFront, echoFront);
  Serial.print("Front: ");
  Serial.print(frontDist);
  Serial.print(" cm | Turned: ");
  Serial.print(totalRotation);
  Serial.print("° | TotalRot: ");
  Serial.println(cumulativeRotation);

  if (!turning && frontDist > 0 && frontDist <= 60) {
    Serial.println("🚧 Wall detected!");
    if (turnDirection == "Right") {
      Serial.println("🔄 Turning RIGHT...");
      myServo.write(135);
    } else {
      Serial.println("🔃 Turning LEFT...");
      myServo.write(45);
    }
    turning = true;
    totalRotation = 0;
  }

  if (turning) {
    if (turnDirection == "Right" && totalRotation >= angleThresholdRight) {
      Serial.println("✅ Right turn done. Resetting to center.");
      myServo.write(90);
      turning = false;
    } else if (turnDirection == "Left" && totalRotation >= angleThresholdLeft) {
      Serial.println("✅ Left turn done. Resetting to center.");
      myServo.write(90);
      turning = false;
    }
  }

  if (cumulativeRotation >= 1180.0) {
    Serial.println("🎉 3 laps completed (1260°)! Stopping bot.");
    delay(4500);
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW);
    analogWrite(motorEnable, 0);
    myServo.write(90);
    completedLaps = true;
  }

  delay(50);
}



