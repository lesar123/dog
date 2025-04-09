#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>
#include <LiquidCrystal.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
BluetoothSerial SerialBT;

// Servo Channels
#define LEG1_UPPER_SERVO 0
#define LEG1_LOWER_SERVO 1
#define LEG2_UPPER_SERVO 2
#define LEG2_LOWER_SERVO 3
#define LEG3_UPPER_SERVO 4
#define LEG3_LOWER_SERVO 5
#define LEG4_UPPER_SERVO 6
#define LEG4_LOWER_SERVO 7

// Servo Pulse Width Ranges
#define SERVO_MIN 150
#define SERVO_MAX 600

// Movement Parameters
const int BASE_ANGLE = 90;
const int TURN_ANGLE = 30;
const int STEP_DELAY = 15;

int currentAngles[8] = {BASE_ANGLE, BASE_ANGLE, BASE_ANGLE, BASE_ANGLE,
                        BASE_ANGLE, BASE_ANGLE, BASE_ANGLE, BASE_ANGLE};

bool isRunning = false;
bool shouldStop = false;

// OLED setup (change pins from default I2C)
#define OLED_SDA 33
#define OLED_SCL 32
TwoWire OLED_I2C = TwoWire(1);
Adafruit_SSD1306 display(128, 64, &OLED_I2C, -1);

// LCD Pins: RS, E, D4, D5, D6, D7
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);

// Ultrasonic sensors
#define TRIG1 5
#define ECHO1 4
#define TRIG2 13
#define ECHO2 12

// Battery read pin
#define BATTERY_PIN 34

// LEDs
#define OBSTACLE_LED 25
#define WALKING_LED 26
#define STANDING_LED 27

void setup() {
  Serial.begin(115200);
  SerialBT.begin("QuadrupedRobot");
  Serial.println("Bluetooth device is ready to pair");

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(OBSTACLE_LED, OUTPUT);
  pinMode(WALKING_LED, OUTPUT);
  pinMode(STANDING_LED, OUTPUT);

  pwm.begin();
  pwm.setPWMFreq(50);

  for (int i = 0; i < 8; i++) {
    moveServoDirect(i, BASE_ANGLE);
  }

  lcd.begin(16, 2);
  OLED_I2C.begin(OLED_SDA, OLED_SCL);  // Init I2C with custom pins

  // Initialize OLED
  Serial.println("Initializing OLED...");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 failed!"));
    while (1);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Quadruped Ready");
  display.display();

  delay(2000);
  Serial.println("Quadruped Ready - Bluetooth Control");
  SerialBT.println("Ready. Send commands like: G, S, T, or servo,angle (e.g., 3,120)");
}

long readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  long distance = duration * 0.034 / 2;
  return distance;
}

int getBatteryPercentage() {
  int raw = analogRead(BATTERY_PIN);
  float voltage = raw * (3.3 / 4095.0) * 2;  // Assuming 1:1 voltage divider
  return map(voltage * 100, 300, 420, 0, 100);
}

void updateDisplays(const String& state) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("State: " + state);
  display.display();

  lcd.setCursor(0, 0);
  lcd.print("Battery: ");
  lcd.print(getBatteryPercentage());
  lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("State: " + state);
}

void moveServoDirect(uint8_t servoNum, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(servoNum, 0, pulse);
  currentAngles[servoNum] = angle;
}

void moveServoIncremental(uint8_t servoNum, int targetAngle) {
  targetAngle = constrain(targetAngle, 0, 180);

  while (currentAngles[servoNum] != targetAngle && !shouldStop) {
    currentAngles[servoNum] += (currentAngles[servoNum] < targetAngle) ? 1 : -1;
    moveServoDirect(servoNum, currentAngles[servoNum]);
    delay(STEP_DELAY);
    checkBluetoothCommands();
  }
}

void turnLeftIncremental() {
  int leg1Target = BASE_ANGLE + TURN_ANGLE;
  int leg2Target = BASE_ANGLE + TURN_ANGLE;
  int leg3Target = BASE_ANGLE - TURN_ANGLE;
  int leg4Target = BASE_ANGLE - TURN_ANGLE;

  bool movementComplete = false;
  while (!movementComplete && !shouldStop) {
    movementComplete = true;
    checkBluetoothCommands();

    if (currentAngles[LEG1_LOWER_SERVO] != leg1Target) {
      moveServoIncremental(LEG1_LOWER_SERVO, leg1Target);
      movementComplete = false;
    }
    if (currentAngles[LEG2_LOWER_SERVO] != leg2Target) {
      moveServoIncremental(LEG2_LOWER_SERVO, leg2Target);
      movementComplete = false;
    }
    if (currentAngles[LEG3_LOWER_SERVO] != leg3Target) {
      moveServoIncremental(LEG3_LOWER_SERVO, leg3Target);
      movementComplete = false;
    }
    if (currentAngles[LEG4_LOWER_SERVO] != leg4Target) {
      moveServoIncremental(LEG4_LOWER_SERVO, leg4Target);
      movementComplete = false;
    }
    delay(STEP_DELAY);
  }
  if (!shouldStop) delay(300);
}

void returnToNeutral() {
  for (int i = 0; i < 8 && !shouldStop; i++) {
    moveServoIncremental(i, BASE_ANGLE);
  }
}

void checkBluetoothCommands() {
  if (SerialBT.available()) {
    String input = SerialBT.readStringUntil('\n');
    input.trim();

    if (input.length() == 1) {
      char command = input.charAt(0);
      switch (command) {
        case 'S':
          shouldStop = true;
          isRunning = false;
          updateDisplays("Stopped");
          digitalWrite(OBSTACLE_LED, LOW);
          digitalWrite(WALKING_LED, LOW);
          digitalWrite(STANDING_LED, HIGH);
          SerialBT.println("Stopping...");
          returnToNeutral();
          break;
        case 'W':
          shouldStop = false;
          isRunning = true;
          updateDisplays("Walking");
          digitalWrite(OBSTACLE_LED, LOW);
          digitalWrite(WALKING_LED, HIGH);
          digitalWrite(STANDING_LED, LOW);
          SerialBT.println("Starting...");
          break;
        case 'TL':
          if (isRunning) {
            updateDisplays("Turning");
            digitalWrite(OBSTACLE_LED, LOW);
            digitalWrite(WALKING_LED, HIGH);
            digitalWrite(STANDING_LED, LOW);
            SerialBT.println("Turning left...");
            turnLeftIncremental();
            returnToNeutral();
          }
          break;
      }
    } else {
      int commaIndex = input.indexOf(',');
      if (commaIndex != -1) {
        int servoNum = input.substring(0, commaIndex).toInt();
        int angle = input.substring(commaIndex + 1).toInt();

        if (servoNum >= 0 && servoNum <= 7) {
          moveServoDirect(servoNum, angle);
          SerialBT.printf("Moved Servo %d to %d degrees\n", servoNum, angle);
        } else {
          SerialBT.println("Invalid servo number. Use 0-7.");
        }
      } else {
        SerialBT.println("Invalid input. Use format: servo,angle");
      }
    }
  }
}

void loop() {
  checkBluetoothCommands();

  long d1 = readDistanceCM(TRIG1, ECHO1);
  long d2 = readDistanceCM(TRIG2, ECHO2);

  if ((d1 > 0 && d1 < 20) || (d2 > 0 && d2 < 20)) {
    shouldStop = true;
    isRunning = false;
    updateDisplays("Obstacle");
    digitalWrite(OBSTACLE_LED, HIGH);
    digitalWrite(WALKING_LED, LOW);
    digitalWrite(STANDING_LED, LOW);
    returnToNeutral();
  }

  if (isRunning && !shouldStop) {
    updateDisplays("Walking");
    digitalWrite(WALKING_LED, HIGH);
    digitalWrite(OBSTACLE_LED, LOW);
    digitalWrite(STANDING_LED, LOW);

    turnLeftIncremental();
    returnToNeutral();
    if (!shouldStop) delay(500);
  } else if (!isRunning && !shouldStop) {
    updateDisplays("Standing");
    digitalWrite(STANDING_LED, HIGH);
    digitalWrite(WALKING_LED, LOW);
    digitalWrite(OBSTACLE_LED, LOW);
  }
}
