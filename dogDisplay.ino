#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

unsigned long previousMillis = 0;
int seconds = 0;
int minutes = 0;
int hours = 0;
int day = 1;
int month = 1;
int year = 2025;

void setup() {
  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Loading...");
  display.display();
  delay(1000);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    seconds++;

    if (seconds >= 60) {
      seconds = 0;
      minutes++;
    }
    if (minutes >= 60) {
      minutes = 0;
      hours++;
    }
    if (hours >= 24) {
      hours = 0;
      day++;
    }

    // Very basic date increment (doesn't account for months with 30 days, leap years, etc.)
    if (day > 30) {
      day = 1;
      month++;
    }
    if (month > 12) {
      month = 1;
      year++;
    }

    // Display Time
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("QUadrupedRobot time:");
    display.setTextSize(1);
    display.setCursor(0, 30);
    if (hours < 10) display.print('0');
    display.print(hours);
    display.print(':');
    if (minutes < 10) display.print('0');
    display.print(minutes);
    display.setTextSize(1);
    display.setCursor(0, 40);
    display.print(day);
    display.print('/');
    display.print(month);
    display.print('/');
    display.print(year);
    display.display();
  }
}
