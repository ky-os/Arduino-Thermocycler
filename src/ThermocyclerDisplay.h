#ifndef THERMOCYCLEDISPLAY_H
#define THERMOCYCLEDISPLAY_H

#include <LiquidCrystal_I2C.h>
#include <Arduino.h>

// Define cycle character for LCD display
byte cycleChar[] = {
    B00000,
    B11100,
    B10010,
    B10111,
    B10010,
    B10010,
    B11110,
    B00000};

// Get the formatted temperature display for the given value
String getTemperatureDisplay(float inputTemp)
{
  return String(inputTemp, 1) + (char)223 + "C";
}

String formatTime(unsigned long millis)
{
  unsigned long seconds = millis / 1000;
  unsigned long minutes = seconds / 60;
  seconds %= 60;
  return String(minutes) + ":" + String(seconds);
}

class ThermocyclerDisplay
{
private:
  LiquidCrystal_I2C lcd;
  unsigned long displayTimer;

public:
  ThermocyclerDisplay() : lcd(0x27, 16, 2)
  {
  }

  void init()
  {
    // Initialize the LCD display
    lcd.init();
    lcd.clear();
    lcd.backlight();
    lcd.createChar(0, cycleChar);

    // Display initialization message for 3 seconds
    lcd.setCursor(0, 0);
    lcd.print("Thermocycler");
    lcd.setCursor(0, 1);
    lcd.print("Initializing...");
    delay(3000);
    lcd.clear();
  }

  void display(String stepName, int currentCycle, float inputTemp, unsigned long duration, unsigned long elapsedTime)
  {
    if (millis() - displayTimer >= 1000)
    {
      lcd.clear();

      // Display the current step name and cycle number
      lcd.setCursor(0, 0);
      lcd.print(stepName);
      lcd.setCursor(14, 0);
      lcd.print(String(currentCycle));
      lcd.write(0);

      // Display the current temperature
      lcd.setCursor(0, 1);
      lcd.print(getTemperatureDisplay(inputTemp));

      // Display the time elapsed and remaining duration
      lcd.setCursor(6, 1);
      lcd.print(formatTime(elapsedTime));
      lcd.print("/");
      lcd.print(formatTime(duration));
      displayTimer = millis();
    }
  }

  void displayEquilibrating(float inputTemp, float setpointTemp)
  {
    if (millis() - displayTimer >= 1000)
    {
      lcd.clear();

      // Display "Equilibrating" message
      lcd.setCursor(0, 0);
      lcd.print("Equilibrating");

      // Display the current temperature and setpoint temperature
      lcd.setCursor(0, 1);
      lcd.print(getTemperatureDisplay(inputTemp));
      lcd.print(" -> ");
      lcd.print(getTemperatureDisplay(setpointTemp));
      displayTimer = millis();
    }
  }

  void displayIdle(float inputTemp, float setpointTemp, bool preHeating)
  {
    if (millis() - displayTimer >= 1000)
    {
      lcd.clear();

      // Display "Idle" message
      lcd.setCursor(0, 0);
      lcd.print("Idle");

      if (preHeating)
      {
        lcd.setCursor(6, 0);
        lcd.print("- Pre-heat");
        // Display the current temperature and setpoint temperature
        lcd.setCursor(0, 1);
        lcd.print(getTemperatureDisplay(inputTemp));
        lcd.print(" -> ");
        lcd.print(getTemperatureDisplay(setpointTemp));
      }
      else
      {
        // Display the current temperature
        lcd.setCursor(0, 1);
        lcd.print(getTemperatureDisplay(inputTemp));
      }

      displayTimer = millis();
    }
  }

  void displayTuning(float inputTemp, double Kp, double Ki, double Kd)
  {
    if (millis() - displayTimer >= 1000)
    {
      lcd.clear();

      // Display "Tuning" message
      lcd.setCursor(0, 0);
      lcd.print("PID Tuning");

      lcd.setCursor(12, 0);
      lcd.print(getTemperatureDisplay(inputTemp));

      // Display the current PID
      lcd.setCursor(0, 1);
      lcd.print("P");
      lcd.print(Kp, 1);
      lcd.print(" I");
      lcd.print(Ki, 2);
      lcd.print(" D");
      lcd.print(Kd, 1);
      displayTimer = millis();
    }
  }

  void programStopped()
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Cycle stopped!");
    lcd.setCursor(0, 1);
    lcd.print("Remove samples.");
  }

  void programComplete()
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Cycle complete!");
    lcd.setCursor(0, 1);
    lcd.print("Remove samples.");
  }
};

#endif