
// Include necessary libraries
#include <SerialCommand.h>
#include <MAX6675.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <ThermocycleStep.h>
#include <EEPROM.h>

#define THERMISTOR_PIN A2           // Pin connected to thermistor
#define REF_RESISTOR 10000          // Resistance of the reference resistor in ohms
#define ROOM_TEMP_RESISTANCE 100000 // Resistance of the thermistor at room temperature in ohms

// Initialize SerialCommand instance
SerialCommand sCmd;

// Define the number of samples to use in the moving average filter
int numSamples = 10;

// Define an array to store the previous temperature samples
double samples[10];

// Define pins for the motor driver
int powerSupplyVoltage = 24;
int motorVoltage = 15;
double powerLimit = (motorVoltage / powerSupplyVoltage) * 255; // Calculate the maximum power limit
int motorPin1 = 6;
int motorPin2 = 5;
int enablePin1 = 3;
int enablePin2 = 4;

// Define pin for the heater
int heaterPin = 10;

// Define power pins
int vcc[] = {2, 15, 12}; // List of VCC pin numbers
int gnd[] = {7, 14};     // List of GND pin numbers

// Define pins for the LCD display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define the PID parameters
double Setpoint, Input, Output;
double Kp = 9.4, Ki = 0.11, Kd = 8.2; // PID constants
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Define enums for the program and thermal states
enum ProgramState
{
  Idle,
  Running,
  Stopped,
  PID_Tune,
  ErrorProgram
};

enum ThermalState
{
  Heating,
  Cooling,
  Holding,
  ErrorThermal
};

bool isDataLogging = false;

// Define program and thermal state variables
float tolerance = 1.5;
bool equilibrating = false;
bool preHeating = false;
float preHeatTemp = 90;
ProgramState programState = Idle;
ThermalState thermalState;

// Define cycle count and number of cycles
int cycleCount = 0;
int numCycles = 2;

// Define the thermocycler program as a sequence of ThermocycleStep objects
const ThermocycleStep program[] = {
    //(in °C, in seconds, ramp rate)
    ThermocycleStep("Denaturation", 95, 30, 0), // Denaturation
    ThermocycleStep("Annealing", 55, 30, 0),    // Annealing
    ThermocycleStep("Extension", 72, 45, 0),    // Extension
    ThermocycleStep("Final", 72, 600, 0),       // Final Extension and Cooling
};

// Define variables for the thermocycler program
unsigned int currentStep = 0;
unsigned long startTime = 0;
unsigned long readTemperatureTimer = 0; // Timer for temperature
unsigned long serialTimer = 0;          // Timer for serial communication
unsigned long displayTimer = 0;         // Timer for LCD display
ThermocycleStep currentThermocycleStep = program[currentStep];

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

void getPID()
{
  // Send current PID values over serial
  Serial.print("Current PID values: Kp = ");
  Serial.print(Kp);
  Serial.print(", Ki = ");
  Serial.print(Ki);
  Serial.print(", Kd = ");
  Serial.println(Kd);
}

// Get the formatted temperature display for the given value
String getTemperatureDisplay(float inputTemp)
{
  return String(inputTemp, 1) + (char)223 + "C";
}

float getTemperature()
{
  int reading = analogRead(THERMISTOR_PIN);                                                                  // Read analog input from thermistor pin
  float resistance = REF_RESISTOR * (1023.0 / reading - 1.0);                                                // Calculate thermistor resistance using voltage divider formula
  float temperature = 1.0 / (1.0 / 298.15 + 1.0 / 3977.0 * log(resistance / ROOM_TEMP_RESISTANCE)) - 273.15; // Calculate temperature using Steinhart-Hart equation
  return temperature;
}

String formatTime(unsigned long millis)
{
  unsigned long seconds = millis / 1000;
  unsigned long minutes = seconds / 60;
  seconds %= 60;
  return String(minutes) + ":" + String(seconds);
}

void setPID()
{
  String param = sCmd.next();
  // extract and set P, I, and D values
  while (param != NULL)
  {
    if (param.startsWith("P="))
    {
      Kp = param.substring(2).toDouble();
      // Set P value in PID controller
    }
    else if (param.startsWith("I="))
    {
      Serial.println(param);
      Ki = param.substring(2).toDouble();
      // Set I value in PID controller
    }
    else if (param.startsWith("D="))
    {
      Kd = param.substring(2).toDouble();
      // Set D value in PID controller
    }
    myPID.SetTunings(Kp, Ki, Kd);
    getPID();
    param = sCmd.next();
  }
}

// Function to load PID values from EEPROM
void loadPIDValues(float &savedKp, float &savedKi, float &savedKd, float &flag)
{
  // Load the PID values from EEPROM if they have been saved before
  int addr = 0;
  EEPROM.get(addr, savedKp);
  addr += sizeof(savedKp);
  EEPROM.get(addr, savedKi);
  addr += sizeof(savedKi);
  EEPROM.get(addr, savedKd);
  addr += sizeof(savedKd);
  EEPROM.get(addr, flag);
}

void initPIDValue()
{

  float savedKp, savedKi, savedKd, flag;
  loadPIDValues(savedKp, savedKi, savedKd, flag);

  if (flag == 1)
  {
    // If the flag is set, load the saved values and update the PID controller
    Kp = savedKp;
    Ki = savedKi;
    Kd = savedKd;

    Serial.println("PID values loaded from EEPROM.");
  }
  else
  {
    // If the flag is not set, use the default values and save them to EEPROM
    EEPROM.put(0, Kp);
    EEPROM.put(sizeof(Kp), Ki);
    EEPROM.put(sizeof(Kp) + sizeof(Ki), Kd);
    EEPROM.put(sizeof(Kp) + sizeof(Ki) + sizeof(Kd), 1);

    Serial.println("Default PID values saved to EEPROM.");
  }
}

// PIDTune function to handle PID tuning.
void PIDTune()
{
  programState = PID_Tune;    // set program state to PID tuning
  String param = sCmd.next(); // get the next parameter from serial monitor

  // loop through all the parameters
  while (param != NULL)
  {
    // set the PID values
    if (param.startsWith("SET_PID"))
    {
      setPID(); // call setPID function
    }
    // exit PID tuning mode
    else if (param.startsWith("DONE"))
    {
      programState = Idle;                // set program state to Idle
      Serial.println("PID tuning done!"); // print message to serial monitor

      // Stop the motor and turn off the motor driver
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      analogWrite(enablePin1, 0);
      analogWrite(enablePin2, 0);

      break; // exit the loop
    }
    // save the PID values to EEPROM
    else if (param.startsWith("SAVE_PID"))
    {
      float savedKp, savedKi, savedKd, flag;
      loadPIDValues(savedKp, savedKi, savedKd, flag); // load PID values from EEPROM

      // check if there are any changes made to the PID values
      if (savedKp == Kp || savedKi == Ki || savedKd == Kd)
      {
        Serial.println("No changes made"); // print message to serial monitor
        break;                             // exit the loop
      }
      // save the PID values to EEPROM
      int addr = 0;
      EEPROM.put(addr, Kp);
      addr += sizeof(Kp);
      EEPROM.put(addr, Ki);
      addr += sizeof(Ki);
      EEPROM.put(addr, Kd);
      addr += sizeof(Kd);
      EEPROM.put(addr, 1); // save a flag to indicate that the values have been saved

      // send a response back to the serial monitor
      Serial.println("PID values updated and saved to EEPROM.");
    }
    // set the target value
    else if (param.startsWith("TARGET="))
    {
      Setpoint = param.substring(7).toDouble();              // get the target value and convert it to double
      Serial.println("Target sets to: " + String(Setpoint)); // print message to serial monitor
    }
    param = sCmd.next(); // get the next parameter from serial monitor
  }
}

void preHeat()
{
  // Check if the program is not already running
  if (programState != Running)
  {
    String param = sCmd.next(); // get the next parameter from serial monitor

    if (!preHeating)
    {
      preHeating = true;
      Setpoint = preHeatTemp;
    }

    if (param != NULL)
    {
      // set the target value
      if (param.startsWith("TARGET="))
      {
        Setpoint = param.substring(7).toDouble();              // get the target value and convert it to double
        Serial.println("Target sets to: " + String(Setpoint)); // print message to serial monitor
      }
    }
  }
  else
  {
    Serial.println("Info: Preheating a running program is not possible.");
  }
}

void cooldown()
{
  // Check if the program is not already running
  if (programState != Running && programState == Idle)
  {
    if (preHeating)
    {
      preHeating = false;
      Setpoint = 0;

      // Stop the motor and turn off the motor driver
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      analogWrite(enablePin1, 0);
      analogWrite(enablePin2, 0);

      Serial.println("Cooling down!"); // print message to serial monitor
    }
  }
  else
  {
    Serial.println("Info: Cooling down a running program is not possible.");
  }
}

void getPrograms()
{
  for (int i = 0; i < 4; i++)
  {

    ThermocycleStep step = program[i];

    Serial.print(step.getName());
    Serial.print("\t");
    Serial.print(getTemperatureDisplay(step.getTemperature()));
    Serial.print("\t");
    Serial.print(formatTime(step.getDuration() * 1000));
    Serial.print("\t");
    Serial.println(step.getRampRate());
  }
}

// Function to start the thermocycling program
void startProgram()
{
  // Check if the program is not already running
  if (programState != Running)
  {
    programState = Running;
    preHeating = false;

    Serial.println("Program running!");

    // Display the name and duration of the current step
    Serial.print("\n\n\nStarting step ");
    Serial.print(currentStep + 1);
    Serial.print(": ");
    Serial.print(currentThermocycleStep.getName());
    Serial.print(" for ");
    Serial.print(currentThermocycleStep.getDuration());
    Serial.println(" seconds.\n\n\n");
  }
  else
  {
    Serial.println("Program already running");
  }
}

// Function to stop the thermocycling program
void stopProgram()
{
  // Check if the program is running
  if (programState == Running)
  {
    // Stop the motor and turn off the motor driver
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin1, 0);
    analogWrite(enablePin2, 0);

    // Display message on the LCD screen
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Cycle stopped!");
    lcd.setCursor(0, 1);
    lcd.print("Remove samples.");

    // Print message to serial monitor
    Serial.println("Program stopped!");

    // Reset the setpoint and program state
    Setpoint = 0;
    programState = Stopped;
  }
  else
  {
    Serial.println("Program not running");
  }
}

// Function to indicate completion of the thermocycling program
void programComplete()
{
  // Check if the program is running
  if (programState == Running)
  {
    // Stop the motor and turn off the motor driver
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin1, 0);
    analogWrite(enablePin2, 0);

    // Display message on the LCD screen
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Cycle complete!");
    lcd.setCursor(0, 1);
    lcd.print("Remove samples.");

    // Print message to serial monitor
    Serial.println("Program complete!");

    // Reset the setpoint, cycle count, current step, and program state
    Setpoint = 0;
    cycleCount = 0;
    currentStep = 0;
    currentThermocycleStep = program[0];
    programState = Stopped;
  }
  else
  {
    Serial.println("Program not running");
  }
}

void readTemperature()
{
  // Read the temperature from the MAX6675 module every 1 second
  if (millis() - readTemperatureTimer >= 250)
  {
    // Add the current temperature to the array of previous samples
    for (int i = 0; i < numSamples - 1; i++)
    {
      samples[i] = samples[i + 1];
    }
    samples[numSamples - 1] = getTemperature();
    ;

    // Compute the moving average temperature
    double sum = 0;
    for (int i = 0; i < numSamples; i++)
    {
      sum += samples[i];
    }

    double movingAverage = sum / numSamples;

    Input = movingAverage;
    readTemperatureTimer = millis();
  }
}

void displayInit()
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

void display(String stepName, int currentCycle, float inputTemp, float setpointTemp, unsigned long duration, unsigned long elapsedTime)
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
    lcd.print(getTemperatureDisplay(Input));

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

void displayIdle(float inputTemp, float setpointTemp)
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

void displayTuning(float inputTemp)
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

void updateTemperatureControl()
{

  // Execute the PID algorithm to control the motor driver PWM signal
  myPID.Compute();

  // Set the motor driver direction and speed based on the PID output
  if (Output > 0)
  {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, Output);
    thermalState = Heating;
  }
  else if (Output < 0)
  {
    analogWrite(motorPin1, Output);
    analogWrite(motorPin2, 0);
    thermalState = Cooling;
  }
  else
  {
    // Stop the motor and set the thermal state to holding
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
    thermalState = Holding;
  }
}

void programRunning()
{

  //// Set the target temperature based on the current step of the thermocycle with ramp rate
  //// Setpoint = currentThermocycleStep.getTemperature() + (currentThermocycleStep.getRampRate() * (millis() - startTime) / 1000);

  // Set the target temperature based on the current step of the thermocycle
  Setpoint = currentThermocycleStep.getTemperature();

  // Run the Peltier element to heat/cool the system
  updateTemperatureControl();

  // Check if the temperature is within the tolerance range
  if (abs(Input - Setpoint) <= tolerance)
  {
    // Get the duration of the current step
    unsigned long duration = currentThermocycleStep.getDuration();

    // Display the current temperature and thermocycle information
    if (millis() - displayTimer >= 1000)
    {
      display(currentThermocycleStep.getName(), cycleCount, Input, Setpoint, duration * 1000, (millis() - startTime));
    }

    // Check if the current thermocycle step is complete
    if (millis() - startTime >= duration * 1000)
    {
      // If the current step is the last step of a cycle, increment the cycle count
      if (currentStep == 2)
      {
        cycleCount++;

        // Check if the desired number of cycles has been completed
        if (cycleCount == numCycles)
        {
          Serial.println("Cycles completed.");
          currentStep++;
        }
        else
        {
          // Display the cycle count and reset the step to zero for the next cycle
          Serial.print("Cycle ");
          Serial.print(cycleCount);
          Serial.println(" completed.");
          currentStep = 0;
        }
      }
      else
      {
        // If the current step is not the last step, increment the step count
        currentStep++;
      }

      // Check if the program is complete
      if (currentStep >= sizeof(program) / sizeof(program[0]))
      {
        programComplete();
      }
      else
      {
        // Set the current thermocycle step and start the timer
        currentThermocycleStep = program[currentStep];
        startTime = millis();

        // Display the name and duration of the current step
        Serial.print("\n\n\nStarting step ");
        Serial.print(currentStep + 1);
        Serial.print(": ");
        Serial.print(currentThermocycleStep.getName());
        Serial.print(" for ");
        Serial.print(currentThermocycleStep.getDuration());
        Serial.println(" seconds.\n\n\n");
      }
    }
  }
  else
  {
    // If the temperature is not within the tolerance range, display the equilibrating temperature
    if (millis() - displayTimer >= 1000)
    {
      displayEquilibrating(Input, Setpoint);
    }
    startTime = millis();
  }
}

void programIdle()
{
  if (preHeating)
  {
    updateTemperatureControl();
  }
  displayIdle(Input, Setpoint);
}

void dataSerialLog()
{
  // Print debug information to the serial monitor every 250ms
  if (millis() - serialTimer >= 250)
  {
    Serial.print(" Set point: ");
    Serial.print(Setpoint);
    Serial.print(" Ouput: ");
    Serial.print(Output);
    Serial.print(" Input: ");
    Serial.print(Input);
    Serial.println("");
    serialTimer = millis();
  }
}

void programPIDTune()
{
  updateTemperatureControl();
  displayTuning(Input);
  dataSerialLog();
}

void setup()
{
  // Initialize the serial communication
  Serial.begin(9600);

  // Initialize the PID saved values
  initPIDValue();

  // Set VCC pins as output and set them HIGH
  for (int i = 0; i < 3; i++)
  {
    pinMode(vcc[i], OUTPUT);
    digitalWrite(vcc[i], HIGH);
  }

  // Set GND pins as output and set them LOW
  for (int i = 0; i < 2; i++)
  {
    pinMode(gnd[i], OUTPUT);
    digitalWrite(gnd[i], LOW);
  }

  // Set the motor driver pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  // Set the heater pin as an output
  pinMode(heaterPin, OUTPUT);

  // Turn on the motor driver by setting enable pins HIGH
  digitalWrite(enablePin1, HIGH);
  digitalWrite(enablePin2, HIGH);

  // Initialize the PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-158, 158);

  // Add serial commands for starting/stopping program, setting/getting PID
  sCmd.addCommand("START", startProgram);
  sCmd.addCommand("STOP", stopProgram);
  sCmd.addCommand("GET_PID", getPID);
  sCmd.addCommand("PID_TUNE", PIDTune);
  sCmd.addCommand("PRE_HEAT", preHeat);
  sCmd.addCommand("COOLDOWN", cooldown);
  sCmd.addCommand("GET_PROGRAMS", getPrograms);

  // Initialize LCD display
  displayInit();
}

void loop()
{
  // Read serial input and update program state
  sCmd.readSerial(); // Read any incoming serial data and execute commands accordingly

  // Read temperature from sensor
  readTemperature(); // Read the temperature from the sensor and update the temperature variables

  // Check if the program is currently running
  if (programState == Running)
  {
    programRunning(); // Execute the code for running the program (heating/cooling control)
  }

  // If the program is not running (Idle state), display the idle screen
  if (programState == Idle)
  {
    programIdle(); // Execute the code for displaying the idle screen
  }

  // If the program is in PID tuning mode, execute the code for tuning the PID values
  if (programState == PID_Tune)
  {
    programPIDTune(); // Execute the code for PID tuning
  }

  // If data logging is enabled, send data to serial port for logging
  if (isDataLogging)
  {
    dataSerialLog(); // Execute the code for logging data to serial port
  }
}

/*

arduino thermocycle serial command handling

to start the program this is the command

START

to stop the program this is the command

STOP

to preheat the heat block this is the command with a default temperature of 90c

PRE_HEAT

to set preheat temperature this is the command

PRE_HEAT TARGET=95

to cooldown the heat block this is the command

COOLDOWN

to get the PID value this is the command

GET_PID

to set the PID tune value this is the command

PID_TUNE SET_PID P=2 I=1 D=1

to set the PID tune temperature this is the command

PID_TUNE TARGET=95

to get the list of program this is the command

GET_PROGRAMS

*/
