/*
 * 
 * 
 *   ***DOUBLE CLICK TO PULSE HEAT***
 * 
 * turns on the IH module and light up LED1 when the trigger button is pressed
 * turns on pulse heating with double click of the button
 * uses the RGB LED as a battery indicator – green when battery is good, red when battery is low.
 * temperature cutoff monitors the temperature and will not activate the IH if over 60 deg Centigrade
 * 
 * This code is free to use and modify for personal use. It is released on the "You owe me a bud" licence - if we ever meet, you promise to provide me with a bud.
 
 Change Tools->Processor->Atmega 328P to Atmega 328P (Old Bootloader). Remind that the Processor option is only available when you select some specific Boards at Tools>Board.

This worked for my Arduino Nano
 
 
 */

  const char* version = "3.5"; // Updated version number
 
#include <Adafruit_MLX90614.h> //include library for infrared temp sensor
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

//include libraries for OLED and define it
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//pins
int triggerPin = 3;

//LEDs
int LED12v = 2;
int LED1 = 5; //I've changed this as my PCB was damaged
int LED2 = 4;
int intLED = 13;

int blueRGB = 9;
int greenRGB = 10;
int redRGB = 11;

int relaySwitch = A2;  // Mute switch
int relayPin = 12;

int handlessSwitch = 6;

//inputs
int buttonPin = 8;
int potPin = A7;
int SetTemp;
int potRawValue;
float InfraPin;

//sensors
int tempPin = A1;
int voltSensorPin = A6;

float voltage; //battery voltage
float storedVoltage; // Stored battery voltage when oven is not on
double lowBatteryThreshold = 10;
double twentybattery = 20;
double thirtybattery = 30;
double fortybattery = 40;
double fiftybattery = 50;
double sixtybattery = 60;
double seventybattery = 70;
double eightybattery = 80;
double ninetybattery = 90;
double hundybattery = 100;


//vaiables for the button press and timeout function
bool buttonState;
bool lastButtonState = LOW;
long buttonPressStart;

//hands-free
bool handsFreeMode = false;
unsigned long handsFreeStartTime = 0;
const unsigned long handsFreeTimeout = 30000; // 30 seconds safety timer

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 25;    // Reduced debounce delay to decrease lag

bool ovenOn = LOW;

int tempC;
int tempCutOff = 60;  //temperature in Celcius to stop IH activating

int dynatempC;
int Dynatemp;

//led duty cycle
int dutycycle = 90;

//pulse mode double click bits
int stateCount = 0;

long firstPressTime;

long buttonTimer = 600;

long timeOut = 800;
long timeOutTimer = 0;

bool pulseMode = false;
long pulseModeStartTime = 0;
int pulseModeDelay = 5000;

//for the pulse heat mode
long pulseOvenTime = 500; //adjust this value to change speed of pulse
long pulseOvenStart = 0;  

float loopTime;

bool previousOvenOn = LOW;
bool muteOn = false;
bool handsFreeActive = false; // Track if hands-free mode is actually activated (button pressed)

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int getTemp(){                 //returns temperature in celcius from TMP36 sensor 
    
  int tempReading = analogRead(tempPin); 
 
  // converting to voltage
   double tempvoltage = tempReading * 5.0;
  tempvoltage /= 1024.0; 

  // print the temperature
  double tempCdoub = (tempvoltage - 0.5) * 100 ;  
 
   int tempCint = tempCdoub;
 
  return tempCint;
}
 
int getVoltage() {          // Returns the voltage from the voltage sensor and maps it to a percentage
    int data = 0;
    float V = 0.0;
    const float R1 = 47000.0;  
    const float R2 = 22000.0; 

    data = analogRead(voltSensorPin); // Read raw analog data
    V = (data * 5.0) / 1024.0;        // Convert to voltage

    // Calculate the actual battery voltage
    float batteryVoltage = V / (R2 / (R1 + R2));
    Serial.print("Battery Voltage: ");
    Serial.println(batteryVoltage, 2); // Print with 2 decimal places for clarity

    // Map the battery voltage to a percentage (assuming a range of 9V to 12.5V)
    int batteryPercentage = mapFloat(batteryVoltage, 9.0, 12.5, 0, 100);

    // Clamp the value between 0 and 100
    batteryPercentage = constrain(batteryPercentage, 0, 100);
    Serial.print("Battery Percentage: ");
    Serial.println(batteryPercentage);

    return batteryPercentage; // Return as an integer
}

// Function to adjust LED color based on battery level
void setBatteryIndicator(float batteryPercentage) {
    // Define RGB colors 

    //9-100% - GREEN
    int red90TO100 = 0;
    int green90TO100 = 128;
    int blue90TO100 = 0;
    //80-90%  -  TURQUOISE
    int red80TO90 = 0;
    int green80TO90 = 128;
    int blue80TO90 = 3;
    //70-80%  -  CYAN
    int red70TO80 = 0;
    int green70TO80 = 110;
    int blue70TO80 = 10;
    //60-70%  -  SKY BLUE
    int red60TO70 = 0;
    int green60TO70 = 100;
    int blue60TO70 = 20;
    //50-60%  -  BLUE
    int red50TO60 = 20;
    int green50TO60 = 40;
    int blue50TO60 = 40;
    //40-50%  -  VIOLET
    int red40TO50 = 128;
    int green40TO50 = 0;
    int blue40TO50 = 40;
    //30-40%  -  MAGENTA
    int red30TO40 = 255;
    int green30TO40 = 0;
    int blue30TO40 = 20;
    //20-30%  -  ROSE
    int red20TO30 = 255;
    int green20TO30 = 10;
    int blue20TO30 = 10;
    //10-20%  -  ORANGE
    int red10TO20 = 255;
    int green10TO20 = 32;
    int blue10TO20 = 0;
    //0-10%  -  RED
    int red0TO10 = 255;
    int green0TO10 = 0;
    int blue0TO10 = 0;

    int red, green, blue;

  // Determine the color based on the battery percentage
  if (batteryPercentage > 90) {
      red = red90TO100;
      green = green90TO100;
      blue = blue90TO100;
  } else if (batteryPercentage > 80) {
      red = red80TO90;
      green = green80TO90;
      blue = blue80TO90;
  } else if (batteryPercentage > 70) {
      red = red70TO80;
      green = green70TO80;
      blue = blue70TO80;
  } else if (batteryPercentage > 60) {
      red = red60TO70;
      green = green60TO70;
      blue = blue60TO70;
  } else if (batteryPercentage > 50) {
      red = red50TO60;
      green = green50TO60;
      blue = blue50TO60;
  } else if (batteryPercentage > 40) {
      red = red40TO50;
      green = green40TO50;
      blue = blue40TO50;
  } else if (batteryPercentage > 30) {
      red = red30TO40;
      green = green30TO40;
      blue = blue30TO40;
  } else if (batteryPercentage > 20) {
      red = red20TO30;
      green = green20TO30;
      blue = blue20TO30;
  } else if (batteryPercentage > 10) {
      red = red10TO20;
      green = green10TO20;
      blue = blue10TO20;
  } else { // batteryPercentage <= 10
      red = red0TO10;
      green = green0TO10;
      blue = blue0TO10;
  }
    // Set the RGB LED to the determined color
    analogWrite(redRGB, red);
    analogWrite(greenRGB, green);
    analogWrite(blueRGB, blue);
}

void setup() {
  unsigned long startTime = millis();  // Record start time

  // Pin setup
  pinMode(buttonPin, INPUT);
  pinMode(potPin, INPUT);
  pinMode(InfraPin, INPUT);
  pinMode(triggerPin, OUTPUT);
  pinMode(LED12v, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(blueRGB, OUTPUT);
  pinMode(greenRGB, OUTPUT);
  pinMode(redRGB, OUTPUT);
  pinMode(intLED, OUTPUT);
  pinMode(relaySwitch, INPUT_PULLUP);
  pinMode(relayPin, OUTPUT);
  pinMode(handlessSwitch, INPUT_PULLUP);

  // OLED Initialization
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  // MLX90614 Infrared Sensor Initialization
  mlx.begin();  

  // Battery voltage check
  storedVoltage = getVoltage();
  setBatteryIndicator(storedVoltage);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.print(version);
    
  //      COLUMN 1
  display.setCursor(0, 0);
  display.println("Mute:");

  delay(1000);  // Delay for 1 second

  // Turn off the RGB LED
  digitalWrite(redRGB, LOW);
  digitalWrite(greenRGB, LOW);
  digitalWrite(blueRGB, LOW);

  // NOW start serial
  Serial.begin(9600);
  Serial.println("Setup complete!");
  Serial.print("Total setup time: ");
  Serial.println(millis() - startTime);
}

void loop() {
  loopTime = millis();

  //read sensors
  //check the box temperature
  tempC = getTemp();

  //check infrared temp 
  dynatempC = mlx.readObjectTempC();

  // Read mute switch state - LOW when switch is ON (mute is inactive)
  muteOn = digitalRead(relaySwitch) == LOW;
  
  // Read hands-free switch state - LOW when switch is ON
  bool handsFreeSwitchState = digitalRead(handlessSwitch) == LOW; // Hands-free switch is ON when LOW
  
  //check the battery voltage (if the oven isn't on)
  if (ovenOn == LOW && pulseMode == LOW) {
    storedVoltage = getVoltage(); // Only update stored voltage when oven is off
  }

  // Debug output
  Serial.print("buttonState:");
  Serial.println(buttonState);
  Serial.print("handsFreeMode:");
  Serial.println(handsFreeMode);  
  Serial.print("muteOn:");
  Serial.println(muteOn);
  Serial.print("dynatempC:");
  Serial.println(dynatempC);
  Serial.print("SetTemp:");
  Serial.println(SetTemp);

  //check button - do some stuff
  //debounce
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = loopTime;
  }

  //button has changed
  if ((loopTime - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      //record the click  
      stateCount++;
      timeOutTimer = loopTime;
      buttonState = reading;
    
      //reset the button time out
      buttonPressStart = loopTime;
    
      //reset first press timer
      if(stateCount == 1){
        firstPressTime = loopTime;
      }
    }
  }

  lastButtonState = reading;

  // Handle hands-free mode activation properly
  // Hands-free is only enabled when both the switch is ON and button is pressed
  if (handsFreeSwitchState) {
    // Hands-free switch is ON
    if (buttonState == HIGH && !handsFreeMode) {
      // Button is pressed while switch is ON and hands-free not yet active
      handsFreeMode = true;
      handsFreeStartTime = loopTime;
      Serial.println("Hands-free mode activated");
    }
  } else {
    // Hands-free switch is OFF, disable hands-free mode
    handsFreeMode = false;
  }

  //check button state count and activate oven 
  //not pressed, everything is off
  if (stateCount == 0 && buttonState == LOW && !handsFreeMode){
    ovenOn = LOW;
    pulseMode = LOW;
  }

  //first press, turn oven on normally
  if(stateCount == 1 && buttonState == HIGH){
    ovenOn = HIGH;
    pulseMode = LOW;
  }
  
  //something is amiss, reset
  if(stateCount == 1 && buttonState == LOW){
    if (!handsFreeMode) { // Only turn off if not in hands-free mode
      ovenOn = LOW;
    }
    pulseMode = LOW;
    stateCount = 0;
  }
  
  //released button after first press, turn oven off
  if (stateCount == 2 && buttonState == LOW){
    if (!handsFreeMode) { // Only turn off if not in hands-free mode
      ovenOn = LOW;
    }
    pulseMode = LOW;
  }
  
  //something is amiss, reset
  if(stateCount == 2 && buttonState == HIGH){
    if (!handsFreeMode) { // Only turn off if not in hands-free mode
      ovenOn = LOW;
    }
    pulseMode = LOW;
    stateCount = 0;
  }
  
  //second press of double click, enter pulse mode
  if (stateCount == 3 && buttonState == HIGH){
    pulseMode = HIGH;
    if (handsFreeMode) {
      handsFreeMode = false; // Disable hands-free when entering pulse mode
    }
  }
  
  //something is amiss, reset
  if(stateCount == 3 && buttonState == LOW){
    if (!handsFreeMode) { // Only turn off if not in hands-free mode
      ovenOn = LOW;
    }
    pulseMode = LOW;
    stateCount = 0;
  }
  
  //more than the double click, reset the counter
  if(stateCount >= 4){
    stateCount = 0;
  }
  
  //timer for double click
  if(loopTime - firstPressTime > buttonTimer){
    stateCount = 0;
  }
  
  //timer to reset stateCount in case of errors
  if((loopTime - timeOutTimer) > timeOut && buttonState == LOW){
    stateCount = 0;
  }  
  
  //if the temperature is above the cutoff, set the oven to off
  if(tempC >= tempCutOff){
    ovenOn = LOW;
    pulseMode = LOW;
    handsFreeMode = false; // Also disable hands-free mode for safety
  }

  // Hands-free mode logic with timeout and temperature control
  if (handsFreeMode) {
    if (loopTime - handsFreeStartTime >= handsFreeTimeout) {
      handsFreeMode = false;  // Timeout reached, disable hands-free
      ovenOn = LOW;
      Serial.println("Hands-free mode timed out.");
    }
    else {
      ovenOn = HIGH; // Keep oven on while in hands-free mode
    }
  }
  
  // Check if pulse mode should stop due to temperature
  if (pulseMode == HIGH && dynatempC >= SetTemp) {
    pulseMode = LOW;  // Turn off pulse mode when temperature is reached
    ovenOn = LOW;
    Serial.println("Pulse mode stopped at target temperature");
  }
  
  // Check if normal heating mode should stop due to temperature
  if (ovenOn == HIGH && !pulseMode && dynatempC >= SetTemp) {
    if (!handsFreeMode) { // Only turn off if not in hands-free mode
      ovenOn = LOW;
      Serial.println("Oven turned off at target temperature");
    }
  }

  // on off switch for oven
  if (ovenOn == HIGH) {
    digitalWrite(intLED, HIGH);
    digitalWrite(LED1, HIGH);
    digitalWrite(triggerPin, HIGH);
    // show battery voltage on the LED
    setBatteryIndicator(storedVoltage); // Use stored battery voltage   
  }

  // Relay control logic - only activate the relay when:
  // 1. Mute switch is ON (muteOn is true)
  // 2. AND one of the heating modes is active (ovenOn or pulseMode)
  if ((ovenOn == HIGH || pulseMode == HIGH) && muteOn) {
    digitalWrite(relayPin, HIGH);
  } else {
    digitalWrite(relayPin, LOW);
  }
  
  // Pulse mode logic
  if (pulseMode == HIGH) {
    if (loopTime - pulseOvenStart < pulseOvenTime) {
      digitalWrite(intLED, HIGH);
      digitalWrite(LED1, HIGH);
      digitalWrite(triggerPin, HIGH);  
    }
    else {
      digitalWrite(intLED, LOW);
      digitalWrite(LED1, LOW);
      digitalWrite(triggerPin, LOW);
    }
      
    if (loopTime - pulseOvenStart > pulseOvenTime*2){
      pulseOvenStart = loopTime;
    }
  }
  
  // Turn everything off if neither mode is active
  if (ovenOn == LOW && pulseMode == LOW) {
    digitalWrite(intLED, LOW);
    digitalWrite(LED1, LOW);
    digitalWrite(triggerPin, LOW);
    digitalWrite(redRGB, LOW);
    digitalWrite(greenRGB, LOW);
    digitalWrite(blueRGB, LOW);
  }

  // Display information on the OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  //      COLUMN 1
  display.setCursor(0, 0);
  display.println("Mute:");

  display.setCursor(70, 0);
  display.println("H-F:");

  display.setCursor(0, 13);
  display.println("Dyna temp:");

  display.setCursor(0, 26);
  display.println("Set temp:");

  display.setCursor(0, 39);
  display.println("Battery: ");

  display.setCursor(0, 52);
  display.println("Box temp");   

  /*  Column 2   */
  display.setCursor(40, 0);
  if (muteOn) {
    display.println("Off");
  } else {
    display.println("On");
  }

  display.setCursor(100, 0);
  if (handsFreeMode) {
    display.println("On");
  } else {
    display.println("Off");
  }

  display.setCursor(70, 13);
  display.println(dynatempC);

  display.setCursor(70, 26);
  display.println(SetTemp);

  display.setCursor(70, 39);
  display.println(int(storedVoltage)); // Use stored voltage value that doesn't fluctuate

  display.setCursor(70, 52);
  display.println(tempC);

  // THIRD COLUMN CAN BE UNITS
  display.setCursor(100, 13);
  display.cp437(true);
  display.write(167);
  display.print("C");

  potRawValue = analogRead(potPin);
  SetTemp = map(potRawValue, 0, 1023, 80, 160); // ADJUST THESE VALUES IN 3 AND 4TH POSITION TO CHANGE RANGE OF TEMPERATURES ON POTENTIOMETER
  
  display.setCursor(100, 26);
  display.cp437(true);
  display.write(167);
  display.print("C");

  display.setCursor(100, 39);
  display.print(" %");

  display.setCursor(100, 52);
  display.cp437(true);
  display.write(167);
  display.print("C");

  display.display();
}