/*
 * Advanced code for the VapOven Arduino Shield
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
//int LED12v = 2;

int LED1 = 4;
int LED2 = 5;
int intLED = 13;

int blueRGB = 9;
int greenRGB = 10;
int redRGB = 11;

//inputs
int buttonPin = 8;
int potPin = A7;
int SetTemp;
int potRawValue;
float InfraPin;

//relay
int relayPin = 6;
int relaySwitch = 7;

//sensors
int tempPin = A1;
int voltSensorPin = A6;

float voltage; //battery voltage ---------------- i should be able to add code to this to create a spectrum of colour as the battery decreases from 100-0 - search google for how a RGB LED works with arduino for info
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


unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // debounce delay - can adjust this to make button less sensitive

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

bool relayState = false;

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

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

 // Returns the voltage from the voltage sensor and maps it to a percentage
float getVoltage() {
    int data = 0;
    float V = 0.0;
    float R1 = 47000.0;  
    float R2 = 22000.0; 

    data = analogRead(voltSensorPin);
    V = (data * 5.0) / 1024.0; 

    // Calculate the actual battery voltage
    float batteryVoltage = V / (R2 / (R1 + R2));
      Serial.print("Battery Voltage:");
      Serial.println(batteryVoltage);

    // Map the battery voltage to a percentage (assuming a range of 9V to 12.5V)
    float batteryPercentage = mapFloat(batteryVoltage, 9.0, 12.5, 0, 100);


    // Clamp the value between 0 and 100
    batteryPercentage = constrain(batteryPercentage, 0, 100);
    Serial.print("Battery Percentage");
    Serial.println(batteryPercentage);

    return batteryPercentage;
}

  //returns temperature in celcius from TMP36 sensor 
int getTemp(){
    
  int tempReading = analogRead(tempPin); 
 
  // converting to voltage --------00000000000000000000000000000-------------- I've changed the "tempvoltage" from "voltage" because voltage was being used above for battery level (row 132)
   double tempvoltage = tempReading * 5.0;
  tempvoltage /= 1024.0; 

  // print the temperature
  double tempCdoub = (tempvoltage - 0.5) * 100 ;  
 
   int tempCint = tempCdoub;
 
  return tempCint;
}

void setup() {
  
  //set up all the pins  
  pinMode(buttonPin, INPUT);
  pinMode(potPin, INPUT);
  pinMode(InfraPin, INPUT);
  pinMode(triggerPin, OUTPUT);
  //  pinMode(LED12v, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(blueRGB, OUTPUT);
  pinMode(greenRGB, OUTPUT);
  pinMode(redRGB, OUTPUT);
  pinMode(intLED, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(relaySwitch, INPUT_PULLUP);

  digitalWrite(relayPin, LOW);

  
  delay(100);

  // Initialize the OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Clear the display to remove the Adafruit logo
  display.clearDisplay();
  display.display();

  //begin serial output and print the temp of infrared sensor
  Serial.begin(9600);
  while (!Serial);

   Serial.println("Adafruit MLX90614 - infrared sensor test");

  if (!mlx.begin()) {
    Serial.println("IR sensor error.");
    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor(5,40);             // Start at top-left corner
    display.println("IR sensor error");
    display.display();
  }

  else {
    Serial.println("IR sensor okay.");
    display.setCursor(5,40);             // Start at top-left corner
    display.println("IR sensor okay");
    display.display();
    }
  

  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());
  Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
  Serial.println();


  // show battery voltage on the LED
    voltage = getVoltage();
    getVoltage();
    setBatteryIndicator(voltage);
    delay(1000);  // Delay for 1 second

    // Turn off the RGB LED
    digitalWrite(redRGB, LOW);
    digitalWrite(greenRGB, LOW);
    digitalWrite(blueRGB, LOW);
}



void loop() {
  
  loopTime = millis();

  //read sensors
  //check the box temperature
  tempC = getTemp();

  //check infrared temp 
  dynatempC = mlx.readObjectTempC();

  if(relaySwitch == HIGH){ //maybe low instead
    relayState == !relayState;  }

  //check the battery voltage (if the button isn't pressed)
  if(ovenOn == LOW && pulseMode == LOW) voltage = getVoltage();



  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(5,10);             // Start at top-left corner
  display.println("Dyna temp");
  display.setCursor(5,20);             // Start lower down i hope
  display.println(dynatempC);
  display.setCursor(20,20);             // Start to the right i hope
  display.cp437(true);
  display.write(167);
  display.print("C");
  

  //check potentiometer position if button is not pressed
  if(buttonState == LOW) { 
    analogRead(potPin); //might be wrong poteniometer identifier
    potRawValue = analogRead(potPin);          //Read and save analog value from potentiometer
    SetTemp = map(potRawValue, 0, 1023, 60, 120); //Map value 0-1023 to 165-220 temperature range.
    }
  display.setCursor(70,10);             // Start at top-left corner
  display.println("Set temp");
  display.setCursor(70,20);             // Start lower down i hope
  display.println(SetTemp);
  display.setCursor(90,20);             // Start to the right i hope
  display.cp437(true);
  display.write(167);
  display.print("C");

  // Display the battery percentage
  display.setCursor(5,40);             // Start at top-left corner
  display.println("Battery: ");
  display.setCursor(5,50);
  display.print(int(voltage));  // voltage now holds the battery level in percentage
  display.print(" %");


  //temp of controller
  display.setCursor(70,40);             // Start at top-left corner
  display.println("Box temp");
  display.setCursor(70,50);
  display.println(tempC);

  display.display(); // display on OLED



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



  //check button state count and activate oven 

  //not pressed, everything is off
  if (stateCount == 0 && buttonState == LOW){
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
      ovenOn = LOW;
      pulseMode = LOW;
      stateCount = 0;
    }
    
    //released button after first press, turn oven off
    if (stateCount == 2 && buttonState == LOW){
      ovenOn = LOW;
      pulseMode = LOW;
    }
    
    //something is amiss, reset
    if(stateCount == 2 && buttonState == HIGH){
      ovenOn = LOW;
      pulseMode = LOW;
      stateCount = 0;
    }
    
    //second press of double click, enter pulse mode
    if (stateCount == 3 && buttonState == HIGH){
      pulseMode = HIGH;
    }
    
    //something is amiss, reset
    if(stateCount == 3 && buttonState == LOW){
      ovenOn = LOW;
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
    
    //timer to reset stateCount  in case of errors
    if((loopTime - timeOutTimer) > timeOut && buttonState == LOW){
      stateCount = 0;
    }
    
    //if the button is not pressed set oven to off
    if(buttonState == LOW){
      ovenOn = LOW;
      pulseMode = LOW;
    }
    
    
    //if the temperature is above the cutoff, set the oven to off----------------------- OR if the chamber temp is above the set temp -> turn off induction
    if(tempC >= tempCutOff || dynatempC >= SetTemp ){
      ovenOn = LOW;
      pulseMode = LOW;
    }
  
  
  // on off switch for oven
  if(ovenOn == HIGH){
    digitalWrite(intLED, HIGH);
    digitalWrite(LED1, HIGH);
    digitalWrite(triggerPin, HIGH);
    // show battery voltage on the LED

    setBatteryIndicator(voltage);

    if(relayState == true){
      digitalWrite(relayPin, HIGH);
      }
    else(relayPin = LOW);{} // relay click control    
  
  }
  
  if(pulseMode == HIGH){

    digitalWrite(relayPin, HIGH);

    if(loopTime - pulseOvenStart < pulseOvenTime){
      digitalWrite(intLED, HIGH);
      digitalWrite(LED1, HIGH);
      digitalWrite(triggerPin, HIGH);  
    }
    else{
      digitalWrite(intLED, LOW);
      digitalWrite(LED1, LOW);
      digitalWrite(triggerPin, LOW);
    }
      
    if (loopTime - pulseOvenStart > pulseOvenTime*2){
      pulseOvenStart = loopTime;
    }
    
  }
  
  if(ovenOn == LOW && pulseMode == LOW){
  
    digitalWrite(intLED, LOW);
    digitalWrite(LED1, LOW);
    digitalWrite(triggerPin, LOW);
    digitalWrite(redRGB, LOW);
    digitalWrite(greenRGB, LOW);
    digitalWrite(blueRGB, LOW);
    if(relayState == true){
      digitalWrite(relayPin, HIGH);}
      else(relayPin, LOW);{} // relay click control 
  }

  if(Dynatemp >= SetTemp){

    digitalWrite(intLED, LOW);
    digitalWrite(LED1, LOW);
    digitalWrite(triggerPin, LOW);
    digitalWrite(relayPin, LOW);
    }


}








