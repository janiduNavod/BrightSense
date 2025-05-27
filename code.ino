#include <Wire.h>
#include <APDS9930.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <IRremote.h>
#define DHTTYPE DHT11
#include <Servo.h>

//PId controling values
#define kp 3.4
#define ki 0.000009
#define kd 0.000015

//sevo varibles
Servo myservo;
int val = 0;  

// Global Variables
APDS9930 apds = APDS9930();
float ambient_light = 0; // can also be an unsigned long
float error, previous_error = 0,I=0,u=0,pwm;


// Set the LCD address to 0x27 for a 16x2 display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define the pin connected to the IR receiver output
const int IR_PIN = 8;  // Example pin number, adjust as per your setup
IRrecv irrecv(IR_PIN);
decode_results results;

// Define the pin connected to the relay control
const int RELAY_PIN = 3;  // Example pin number, adjust as per your setup

// Define the PWM pin connected to the MOSFET
const int pwmPin = 6;
const int potPin = A0; // Potentiometer connected to A0
const int motionSensorPin = 2;  // Pin connected to motion sensor output
const int Pin = 12;
int motionDetected = 1;

//temperature value 
float temperature_value;
float humidity_value;
uint8_t DHTPin = 4;
DHT dht(DHTPin, DHTTYPE);

byte sun[8] = {
  0b00100,
  0b10101,
  0b01110,
  0b11111,
  0b01110,
  0b10101,
  0b00100,
  0b00000
};

byte bulb[8] = {
  0b00100, //   *
  0b01110, //  ***
  0b01110, //  ***
  0b01110, //  ***
  0b00100, //   *
  0b01110, //  ***
  0b01110, //  ***
  0b00000  //      
};
byte powerOn[8] = {
  0b00100, //   *
  0b01010, //  * *
  0b01010, //  * *
  0b01010, //  * *
  0b00100, //   *
  0b00000, //     
  0b00100, //   *
  0b00000  //     
};
byte powerOff[8] = {
  0b00100, //   *
  0b01110, //  ***
  0b01110, //  ***
  0b01110, //  ***
  0b01110, //  ***
  0b01110, //  ***
  0b01110, //  ***
  0b00000  //     
};
byte thermometer[8] = {
  0b00100,
  0b01010,
  0b01010,
  0b01110,
  0b01110,
  0b11111,
  0b11111,
  0b01110
};
byte droplet[8] = {
  0b00100,
  0b00100,
  0b01110,
  0b01110,
  0b11111,
  0b11111,
  0b11111,
  0b01110
};
byte ambientSymbol[8] = {
  0b00100,
  0b01010,
  0b10001,
  0b10001,
  0b11111,
  0b01110,
  0b01010,
  0b00000
};
byte ledOn[8] = {
  0b00100,
  0b01010,
  0b01010,
  0b01010,
  0b01010,
  0b01110,
  0b01110,
  0b01110
};

byte ledOff[8] = {
  0b00100,
  0b01010,
  0b01010,
  0b01010,
  0b01010,
  0b01110,
  0b10001,
  0b01110
};
void setup() {
  // Initialize Serial port
  Serial.begin(9600);
  Serial.println();
  
  dht.begin();

  pinMode(motionSensorPin, INPUT);

  myservo.attach(9);

  // Initialize LCD
  lcd.init();
  lcd.backlight();

 
  // Initialize APDS-9930 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9930 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9930 init!"));
    lcd.setCursor(0, 1);
    lcd.print("APDS-9930 init fail");
  }
  
  // Start running the APDS-9930 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {
    Serial.println(F("Light sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during light sensor init!"));
    lcd.setCursor(0, 1);
    lcd.print("Light sensor fail");
  }

  // Wait for initialization and calibration to finish
  delay(50);
  
  // Set the PWM pin as output
  pinMode(pwmPin, OUTPUT);
  pinMode(12,INPUT);
  int switchState = 0;
}

void loop() {

  int motionDetected = digitalRead(motionSensorPin);

  
  //read temp and huminity
  temperature_value = dht.readTemperature();
  humidity_value = dht.readHumidity();
  Serial.println(temperature_value);
  Serial.println(humidity_value);

  // Read the potentiometer value
  int potValue = analogRead(potPin);

  // Map the potentiometer value to an ambient light threshold
  int lightThreshold = map(potValue, 0, 1023, 0, 40); // Adjust mapping as needed
  //int lightThresholdmap = map(lightThreshold, 0, 60, 0, 100);

  
  // Read the light levels (ambient)
  if (!apds.readAmbientLightLux(ambient_light)) {
    Serial.println(F("Error reading light values"));
    lcd.setCursor(0, 1);
    lcd.print("Error reading light");
  } else {
    Serial.print(F("Ambient: "));
    Serial.println(ambient_light);
  }

  // Wait 1 second before next reading
  delay(10);

    int switchState = digitalRead(Pin);  


  if(switchState == HIGH){
    //pwm cotroller for blub brightness adgesment 
    float error = lightThreshold - ambient_light;
    I = I + error;
    previous_error = error;
    u = kp * error + ki * I + kd * (error - previous_error);
  
    pwm = pwm + u;
    if (pwm > 255)
    {
      pwm = 255;
    }
    if (pwm < 0)
    {
      pwm = 0;
    }

    // mortionsensor 
    if (motionDetected == HIGH) {
    // Adjust brightness based on ambient light and threshold
    analogWrite(pwmPin, pwm);
    delay(100);
    } 
    else {
      analogWrite(pwmPin, 0); // Turn off the light if no motion detected
    }
    
  }
  else{
    analogWrite(pwmPin, 0);
  }


Serial.println(switchState);
  

// Display the ambient light value and threshold on the LCD

    lcd.createChar(0, bulb);
    lcd.createChar(1, sun);
    lcd.createChar(2, powerOn);
    lcd.createChar(3, powerOff);
    lcd.createChar(4, thermometer);
    lcd.createChar(5, droplet);
    lcd.createChar(6, ambientSymbol);
    lcd.createChar(7, ledOn);
    lcd.createChar(8, ledOff);

    lcd.setCursor(0, 0);

    if(pwm >= 0 || results.value == 0xFFA25D){
      lcd.write(byte(7));
      delay(100);
      }
    else{
      lcd.write(byte(8));
      }

    lcd.setCursor(3, 0);

    lcd.write(byte(6));
    lcd.print(" ");
    lcd.print(ambient_light);
    lcd.print(" ");

    lcd.write(byte(1));//sun emoji
    lcd.print(" ");
    lcd.print(lightThreshold);
    lcd.print("%");
    
    lcd.setCursor(0, 1);
    lcd.write(byte(4));
    lcd.print(" ");
    lcd.print(temperature_value);
    lcd.print("C ");
    lcd.write(byte(5));
    lcd.print(" ");
    lcd.print(humidity_value);
    lcd.print("% "); // clear any leftover characters

    
    if(temperature_value >= 30){
      val = val+2;
      }
    else {
       val = val-2;
      }
     if(val>47) {
       val=47;
     }else if (val<0){
       val=1;
      }
    myservo.write(val);
  Serial.println(val);

}
