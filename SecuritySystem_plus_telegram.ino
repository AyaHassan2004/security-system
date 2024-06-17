#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Stepper.h>
#ifndef PASSWORD_H
#define PASSWORD_H
#define BUZZER_PIN 11
#define MAX_PASSWORD_LENGTH (20)
#define STRING_TERMINATOR '\0'
#define ledPin1 13
#define ledPin2 12

#define ENABLE 9
#define DIRA 17
#define DIRB 3
#define pirPin 15    
class Password {
public:
  Password(char* pass);
  void set(char* pass);
  bool is(char* pass);
  bool append(char character);
  void reset();
  bool evaluate();
  Password &operator=(char* pass);
  bool operator==(char* pass);
  bool operator!=(char* pass);
  Password &operator<<(char character);
private:
  char* target;
  char guess[MAX_PASSWORD_LENGTH];
  byte currentIndex;
};

#endif

Password::Password(char* pass) {
  set(pass);
  reset();
}

void Password::set(char* pass) { target = pass; }

bool Password::is(char* pass) {
  byte i = 0;
  while (*pass && i < MAX_PASSWORD_LENGTH) {
    guess[i] = pass[i];
    i++;
  }
  return evaluate();
}

bool Password::append(char character) {
  if (currentIndex + 1 == MAX_PASSWORD_LENGTH) {
    return false;
  } else {
    guess[currentIndex++] = character;
    guess[currentIndex] = STRING_TERMINATOR;  
  }
  return true;
}

void Password::reset() {
  currentIndex = 0;
  guess[currentIndex] = STRING_TERMINATOR;
}

bool Password::evaluate() {
  char pass = target[0];
  char guessed = guess[0];
  for (byte i = 1; i < MAX_PASSWORD_LENGTH; ++i) {
    if ((STRING_TERMINATOR == pass) && (STRING_TERMINATOR == guessed)) {
      return true;  
    } else if ((pass != guessed) || (STRING_TERMINATOR == pass) ||
               (STRING_TERMINATOR == guessed)) {
      return false;  
    }
    pass = target[i];
    guessed = guess[i];
  }
  return false;  
}

Password& Password::operator=(char* pass) {
  set(pass);
  return *this;
}

bool Password::operator==(char* pass) { return is(pass); }

bool Password::operator!=(char* pass) { return !is(pass); }

Password& Password::operator<<(char character) {
  append(character);
  return *this;
}

unsigned long startTime; // Variable to store the start time
unsigned long elapsedTime; // Variable to store elapsed time

void setup1() {
  pinMode(11, OUTPUT); // Set pin 11 as output
  pinMode(13, OUTPUT); // Set pin 13 as output
  startTime = millis(); // Record the start time
   pinMode(ledPin1, OUTPUT);
   pinMode(ledPin2, OUTPUT);

     //---set pin direction
  pinMode(ENABLE,OUTPUT);
  pinMode(DIRA,OUTPUT);
  pinMode(DIRB,OUTPUT);
  Serial.begin(9600);
}

void loop1() {
  elapsedTime = millis() - startTime; // Calculate elapsed time
  if (elapsedTime < 500) { // Check if less than 5 seconds have elapsed
    digitalWrite(11, HIGH); // Turn on buzzer
    digitalWrite(13, LOW);  // Ground the other end of the buzzer
    delay(1000);            // Wait for 1 second
    digitalWrite(11, LOW);  // Turn off buzzer
    digitalWrite(13, LOW);  // Ground the other end of the buzzer
    delay(1000);            // Wait for 1 second
  } else { // If 5 seconds have elapsed
    digitalWrite(11, LOW);  // Turn off buzzer
    digitalWrite(13, LOW);  // Ground the other end of the buzzer

    
  }
}

#define buzzer 11
#define buttonPin 12

int buttonState = 0;

Servo servo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

String newPasswordString; 
char newPassword[6]; 
byte a = 5;
bool value = true;
char defaultPassword[]="012";
Password password = Password(defaultPassword);

byte maxPasswordLength = 6;
byte currentPasswordLength = 0;

const byte ROWS = 4; 
const byte COLS = 4; 

char keys[ROWS][COLS] = {
  {'D', 'C', 'B', 'A'},
  {'#', '9', '6', '3'},
  {'0', '8', '5', '2'},
  {'*', '7', '4', '1'},
};

byte rowPins[ROWS] = {2, 16, 4, 5};
byte colPins[COLS] = {6, 7, 8, 13};

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
const int sensorMin = 0;     //  sensor minimum
const int sensorMax = 1024;  // sensor maximum
////////////////////////////////////////
//the time we give the sensor to calibrate (10-60 secs according to the datasheet)
int calibrationTime = 10;        

//the time when the sensor outputs a low impulse
long unsigned int lowIn;         

//the amount of milliseconds the sensor has to be low 
//before we assume all motion has stopped
long unsigned int pause = 5000;  

boolean lockLow = true;
boolean takeLowTime;  
///////////////////////////////////////
void setup() {
  Serial.begin(9600);
  pinMode(pirPin, INPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(pirPin, LOW);

  //give the sensor some time to calibrate
  Serial.print("calibrating sensor ");
    for(int i = 0; i < calibrationTime; i++){
      Serial.print(".");
      delay(1000);
      }
    Serial.println(" done");
    Serial.println("SENSOR ACTIVE");
    delay(50);
    /////////////////////////
  Serial.begin(9600);
  pinMode(buzzer, OUTPUT);
 
  servo.attach(10);
  servo.write(0); // Set initial position to default (90 degrees)
  delay(1000); // Wait for servo to stabilize (adjust delay as needed)
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("WELCOME TO");
  lcd.setCursor(0, 1);
  lcd.print("SECURITY SYSTEM");
  delay(3000);
  lcd.clear();
  pinMode(buzzer, OUTPUT); // Set buzzer pin as output
  pinMode(buttonPin, INPUT);
  int buzz = 11;
int pir = 1;
unsigned long previousMillis = 0;
const long buzzerDuration = 5000; // 5 


  // Sets the buzzer as an OUTPUT & PIR sensor as an INPUT
  pinMode(buzz, OUTPUT);
  pinMode(pir, INPUT);
  
  const int motionSensorPin = 15;

  const int buzzerPin=11;
  // Set motion sensor pin as input
  pinMode(motionSensorPin, INPUT);
  // Set buzzer pin as output
  pinMode(buzzerPin, OUTPUT);
////////////////////////////////////


}


int ledBrightness = 20; // Initialize LED brightness PWM value
void loop()
{
  
analogWrite(ledPin2, ledBrightness);
  if(digitalRead(pirPin) == HIGH){
       digitalWrite(ledPin2, HIGH);   //the led visualizes the sensors output pin state
       tone(BUZZER_PIN,500);
       if(lockLow){  
         //makes sure we wait for a transition to LOW before any further output is made:
         lockLow = false;            
         Serial.println("---");
         Serial.print("Motion Detected at ");
         Serial.print(millis()/1000);
         Serial.println(" sec"); 
         delay(50);
         }         
         takeLowTime = true;
       }

     if(digitalRead(pirPin) == LOW){       
       digitalWrite(ledPin2, LOW);  //the led visualizes the sensors output pin state
       noTone(BUZZER_PIN);
       if(takeLowTime){
        lowIn = millis();          //save the time of the transition from high to LOW
        takeLowTime = false;       //make sure this is only done at the start of a LOW phase
        }
       //if the sensor is low for more than the given pause, 
       //we assume that no more motion is going to happen
       if(!lockLow && millis() - lowIn > pause){  
           //makes sure this block of code is only executed again after 
           //a new motion sequence has been detected
           lockLow = true;                        
           Serial.print("Motion Ended at ");      //output
           Serial.print((millis() - pause)/1000);
           Serial.println(" sec");
           delay(50);
           }
       }
       //////////////////////////////////////////////////////////////////////////////////////////
   buttonState = digitalRead(buttonPin); // Read button state

 
  lcd.setCursor(1, 0);
  lcd.print("ENTER PASSWORD");

  char key = keypad.getKey();

  if (key != NO_KEY) {
    delay(60);
    if (key == 'C')
    {
      resetPassword();
      
    } 
    else if (key == 'D') 
    {
      if (value == true) 
      {
        dooropen();
        value = false;
        
        
      }
      else if (value == false)
      {
        dooropen();
        value = true;
        
      }
    }
    else {
      processNumberKey(key);
      
    }
    const int motionSensorPin = 1;
    int motionState = digitalRead(motionSensorPin);
  
      
  const int buzzerPin=12;

    
    }
    const int sensorMin = 0;     //  sensor minimum
const int sensorMax = 1024;  // sensor maximum
   int sensorReading  = analogRead(A0);
 // map the sensor range (four options):
  // ex: 'long  int map(long int, long int, long int, long int, long int)'
  int range = map(sensorReading,  sensorMin, sensorMax, 0, 3);
    // range value:
 switch (range) {
 case 0:    // A fire closer than 1.5 feet away.
   Serial.println("FIRE DETECTED");
    beepBuzzer(10);
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print("FIRE DETECTED");
   delay(5000);
       
   break;
  case 1:    // A fire between 1-3 feet away.
  // Serial.println("**  Distant Fire **");
   
   break;
 case 2:    // No fire detected.
  //  Serial.println("No  Fire");
    break;
}

 

}

  
void processNumberKey(char key) {
  lcd.setCursor(a, 1);
  lcd.print("*");
  a++;
  if (a == 11) {
    a = 5;
  }
  currentPasswordLength++;
  password.append(key);

  if (currentPasswordLength == maxPasswordLength) 
  {
    
    dooropen();
  }
  
}

void resetPassword(){
  password.reset();
  currentPasswordLength=0;
  lcd.clear();
  a=5;
}

void beepBuzzer(int numBeeps)
{
  pinMode(buzzer, OUTPUT);
  for (int i = 0; i < numBeeps; i++) {
    digitalWrite(buzzer, HIGH);
    delay(120);
    digitalWrite(buzzer, LOW);
    delay(120);
  }
  
}






void doorControl(bool isOpen) {
    int startAngle = isOpen ? 0 : 180; // Starting angle based on whether opening or closing
    int endAngle = isOpen ? 180 : 0; // Ending angle based on whether opening or closing

    int increment = isOpen ? 1 : -1; // Increment or decrement based on whether opening or closing
    int currentAngle = startAngle;

    while (currentAngle != endAngle) {
        servo.write(currentAngle); // Rotate servo to current angle
        delay(15); // Small delay for smooth rotation
        currentAngle += increment;
    }

    delay(1000); // Wait for 1 second after door action

    lcd.setCursor(0, 0);
    lcd.clear();
    if (isOpen) {
    lcd.print("DOOR OPENED");
} else {
    lcd.print("DOOR CLOSED");
}
    delay(2000);
    lcd.clear();
    a = 5;
}





void operateMotor1(bool condition) {
  if (condition) {
    digitalWrite(ENABLE, 20); // Enable the motor
    digitalWrite(DIRA, HIGH);   // One direction (clockwise)
    digitalWrite(DIRB, LOW);
  } else {
    digitalWrite(ENABLE, LOW); // Disable the motor
    digitalWrite(DIRA, LOW);
    digitalWrite(DIRB, LOW);
  }
}

void operateMotor2(bool condition) {
  if (condition) {
    digitalWrite(ENABLE, 20); // Enable the motor
    digitalWrite(DIRA, LOW);   
    digitalWrite(DIRB, HIGH);   // Opposite direction (anti-clockwise)
  } else {
    digitalWrite(ENABLE, LOW); // Disable the motor
    digitalWrite(DIRA, LOW);
    digitalWrite(DIRB, LOW);
  }
}
void stopMotor() {
  digitalWrite(ENABLE, LOW); // Disable the motor
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
}

//int ledBrightness = 20; // Initialize LED brightness PWM value
int wrongAttempts=0;

void dooropen() {
  if (password.evaluate()) 
  {
    if(wrongAttempts == 3)
    {
      operateMotor2(true);
    delay(500);
    stopMotor(); 
    wrongAttempts = 0;
    Serial.println("Free the intruder!");
    }
    else
    {
     doorControl(true); // Open the door
     Serial.println("The Door is Opened, Welcome!");
    delay(2000); // Wait for 2 seconds after opening
    doorControl(false); // Close the door 
    }
    
   
    
  } else {
    lcd.setCursor(0, 0);
    lcd.print("WRONG PASSWORD!");
    lcd.setCursor(0, 1);
    lcd.print("PLEASE TRY AGAIN");
    
    if (wrongAttempts < 2) {
      // First and second wrong attempts behavior
      analogWrite(ledPin1, ledBrightness);
      analogWrite(ledPin2, ledBrightness);
      digitalWrite(ledPin1, HIGH);
      digitalWrite(ledPin2, HIGH);
      delay(80);
      digitalWrite(ledPin1, LOW);
      digitalWrite(ledPin2, LOW);
      delay(80);
      
      tone(buzzer, 1500); // High-pitched alarm sound (1500 Hz)
      delay(1000); // Wait for 1 second for the alarm
      noTone(buzzer); // Stop the alarm sound
      Serial.println("Wrong Password!");
    } else if (wrongAttempts == 2) {
      // Third wrong attempt behavior
      
 ///////////////////////////////////////////////   
operateMotor1(true);
delay(500);
 stopMotor();
 Serial.println("Lock up the intruder!");
  //////////////////////////////////////////////
      analogWrite(ledPin1, ledBrightness);
      analogWrite(ledPin2, ledBrightness);
      for (int i = 0; i < 20; i++) {
        beepBuzzer(1);
        digitalWrite(ledPin1, HIGH);
        digitalWrite(ledPin2, HIGH);
        delay(80);
        digitalWrite(ledPin1, LOW);
        digitalWrite(ledPin2, LOW);
        delay(80);
      
      }
       
     
       
    }
    
    lcd.clear();
    setup1();
    loop1();
    a = 5; // Assuming 'a' is some control variable
    wrongAttempts++;
  } 
  resetPassword();
  
 
}





