/* Collection Box Firmware
 *  
 
 24 May 2019
 by Arron Li
*/

//#include <Servo.h>
#include <VarSpeedServo.h>
#include <hcsr04.h>
#include <Bounce2.h>

#define TRIG_PIN 2
#define ECHO_PIN 3

#define red_led 10
#define yellow_led 9
#define green_led 8

//states for the box
#define CLOSED 1
#define PREREADY 2
#define READY 3
#define OPEN 4
#define FLAG 5

//Message types
#define COMMAND 1
#define QUERY 2
#define MARK 3

#define glass1Pin 4
#define glass2Pin 5

#define door1Pin 11

//Speed of servo opening and closing
#define OP_SPEED 20
#define CL_SPEED 20

//Number of buttons and boxes: Should be the same number
#define NUM_BUTTONS 4
#define NUM_BOXES 4

//Declare all the button pins here
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {7};

const uint8_t GLASS_PINS[NUM_BOXES] = {4,5};

int ledState = LOW;

Bounce * buttons = new Bounce[NUM_BUTTONS];



    // Checks whether it is time to turn on or off the LED.
    /*
    void check() {
      unsigned long currentTime = millis();
 
      if(currentTime >= nextChangeTime) {
 
        if(ledState) {
          // LED is currently turned On. Turn Off LED.
          ledState = LOW;
          nextChangeTime = currentTime + timeLedOff;
        }
        else{
          // LED is currently turned Off. Turn On LED.
          ledState = HIGH;
          nextChangeTime = currentTime + timeLedOn;
        }
 
        digitalWrite(pinLED, ledState);
      }
      */
    
VarSpeedServo door1;// create Variable speed Servo object to control a servo
//Servo door1;  // create servo object to control a servo
// twelve servo objects can be created on most boards

HCSR04 sensor1(TRIG_PIN, ECHO_PIN, 20, 4000);
int pos = 0;    // variable to store the servo position

int boxState [NUM_BOXES]; 


void setup() 
{
  door1.attach(11);  // attaches the servo on pin 11 to the servo object

    Serial.begin(115200);
  
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(yellow_led, OUTPUT); 

  //pinMode(button1Pin, INPUT);

  pinMode(glass1Pin, OUTPUT);
  //initialise the relay to be OFF (Active LOW)
  digitalWrite(glass1Pin,HIGH);

  pinMode(glass2Pin, OUTPUT);
  digitalWrite(glass2Pin,HIGH);

  for (int i=0; i<NUM_BOXES; i++)
  {
    boxState[i]=CLOSED;
  }

  for (int i = 0; i < NUM_BUTTONS; i++) 
  {
    buttons[i].attach( BUTTON_PINS[i] , INPUT_PULLUP  );       //setup the bounce instance for the current button
    buttons[i].interval(25);              // interval in ms
  }

}

//int buttonState = LOW;

//Contains the {"Message Type","Box Number"}
int message[2]={6,6}  ;

void loop() 
{

  

  if(recieveMessage(message))
  {
    Serial.println("processing message");
    Serial.print("Command:");
    Serial.println(message[0]);
    Serial.print("Box Num:");
    Serial.println(message[1]);
    //Checks what command is given
    switch(message[0])
    {
      case COMMAND:
        //Serial.println("Command");
        //Serial.println(boxState[message[1]]);
        if(boxState[message[1]]==CLOSED){
          Serial.println("Box is closed, switching to preready");
          boxState[message[1]]=PREREADY;
          Serial.print("BoxState is:");
          Serial.println(boxState[message[1]]);
        }
    
        else
          Serial.println("Box not in CLOSED state, cannot switch to PREREADY");
        delay(1000);  
        break;
  
      case QUERY:
        Serial.println("Query");
        for (int i=0; i<NUM_BOXES; i++){
          Serial.print(boxState[i]);
          if(i<=NUM_BOXES-1)
            Serial.print(",");
        }
        Serial.print("\n");
        delay(1000);
        break;
  
      case MARK:
        Serial.println("Mark");
        if(boxState[message[1]]==FLAG)
          boxState[message[1]]=CLOSED;
        else
          boxState[message[1]]=FLAG;
        delay(1000);   
        break;  

      default:
        Serial.println("Default");
        break;
    }
    
  }

//  //Check all the buttons first 
//  for (int i = 0; i < NUM_BUTTONS; i++)  
//  {
//    // Update the Bounce instance :
//    buttons[i].update();
//    // If it fell, activate sequence for that door
//    if ( buttons[i].fell() ) 
//    {
//      //needToToggleLed = true;
//
//      //Check if the door is CLOSED
//      if(boxState[i]==CLOSED)
//    {
//    	//Check if food is placed inside
//
//    	digitalWrite(glass1Pin,LOW);
//    }
//
//    } 
//  }
//    if(buttonState==HIGH)
//  {
//    
//      delay(500);
//      //Serial.println("door open");
//
//      
//      do{buttonState=digitalRead(button1Pin);}
//      while(buttonState==LOW);
//
//      openDoorVar(door1, boxState[1]);
//      
//      while(!sensorClear(sensor1))
//      {
//        //Serial.println("food not retrieved");
//        digitalWrite(green_led, LOW);
//        digitalWrite(yellow_led, HIGH);   // turn the LED on (HIGH is the voltage level)
//        delay(10);
//        //Serial.println("Food Inside:");
//      }
//      
//      
//
//      
//      closeDoorVar(door1,boxState[1]);  
//
//      digitalWrite(glass1Pin,HIGH);
//    }
//    else
//    {      
//      closeDoorVar(door1,boxState[1]);
//    }
//  
//    
    delay(200);
}





void openDoorVar(VarSpeedServo &foo, int &tempboxState)
  {
    if(tempboxState == CLOSED)
    {
     foo.write(90, OP_SPEED);
      tempboxState=OPEN; //sets the door state to OPEN
    }
    
  }



void closeDoorVar(VarSpeedServo &foo, int &tempboxState)
  {
    if(tempboxState == OPEN)
    {
     foo.write(0, CL_SPEED);
      tempboxState=CLOSED; //sets the door state to OPEN
    }
    
  }


bool sensorClear(HCSR04 &tempSensor)
{
  delay(10);
  int tempdist=0;

  for (int j=0;j<8;j++)
  {
    tempdist=tempdist+tempSensor.distanceInMillimeters();
    //Serial.println(tempdist);
    delay(10);
  }
  //Serial.print("Averaged Distance sensed:");
  //Serial.println(tempdist/8);
  
  if (tempdist/8>100)
  {
    return true;
  }
  
  else
  {
    return false;
  }

}

bool recieveMessage(int param[2])
{
  Serial.println("Recieving Message");
	if (Serial.available()) {
    // read the incoming byte:
    Serial.println("Message Available");
    param[0] = Serial.parseInt();
    // do it again:
    param[1] = Serial.parseInt();

    if (Serial.read() == '\n') {         
        // print the three numbers in one string as hexadecimal:
        Serial.print(param[0]);
        Serial.print(",");
        Serial.println(param[1]);

    }
    delay(1000);
    return true;
	}
 return false;
}

void glassOn()

// void glassOff()
 
