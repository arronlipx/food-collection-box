/* Collection Box Firmware
 *  
 
 24 May 2019
 by Arron Li
*/

//#include <Servo.h>
#include <VarSpeedServo.h>
#include <hcsr04.h>
#include <Bounce2.h>
#include <SharpIR.h>

#define TRIG_PIN 2
#define ECHO_PIN 3

// #define red_led 10
// #define yellow_led 9
// #define green_led 8

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

//Speed of servo opening and closing
#define OP_SPEED 20
#define CL_SPEED 20

//Number of buttons and boxes: Should be the same number
#define NUM_BUTTONS 2
#define NUM_BOXES 2

//Declare all the button pins here
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {7,6};

//Declare all the smart glass relay pins here
const uint8_t GLASS_PINS[NUM_BOXES] = {4,5};

//Declare all the door servo relay pins here
const uint8_t DOOR_PINS[NUM_BOXES] = {11,12};

//Declare all the IR Sensors here


int ledState = LOW;

//Delay before the door closes after food collected
const unsigned long doorCloseDelay=5000;

// create array of Buttons to trigger OPEN
Bounce * buttons = new Bounce[NUM_BUTTONS];

// create array of Variable speed Servo object to control door servo
VarSpeedServo * door = new VarSpeedServo[NUM_BOXES];

//array of sensors to detect box empty
SharpIR sensor[NUM_BOXES]= {SharpIR( SharpIR::GP2Y0A41SK0F, A0 ), SharpIR( SharpIR::GP2Y0A41SK0F, A1 ) };
//HCSR04  sensor[NUM_BOXES]= {HCSR04(TRIG_PIN, ECHO_PIN, 20, 4000),HCSR04(8, 9, 20, 4000)};


//VarSpeedServo door1;
//Servo door1;  // create servo object to control a servo
// twelve servo objects can be created on most boards

//HCSR04 sensor1(TRIG_PIN, ECHO_PIN, 20, 4000);
int pos = 0; // variable to store the servo position

int boxState [NUM_BOXES];
int doorState [NUM_BOXES];
unsigned long boxTimer[NUM_BOXES];

void setup() 
{
	//remember to attach all the doors
 	//door1.attach(11);  // attaches the servo on pin 11 to the servo object

	Serial.begin(115200);

	// pinMode(red_led, OUTPUT);
	// pinMode(green_led, OUTPUT);
	// pinMode(yellow_led, OUTPUT);

	for (int i = 0; i < NUM_BUTTONS; i++) 
	{
	    pinMode(GLASS_PINS[i], OUTPUT); //setup the bounce instance for the current button
	    digitalWrite(GLASS_PINS[i],HIGH);     // interval in ms
	  }

	  for (int i=0; i<NUM_BOXES; i++)
	  {
	  	boxState[i]=CLOSED;
	  	doorState[i]=CLOSED;

	  	door[i].attach(11+i);
	  }




	  for (int i = 0; i < NUM_BUTTONS; i++)
	  {
      pinMode(BUTTON_PINS[i], INPUT_PULLUP);
      buttons[i].attach(BUTTON_PINS[i]);       //setup the bounce instance for the current button
      buttons[i].interval(25);              // interval in ms
    }

  }

//int buttonState = LOW;

//Contains the {"Message Type","Box Number"}
int message[2]={6,6}  ;
bool msgRcv=false ;

void loop() 
{
  for (int i=0; i<NUM_BUTTONS; i++)
  {
    buttons[i].update();
  }

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
	//runs through all the boxes and performs the actions with each state
	for (int i=0; i<NUM_BOXES; i++)
	{
		switch(boxState[i])
		{
			case CLOSED:
			if (doorState[i]!=CLOSED)
				closeDoorVar(door[i],doorState[i]);
			break;

			case PREREADY:  
			if (sensorClear(sensor[i]))
				Serial.println("Not ready");
			else
			{
				Serial.print("Box ");
				Serial.print(i);
				Serial.println(" is ready");
				boxState[i]=READY;
			}
			break;
      	//Glass will turn clear and wait for button press to transition state
			case READY:
			glassOn(i);
			if(buttons[i].fell())
     {
      Serial.println("Button Press Detected");
				boxState[i]=OPEN;
     }
			break;
      	//
			case OPEN:
			//if door is not open, open the door
			if (doorState[i]!=OPEN)
				openDoorVar(door[i],doorState[i]);
			else
			{
	      //door is open, check sensor to see if food has been collected
				if(sensorClear(sensor[i]))
				{
	        //means timer has not been started
					if(boxTimer[i]==0)
						boxTimer[i] = millis();
					//timer has started, check how much time has elapsed
          else
          {          
          	if(millis()-boxTimer[i]>doorCloseDelay)
          	{	
          		//if sensor is clear after wait, change state to closed, reset timer and off glass
          		if(sensorClear(sensor[i]))
          		{
          			boxState[i]=CLOSED;
          			glassOff(i);
          			boxTimer[i]=0;
          		}
          		//reset timer if sensor is not clear
        			else
        				boxTimer[i]=0;
          		
          	}
          }
        }
      }
          	break;

    	case FLAG:
    	break;

    	default:
    	break;
    }
	          
	          delay(20);
	}
}





void openDoorVar(VarSpeedServo &foo, int &tempboxState)
{
  Serial.println("Opening Door");
	if(tempboxState == CLOSED)
	{ 
    Serial.println("turning door to 90");
		foo.write(90, OP_SPEED);
    tempboxState=OPEN; //sets the door state to OPEN
  }

}



void closeDoorVar(VarSpeedServo &foo, int &tempboxState)
{
  Serial.println("Closing Door");
	if(tempboxState == OPEN)
	{
    Serial.println("turning door to 0");
		foo.write(0, CL_SPEED);
    tempboxState=CLOSED; //sets the door state to OPEN
  }

}


bool sensorClear(SharpIR &tempSensor)
{
	delay(10);
	int tempdist=0;

	for (int j=0;j<8;j++)
	{
		tempdist=tempdist+tempSensor.getDistance();
  //Serial.println(tempdist);
		delay(10);
	}
//Serial.print("Averaged Distance sensed:");
//Serial.println(tempdist/8);

	if (tempdist/8>15)
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

void glassOn(int doorNum){
  Serial.print("glasson doornum:");
  Serial.print(doorNum);
  Serial.print(" Pin:");
  Serial.println(GLASS_PINS[doorNum]);
	digitalWrite(GLASS_PINS[doorNum],LOW);
}

void glassOff(int doorNum){
  Serial.println("glassoff");
	digitalWrite(GLASS_PINS[doorNum],HIGH);
}
