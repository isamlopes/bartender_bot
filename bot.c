int numberCollector;  // to build the number from serial
int cancelCollector;
byte serialNumber = 0;  // to store the ASCI number from serial
byte serialCancel = 0;
int opticCount = 3;  // how many optic stations there are - you also need to update line 7
int parameterCount = 4; //number of seeting to be stored in array for each optic
int parameterSize = 3; //the required number of digit for each parameter being sent
int SemBebida = 0;

int drinkMatrix[3][4] = {
  };        // linha e cada garrafa correspondente, coluna e cada parametro (0 e posicao, 1 tempo da dose, 2 quantidade de dose, 3 quantidade de dose disponivel)
 
//The below is for the contact switch.

const int  buttonPin = 13;    // the pin that the pushbutton is attached to
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

//----------------------

//The below is for the stepper motors

#include <AFMotor.h>
AF_Stepper motor1(48, 1);
AF_Stepper motor2(48, 2);

//---------------------

#include <SoftwareSerial.h>
SoftwareSerial HM10(0,1);



// Below is for the RGB LED

int greenPin = 10;
int bluePin = A1;
int redPin = 9;

//---------------------

//The below are for running the machine:

int raftFound = false;
int drinkRequested = false;
int drinkCancel = false;

//---------------------

void setup() {

// for the contact switch 

  // initialize the button pin as a input:
  pinMode(buttonPin, INPUT);
  // initialize the LED as an output:
  pinMode(LED_BUILTIN, OUTPUT);
  // initialize serial communication:
  Serial.begin(9600);
  HM10.begin(9600);
  
//--------------------------------

//  For the stepper motors

  motor1.setSpeed(600);
   motor2.setSpeed(600);

//--------------------------------

// For LED

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

//-------------------------------

}


void loop(){ 
  
//Lets find the location of the float:
if (raftFound == false){
  Serial.println("Raft location not known yet");
  setColor(255, 0, 0);  // red
  delay(1000);
  Serial.print("Looking for the raft...");
  buttonState = digitalRead(buttonPin); // read the pushbutton input pin:
//move the stepper until the contact switch is triggered
  while(buttonState == LOW && raftFound == false){
    motor1.step(10, BACKWARD, SINGLE); 
    buttonState = digitalRead(buttonPin);
  }
 raftFound = true;
 Serial.println("ahh! There it is. :)");
 motor1.release();
 setColor(0, 255, 0);  // blue
 delay(700); // Delay a little bit to calm everything down
} 


//working through the dispensing instructions until drink is complete and paddle is at last optic position:
if (drinkRequested == true){
  Serial.println("alarme");  
  while(drinkCancel == false){
    if (Serial.available()) {
        Serial.println("Cancel");
        serialCancel = Serial.read();
        serialCancel = serialNumber -48;
        cancelCollector = cancelCollector * 10 + serialCancel;
        if (cancelCollector == 1){
        drinkCancel = true;
        cancelCollector = 0;
        Serial.println("Cancel");}
    }
    
    for (int optic = 0; optic < opticCount; optic++) {
      if ((drinkMatrix[optic][3]-drinkMatrix[optic][2])<0){
        SemBebida=1;
      }
    }
    Serial.println("opticCount");
    if (SemBebida==0){
      for (int optic = 0; optic < opticCount; optic++) {
       //Move to pump
        motor1.step((drinkMatrix[optic][0] * 10), FORWARD, SINGLE); //move the paddle according to instruciton, x10 to allow us to compress serial data transfer length
        motor1.release(); // let the motor's magnets relax
    
       //dispense what is required then resume moving to the next position:
        while(drinkMatrix[optic][2] > 0 && raftFound == true){
          delay(500);
          motor2.step(2100, FORWARD, DOUBLE);
          delay((drinkMatrix[optic][1]) * 100);
          motor2.step(2100, BACKWARD, DOUBLE);
          motor2.release();
          drinkMatrix[optic][3] = drinkMatrix[optic][3]-1;
          drinkMatrix[optic][2] = drinkMatrix[optic][2]-1;
          delay(500);
        }
      }

      //Drink complete  
      Serial.println("Drinks ready, enjoy.");
      setColor(0, 0, 255);  // green
      drinkCancel= true;
    }
    else{
      drinkCancel=true;
      SemBebida=0;
    }
  }
  if (drinkCancel == true){
    drinkRequested = false;
    raftFound = false;
    drinkCancel = false;
  }
}

 while(drinkRequested == false){
 delay(200);

   if (HM10.available()) {
    for (int optic = 0; optic < opticCount; optic++){
      for (int parameter = 0; parameter < parameterCount; parameter++){     
        for (int parameterMeasure = 0; parameterMeasure < parameterSize; parameterMeasure++){
          if (HM10.available()) {
            serialNumber = HM10.read(); /* load the number from serial buffer */
            serialNumber = serialNumber - 48; /* convert number to text number */
            numberCollector = numberCollector * 10 + serialNumber; /* store and build the number */
          } else {
            delay(250);
            serialNumber = HM10.read(); /* load the number from serial buffer */
            serialNumber = serialNumber - 48; /* convert number to text number */
            numberCollector = numberCollector * 10 + serialNumber; /* store and build the number */     
          }
        }
       drinkMatrix[optic][parameter] = numberCollector;   /* store the value in the array  */
       numberCollector =  0;   /* Prepare variable for next number  */
       serialNumber = HM10.read(); /* to clear the comma from the buffer */
      }
      }
    CheckArray();
    Serial.println("Done loading");
    drinkRequested = true;
    Serial.println(drinkRequested);
    }  
 }  
}


void CheckArray(){
//print out the array to check it:
for(int i = 0; i < opticCount; i++) {
  for(int j = 0; j < 4; j++) {
    Serial.print(drinkMatrix[i][j]);
    Serial.print(",");
  }
  Serial.println();
}
}

void setColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}
