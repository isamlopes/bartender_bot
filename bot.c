int raftFound = false;
int numberCollector;    // coleta numero do serial
byte serialNumber = 0;  // guarda o numero em ASCII do serial
int opticCount = 3;     // quantidade de garrafas
int parameterCount = 4; // contagem de parametros. nesse caso: posicao, tempo, numero de doses e quantas doses tem cada garrafa
int parameterSize = 3;  // numero de digitos de cada parametro
int finish = false;

int drinkMatrix[3][4] = {
  };

//The below is for the contact switch.

const int  buttonPin = 13;   // the pin that the pushbutton is attached to
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

//----------------------

//The below is for the stepper motors

#include <AFMotor.h>
AF_Stepper motor1(48, 1);
AF_Stepper motor2(48, 2);

//---------------------

#include <SoftwareSerial.h>
SoftwareSerial mySerial(0,1);
// Below is for the RGB LED

int greenPin = 10;
int bluePin = A1;
int redPin = 9;

//---------------------

//The below are for running the machine:

int drinkRequested = false;

//---------------------

void setup() {

// for the contact switch 
  mySerial.begin(9600);
  // initialize the button pin as a input:
  pinMode(buttonPin, INPUT);
  // initialize the LED as an output:
  pinMode(LED_BUILTIN, OUTPUT);
  // initialize serial communication:
  Serial.begin(9600);
  
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

  void CheckArray(){
//print out the array to check it:
for(int i = 0; i < opticCount; i++) {
  for(int j = 0; j < 3; j++) {
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

#define NUM_ESTADOS 2

#define NUM_EVENTOS 5





//ESTADOS

enum ESTADO{

    IDLE,

    OCUPADO

};



// EVENTOS

enum EVENTOS{

    RECEITA,

    SBRECEITA,

    CANCELA,

    TERMINOU,

    NENHUM_EVENTO

};



// ACOES

enum ACOES{

    PREPARA,

    RESETA,
    
    VOLTA,

    NENHUMA_ACAO

};



int codigoEvento;

int codigoAcao;

int estado;

int acao_matrizTransicaoEstados[NUM_ESTADOS][NUM_EVENTOS];

int proximo_estado_matrizTransicaoEstados[NUM_ESTADOS][NUM_EVENTOS];



void executarAcao(int codigoAcao) {

    if (codigoAcao == NENHUMA_ACAO)

        return;



    switch(codigoAcao)

    {

    case RESETA:

        //Lets find the location of the float:
if (raftFound == false){
  Serial.println("Raft location not known yet");
  setColor(255, 0, 0);  // red
  delay(1000);
  Serial.print("Looking for the raft...");
  buttonState = digitalRead(buttonPin); // read the pushbutton input pin:
//move the stepper until the contact switch is triggered
  while(buttonState == LOW && raftFound == false){
    motor1.step(10, FORWARD, SINGLE); 
    buttonState = digitalRead(buttonPin);
  }
 raftFound = true;
 Serial.println("ahh! There it is. :)");
 motor1.release();
 setColor(0, 255, 0);  // blue
 delay(700); // Delay a little bit to calm everything down
} 

        break;

    case PREPARA:
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
      finish=true;
  

        break;

    case  VOLTA:
    drinkRequested = false;
    raftFound = false;
    if (raftFound == false){
  Serial.println("Raft location not known yet");
  setColor(255, 0, 0);  // red
  delay(1000);
  Serial.print("Looking for the raft...");
  buttonState = digitalRead(buttonPin); // read the pushbutton input pin:
//move the stepper until the contact switch is triggered
  while(buttonState == LOW && raftFound == false){
    motor1.step(10, FORWARD, SINGLE); 
    buttonState = digitalRead(buttonPin);
  }
 raftFound = true;
 Serial.println("ahh! There it is. :)");
 motor1.release();
 setColor(0, 255, 0);  // blue
 delay(700); // Delay a little bit to calm everything down
} 


    }
}




void iniciaMaquinaEstados()

{

  int i;

  int j;



  for (i=0; i < NUM_ESTADOS; i++) {

    for (j=0; j < NUM_EVENTOS; j++) {

       acao_matrizTransicaoEstados[i][j] = NENHUMA_ACAO;

       proximo_estado_matrizTransicaoEstados[i][j] = i;

    }

  }

  proximo_estado_matrizTransicaoEstados[IDLE][RECEITA] = OCUPADO;

  acao_matrizTransicaoEstados[IDLE][RECEITA] =PREPARA;



  proximo_estado_matrizTransicaoEstados[OCUPADO][TERMINOU] = IDLE;

  acao_matrizTransicaoEstados[OCUPADO][TERMINOU] = RESETA;



  proximo_estado_matrizTransicaoEstados[OCUPADO][CANCELA] = IDLE;

  acao_matrizTransicaoEstados[OCUPADO][CANCELA] = CANCELA;


} // initStateMachine



void iniciaSistema()

{

   iniciaMaquinaEstados();

   if (raftFound == false){
  Serial.println("Raft location not known yet");
  setColor(255, 0, 0);  // red
  delay(1000);
  Serial.print("Looking for the raft...");
  buttonState = digitalRead(buttonPin); // read the pushbutton input pin:
//move the stepper until the contact switch is triggered
  while(buttonState == LOW && raftFound == false){
    motor1.step(10, FORWARD, SINGLE); 
    buttonState = digitalRead(buttonPin);
  }
 raftFound = true;
 Serial.println("ahh! There it is. :)");
 motor1.release();
 setColor(0, 255, 0);  // blue
 delay(700); // Delay a little bit to calm everything down
} 

} // initSystem



int obterEvento() {

while(drinkRequested == false){
 delay(200);

   if (Serial.available()) {
    for (int optic = 0; optic < opticCount; optic++){
      for (int parameter = 0; parameter < parameterCount; parameter++){     
        for (int parameterMeasure = 0; parameterMeasure < parameterSize; parameterMeasure++){
          if (Serial.available()) {
            serialNumber = Serial.read(); /* load the number from serial buffer */
            serialNumber = serialNumber - 48; /* convert number to text number */
            numberCollector = numberCollector * 10 + serialNumber; /* store and build the number */
          } else {
            delay(250);
            serialNumber = Serial.read(); /* load the number from serial buffer */
            serialNumber = serialNumber - 48; /* convert number to text number */
            numberCollector = numberCollector * 10 + serialNumber; /* store and build the number */     
          }
        }
       drinkMatrix[optic][parameter] = numberCollector;   /* store the value in the array  */
       numberCollector =  0;   /* Prepare variable for next number  */
       serialNumber = Serial.read(); /* to clear the comma from the buffer */
      }
      }
    CheckArray();
    Serial.println("Done loading");
    drinkRequested = true;
    Serial.println(drinkRequested);
    }
      for (int optic = 0; optic < opticCount; optic++) {
        if ((drinkMatrix[optic][3]-drinkMatrix[optic][2])<0){
          return SBRECEITA;
          break;
        }
        else {
          return RECEITA;

        }
      } 
 }

if(finish==true){
  finish=false;
  return TERMINOU;

}   

while(drinkRequested == true){
  delay(200);

   if (Serial.available()) {
          if (Serial.available()) {
            serialNumber = Serial.read(); /* load the number from serial buffer */
            if(serialNumber==1){
              serialNumber=0;
              return CANCELA;
            }
          } else {
            delay(250);
            serialNumber = Serial.read(); /* load the number from serial buffer */
            if(serialNumber==1){
              serialNumber=0;
              return CANCELA;
            }    
          }
        }
}
}





int obterAcao(int estado, int codigoEvento) {

  return acao_matrizTransicaoEstados[estado][codigoEvento];

} // obterAcao



int obterProximoEstado(int estado, int codigoEvento) {

  return proximo_estado_matrizTransicaoEstados[estado][codigoEvento];

} // obterEstado



int main() {



  int codigoEvento;

  int codigoAcao;

  int estado;



  estado = IDLE;



  iniciaSistema();

  printf ("Maquina iniciada\n");

  for( ; ; ) {

    codigoEvento = obterEvento();



    if (codigoEvento != NENHUM_EVENTO)

    {

       codigoAcao = obterAcao(estado, codigoEvento);

       estado = obterProximoEstado(estado, codigoEvento);

       printf("Estado: %d Evento: %d Acao:%d\n", estado, codigoEvento, codigoAcao);

    }

  } // while true

} // main
