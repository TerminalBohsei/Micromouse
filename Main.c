#include 

/*Dava if you're reading this and theres like a better way, any suggestions / tips to make my code better, 
or just the whole code is terrible lemme know - Kohsei*/

// I'm just  tryna set everything up but ngl I'm a bit confused on how we should do the left-side wall algorithm. - Kohsei


//Pin number hasn't been added because idk what pins align with which ones
//                        MODULE PINS
//MOTOR SPEED CONTROL
#define MOTOR1PIN1 11
#define MOTOR1PIN214

#define MOTOR2PIN1 12
#define MOTOR2PIN2 16

//TIME OF FLIGHT SENSORS - Data
#define LTOFSDA 33
#define MTOFSDA 33
#define RTOFSDA 33

#define SCL 36

void setup(){
    Serial.begin(9600);
    delay(500);

    //L298N Motor Speed Control
    pinMode(motor1pin_, OUTPUT);
    pinMode(motor1pin_, OUTPUT);
    pinMode(motor2pin_, OUTPUT);
    pinMode(motor2pin_, OUTPUT);

    //TIME OF FLIGHT SENSORS
    //LEFT TOF
    pinMode(toflsendpin, OUTPUT);
    pinMode(toflrecpin_, INPUT);

    //MIDDLE TOF
    pinMode(tofmsendpin_, OUTPUT);
    pinMode(tofmrecpin_, INPUT);
    
    //RIGHT TOF
    pinMode(tofrsendpin_, OUTPUT);
    pinMode(tofrrecpin_, INPUT);
}



//TOF readings of left, middle, right distances
void printReading(){
    long cml, cmm, cmr;

}

void loop(){
    printReading();
    delay(100)
}