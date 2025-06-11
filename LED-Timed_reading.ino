#include <TimerOne.h>

#include <PCA9685.h>

#include <Wire.h>

#define Motor_en A2

PCA9685 led_left(B001001);
PCA9685 led_right(B001000);

PCA9685 led_all(PCA9685_I2C_DEF_ALLCALL_PROXYADR);


uint16_t R_0 = 0;uint16_t G_0 = 0;uint16_t B_0 = 2048; 
uint16_t R_1 = 0;uint16_t G_1 = 0;uint16_t B_1 = 2048;
uint16_t R_2 = 0;uint16_t G_2 = 0;uint16_t B_2 = 2048;
uint16_t R_3 = 0;uint16_t G_3 = 0;uint16_t B_3 = 2048;

bool ESTOP;

void LED_set(){

//if(CanBus){}



if(ESTOP == LOW){

led_left.setChannelPWM(0, 2048);
led_left.setChannelPWM(3, 2048);

led_right.setChannelPWM(0, 2048);
led_right.setChannelPWM(3, 2048);

}


else{


led_left.setChannelPWM(0, R_0);
led_left.setChannelPWM(1, G_0);
led_left.setChannelPWM(2, B_0);

led_left.setChannelPWM(3, R_2);
led_left.setChannelPWM(4, G_2);
led_left.setChannelPWM(5, B_2);

//right master board is on the back side of the robot hence the inversion here

led_right.setChannelPWM(0, R_3);
led_right.setChannelPWM(1, G_3);
led_right.setChannelPWM(2, B_3);

led_right.setChannelPWM(3, R_1);
led_right.setChannelPWM(4, G_1);
led_right.setChannelPWM(5, B_1);
}


}


void setup() {


//delay here is to let the PCA 9685 chip time to setup


// initiating all the necessary variables and startup sequence to use the pca chip correctly
Serial.begin(9600);
Wire.begin();
led_all.resetDevices();
led_right.init();
led_left.init();

//Frequency adapted to the lked board's led see led warthog for the individuals led's datasheet
led_all.setPWMFrequency(1200);

//settings up the motor_en reading pin this pin will be LOW when ESTOP is not in function
pinMode(A2, INPUT);
pinMode(Motor_en, INPUT);

//Timmed interrrupt to manage the led's color managment 100 hz 
Timer1.initialize(100000000); //0.001 second (unit is µs)
Timer1.attachInterrupt(LED_set);


}



void loop() {

// Highest priorrity
//canbus buffer reading 

//pending #############


// medium priority
// digital conversion of the motor enable signal from a 12v signal into a step down resistor setup -> 4v == HIGH
ESTOP = digitalRead(A2);



// lowest priority
//Ros interface when ros send a command via serial comms the values are changed accordingly
if (Serial.available() > 0) {
  String Data = Serial.readStringUntil('\n');
  
  Data.trim(); 

  char inputBuffer[50];  
  Data.toCharArray(inputBuffer, sizeof(inputBuffer)); 

  sscanf(inputBuffer, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", &R_0, &G_0, &B_0, &R_1, &G_1, &B_1, &R_2, &G_2, &B_2, &R_3, &G_3, &B_3);

}


if(ESTOP == LOW){

led_left.setChannelPWM(0, 2048);
led_left.setChannelPWM(3, 2048);

led_right.setChannelPWM(0, 2048);
led_right.setChannelPWM(3, 2048);

}





}
