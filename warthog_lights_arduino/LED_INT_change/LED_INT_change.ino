#include <TimerOne.h>
#include <SPI.h>
#include <mcp_canbus.h>
#include <PCA9685.h>
#include <Wire.h>

// this pin is connected on the protoboard to a resistor setup to convert the 12V motor_enable signal to a digitaly readable value
#define MOTOR_EN A3
//  see datasheet of the can bus shield
#define SPI_CS_PIN 9
// maximum battery soc to show the green color
#define MAX_BATTERY 93.0
// minimum battery voltage to show full red color 
#define MIN_BATTERY 25.0
// the time it take the arduino to erase the last ros command
#define INTERVAL_ROS 1000

// magic numbers
#define BATTERY_CHARGING_STATE 3
#define BATTERY_CHARGED_STATE 8
#define CAN_BUS_BUFFER_SIZE 8
#define BATTERY_THRESHOLD 30
#define BLINK_INTERVAL 500000
#define DOUBLE_BLINK_INTERVAL 200000
#define PULSE_SPEED 15000

enum LED_MODES{
PULSE,
STEADY,
BLINK,
DOUBLE_BLINK,
ROS_MODE
};
const uint16_t RED[] = {2048, 0, 0};
// these ratios are to be used in the white balance in led_set
const uint16_t WHITE[] = {1048,2048,1500};
const uint16_t BLANK[] = {0,0,0};

// Can bus initilazation
MCP_CAN CAN(SPI_CS_PIN);
// PCA 9685 initialization the adresse are determined by a physical switch on the pcb
PCA9685 led_left(B001001);
PCA9685 led_right(B001000);
PCA9685 led_all(PCA9685_I2C_DEF_ALLCALL_PROXYADR);

uint16_t R_0 = 0;uint16_t G_0 = 0;uint16_t B_0 = 0; 
uint16_t R_1 = 0;uint16_t G_1 = 0;uint16_t B_1 = 0;
uint16_t R_2 = 0;uint16_t G_2 = 0;uint16_t B_2 = 0;
uint16_t R_3 = 0;uint16_t G_3 = 0;uint16_t B_3 = 0;

bool ESTOP_STATE = LOW;

void setup() {
// initiating all the necessary variables and startup sequence to use the pca chip correctly
Serial.begin(115200);
Wire.begin();

Serial.println("mon cul");

led_right.resetDevices();

// led_all.resetDevices();

led_right.init();
led_left.init();
//Frequency adapted to the led board's led see led warthog for the individuals led's datasheet
led_all.setPWMFrequency(1200);
led_left.setChannelPWM(0, 2048);
//settings up the MOTOR_EN reading pin this pin will be LOW when ESTOP_STATE is not in function
pinMode(MOTOR_EN, INPUT);
//setting up the canbus with the proper rate

while(!Serial);
while (CAN_OK != CAN.begin(CAN_250KBPS))
    {    

        delay(100);
    }     

//setting up the masks and filter for canbus operations
CAN.init_Mask(0, 0, 0x7ff);
CAN.init_Mask(1, 0, 0x7ff);
CAN.init_Filt(0, 0, 0x1e0);
}
uint16_t current_color[3];
int current_mode;
int last_mode;
int changed =1;
int opacity = 50;
int opacity_scaler = 1;
int blinker = 1;
int db = 0;
unsigned char len = 0;
unsigned char buf[CAN_BUS_BUFFER_SIZE];
int soc = 100;
int state = 0;
unsigned long last_ros = 1000;
uint16_t ROS_COLOR_LF[3];
uint16_t ROS_COLOR_LR[3];
uint16_t ROS_COLOR_RF[3];
uint16_t ROS_COLOR_RR[3];
uint16_t soc_color[3];
float temp;
float mul_G;
float mul_R;

void color_set(const uint16_t left_f[3], const uint16_t right_f[3],
  const uint16_t left_r[3], const uint16_t right_r[3])
    {
      R_0 = left_f[0];R_1 = right_f[0];R_2 = left_r[0];R_3 = right_r[0];
      G_0 = left_f[1];G_1 = right_f[1];G_2 = left_r[1];G_3 = right_r[1];
      B_0 = left_f[2];B_1 = right_f[2];B_2 = left_r[2];B_3 = right_r[2];
      }
void current_color_set(const uint16_t new_color[3]){
  for(int i=0;i<3;i++){
    current_color[i] = new_color[i];
  }}
void color_set(const uint16_t all[3]){
    color_set(all,all,all,all);
    }

void mode_set(int New_Mode){
  if(last_mode == New_Mode){
    return;}
  last_mode = New_Mode;
  Timer1.detachInterrupt();  
  switch(New_Mode){
    case BLINK:
    { blinker = 1;
      Timer1.initialize(BLINK_INTERVAL);
      Timer1.attachInterrupt(blink_int);
      break;}
    case PULSE: 
    { 
      Timer1.initialize(PULSE_SPEED);
      Timer1.attachInterrupt(pulse_int);
      break;}
    case STEADY:
    {
      changed = 1;
      color_set(current_color);
      break;}
    case DOUBLE_BLINK:
    { db = 0;
      Timer1.initialize(DOUBLE_BLINK_INTERVAL);
      Timer1.attachInterrupt(double_blink_int);
      break;}
    default:
      break;
  }}

void pulse_int(){
  uint16_t pulsing[3];
    for(int i = 0;i<3; i++){
      pulsing[i] = current_color[i] / opacity;}
    opacity += opacity_scaler;
    if(opacity >=100 || opacity <= 5){
      opacity_scaler *= -1;}
    color_set(pulsing);
    changed = 1;    
}
void blink_int(){
  if(blinker == 1){color_set(current_color);}
  if(blinker == -1){color_set(BLANK);}
  changed = 1;
  blinker *= -1;
}

void double_blink_int(){
  if(db == 0){
    color_set(current_color);
  }
  if(db == 1){
    color_set(BLANK);
  }
  if(db == 2){
    color_set(current_color);
  }
  if(db == 3){
    color_set(BLANK);
  }
  if(db == 4){
    color_set(current_color);
  }
  if(db == 5){
    color_set(current_color);
  }
  if(db == 7){
    db = 0;
  }
  changed = 1;
  db += 1;
}

void LED_SET(uint16_t R = 0, uint16_t G = 0, uint16_t B = 0)
  {
  if(R != 0 || G != 0 || B != 0){R_0 = R_1 = R_2 = R_3 = R; G_0 = G_1 = G_2 = G_3 = G; B_0 = B_1 = B_2 = B_3 = B;}
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

void check_can_bus(){
if(CAN_MSGAVAIL == CAN.checkReceive())            
    {   CAN.readMsgBuf(&len, buf);    
        state = buf[0];
        soc = buf[2];  
      }
if(soc > MAX_BATTERY){
  soc = MAX_BATTERY;
}
if(soc <= MIN_BATTERY){
  soc = MIN_BATTERY;
}
temp = ((soc - MIN_BATTERY)/(MAX_BATTERY - MIN_BATTERY));
mul_G = temp;
mul_R = 1 - temp;

soc_color[0] = 2048*mul_R/2;
soc_color[1] = 2048*mul_G;
soc_color[2] = 0;

}

void check_serial(){

  if (Serial.available() > 0 ) {
    String Data = Serial.readStringUntil('\n');
    Data.trim(); 
    char inputBuffer[80];  
    Data.toCharArray(inputBuffer, sizeof(inputBuffer)); 
    sscanf(inputBuffer, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", ROS_COLOR_LF[0], ROS_COLOR_LF[1], ROS_COLOR_LF[2], ROS_COLOR_RF[0], ROS_COLOR_RF[1], ROS_COLOR_RF[2], ROS_COLOR_LR[0], ROS_COLOR_LR[1], ROS_COLOR_LR[2], ROS_COLOR_LR[0], ROS_COLOR_RR[1], ROS_COLOR_RR[2]);
    last_ros = millis();
    }

  
 

}

int last_serial = 0;
void loop() { 
current_mode = 0; 
// Highest priority
// reading state of charge through canbus

ESTOP_STATE = digitalRead(MOTOR_EN);

if(digitalRead(MOTOR_EN) && (millis() - last_serial) > 100){
  Serial.write("0\n");
  last_serial = millis();
  }
if(!digitalRead(MOTOR_EN) && (millis() - last_serial) > 100){
  Serial.write("1\n");
  last_serial = millis();
  }
check_can_bus();

check_serial();
  if(((millis()- last_ros) < INTERVAL_ROS)){
    // current_mode == ROS_MODE
    current_mode = STEADY;
    color_set(ROS_COLOR_LF,ROS_COLOR_RF,ROS_COLOR_LR,ROS_COLOR_RR);
  }
  if(ESTOP_STATE == HIGH){
    current_color_set(soc_color);
    current_mode = STEADY;
    }
  if(state == BATTERY_CHARGED_STATE){
    current_mode = STEADY;
    color_set(soc_color);
    }
     
  if(ESTOP_STATE == LOW ){
    current_color_set(soc_color);
    current_mode = BLINK;
  }
  if(soc <= BATTERY_THRESHOLD){
    current_mode = DOUBLE_BLINK;
    current_color_set(soc_color);
    }
  if(state == BATTERY_CHARGING_STATE){
    current_color_set(soc_color);
    current_mode = PULSE;
    }
  
mode_set(current_mode);

if(changed == 1){
LED_SET();
changed = 0;

}
}