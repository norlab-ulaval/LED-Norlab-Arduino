#include <TimerOne.h>
#include <SPI.h>
#include <mcp_can.h>

#include <PCA9685.h>
#include <Wire.h>


// DO NOT TOUCH
#define MOTOR_EN A3                       //  Pin that reads motor_en signal
#define SPI_CS_PIN 10                     //  See datasheet of the can bus shield
#define BMS_STATE_FRAME 0x1e0             //  CAN frame ID from Sysnergie
#define BATTERY_CHARGING_STATE 3          //  From Sysnergie
#define BATTERY_CHARGED_STATE 8           //  From Sysnergie
#define CAN_BUS_BUFFER_SIZE 8             //  Receiving buffer for CAN Bus
#define PWM_FREQUENCY 1200                //  Frequency of the PWM signal (LED datasheet)

// USER CONSTANTS
#define INTERVAL_ROS 500                 // The time it take the arduino to erase the last ros command
#define BLINK_INTERVAL 500000             // Blinking speed (us)
#define DOUBLE_BLINK_INTERVAL 200000      // Double blinking speed (us)
#define PULSE_SPEED 15000                 // Period of pulsing (us)
#define STEADY_SPEED 100000               // Time between updates of steady (us)
#define MAX_BATTERY 93.0                  // Maximum battery state of charge to show the green color
#define MIN_BATTERY 25.0                  // Minimum battery state of charge to show full red color
#define BATTERY_THRESHOLD 30              // State of charge below which the leds double blink
#define WHITE_BALANCE_R 0.5               // Multiplicator applied to red channel
#define WHITE_BALANCE_G 1.0               // Multiplicator applied to green channel
#define WHITE_BALANCE_B 0.75              // Multiplicator applied to blue channel


// COLOR MAP
const uint16_t RED[] = {2048, 0, 0};
const uint16_t WHITE[] = {2048, 2048, 2048};
const uint16_t BLANK[] = {0, 0, 0};
const uint16_t GREEN[] = {0, 2048, 0};
const uint16_t BLUE[] = {0, 0, 2048};

const uint16_t SIZE_OF_RGB = sizeof(uint16_t) * 3;

// Can bus 
MCP_CAN CAN(SPI_CS_PIN);

// PCA 9685 initialization the adresse are determined by a physical switch on the pcb
PCA9685 led_left(B001001);
PCA9685 led_right(B001000);

uint16_t front_left[3] = {0, 0, 0};
uint16_t front_right[3] = {0, 0, 0};
uint16_t rear_left[3] = {0, 0, 0};
uint16_t rear_right[3] = {0, 0, 0};

enum LED_MODES
{
  STEADY,
  BLINK,
  DOUBLE_BLINK,
  PULSE,
  IDLE
};

// GLOBAL VARIABLES
bool estop_state = LOW;
uint16_t current_color[3];
int last_mode = 1000;
int opacity = 50;
int opacity_scaler = 1;
int blink_counter = 0;
unsigned char len = 0;
unsigned char buf[CAN_BUS_BUFFER_SIZE];
int soc = 100;
int state = 0;
unsigned long last_ros;
uint16_t ros_color_front_left[3];
uint16_t ros_color_rear_left[3];
uint16_t ros_color_front_right[3];
uint16_t ros_color_rear_right[3];
uint16_t soc_color[3];
float mul_G;
float mul_R;
bool ros_bool = false;
long unsigned int canId;
int last_estop = 0;



void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(100);
  Wire.begin();

  led_right.resetDevices();
  led_left.resetDevices();
  led_right.init();
  led_left.init();
  led_left.setPWMFrequency(PWM_FREQUENCY);
  led_right.setPWMFrequency(PWM_FREQUENCY);
  
  pinMode(MOTOR_EN, INPUT);

  while (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK)
  {
    Serial.println("waiting for CAN bus to start");
  }

  // TODO: Figure out how to make filtering work
  CAN.init_Mask(0, 0, 0x7ff);
  CAN.init_Mask(1, 0, 0x7ff);
  CAN.init_Filt(0, 0, BMS_STATE_FRAME);
  CAN.init_Filt(1, 0, BMS_STATE_FRAME);
  CAN.setMode(MCP_NORMAL);

  setMode(STEADY);
}

void loop()
{    
  readCanBus();
  int current_mode = IDLE;
  memcpy(current_color, soc_color, SIZE_OF_RGB);
  
  if (state == BATTERY_CHARGING_STATE)
  {
    current_mode = PULSE;
  }
  else if (soc <= BATTERY_THRESHOLD)
  {
    current_mode = DOUBLE_BLINK;
  }
  else if (checkEstop())
  {
    current_mode = BLINK;
  }
  else if (state == BATTERY_CHARGED_STATE)
  {
    current_mode = STEADY;
  }
  else if (checkRosMsg())
  {
    setColor(ros_color_front_left, ros_color_front_right, ros_color_rear_left, ros_color_rear_right);
  }
  else
  {
    current_mode = STEADY;
  }

  setMode(current_mode);
  writeColor();
}

void readCanBus()
{
  if (CAN.checkReceive() == CAN_MSGAVAIL)
  {
    CAN.readMsgBuf(&canId, &len, buf);
      if(canId == BMS_STATE_FRAME)
      {
        state = buf[0];
        soc = constrain(buf[2], MIN_BATTERY, MAX_BATTERY);
      }
  }
  float soc_norm = ((soc - MIN_BATTERY) / (MAX_BATTERY - MIN_BATTERY));

  soc_color[0] = 2048 * (1 - soc_norm);
  soc_color[1] = 2048 * soc_norm;
  soc_color[2] = 0;
}

bool checkEstop()
{
  estop_state = digitalRead(MOTOR_EN);
  
  if ((millis() - last_estop) > 100)
  {
    char* estop_msg = estop_state ? "0\n" : "1\n";

    Serial.write(estop_msg);
    last_estop = millis();
  }


  return !estop_state;
}

bool checkRosMsg()
{
  
  if (Serial.available() > 0)
  {
    String Data = Serial.readStringUntil('\n');
    Data.trim();
    char inputBuffer[80];
    Data.toCharArray(inputBuffer, sizeof(inputBuffer));
    sscanf(inputBuffer, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
           &ros_color_front_left[0], &ros_color_front_left[1], &ros_color_front_left[2],
           &ros_color_front_right[0], &ros_color_front_right[1], &ros_color_front_right[2],
           &ros_color_rear_left[0], &ros_color_rear_left[1], &ros_color_rear_left[2],
           &ros_color_rear_right[0], &ros_color_rear_right[1], &ros_color_rear_right[2]);
    last_ros = millis();
    return true;
  }
  
  else if ((millis() - last_ros) > INTERVAL_ROS)
  {
    // Reset the ROS colors to blank
    ros_color_front_left[0] = 0;
    ros_color_front_left[1] = 0;
    ros_color_front_left[2] = 0;
    ros_color_front_right[0] = 0;
    ros_color_front_right[1] = 0;
    ros_color_front_right[2] = 0;
    ros_color_rear_left[0] = 0;
    ros_color_rear_left[1] = 0;
    ros_color_rear_left[2] = 0;
    ros_color_rear_right[0] = 0;
    ros_color_rear_right[1] = 0;
    ros_color_rear_right[2] = 0;
    return false;
  }
  return !((millis() - last_ros) > INTERVAL_ROS);
}

void setColor(const uint16_t left_f[3], const uint16_t right_f[3],
              const uint16_t left_r[3], const uint16_t right_r[3])
{
  memcpy(front_left, left_f, SIZE_OF_RGB);
  memcpy(front_right, right_f, SIZE_OF_RGB);
  memcpy(rear_left, left_r, SIZE_OF_RGB);
  memcpy(rear_right, right_r, SIZE_OF_RGB);
}

void setColor(const uint16_t all[3])
{
  setColor(all, all, all, all);
}

void setMode(int new_mode)
{
  if (last_mode == new_mode) return;
  
  Timer1.detachInterrupt();
  switch (new_mode)
  {
  case STEADY:
  {
    Timer1.initialize(STEADY_SPEED);
    Timer1.attachInterrupt(steadyInterrupt);
    break;
  }
  case BLINK:
  {
    blink_counter = 0;
    Timer1.initialize(BLINK_INTERVAL);
    Timer1.attachInterrupt(blinkInterrupt);
    break;
  }
  case DOUBLE_BLINK:
  {
    blink_counter = 0;
    Timer1.initialize(DOUBLE_BLINK_INTERVAL);
    Timer1.attachInterrupt(doubleBlinkInterrupt);
    break;
  }
  case PULSE:
  {
    Timer1.initialize(PULSE_SPEED);
    Timer1.attachInterrupt(pulseInterrupt);
    break;
  }
  default:
    break;
  }
  last_mode = new_mode;

}

void steadyInterrupt()
{
  setColor(current_color);
}

void pulseInterrupt()
{
  uint16_t pulsing[3];
  for (int i = 0; i < 3; i++)
  {
    pulsing[i] = current_color[i] / opacity;
  }
  opacity += opacity_scaler;
  if (opacity >= 100 || opacity <= 5)
  {
    opacity_scaler *= -1;
  }
  setColor(pulsing);
}

void blinkInterrupt()
{
  if (blink_counter)
    setColor(current_color);
  else
    setColor(BLANK);
  blink_counter = (blink_counter + 1) % 2;
}

void doubleBlinkInterrupt()
{
  switch (blink_counter)
  {
    case 0:
      setColor(current_color);
      break;
    case 1:
      setColor(BLANK);
      break;
    case 2:
      setColor(current_color);
      break;
    case 3:
      setColor(BLANK);
      break;
    case 4:
      setColor(current_color);
      break;
    default:
      break;
  }
  blink_counter = (blink_counter + 1) % 8;
}

void writeColor()
{

  led_left.setChannelPWM(0, (uint16_t) (WHITE_BALANCE_R * front_left[0]));
  led_left.setChannelPWM(1, (uint16_t) (WHITE_BALANCE_G * front_left[1]));
  led_left.setChannelPWM(2, (uint16_t) (WHITE_BALANCE_B * front_left[2]));
  led_left.setChannelPWM(3, (uint16_t) (WHITE_BALANCE_R * rear_left[0]));
  led_left.setChannelPWM(4, (uint16_t) (WHITE_BALANCE_G * rear_left[1]));
  led_left.setChannelPWM(5, (uint16_t) (WHITE_BALANCE_B * rear_left[2]));

  led_right.setChannelPWM(0, (uint16_t) (WHITE_BALANCE_R * front_right[0]));
  led_right.setChannelPWM(1, (uint16_t) (WHITE_BALANCE_G * front_right[1]));
  led_right.setChannelPWM(2, (uint16_t) (WHITE_BALANCE_B * front_right[2]));
  led_right.setChannelPWM(3, (uint16_t) (WHITE_BALANCE_R * rear_right[0]));
  led_right.setChannelPWM(4, (uint16_t) (WHITE_BALANCE_G * rear_right[1]));
  led_right.setChannelPWM(5, (uint16_t) (WHITE_BALANCE_B * rear_right[2]));
}


