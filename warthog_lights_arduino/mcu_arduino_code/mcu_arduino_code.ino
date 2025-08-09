#include <TimerOne.h>
#include <SPI.h>
#include <mcp_can.h>

#include <PCA9685.h>
#include <Wire.h>

// this pin is connected on the protoboard to a resistor setup to convert the 12V motor_enable signal to a digitaly readable value
#define MOTOR_EN A3
//  see datasheet of the can bus shield
#define SPI_CS_PIN 10
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
#define STEADY_SPEED 100000
#define BMS_STATE_FRAME 0x1e0

enum LED_MODES
{
  PULSE,
  STEADY,
  BLINK,
  DOUBLE_BLINK,
  ROS_MODE,
  INIT
};
const uint16_t RED[] = {2048, 0, 0};
// these ratios are to be used in the white balance in led_set
const uint16_t WHITE[] = {1048, 2048, 1500};
const uint16_t BLANK[] = {0, 0, 0};
const uint16_t GREEN[] = {0, 2048, 0};
const uint16_t BLUE[] = {0, 0, 2048};

const uint16_t SIZE_OF_RGB = sizeof(uint16_t) * 3;

// Can bus initilazation
MCP_CAN CAN(SPI_CS_PIN);
long unsigned int canId;

// PCA 9685 initialization the adresse are determined by a physical switch on the pcb
PCA9685 led_left(B001001);
PCA9685 led_right(B001000);
PCA9685 led_all(PCA9685_I2C_DEF_ALLCALL_PROXYADR);

uint16_t left_front[3] = {0, 0, 0};
uint16_t right_front[3] = {0, 0, 0};
uint16_t left_rear[3] = {0, 0, 0};
uint16_t right_rear[3] = {0, 0, 0};

bool ESTOP_STATE = LOW;

void setup()
{
  // initiating all the necessary variables and startup sequence to use the pca chip correctly
  Serial.begin(115200);
  Serial.setTimeout(100);
  Wire.begin();

  led_right.resetDevices();
  led_left.resetDevices();
  // led_all.resetDevices();

  led_right.init();
  led_left.init();
  // Frequency adapted to the led board's led see led warthog for the individuals led's datasheet
  led_all.setPWMFrequency(1200);

  // settings up the MOTOR_EN reading pin this pin will be LOW when ESTOP_STATE is not in function
  pinMode(MOTOR_EN, INPUT);
  // setting up the canbus with the proper rate

  while (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK)
  {
    Serial.println("waiting...");
  }

  // setting up the masks and filter for canbus operations
  CAN.init_Mask(0, 0, 0x7ff);
  CAN.init_Mask(1, 0, 0x7ff);
  CAN.init_Filt(0, 0, 0x1e0);
  CAN.setMode(MCP_NORMAL);
}
uint16_t current_color[3];
int current_mode = INIT;
int last_mode = INIT;
int changed = 1;
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
bool ros_bool = 0;

void color_set(const uint16_t left_f[3], const uint16_t right_f[3],
               const uint16_t left_r[3], const uint16_t right_r[3])
{
  memcpy(left_front, left_f, SIZE_OF_RGB);
  memcpy(right_front, right_f, SIZE_OF_RGB);
  memcpy(left_rear, left_r, SIZE_OF_RGB);
  memcpy(right_rear, right_r, SIZE_OF_RGB);
}

void color_set(const uint16_t all[3])
{
  color_set(all, all, all, all);
}

void mode_set(int New_Mode)
{
  if (last_mode == New_Mode)
  {
    return;
  }
  last_mode = New_Mode;
  Timer1.detachInterrupt();
  switch (New_Mode)
  {
  case BLINK:
  {
    blinker = 1;
    Timer1.initialize(BLINK_INTERVAL);
    Timer1.attachInterrupt(blink_int);
    break;
  }
  case PULSE:
  {
    Timer1.initialize(PULSE_SPEED);
    Timer1.attachInterrupt(pulse_int);
    break;
  }
  case STEADY:
  {
    Timer1.initialize(STEADY_SPEED);
    Timer1.attachInterrupt(steady_int);
    color_set(current_color);
    break;
  }
  case DOUBLE_BLINK:
  {
    db = 0;
    Timer1.initialize(DOUBLE_BLINK_INTERVAL);
    Timer1.attachInterrupt(double_blink_int);
    break;
  }
  case ROS_MODE:
  {
    Timer1.initialize(BLINK_INTERVAL);
    Timer1.attachInterrupt(ros_int);
    break;
  }
  default:
    break;
  }
}

void steady_int()
{
  color_set(current_color);
  changed = 1;
}

void ros_int()
{
  color_set(BLUE);
  changed = 1;
}

void pulse_int()
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
  color_set(pulsing);
  changed = 1;
}
void blink_int()
{
  if (blinker == -1)
  {
    color_set(current_color);
  }
  if (blinker == 1)
  {
    color_set(BLANK);
  }
  changed = 1;
  blinker *= -1;
}

void double_blink_int()
{
  switch (db)
  {
  case 0:
  case 2:
  case 4:
  case 5:
    color_set(current_color);
    break;
  case 1:
  case 3:
    color_set(BLANK);
    break;
  case 7:
    db = 0;
    break;
  default:
    break;
  }
  changed = 1;
  db += 1;
}

void LED_SET()
{
  led_left.setChannelPWM(0, left_front[0]);
  led_left.setChannelPWM(1, left_front[1]);
  led_left.setChannelPWM(2, left_front[2]);
  led_left.setChannelPWM(3, left_rear[0]);
  led_left.setChannelPWM(4, left_rear[1]);
  led_left.setChannelPWM(5, left_rear[2]);
  // right master board is on the back side of the robot hence the inversion here
  led_right.setChannelPWM(0, right_front[0]);
  led_right.setChannelPWM(1, right_front[1]);
  led_right.setChannelPWM(2, right_front[2]);
  led_right.setChannelPWM(3, right_rear[0]);
  led_right.setChannelPWM(4, right_rear[1]);
  led_right.setChannelPWM(5, right_rear[2]);
}

void check_can_bus()
{
  if (CAN.checkReceive() == CAN_MSGAVAIL)
  {
    CAN.readMsgBuf(&canId, &len, buf);
    if (canId == BMS_STATE_FRAME)
    {
      state = buf[0];
      soc = buf[2];
    }
  }
  soc = constrain(soc, MIN_BATTERY, MAX_BATTERY);
  temp = ((soc - MIN_BATTERY) / (MAX_BATTERY - MIN_BATTERY));
  mul_G = temp;
  mul_R = 1 - temp;

  soc_color[0] = 2048 * mul_R / 2;
  soc_color[1] = 2048 * mul_G;
  soc_color[2] = 0;
}
int last_serial = 0;
void check_serial()
{

  if (Serial.available() > 0)
  {
    String Data = Serial.readStringUntil('\n');
    Data.trim();
    char inputBuffer[80];
    Data.toCharArray(inputBuffer, sizeof(inputBuffer));
    sscanf(inputBuffer, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
           &ROS_COLOR_LF[0], &ROS_COLOR_LF[1], &ROS_COLOR_LF[2],
           &ROS_COLOR_RF[0], &ROS_COLOR_RF[1], &ROS_COLOR_RF[2],
           &ROS_COLOR_LR[0], &ROS_COLOR_LR[1], &ROS_COLOR_LR[2],
           &ROS_COLOR_RR[0], &ROS_COLOR_RR[1], &ROS_COLOR_RR[2]);
    last_ros = millis();
    ros_bool = 1;
  }
  else if ((millis() - last_ros) > INTERVAL_ROS)
  {
    ros_bool = 0;
  }

}

void check_estop()
{
  ESTOP_STATE = digitalRead(MOTOR_EN);
  if (ESTOP_STATE == HIGH)return;
  
  else if ((millis() - last_serial) > 100)
  {
    Serial.write("1\n");
    last_serial = millis();
  }
}

void loop()
{    
  current_mode = INIT;

  check_can_bus();

  check_serial();

  check_estop();

  if (ESTOP_STATE == HIGH)
  {
    memcpy(current_color, soc_color, SIZE_OF_RGB);
    current_mode = STEADY;
  }
  if (ros_bool)
  {
    current_mode = ROS_MODE;
    // color_set(ROS_COLOR_LF, ROS_COLOR_RF, ROS_COLOR_LR, ROS_COLOR_RR);
  }
  if (state == BATTERY_CHARGED_STATE)
  {
    current_mode = STEADY;
    memcpy(current_color, soc_color, SIZE_OF_RGB);
  }

  if (ESTOP_STATE == LOW)
  {
    memcpy(current_color, soc_color, SIZE_OF_RGB);
    current_mode = BLINK;
  }
  if (soc <= BATTERY_THRESHOLD)
  {
    current_mode = DOUBLE_BLINK;
    memcpy(current_color, RED, SIZE_OF_RGB);
  }
  if (state == BATTERY_CHARGING_STATE)
  {
    memcpy(current_color, soc_color, SIZE_OF_RGB);
    current_mode = PULSE;
  }

  mode_set(current_mode);

  if (changed == 1)
  {
    LED_SET();
    changed = 0;
  }
}