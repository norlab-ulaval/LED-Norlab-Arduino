# Warthog Lights Arduino
Arduino code for LED-Personalisation and E-Stop state publishing

<img width="723" height="565" alt="image" src="https://github.com/user-attachments/assets/3e5d3079-10a2-4017-95c6-6bcad06100d6" />

The base Clearpath configuration for the LED has been kept, to customize the lights color's the user must publish on the `warthog/platform/cmd_lights` topic all three RGB componnents must be published at a minimal rate of 1 `Hz` which is the maximum time interval set for ros in the arduino code see an example [here](https://github.com/norlab-ulaval/LED-Norlab-Arduino/blob/main/warthog_lights_arduino/warthog_mcu_link/ros_dummy_publisher.py) . A white balance is also integrated in the arduino code here are the rations `R:0.5` `G:1` `B:0.75` to change see user constants on the top of the arduino sketch.



# LED Behavior 


From the LED control wiki page here is the table of behaviors for the led and the order of priority in which they interact with each other. To change the behaviors of different state, the order of priority or add new behaviors modify the placement of the different `else if` blocks in the main loop of the arduino code [here](https://github.com/norlab-ulaval/LED-Norlab-Arduino/blob/main/Arduino/mcu_arduino_code.ino).


 Priority ranked LED behavior 
| State        | Logic trigger              | Behavior                                 |
|--------------|----------------------------|------------------------------------------|
| Charging     | BMS state 3                | pulsing following soc color gradient     |
| Charged      | BMS state 8                | steady soc full color                    |
| Low battery  | BMS soc below 30           | blinking rapidly soc empty color         |
| E-stop       | Estop state low            | blinking current soc color               |
| ROS          | ros message received       | solid color determined by user           |
| Stand-by     | no higher priority behavior| current soc color                        |

## BMS state of charge(soc) color gradient

Empty ---------------------------> Full

🟥 🟥 🟧 🟧 🟨 🟨 🟩 🟩 🟩 🟩



