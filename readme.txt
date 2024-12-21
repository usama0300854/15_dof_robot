sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install libserial-dev
sudo apt install ros-humble-xacro
sudo apt install ros-humble-moveit-configs-utils
sudo apt install python3-pip
pip install moveit_configs_utils
sudo apt install ros-humble-moveit
sudo apt install ros-humble-ament-cmake
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/ros/humble/share/ament_cmake
sudo apt install ros-humble-ament-cmake
sudo apt-get install ros-humble-gazebo-ros
sudo apt-get update && sudo apt-get install ros-humble-chomp* && sudo apt-get install ros-humble-moveit*



sudo apt remove ros-humble-moveit-planners-chomp ros-humble-moveit-chomp-optimizer-adapter











--------------------------------------------------------------------------------------------------------
https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html

sudo add-apt-repository ppa:canonical-kernel-team/proposed
sudo apt install linux-image-lowlatency


sudo nano /etc/security/limits.conf
@realtime  soft  rtprio  99
@realtime  hard  rtprio  99
@realtime  soft  memlock 102400
@realtime  hard  memlock 102400

sudo usermod -aG realtime $USER
sudo apt install rt-tests
sudo cyclictest -p 80 -n -i 10000 -l 100000
sudo cyclictest -p 80 -i 10000 -l 100000 -t 4 -N -v




--------------------------------------------------------------------------------------------------------
odrive 


sudo apt install git tup gcc-arm-none-eabi build-essential
sudo apt-get install libusb-1.0-0-dev ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro

git clone https://github.com/Wetmelon/ODrive.git                   #change to branch you need
cd ODrive/Firmware
cp tup.config.default tup.config
nano tup.config # you need to edit this file to put in your board type and version
tup
cd ..
tools/odrivetool dfu Firmware/build/ODriveFirmware.hex   # to flash the firmware that you just `

-----------

git clone https://github.com/madcowswe/ODrive.git                  #change to branch you need
cd ODrive/Firmware
git checkout rc-v0.5.1
cp tup.config.default tup.config
nano tup.config # you need to edit this file to put in your board type and version
tup
cd ..
tools/odrivetool dfu Firmware/build/ODriveFirmware.hex   # to flash the firmware that you just `



pip3 install PyYAML
pip3 install jsonschema

pip3 install python-can

------------------------
useful info :

You want the SPI / absolute encoder settings.

odrv0.encoder.config.abs_spi_gpio_pin = 3
odrv0.encoder.config.mode = <whatever the AMS mode is>
odrv0.encoder.config.cpr = 2**14

---------------------------------------

Arduino can example :
https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html
https://github.com/sandeepmistry/arduino-CAN/

for teensy :  https://github.com/tonton81/FlexCAN_T4

Teensy 4.0, only CAN3 supports CANFD
FIFO is only supported in CAN2.0


----------------------------
CAN Bus Guide:
https://docs.odriverobotics.com/v/latest/guides/can-guide.html#can-hardware-setup

CAN Protocol:
https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#can-protocol

--------------
buy:
ODrive USB-CAN Adapter
https://docs.odriverobotics.com/v/latest/hardware/usb-can-datasheet.html
https://www.peak-system.com/PCAN-USB-FD.365.0.html?&L=1


https://makerbase3d.com/product/makerbase-canable-v2/
https://www.youtube.com/watch?v=6MChPbeG6D0&list=PLc2RScfrSFEBboq8GDhRPEapSpJCIDf9dhttps://www.youtube.com/watch?v=6MChPbeG6D0&list=PLc2RScfrSFEBboq8GDhRPEapSpJCIDf9d


---------------------

usb isolators :
https://www.aliexpress.com/item/33016336073.html?spm=a2g0s.9042311.0.0.57ec4c4dDADzZo
https://www.aliexpress.com/item/4000060726013.html?spm=a2g0s.9042311.0.0.57ec4c4dDADzZo

----------------------------------------
project:
https://www.hackster.io/jlamperez10/karp-petalinux-and-odrive-motor-controller-eb2ad2
https://gitlab.com/towen/ODrive/-/tree/TOwen/devel/tools/CAN
https://github.com/belovictor/odrive_can_ros_driver


----------------
odrive ros package :
https://github.com/Factor-Robotics/odrive_ros2_control

-----------------
as5047p encoder :
Another thing to remember is that this board needs to be changed to the 3.3V setup for SPI,
I added a 22uF/63V electrolytic capacitor from VDD3V terminal to ground.
The distance should be chosen such that the magnetic field on the die surface is within the specified limits The typical distance “z” between the magnet and the package surface is 0.5mm to 2.5mm,
provided the use of the recommended magnet material and dimensions (6mm x 3mm).
https://discourse.odriverobotics.com/t/as5047p-ts-ek-ab-spi-3v3-soldering-needed/5163
 For 5V operation R1 has to be populated and R2 has to be removed
(default case). Vice versa for 3.3V operation.”


-----------------
useful video of odrive :
https://www.youtube.com/watch?app=desktop&v=Htb2Q0Yw1FU


--------------------
firmware of odrive 3.6:
https://github.com/odriverobotics/ODrive/tree/v0.5.1-rc5


-------------------
odrive can guide :
https://github.com/odriverobotics/ODrive/blob/devel/docs/can-guide.rst
https://docs.odriverobotics.com/v/0.5.6/getting-started.html


--------------------
odrive info :
 the odrive only works with the 8MHz mode
 Move to position” uses Trajectory control 
ODrive v3 supports CAN 2.0b
 GPIOs 1 and 2 are usually used by UART
 


---------------------
linux can command sender software:
sudo apt install can-utils 



---------------------
odrivetool:
controller.config.input_mode=INPUT_MODE_TRAP_TRAJ
set axis.config.

<axis>.encoder.config.abs_spi_cs_gpio_pin = 4  # or which ever GPIO pin you choose
<axis>.encoder.config.mode = ENCODER_MODE_SPI_ABS_CUI   # or ENCODER_MODE_SPI_ABS_AMS
<axis>.encoder.config.cpr = 2**14              # or 2**12 for AMT232A and AMT233A
<odrv>.save_configuration()
<odrv>.reboot()

https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.ControlMode
https://github.com/makerbase-motor/ODrive-MKS/blob/main/03_Makerbase%20ODrive%20related%20components/02_X2212%20Motor%20configuration/X2212%20Motor%20configuration.txt



-------------------------
Standard CAN :

SOF–The single dominant start of frame (SOF) bit marks the start of a message, and is used to
synchronize the nodes on a bus after being idle.
• Identifier-The Standard CAN 11-bit identifier establishes the priority of the message. The lower the
binary value, the higher its priority.
• RTR–The single remote transmission request (RTR) bit is dominant when information is required from
another node. All nodes receive the request, but the identifier determines the specified node. The
responding data is also received by all nodes and used by any node interested. In this way, all data
being used in a system is uniform.
• IDE–A dominant single identifier extension (IDE) bit means that a standard CAN identifier with no
extension is being transmitted.
• r0–Reserved bit (for possible use by future standard amendment).
• DLC–The 4-bit data length code (DLC) contains the number of bytes of data being transmitted.
• Data–Up to 64 bits of application data may be transmitted.
• CRC–The 16-bit (15 bits plus delimiter) cyclic redundancy check (CRC) contains the checksum
(number of bits transmitted) of the preceding application data for error detection.
• ACK–Every node receiving an accurate message overwrites this recessive bit in the original message
with a dominate bit, indicating an error-free message has been sent. Should a receiving node detect an
error and leave this bit recessive, it discards the message and the sending node repeats the message
after rearbitration. In this way, each node acknowledges (ACK) the integrity of its data. ACK is 2 bits,
one is the acknowledgment bit and the second is a delimiter.
• EOF–This end-of-frame (EOF), 7-bit field marks the end of a CAN frame (message) and disables bit-
stuffing, indicating a stuffing error when dominanthttps://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.ControlMode. When 5 bits of the same logic level occur in
succession during normal operation, a bit of the opposite logic level is stuffed into the data.
• IFS–This 7-bit interframe space (IFS) contains the time required by the controller to move a correctly
received frame to its proper position in a message buffer area.


CAN H and CAN L must be twisted, at least 40 turns per meter.
The CAN cable must be well shielded. The shield must be connected to the ground from both ends and all stubs (multipoint grounding).
CAN GND must be connected between the devices, the wire must go with the CAN H and CAN L wires inside the same shield.
The CAN bus must be kept as far away as possible from any high voltage cables, minimum 30 cm.
All high voltage cables must be well shielded.

---------------------------
odrive 3.6 hoverboard motor:
https://docs.odriverobotics.com/v/0.5.6/hoverboard.html


-------------------------
install odrivetool on ubuntu:
https://docs.odriverobotics.com/v/latest/interfaces/odrivetool.html



--------------------------------------------------------------------------------------------------------------------------
odrive 3.6 as5047p configuration:

dev0.axis0.encoder.config.abs_spi_cs_gpio_pin = 4
dev0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
dev0.axis0.encoder.config.cpr = 16384  
dev0.axis0.encoder.config.use_index = False
dev0.axis0.encoder.config.calib_range = 0.1



motor:
dev0.axis0.motor.config.pole_pairs = 7
dev0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
dev0.axis0.motor.config.calibration_current = 10
dev0.axis0.motor.config.resistance_calib_max_voltage = 2
dev0.axis0.motor.config.current_lim = 10
dev0.axis0.motor.config.requested_current_range = 24
dev0.axis0.controller.config.vel_limit_tolerance = 3.0
dev0.axis0.controller.config.vel_limit = 5.0
dev0.config.dc_max_negative_current = -15
dev0.config.max_regen_current = 12.0
dev0.axis0.controller.config.pos_gain = 10
dev0.axis0.controller.config.vel_gain = 0.05
dev0.axis0.controller.config.vel_integrator_gain = 0.05
dev0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
dev0.axis0.encoder.config.calib_scan_distance = 43.9822972     50.26548385620117

dev0.save_configuration()
dev0.reboot()

dev0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
dev0.axis0.motor.error

dev0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
dev0.axis0.encoder.error

dev0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
dev0.axis0.motor.config.pre_calibrated = True
dev0.axis0.encoder.config.pre_calibrated = True
dev0.axis0.config.startup_encoder_offset_calibration = False
dev0.axis0.config.startup_closed_loop_control = True
dev0.save_configuration()


dev0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
dev0.axis0.requested_state = AXIS_STATE_IDLE




dev0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH





dev0.axis0.motor.config.phase_inductance
dev0.axis0.motor.config.phase_resistance 

dev0.axis0.controller.start_anticogging_calibration()
dev0.axis0.controller.error
dev0.axis0.controller.config.anticogging.pre_calibrated = True


dev0.axis0.controller.input_pos = 50
dev0.axis0.controller.input_pos = 1000



--------------------------------
error checking:

dump_errors(dev0)
dev0.vbus_voltage

dev0.axis0.error
dev0.axis0.motor.error
dev0.axis0.encoder.error

dev0.axis0.motor.fet_thermistor.temperature

dev0.axis0.encoder.shadow_count 
dev0.axis0.encoder.pos_estimate

dev0.axis0.encoder.set_linear_count(0)    // reset encoder counts

dev0.clear_errors()

start_liveplotter(lambda:[dev0.axis0.encoder.pos_estimate, dev0.axis0.controller.pos_setpoint])

-------------------------

dev0.axis0.motor.config


dev0.axis0.motor.config.current_lim = 16
dev0.axis0.motor.config.current_lim_margin
dev0.axis0.motor.config.dc_calib_tau
dev0.axis0.motor.config.motor_type
dev0.axis0.motor.config.phase_inductance 
dev0.axis0.motor.config.phase_resistance = 0.065
dev0.axis0.motor.config.pole_pairs


dev0.axis0.motor.config.torque_lim



dev0.axis0.controller.pos_setpoint = 1.0






-----------------------------------------
motor:

8.27 / (motor KV).
dev0.axis0.motor.config.torque_constant = 0.0068

dev0.axis0.motor.config.current_control_bandwidth = 100






---------------------------------
ordive pid tuning :
https://docs.odriverobotics.com/v/0.5.6/control.html#tuning



odrive Position control:
https://docs.odriverobotics.com/v/0.5.6/getting-started.html#position-control-of-m0

----------------------------------------

odrive speed control :

dev0.axis0.controller.config.vel_limit = 45.0
dev0.axis0.trap_traj.config.vel_limit = 45.0
dev0.axis0.trap_traj.config.accel_limit = 5.0
dev0.axis0.trap_traj.config.decel_limit = 5.0





---------------------------------------
pip install python-can
pip install odrive_can

odrive can bus settings:

dev0.axis0.config.can.node_id = 1
dev0.can.config.baud_rate = 250000
dev0.axis0.config.can.bus_vi_rate_ms = 100 
dev0.axis0.config.can.encoder_rate_ms = 50
dev0.axis0.config.can.iq_rate_ms = 100
dev0.axis0.requested_state = AXIS_STATE_IDLE
dev0.save_configuration()

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
partab odrive setup:

dev0.axis0.encoder.config.abs_spi_cs_gpio_pin = 4
dev0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
dev0.axis0.encoder.config.cpr = 16384  
dev0.axis0.encoder.config.use_index = False

dev0.axis0.motor.config.requested_current_range = 40




dev0.axis0.motor.config.pole_pairs = 20
dev0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
dev0.axis0.motor.config.calibration_current = 20
dev0.axis0.motor.config.resistance_calib_max_voltage = 5

dev0.config.dc_max_negative_current = -22.2
dev0.config.max_regen_current = 22.2

dev0.axis0.motor.config.current_lim = 50
dev0.axis0.controller.config.vel_limit = 20
dev0.axis0.trap_traj.config.vel_limit = 20
dev0.axis0.controller.config.vel_limit_tolerance = 3.0
dev0.axis0.trap_traj.config.accel_limit = 10
dev0.axis0.trap_traj.config.decel_limit = 10

dev0.axis0.controller.config.pos_gain = 20
dev0.axis0.controller.config.vel_gain = 0.05
dev0.axis0.controller.config.vel_integrator_gain = 0.05
dev0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

dev0.save_configuration()

dev0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
dev0.axis0.motor.error

dev0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
dev0.axis0.encoder.error

dump_errors(dev0)
dev0.clear_errors()

dev0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
dev0.axis0.motor.config.pre_calibrated = True
dev0.axis0.encoder.config.pre_calibrated = True
dev0.axis0.config.startup_encoder_offset_calibration = False
dev0.axis0.config.startup_closed_loop_control = True

dev0.save_configuration()

dev0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
dev0.axis0.requested_state = AXIS_STATE_IDLE



dev0.axis0.controller.input_pos = 50


dev0.axis0.encoder.shadow_count 
dev0.axis0.encoder.pos_estimate


dev0.axis0.motor.fet_thermistor.temperature



dev0.vbus_voltage




-------------------------------------------------------------------------------------------------------------------------------

orignial odrive 3.6 :


odrive 3.6 as5047p configuration:

odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = 4
odrv0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
odrv0.axis0.encoder.config.cpr = 16384  
odrv0.axis0.encoder.config.use_index = False



motor:
odrv0.axis0.motor.config.pole_pairs = 7
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.motor.config.calibration_current = 10
odrv0.axis0.motor.config.resistance_calib_max_voltage = 2
odrv0.axis0.motor.config.requested_current_range = 16
odrv0.axis0.controller.config.vel_limit_tolerance = 3.0
odrv0.axis0.controller.config.vel_limit = 5.0
odrv0.config.dc_max_negative_current = -12
odrv0.config.max_regen_current = 12
odrv0.axis0.controller.config.pos_gain = 10
odrv0.axis0.controller.config.vel_gain = 0.02
odrv0.axis0.controller.config.vel_integrator_gain = 0.02
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

odrv0.save_configuration()
odrv0.reboot()

odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis0.motor.error
dump_errors(odrv0)
odrv0.clear_errors()

odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv0.axis0.encoder.error

odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.config.startup_encoder_offset_calibration = False
odrv0.axis0.config.startup_closed_loop_control = True
odrv0.save_configuration()


odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.requested_state = AXIS_STATE_IDLE




odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH





odrv0.axis0.motor.config.phase_inductance
odrv0.axis0.motor.config.phase_resistance 

odrv0.axis0.controller.start_anticogging_calibration()
odrv0.axis0.controller.error
odrv0.axis0.controller.config.anticogging.pre_calibrated = True


odrv0.axis0.controller.input_pos = 50
odrv0.axis0.controller.input_pos = 1000



--------------------------------
error checking:

dump_errors(odrv0)
odrv0.vbus_voltage

odrv0.axis0.error
odrv0.axis0.motor.error
odrv0.axis0.encoder.error

odrv0.axis0.motor.fet_thermistor.temperature

odrv0.axis0.encoder.shadow_count 
odrv0.axis0.encoder.pos_estimate

odrv0.axis0.encoder.set_linear_count(0)    // reset encoder counts

odrv0.clear_errors()

start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])

-------------------------

odrv0.axis0.motor.config


odrv0.axis0.motor.config.current_lim = 16
odrv0.axis0.motor.config.current_lim_margin
odrv0.axis0.motor.config.dc_calib_tau
odrv0.axis0.motor.config.motor_type
odrv0.axis0.motor.config.phase_inductance 
odrv0.axis0.motor.config.phase_resistance = 0.065
odrv0.axis0.motor.config.pole_pairs


odrv0.axis0.motor.config.torque_lim



odrv0.axis0.controller.pos_setpoint = 1.0






-----------------------------------------
motor:


odrv0.axis0.motor.config.torque_constant = 0.0068
odrv0.axis0.motor.config.current_control_bandwidth = 100






---------------------------------
ordive pid tuning :
https://docs.odriverobotics.com/v/0.5.6/control.html#tuning



odrive Position control:
https://docs.odriverobotics.com/v/0.5.6/getting-started.html#position-control-of-m0

----------------------------------------

odrive speed control :

odrv0.axis0.controller.config.vel_limit = 10
odrv0.axis0.trap_traj.config.vel_limit = 10
odrv0.axis0.trap_traj.config.accel_limit = 2.5
odrv0.axis0.trap_traj.config.decel_limit = 2.5





---------------------------------------
pip install python-can
pip install odrive_can

odrive can bus settings:

dev0.axis0.config.can.node_id = 3
dev0.can.config.baud_rate = 250000
dev0.axis0.config.can.bus_vi_rate_ms = 100 
dev0.axis0.config.can.encoder_rate_ms = 50
dev0.axis0.config.can.iq_rate_ms = 100
dev0.axis0.requested_state = AXIS_STATE_IDLE
dev0.save_configuration()




--------------------------------------------------------------------------------------------------------------------------------

odrive 3.6 end stop:

dev0.axis0.min_endstop.config.gpio_num = 8
dev0.axis0.min_endstop.config.is_active_high = True
dev0.axis0.min_endstop.config.enabled = True

dev0.axis0.min_endstop.config.offset = 50

dev0.axis0.controller.config.homing_speed = 10.0
dev0.axis0.config.startup_homing = True

dev0.config.gpio8_mode = GPIO_MODE_DIGITAL
dev0.config.gpio8_mode = GPIO_MODE_DIGITAL_PULL_UP

dev0.get_gpio_states(8)
dev0.axis0.min_endstop.endstop_state

-------------------------------------------------------------------------------------------------------------------------
anti_cogging:

dev0.axis0.motor.config.enable_anti_cogging = True
dev0.axis0.requested_state = AXIS_STATE_ANTI_COGGING_CALIBRATION
dev0.save_configuration()


