import can
import struct
import time
import threading
import rclpy
from std_msgs.msg import Float32MultiArray

# Global variables
command_lock = threading.Lock()
previous_commands = None

position_1 = 0.0
velocity_limit_1 = 10.0 
current_limit_1 = 16.0  
previous_current_limit_1 = None

position_2 = 0.0
velocity_limit_2 = 10.0 
current_limit_2 = 16.0  
previous_current_limit_2 = None

position_3 = 0.0
velocity_limit_3 = 10.0 
current_limit_3 = 16.0  
previous_current_limit_3 = None

position_4 = 0.0
velocity_limit_4 = 10.0 
current_limit_4 = 16.0  
previous_current_limit_4 = None

position_5 = 0.0
velocity_limit_5 = 10.0 
current_limit_5 = 16.0  
previous_current_limit_5 = None

position_6 = 0.0
velocity_limit_6 = 10.0 
current_limit_6 = 16.0  
previous_current_limit_6 = None

position_7 = 0.0
velocity_limit_7 = 10.0 
current_limit_7 = 16.0  
previous_current_limit_7 = None

position_8 = 0.0
velocity_limit_8 = 10.0 
current_limit_8 = 16.0  
previous_current_limit_8 = None

position_9 = 0.0
velocity_limit_9 = 10.0 
current_limit_9 = 16.0  
previous_current_limit_9 = None

position_10 = 0.0
velocity_limit_10 = 10.0 
current_limit_10 = 16.0  
previous_current_limit_10 = None

position_11 = 0.0
velocity_limit_11 = 10.0 
current_limit_11 = 16.0  
previous_current_limit_11 = None

position_12 = 0.0
velocity_limit_12 = 10.0 
current_limit_12 = 16.0  
previous_current_limit_12 = None

position_13 = 0.0
velocity_limit_13 = 10.0 
current_limit_13 = 16.0  
previous_current_limit_13 = None

position_14 = 0.0
velocity_limit_14 = 10.0 
current_limit_14 = 16.0  
previous_current_limit_14 = None


position_15 = 0.0
velocity_limit_15 = 10.0 
current_limit_15 = 16.0  
previous_current_limit_15 = None



odrive_1 = 1
odrive_2 = 2
odrive_3 = 3
odrive_4 = 4
odrive_5 = 5
odrive_6 = 6
odrive_7 = 7
odrive_8 = 8
odrive_9 = 9
odrive_10 = 10
odrive_11 = 11
odrive_12 = 12
odrive_13 = 13
odrive_14 = 14
odrive_15 = 15

command_send_position = 0x00C  
command_position_estimate = 0x009 
command_get_bus_voltage_current = 0x17  
command_set_limits = 0x0F  

bus = can.interface.Bus(channel='/dev/ttyACM0', interface='slcan', bitrate=250000)

can_id_set_position_1 = command_send_position + (odrive_1 << 5)  
can_id_get_position_1 = command_position_estimate + (odrive_1 << 5)  
can_id_get_bus_voltage_current_1 = command_get_bus_voltage_current + (odrive_1 << 5)  
can_id_set_limits_1 = command_set_limits + (odrive_1 << 5)  

can_id_set_position_2 = command_send_position + (odrive_2 << 5)  
can_id_get_position_2 = command_position_estimate + (odrive_2 << 5)  
can_id_get_bus_voltage_current_2 = command_get_bus_voltage_current + (odrive_2 << 5)  
can_id_set_limits_2 = command_set_limits + (odrive_2 << 5)  

can_id_set_position_3 = command_send_position + (odrive_3 << 5)  
can_id_get_position_3 = command_position_estimate + (odrive_3 << 5)  
can_id_get_bus_voltage_current_3 = command_get_bus_voltage_current + (odrive_3 << 5)  
can_id_set_limits_3 = command_set_limits + (odrive_3 << 5)  

can_id_set_position_4 = command_send_position + (odrive_4 << 5)  
can_id_get_position_4 = command_position_estimate + (odrive_4 << 5)  
can_id_get_bus_voltage_current_4 = command_get_bus_voltage_current + (odrive_4 << 5)  
can_id_set_limits_4 = command_set_limits + (odrive_4 << 5)  

can_id_set_position_5 = command_send_position + (odrive_5 << 5)  
can_id_get_position_5 = command_position_estimate + (odrive_5 << 5)  
can_id_get_bus_voltage_current_5 = command_get_bus_voltage_current + (odrive_5 << 5)  
can_id_set_limits_5 = command_set_limits + (odrive_5 << 5)  

can_id_set_position_6 = command_send_position + (odrive_6 << 5)  
can_id_get_position_6 = command_position_estimate + (odrive_6 << 5)  
can_id_get_bus_voltage_current_6 = command_get_bus_voltage_current + (odrive_6 << 5)  
can_id_set_limits_6 = command_set_limits + (odrive_6 << 5)  

can_id_set_position_7 = command_send_position + (odrive_7 << 5)  
can_id_get_position_7 = command_position_estimate + (odrive_7 << 5)  
can_id_get_bus_voltage_current_7 = command_get_bus_voltage_current + (odrive_7 << 5)  
can_id_set_limits_7 = command_set_limits + (odrive_7 << 5)  

can_id_set_position_8 = command_send_position + (odrive_8 << 5)  
can_id_get_position_8 = command_position_estimate + (odrive_8 << 5)  
can_id_get_bus_voltage_current_8 = command_get_bus_voltage_current + (odrive_8 << 5)  
can_id_set_limits_8 = command_set_limits + (odrive_8 << 5)  

can_id_set_position_9 = command_send_position + (odrive_9 << 5)  
can_id_get_position_9 = command_position_estimate + (odrive_9 << 5)  
can_id_get_bus_voltage_current_9 = command_get_bus_voltage_current + (odrive_9 << 5)  
can_id_set_limits_9 = command_set_limits + (odrive_9 << 5)  

can_id_set_position_10 = command_send_position + (odrive_10 << 5)  
can_id_get_position_10 = command_position_estimate + (odrive_10 << 5)  
can_id_get_bus_voltage_current_10 = command_get_bus_voltage_current + (odrive_10 << 5)  
can_id_set_limits_10 = command_set_limits + (odrive_10 << 5)  

can_id_set_position_11 = command_send_position + (odrive_11 << 5)  
can_id_get_position_11 = command_position_estimate + (odrive_11 << 5)  
can_id_get_bus_voltage_current_11 = command_get_bus_voltage_current + (odrive_11 << 5)  
can_id_set_limits_11 = command_set_limits + (odrive_11 << 5)  

can_id_set_position_12 = command_send_position + (odrive_12 << 5)  
can_id_get_position_12 = command_position_estimate + (odrive_12 << 5)  
can_id_get_bus_voltage_current_12 = command_get_bus_voltage_current + (odrive_12 << 5)  
can_id_set_limits_12 = command_set_limits + (odrive_12 << 5)  

can_id_set_position_13 = command_send_position + (odrive_13 << 5)  
can_id_get_position_13 = command_position_estimate + (odrive_13 << 5)  
can_id_get_bus_voltage_current_13 = command_get_bus_voltage_current + (odrive_13 << 5)  
can_id_set_limits_13 = command_set_limits + (odrive_13 << 5)  

can_id_set_position_14 = command_send_position + (odrive_14 << 5)  
can_id_get_position_14 = command_position_estimate + (odrive_14 << 5)  
can_id_get_bus_voltage_current_14 = command_get_bus_voltage_current + (odrive_14 << 5)  
can_id_set_limits_14 = command_set_limits + (odrive_14 << 5)  

can_id_set_position_15 = command_send_position + (odrive_15 << 5)  
can_id_get_position_15 = command_position_estimate + (odrive_15 << 5)  
can_id_get_bus_voltage_current_15 = command_get_bus_voltage_current + (odrive_15 << 5)  
can_id_set_limits_15 = command_set_limits + (odrive_15 << 5)  

running = True 


def command_callback(msg):
    global previous_commands
    global position_1
    global position_2
    global position_3
    global position_4
    global position_5
    global position_6
    global position_7
    global position_8
    global position_9
    global position_10
    global position_11
    global position_12
    global position_13
    global position_14
    global position_15

    commands = msg.data  

    with command_lock:
        if previous_commands is None or not are_commands_same(previous_commands, commands):
            previous_commands = commands  

            position_1 = commands[0]  
            send_motor_command(position_1, can_id_set_position_1)

            position_2 = commands[1]  
            send_motor_command(position_2, can_id_set_position_2)

            position_3 = commands[2]  
            send_motor_command(position_3, can_id_set_position_3)

            position_4 = commands[3]  
            send_motor_command(position_4, can_id_set_position_4)

            position_5 = commands[4]  
            send_motor_command(position_5, can_id_set_position_5)

            position_6 = commands[5]  
            send_motor_command(position_6, can_id_set_position_6)

            position_7 = commands[6]  
            send_motor_command(position_7, can_id_set_position_7)

            position_8 = commands[7]  
            send_motor_command(position_8, can_id_set_position_8)

            position_9 = commands[8]  
            send_motor_command(position_9, can_id_set_position_9)

            position_10 = commands[9]  
            send_motor_command(position_10, can_id_set_position_10)

            position_11 = commands[10]  
            send_motor_command(position_11, can_id_set_position_11)

            position_12 = commands[11]  
            send_motor_command(position_12, can_id_set_position_12)

            position_13 = commands[12]  
            send_motor_command(position_13, can_id_set_position_13)

            position_14 = commands[13]  
            send_motor_command(position_14, can_id_set_position_14)

            position_15 = commands[14]  
            send_motor_command(position_15, can_id_set_position_15)


def are_commands_same(prev_commands, new_commands):
    return len(prev_commands) == len(new_commands) and all(p == n for p, n in zip(prev_commands, new_commands))

def odrive_reboot(odrive_id):
    global previous_current_limit_1
    global previous_current_limit_2
    global previous_current_limit_3
    global previous_current_limit_4
    global previous_current_limit_5
    global previous_current_limit_6
    global previous_current_limit_7
    global previous_current_limit_8
    global previous_current_limit_9
    global previous_current_limit_10
    global previous_current_limit_11
    global previous_current_limit_12
    global previous_current_limit_13
    global previous_current_limit_14
    global previous_current_limit_15

    command_reboot = 0x16
    can_id_reboot = command_reboot + (odrive_id << 5) 

    reboot_data = struct.pack('<B', 0)  
    msg_reboot = can.Message(arbitration_id=can_id_reboot, data=reboot_data, is_extended_id=False)

    try:
        bus.send(msg_reboot)  
        print(f"Reboot command sent to ODrive {odrive_id}")
        previous_current_limit_1 = None 
        previous_current_limit_2 = None 
        previous_current_limit_3 = None 
        previous_current_limit_4 = None 
        previous_current_limit_5 = None 
        previous_current_limit_6 = None 
        previous_current_limit_7 = None 
        previous_current_limit_8 = None 
        previous_current_limit_9 = None 
        previous_current_limit_10 = None 
        previous_current_limit_11 = None 
        previous_current_limit_12 = None 
        previous_current_limit_13 = None 
        previous_current_limit_14 = None 
        previous_current_limit_15 = None 
        time.sleep(0.5)
      
        
 
    except can.CanError as e:
        print(f"Failed to send reboot command: {e}")

def get_odrive_errors(odrive_id):
    command_get_error = 0x03  
    can_id_get_error = command_get_error + (odrive_id << 5)  

    msg_get_error = can.Message(arbitration_id=can_id_get_error, data=[], is_extended_id=False)

    try:
        bus.send(msg_get_error)  
    except can.CanError as e:
        print(f"Failed to send Get_Error command: {e}")

    msg = bus.recv(timeout=1)
    if msg is not None:
        if msg.arbitration_id == can_id_get_error:
            try:
                active_errors, disarm_reason = struct.unpack('<II', msg.data)
                if active_errors != 0:
                    if active_errors == 4096:
                            print(f"End Stop Pressed on ODrive {odrive_id}")
                    print(f"Error occurred: {active_errors} on ODrive {odrive_id}")
                    odrive_reboot(odrive_id)
                    time.sleep(5)
            except struct.error as e:
                print(f"Error unpacking Get_Error response: {e}")
    else:
        print("No response received for Get_Error command.")

def reset_bus():
    global bus
    print("Resetting CAN bus connection...")
    bus.shutdown()
    time.sleep(1)  
    return can.interface.Bus(channel='/dev/ttyACM0', interface='slcan', bitrate=250000)

def send_motor_command(command, can_id_set_pos):
    global bus
    data = struct.pack('<f', command) + b'\x00' * 4
    msg_set_position = can.Message(arbitration_id=can_id_set_pos, data=data, is_extended_id=False)

    try:
        bus.send(msg_set_position)
    except can.CanError as e:
        print(f"Failed to send position command: {e}")

def get_encoder_data(can_id_get_pos, odrive_id):  
    global bus
    global previous_current_limit_1
    global previous_current_limit_2
    global previous_current_limit_3
    global previous_current_limit_4
    global previous_current_limit_5
    global previous_current_limit_6
    global previous_current_limit_7
    global previous_current_limit_8
    global previous_current_limit_9
    global previous_current_limit_10
    global previous_current_limit_11
    global previous_current_limit_12
    global previous_current_limit_13
    global previous_current_limit_14
    global previous_current_limit_15

  


    msg_get_position = can.Message(arbitration_id=can_id_get_pos, data=[], is_extended_id=False)

    try:
        bus.send(msg_get_position)
    except can.CanError as e:
        print(f"Failed to send position request: {e}")

    msg = bus.recv(timeout=1)
    if msg is not None:
        if msg.arbitration_id == can_id_get_pos:
            try:
                pos_estimate, _ = struct.unpack('<ff', msg.data)
                if pos_estimate != 0.0:
                    print(f"odrive {odrive_id} : Position Estimate : {pos_estimate}")

            except struct.error as e:
                print(f"Error unpacking message: {e}")
    else:
        print(f"No response received from odrive :: {odrive_id} for position estimate, resetting bus...")
        bus = reset_bus()
        time.sleep(1)
        odrive_reboot(odrive_id)
        time.sleep(5)
        previous_current_limit_1 = None  
        previous_current_limit_2 = None  
        previous_current_limit_3 = None  
        previous_current_limit_4 = None  
        previous_current_limit_5 = None  
        previous_current_limit_6 = None  
        previous_current_limit_7 = None  
        previous_current_limit_8 = None  
        previous_current_limit_9 = None  
        previous_current_limit_10 = None  
        previous_current_limit_11 = None  
        previous_current_limit_12 = None  
        previous_current_limit_13 = None  
        previous_current_limit_14 = None  
        previous_current_limit_15 = None  

def odrive_battery_voltage(can_id_get_bus_voltage, odrive_id):
    global bus
    global previous_current_limit_1
    global previous_current_limit_2
    global previous_current_limit_3
    global previous_current_limit_4
    global previous_current_limit_5
    global previous_current_limit_6
    global previous_current_limit_7
    global previous_current_limit_8
    global previous_current_limit_9
    global previous_current_limit_10
    global previous_current_limit_11
    global previous_current_limit_12
    global previous_current_limit_13
    global previous_current_limit_14
    global previous_current_limit_15

    msg_get_bus_voltage_current = can.Message(arbitration_id=can_id_get_bus_voltage, data=[], is_extended_id=False)

    try:
        bus.send(msg_get_bus_voltage_current)
    except can.CanError as e:
        print(f"Failed to send bus voltage/current request: {e}")
        return  

    msg = bus.recv(timeout=1)
    if msg is not None:
        if msg.arbitration_id == can_id_get_bus_voltage:
            try:
                bus_voltage, bus_current = struct.unpack('<ff', msg.data)
            except struct.error as e:
                print(f"Error unpacking bus voltage/current message: {e}")
    else:
        print("No response received for bus voltage/current, resetting bus...")
        bus = reset_bus()
        time.sleep(1)
        odrive_reboot(odrive_id)
        time.sleep(5)
        previous_current_limit_1 = None  
        previous_current_limit_2 = None  
        previous_current_limit_3 = None  
        previous_current_limit_4 = None  
        previous_current_limit_5 = None  
        previous_current_limit_6 = None  
        previous_current_limit_7 = None  
        previous_current_limit_8 = None  
        previous_current_limit_9 = None  
        previous_current_limit_10 = None  
        previous_current_limit_11 = None  
        previous_current_limit_12 = None  
        previous_current_limit_13 = None  
        previous_current_limit_14 = None  
        previous_current_limit_15 = None  

def odrive_motor_current_limit(current_lim, previous_current_lim, can_id_set_lim, velocity_lim, odrive_id):
    global bus
    global previous_current_limit_1
    global previous_current_limit_2
    global previous_current_limit_3
    global previous_current_limit_4
    global previous_current_limit_5
    global previous_current_limit_6
    global previous_current_limit_7
    global previous_current_limit_8
    global previous_current_limit_9
    global previous_current_limit_10
    global previous_current_limit_11
    global previous_current_limit_12
    global previous_current_limit_13
    global previous_current_limit_14
    global previous_current_limit_15

    if current_lim != previous_current_lim:
        limit_data = struct.pack('<ff', velocity_lim, current_lim) 
        msg_set_limits = can.Message(arbitration_id=can_id_set_lim, data=limit_data, is_extended_id=False)
        try:
            bus.send(msg_set_limits)
            if odrive_id == 1:
                previous_current_limit_1 = current_lim  
                print(f"Set Limits Command Sent to odrive {odrive_id} :: {velocity_lim} rev/s, {current_lim} Amperes")

            elif odrive_id == 2:
                previous_current_limit_2 = current_lim  
                print(f"Set Limits Command Sent to odrive {odrive_id} :: {velocity_lim} rev/s, {current_lim} Amperes")

            elif odrive_id == 3:
                previous_current_limit_3 = current_lim  
                print(f"Set Limits Command Sent to odrive {odrive_id} :: {velocity_lim} rev/s, {current_lim} Amperes")

            elif odrive_id == 4:
                previous_current_limit_4 = current_lim  
                print(f"Set Limits Command Sent to odrive {odrive_id} :: {velocity_lim} rev/s, {current_lim} Amperes")

            elif odrive_id == 5:
                previous_current_limit_5 = current_lim  
                print(f"Set Limits Command Sent to odrive {odrive_id} :: {velocity_lim} rev/s, {current_lim} Amperes")

            elif odrive_id == 6:
                previous_current_limit_6 = current_lim  
                print(f"Set Limits Command Sent to odrive {odrive_id} :: {velocity_lim} rev/s, {current_lim} Amperes")

            elif odrive_id == 7:
                previous_current_limit_7 = current_lim  
                print(f"Set Limits Command Sent to odrive {odrive_id} :: {velocity_lim} rev/s, {current_lim} Amperes")

            elif odrive_id == 8:
                previous_current_limit_8 = current_lim  
                print(f"Set Limits Command Sent to odrive {odrive_id} :: {velocity_lim} rev/s, {current_lim} Amperes")

            elif odrive_id == 9:
                previous_current_limit_9 = current_lim  
                print(f"Set Limits Command Sent to odrive {odrive_id} :: {velocity_lim} rev/s, {current_lim} Amperes")

            elif odrive_id == 10:
                previous_current_limit_10 = current_lim  
                print(f"Set Limits Command Sent to odrive {odrive_id} :: {velocity_lim} rev/s, {current_lim} Amperes")

            elif odrive_id == 11:
                previous_current_limit_11 = current_lim  
                print(f"Set Limits Command Sent to odrive {odrive_id} :: {velocity_lim} rev/s, {current_lim} Amperes")

            elif odrive_id == 12:
                previous_current_limit_12 = current_lim  
                print(f"Set Limits Command Sent to odrive {odrive_id} :: {velocity_lim} rev/s, {current_lim} Amperes")

            elif odrive_id == 13:
                previous_current_limit_13 = current_lim  
                print(f"Set Limits Command Sent to odrive {odrive_id} :: {velocity_lim} rev/s, {current_lim} Amperes")

            elif odrive_id == 14:
                previous_current_limit_14 = current_lim  
                print(f"Set Limits Command Sent to odrive {odrive_id} :: {velocity_lim} rev/s, {current_lim} Amperes")

            elif odrive_id == 15:
                previous_current_limit_15 = current_lim  
                print(f"Set Limits Command Sent to odrive {odrive_id} :: {velocity_lim} rev/s, {current_lim} Amperes")                                                

        except can.CanError as e:
            print(f"Failed to send set limits command: {e}")

def set_absolute_position(odrive_id, encoder_pos):
    command_set_absolute_position = 0x019
    can_id_set_position = command_set_absolute_position + (odrive_id << 5) 

   
    position_data = struct.pack('<f', encoder_pos)  
    
    data = position_data + b'\x00' * 4  

    msg_set_position = can.Message(arbitration_id=can_id_set_position, data=data, is_extended_id=False)

    try:
        bus.send(msg_set_position)
        print(f"Set Absolute Position command sent to ODrive {odrive_id}: {encoder_pos} rev")
    except can.CanError as e:
        print(f"Failed to send Set Absolute Position command: {e}")


def can_communication_thread():
    global running
    while running:
        
        # odrive_battery_voltage(can_id_get_bus_voltage_current_1, odrive_1)
        odrive_motor_current_limit(current_limit_1, previous_current_limit_1, can_id_set_limits_1, velocity_limit_1, odrive_1)
        get_odrive_errors(odrive_1)
        get_encoder_data(can_id_get_position_1, odrive_1)

        # odrive_battery_voltage(can_id_get_bus_voltage_current_2, odrive_2)
        odrive_motor_current_limit(current_limit_2, previous_current_limit_2, can_id_set_limits_2, velocity_limit_2, odrive_2)
        get_odrive_errors(odrive_2)
        get_encoder_data(can_id_get_position_2, odrive_2)

        # odrive_battery_voltage(can_id_get_bus_voltage_current_3, odrive_3)
        odrive_motor_current_limit(current_limit_3, previous_current_limit_3, can_id_set_limits_3, velocity_limit_3, odrive_3)
        get_odrive_errors(odrive_3)
        get_encoder_data(can_id_get_position_3, odrive_3)

        # odrive_battery_voltage(can_id_get_bus_voltage_current_4, odrive_4)
        odrive_motor_current_limit(current_limit_4, previous_current_limit_4, can_id_set_limits_4, velocity_limit_4, odrive_4)
        get_odrive_errors(odrive_4)
        get_encoder_data(can_id_get_position_4, odrive_4)

        # odrive_battery_voltage(can_id_get_bus_voltage_current_5, odrive_5)
        odrive_motor_current_limit(current_limit_5, previous_current_limit_5, can_id_set_limits_5, velocity_limit_5, odrive_5)
        get_odrive_errors(odrive_5)
        get_encoder_data(can_id_get_position_5, odrive_5)

        # odrive_battery_voltage(can_id_get_bus_voltage_current_6, odrive_6)
        odrive_motor_current_limit(current_limit_6, previous_current_limit_6, can_id_set_limits_6, velocity_limit_6, odrive_6)
        get_odrive_errors(odrive_6)
        get_encoder_data(can_id_get_position_6, odrive_6)

        # odrive_battery_voltage(can_id_get_bus_voltage_current_7, odrive_7)
        odrive_motor_current_limit(current_limit_7, previous_current_limit_7, can_id_set_limits_7, velocity_limit_7, odrive_7)
        get_odrive_errors(odrive_7)
        get_encoder_data(can_id_get_position_7, odrive_7)

        # odrive_battery_voltage(can_id_get_bus_voltage_current_8, odrive_8)
        odrive_motor_current_limit(current_limit_8, previous_current_limit_8, can_id_set_limits_8, velocity_limit_8, odrive_8)
        get_odrive_errors(odrive_8)
        get_encoder_data(can_id_get_position_8, odrive_8)

        # odrive_battery_voltage(can_id_get_bus_voltage_current_9, odrive_9)
        odrive_motor_current_limit(current_limit_9, previous_current_limit_9, can_id_set_limits_9, velocity_limit_9, odrive_9)
        get_odrive_errors(odrive_9)
        get_encoder_data(can_id_get_position_9, odrive_9)

        # odrive_battery_voltage(can_id_get_bus_voltage_current_10, odrive_10)
        odrive_motor_current_limit(current_limit_10, previous_current_limit_10, can_id_set_limits_10, velocity_limit_10, odrive_10)
        get_odrive_errors(odrive_10)
        get_encoder_data(can_id_get_position_10, odrive_10)

        # odrive_battery_voltage(can_id_get_bus_voltage_current_11, odrive_11)
        odrive_motor_current_limit(current_limit_11, previous_current_limit_11, can_id_set_limits_11, velocity_limit_11, odrive_11)
        get_odrive_errors(odrive_11)
        get_encoder_data(can_id_get_position_11, odrive_11)

        # odrive_battery_voltage(can_id_get_bus_voltage_current_12, odrive_12)
        odrive_motor_current_limit(current_limit_12, previous_current_limit_12, can_id_set_limits_12, velocity_limit_12, odrive_12)
        get_odrive_errors(odrive_12)
        get_encoder_data(can_id_get_position_12, odrive_12)

        # odrive_battery_voltage(can_id_get_bus_voltage_current_13, odrive_13)
        odrive_motor_current_limit(current_limit_13, previous_current_limit_13, can_id_set_limits_13, velocity_limit_13, odrive_13)
        get_odrive_errors(odrive_13)
        get_encoder_data(can_id_get_position_13, odrive_13)

        # odrive_battery_voltage(can_id_get_bus_voltage_current_14, odrive_14)
        odrive_motor_current_limit(current_limit_14, previous_current_limit_14, can_id_set_limits_14, velocity_limit_14, odrive_14)
        get_odrive_errors(odrive_14)
        get_encoder_data(can_id_get_position_14, odrive_14)

        # odrive_battery_voltage(can_id_get_bus_voltage_current_15, odrive_15)
        odrive_motor_current_limit(current_limit_15, previous_current_limit_15, can_id_set_limits_15, velocity_limit_15, odrive_15)
        get_odrive_errors(odrive_15)
        get_encoder_data(can_id_get_position_15, odrive_15)

        # time.sleep(0.015) # 15 milliseconds

def main():
    global running
    rclpy.init()
    node = rclpy.create_node('odrive_controller')
    
    node.create_subscription(Float32MultiArray, 'odrive_commands', command_callback, 10)

    
    can_thread = threading.Thread(target=can_communication_thread)
    can_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        running = False 
        can_thread.join()  
        bus.shutdown()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()