#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode
from std_msgs.msg import Float32MultiArray
import serial
from serial.serialutil import SerialException
import sys
import time
import threading

class ODriveCommandSubscriber(LifecycleNode):

    def __init__(self, serial_port='/dev/ttyACM0', baudrate=115200):
        super().__init__('odrive_command_subscriber')

        self.serial_port = serial_port
        self.baudrate = baudrate
        self.serial_port_instance = None
        self.command_thread = None
        self.command_lock = threading.Lock()
        self.running = True  

       
        self.initialize_serial_port(self.serial_port)

        
        self.create_subscription(Float32MultiArray, 'odrive_commands', self.command_callback, 10)

        
        self.previous_commands = None

        
        self.command_thread = threading.Thread(target=self.process_commands)
        self.command_thread.start()

    def command_callback(self, msg):
        commands = msg.data

        
        with self.command_lock:
            
            if self.previous_commands is None or not self.are_commands_same(self.previous_commands, commands):
                self.previous_commands = commands  
                self.send_commands_to_serial(commands)

    def process_commands(self):
        while self.running:
            time.sleep(0.1) 

    def are_commands_same(self, prev_commands, new_commands):
        
        return len(prev_commands) == len(new_commands) and all(p == n for p, n in zip(prev_commands, new_commands))

    def initialize_serial_port(self, serial_port):
        try:
            self.serial_port_instance = serial.Serial(port=serial_port, baudrate=self.baudrate, timeout=1)
            self.get_logger().info(f'Successfully connected to {serial_port} at {self.baudrate} baud.')
        except SerialException:
            self.get_logger().warning('Teensy board is not Connected !!!')

    def send_commands_to_serial(self, commands):
        if self.serial_port_instance is not None:
            try:
                command_string = ','.join(map(str, commands)) + '\n'
                self.serial_port_instance.write(command_string.encode('utf-8'))
                self.get_logger().info(f'Sent commands: {command_string.strip()}')
            except SerialException:
                self.get_logger().error('Failed to send data. Attempting to reconnect...')
                self.reconnect_serial()

    def reconnect_serial(self):
        if self.serial_port_instance is not None:
            self.serial_port_instance.close()
            self.serial_port_instance = None

        self.get_logger().info('Attempting to reconnect to the serial port...')
        while not self.serial_port_instance:
            self.initialize_serial_port(self.serial_port)
            time.sleep(1)

    def shutdown(self):
        self.running = False  
        if self.command_thread is not None:
            self.command_thread.join() 

def main(args=None):
    rclpy.init(args=args)

    serial_port = '/dev/ttyACM0'
    baudrate = 115200

    
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]
        if len(sys.argv) > 2:
            baudrate = int(sys.argv[2])

    odrive_command_subscriber = ODriveCommandSubscriber(serial_port, baudrate)

    try:
        rclpy.spin(odrive_command_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        odrive_command_subscriber.shutdown()  
        odrive_command_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
