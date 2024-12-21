import tkinter as tk
import subprocess


def arm_launch():
    try:
        # Using gnome-terminal with bash and ROS2 launch command
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'source ~/partab_15dof_ws/install/setup.bash && ros2 launch arduinobot_bringup real_robot.launch.py'])
    except Exception as e:
        print(f"Error launching arm: {e}")


def arm_simulation_launch():
    try:
        # Using gnome-terminal with bash and ROS2 launch command
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'source ~/partab_15dof_ws/install/setup.bash && ros2 launch arduinobot_bringup simulated_robot.launch.py'])
    except Exception as e:
        print(f"Error launching arm: {e}")



def start_can_bus_connection():
    try:
        # Command to start socket connection for CAN bus protocol
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'source ~/partab_15dof_ws/install/setup.bash && python3 ~/partab_15dof_ws/src/arduinobot_controller/scripts/odrive_commands.py /dev/ttyACM0 115200'])
    except Exception as e:
        print(f"Error starting CAN bus connection: {e}")


def close_all_terminals():
    try:
        # Close all gnome-terminals
        subprocess.Popen(['pkill', '-f', 'gnome-terminal'])
    except Exception as e:
        print(f"Error closing terminals: {e}")


def main():
    # Initialize the Tkinter GUI window
    root = tk.Tk()
    root.title("ROS2 Humble 15dof Dual Arm Launch Starter")
    root.geometry("400x200")

    # Add label and buttons to the window
    label = tk.Label(root, text="Click the button to start the 15dof Dual Arm robot:")
    label.pack(pady=10)

    start_custom_launch_button = tk.Button(root, text="Start ROS2 Humble 15dof dual Arms", command=arm_launch)
    start_custom_launch_button.pack()

    start_simulation_launch_button = tk.Button(root, text="15dof Simulation", command=arm_simulation_launch)
    start_simulation_launch_button.pack()

    start_micro_controller_button = tk.Button(root, text="Connect Can bus Protocol", command=start_can_bus_connection)
    start_micro_controller_button.pack()

    close_terminals_button = tk.Button(root, text="Close All Terminals", command=close_all_terminals)
    close_terminals_button.pack()

    # Start the Tkinter main loop
    root.mainloop()


if __name__ == "__main__":
    main()
