#!/usr/bin/env python

import sys
import os
import time
import rospy
import json
import math
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_commander import roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import threading

# Define separate directories for normal positions and freedrive sequences
POSITIONS_DIR = 'positions'
POSITIONS_FD_DIR = 'positions_fd'

def get_position_file(position_id):
    """Get the file path for a normal position."""
    return os.path.join(POSITIONS_DIR, f'position_{position_id}.json')

def get_freedrive_position_file(position_id):
    """Get the file path for a freedrive sequence."""
    return os.path.join(POSITIONS_FD_DIR, f'freedrive_{position_id}.json')

def move_joint(joint_positions):
    """Move the robot to the specified joint positions."""
    group = MoveGroupCommander("manipulator")
    group.go(joint_positions, wait=True)
    group.stop()

def save_position(position_id, position_data):
    """Save a normal position to the positions directory."""
    if not os.path.exists(POSITIONS_DIR):
        os.makedirs(POSITIONS_DIR)
    position_file = get_position_file(position_id)
    with open(position_file, 'w') as f:
        json.dump(position_data, f, indent=4)
    rospy.loginfo(f"Position saved to {position_file}")

def load_position(position_id):
    """Load a normal position from the positions directory."""
    position_file = get_position_file(position_id)
    try:
        with open(position_file, 'r') as f:
            position_data = json.load(f)
        rospy.loginfo(f"Position loaded from {position_file}")
        return position_data
    except FileNotFoundError:
        rospy.logwarn(f"Position file {position_file} not found.")
        return None

def save_freedrive_position(position_id, position_data):
    """Save a freedrive sequence to the positions_fd directory."""
    if not os.path.exists(POSITIONS_FD_DIR):
        os.makedirs(POSITIONS_FD_DIR)
    position_file = get_freedrive_position_file(position_id)
    with open(position_file, 'w') as f:
        json.dump(position_data, f, indent=4)
    rospy.loginfo(f"Freedrive position sequence saved to {position_file}")

def load_freedrive_position(position_id):
    """Load a freedrive sequence from the positions_fd directory."""
    position_file = get_freedrive_position_file(position_id)
    try:
        with open(position_file, 'r') as f:
            position_data = json.load(f)
        rospy.loginfo(f"Freedrive sequence loaded from {position_file}")
        return position_data
    except FileNotFoundError:
        rospy.logwarn(f"Freedrive sequence file {position_file} not found.")
        return None

def get_all_positions():
    """Retrieve all normal positions from the positions directory."""
    positions = []
    if os.path.exists(POSITIONS_DIR):
        for filename in os.listdir(POSITIONS_DIR):
            if filename.startswith('position_') and filename.endswith('.json'):
                try:
                    position_id = int(filename.split('_')[1].split('.')[0])
                    position_data = load_position(position_id)
                    if position_data:
                        positions.append(position_data)
                except (IndexError, ValueError):
                    rospy.logwarn(f"Filename {filename} does not match expected format.")
    positions.sort(key=lambda x: x['PositionID'])

    return positions

def get_all_freedrive_sequences():
    """Retrieve all freedrive sequences from the positions_fd directory."""
    sequences = []
    if os.path.exists(POSITIONS_FD_DIR):
        for filename in os.listdir(POSITIONS_FD_DIR):
            if filename.startswith('freedrive_') and filename.endswith('.json'):
                try:
                    position_id = int(filename.split('_')[1].split('.')[0])
                    position_data = load_freedrive_position(position_id)
                    if position_data:
                        sequences.append(position_data)
                except (IndexError, ValueError):
                    rospy.logwarn(f"Filename {filename} does not match expected format.")
    return sequences

def record_freedrive():
    """
    Records the robot's joint positions while in freedrive mode.
    The recording captures joint positions every 0.2 seconds.
    The recording stops when the user presses Enter.
    """
    recorded_positions = []
    stop_event = threading.Event()
    latest_joint_positions = []

    def joint_states_callback(msg):
        nonlocal latest_joint_positions
        latest_joint_positions = list(msg.position)  # Convert tuple to list

    def wait_for_enter():
        input("Press Enter to stop recording Freedrive motions.\n")
        stop_event.set()

    # Subscribe to joint states
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    rospy.loginfo("Entering Freedrive Recording Mode.")
    rospy.loginfo("Please manually move the robot. Recording will capture positions every 0.2 seconds.")
    rospy.loginfo("Press Enter to stop recording.")

    # Start the thread to listen for Enter key
    input_thread = threading.Thread(target=wait_for_enter)
    input_thread.start()

    # Record positions at 0.2 second intervals
    rate = rospy.Rate(100)  # 5 Hz corresponds to 0.2 seconds
    while not stop_event.is_set() and not rospy.is_shutdown():
        if latest_joint_positions:
            recorded_positions.append({
                'timestamp': time.time(),
                'JointPositions': latest_joint_positions.copy()  # Safe copy
            })
        rate.sleep()

    input_thread.join()

    rospy.loginfo("Stopped recording Freedrive motions.")

    if not recorded_positions:
        rospy.logwarn("No joint positions were recorded.")
        return

    # Save the recorded positions
    sequences = get_all_freedrive_sequences()
    position_id = len(sequences) + 1
    position_name = input("Enter a name for the recorded sequence: ")

    position_data = {
        'PositionID': position_id,
        'PositionName': position_name,
        'JointPositionsSequence': recorded_positions
    }

    save_freedrive_position(position_id, position_data)
    rospy.loginfo(f"Freedrive motion sequence saved with ID: {position_id}")

def play_freedrive_sequence():
    """
    Plays back a recorded freedrive motion sequence.
    """
    sequences = get_all_freedrive_sequences()

    if not sequences:
        rospy.logwarn("No freedrive sequences available to play.")
        return

    print("\nAvailable Freedrive Sequences:")
    for seq in sequences:
        print(f"ID: {seq['PositionID']}, Name: {seq['PositionName']}")

    try:
        selected_id = int(input("Enter the ID of the sequence to play back: "))
    except ValueError:
        rospy.logwarn("Invalid input. Please enter a numeric ID.")
        return

    sequence = next((seq for seq in sequences if seq['PositionID'] == selected_id), None)

    if not sequence:
        rospy.logwarn("Selected sequence ID not found.")
        return

    joint_positions_sequence = sequence.get('JointPositionsSequence', [])

    if not joint_positions_sequence:
        rospy.logwarn("The selected sequence has no recorded joint positions.")
        return

    group = MoveGroupCommander("manipulator")

    rospy.loginfo(f"Playing back freedrive sequence: {sequence['PositionName']} (ID: {sequence['PositionID']})")

    start_time = None
    previous_timestamp = None

    for entry in joint_positions_sequence:
        timestamp = entry['timestamp']
        joint_positions = entry['JointPositions']

        if start_time is None:
            start_time = timestamp
            previous_timestamp = timestamp
        else:
            # Calculate time difference between current and previous pose
            time_diff = timestamp - previous_timestamp
            if time_diff > 0:
                rospy.sleep(time_diff)
            previous_timestamp = timestamp

        try:
            group.go(joint_positions, wait=True)
            group.stop()
        except Exception as e:
            rospy.logwarn(f"Failed to move to joint positions: {e}")
            continue

    rospy.loginfo("Freedrive sequence playback completed.")

def set_joint_positions():
    """Handles the setting and recording of joint positions."""
    # Retrieve all normal positions
    positions = get_all_positions()
    group = MoveGroupCommander("manipulator")
    
    while not rospy.is_shutdown():
        user_input = input("Enter joint positions as comma-separated values, 'd' to get current joint positions, 'f' to record freedrive motions, or 'q' to quit: ")
        if user_input.lower() == 'q':
            break
        elif user_input.lower() == 'd':
            try:
                # Get current joint positions from the robot
                current_joint_positions = group.get_current_joint_values()
                joint_positions_str = ', '.join([f"{pos:.4f}" for pos in current_joint_positions])
                print(f"Current Joint Positions: {joint_positions_str}")
                
                # Prompt user to save the current joint positions
                save_input = input("Would you like to save the current joint positions? (y/n): ").lower()
                if save_input == 'y':
                    position_name = input("Enter a name for this position (or leave blank): ")
                    position_id = len(positions) + 1
                    position_data = {
                        'PositionID': position_id,
                        'PositionName': position_name,
                        'JointPositions': current_joint_positions
                    }
                    save_position(position_id, position_data)
                    rospy.loginfo(f"Current position saved with ID: {position_id}")
                    positions.append(position_data)  # Update the positions list
                else:
                    rospy.loginfo("Current joint positions were not saved.")
            except Exception as e:
                rospy.logwarn(f"Failed to get current joint positions: {e}")
            continue
        elif user_input.lower() == 'f':
            try:
                record_freedrive()
            except Exception as e:
                rospy.logwarn(f"Failed to record freedrive motions: {e}")
            continue
        try:
            joint_positions = [float(x.strip()) for x in user_input.split(',')]
            if len(joint_positions) != 6:
                rospy.logwarn("Please enter exactly 6 joint positions.")
                continue
            position_name = input("Enter a name for this position (or leave blank): ")
            position_id = len(positions) + 1
            position_data = {
                'PositionID': position_id,
                'PositionName': position_name,
                'JointPositions': joint_positions
            }
            save_position(position_id, position_data)
            rospy.loginfo(f"Position saved with ID: {position_id}")
            positions.append(position_data)  # Update the positions list
        except ValueError:
            rospy.logwarn("Invalid input. Please enter numeric values.")

def delete_saved_position():
    """Deletes a saved normal position."""
    positions = get_all_positions()
    if not positions:
        rospy.logwarn("No positions saved.")
        return

    print("Saved Positions:")
    for pos in positions:
        print(f"ID: {pos['PositionID']}, Name: {pos['PositionName']}")
    
    try:
        position_id = int(input("Enter the ID of the position to delete: "))
        position_file = get_position_file(position_id)
        if os.path.exists(position_file):
            os.remove(position_file)
            rospy.loginfo(f"Position with ID {position_id} deleted.")
        else:
            rospy.logwarn("Position ID not found.")
    except ValueError:
        rospy.logwarn("Invalid input. Please enter a numeric ID.")

def cycle_through_positions():
    """Cycles through saved normal positions."""
    positions = get_all_positions()
    if not positions:
        rospy.logwarn("No positions saved to cycle through.")
        return

    # Get number of cycles
    while True:
        try:
            num_cycles = int(input("\nEnter the number of cycles to perform: "))
            if num_cycles > 0:
                break
            else:
                print("Please enter a positive number.")
        except ValueError:
            print("Invalid input. Please enter a positive integer.")

    print("\nCycle Through Positions Options:")
    print("a: Specify a single velocity scaling factor for all movements")
    print("b: Specify movement time for each movement")
    option = input("Select an option (a or b): ").lower()

    if option == 'a':
        # Option A: Specify a single velocity scaling factor
        while True:
            try:
                scale = float(input("\nEnter a velocity scaling factor (0 < scale <= 1): "))
                if 0 < scale <= 1:
                    break
                else:
                    print("Please enter a value between 0 and 1.")
            except ValueError:
                print("Invalid input. Please enter a numeric value.")

        rospy.loginfo(f"Starting to cycle through positions {num_cycles} times with velocity scale: {scale}")
        for cycle in range(num_cycles):
            rospy.loginfo(f"Starting cycle {cycle + 1}/{num_cycles}")
            for i, pos in enumerate(positions):
                if rospy.is_shutdown():
                    break
                next_pos = positions[(i + 1) % len(positions)]
                rospy.loginfo(f"Moving to Position ID: {pos['PositionID']}, Name: {pos['PositionName']} with velocity scale: {scale}")
                group = MoveGroupCommander("manipulator")
                group.set_max_velocity_scaling_factor(scale)
                group.go(pos['JointPositions'], wait=True)
                group.stop()
                # rospy.sleep(0.2)  

        rospy.loginfo("Completed all cycles with the specified velocity scale.")

    elif option == 'b':
        try:
            num_movements = len(positions)
            print(f"\nEnter desired movement time in seconds for each of the {num_movements} movements in the cycle (1 sec corresponds to π rad/s):")
            movement_times = []
            for i in range(num_movements):
                while True:
                    try:
                        time_input = float(input(
                            f"Movement {i + 1} time (sec) (must be > 0): "
                        ))
                        if time_input > 0:
                            movement_times.append(time_input)
                            break
                        else:
                            print("Please enter a positive value.")
                    except ValueError:
                        print("Invalid input. Please enter a numeric value.")
        except Exception as e:
            rospy.logwarn(f"Error obtaining movement times: {e}")
            return

        robot_max_velocities = [
             math.pi,        # Joint 1 maximum velocity (rad/s)
             math.pi,        # Joint 2 maximum velocity (rad/s)
            math.pi,        # Joint 3 maximum velocity (rad/s)
            2 * math.pi,    # Joint 4 maximum velocity (rad/s)
            2 * math.pi,    # Joint 5 maximum velocity (rad/s)
            2 * math.pi     # Joint 6 maximum velocity (rad/s)
        ]

        rospy.loginfo(f"Starting to cycle through positions {num_cycles} times with specified movement times...")
        total_start_time = time.time()
        
        for cycle in range(num_cycles):
            cycle_start_time = time.time()
            rospy.loginfo(f"Starting cycle {cycle + 1}/{num_cycles}")
            
            for i, pos in enumerate(positions):
                if rospy.is_shutdown():
                    break
                next_pos = positions[(i + 1) % len(positions)]
                movement_time = movement_times[i]
                
                # Calculate required velocity scaling factor based on movement_time
                required_scales = []
                for j in range(6):
                    delta = abs(next_pos['JointPositions'][j] - pos['JointPositions'][j])
                    if movement_time == 0:
                        rospy.logwarn("Movement time cannot be zero.")
                        scale = 1.0
                    else:
                        required_vel = delta / movement_time
                        scale = required_vel / robot_max_velocities[j]
                    required_scales.append(scale)

                # Find the maximum scale needed and adjust if it's greater than 1
                max_scale = max(required_scales)
                if max_scale > 1.0:
                    # If we need to move faster than maximum velocity, we need to extend the movement time
                    adjusted_time = movement_time * max_scale
                    rospy.logwarn(
                        f"Movement {i + 1}: Requested time {movement_time:.2f}s is too short. "
                        f"Adjusting to {adjusted_time:.2f}s to respect velocity limits."
                    )
                    overall_scale = 1.0
                else:
                    overall_scale = max_scale

                movement_start_time = time.time()
                group = MoveGroupCommander("manipulator")
                group.set_max_velocity_scaling_factor(overall_scale)
                group.go(next_pos['JointPositions'], wait=True)
                group.stop()
                movement_actual_time = time.time() - movement_start_time
                
                rospy.loginfo(
                    f"Movement {i + 1}: Target time: {movement_time:.2f}s, "
                    f"Actual time: {movement_actual_time:.2f}s, "
                    f"Difference: {movement_actual_time - movement_time:.2f}s"
                )
                rospy.sleep(1)  # Optional: Add a short pause between movements
            
            cycle_time = time.time() - cycle_start_time
            rospy.loginfo(f"Cycle {cycle + 1} completed in {cycle_time:.2f} seconds")
        
        total_time = time.time() - total_start_time
        rospy.loginfo(f"Completed all {num_cycles} cycles in {total_time:.2f} seconds")

    else:
        rospy.logwarn("Invalid option selected. Please choose 'a' or 'b'.")

def load_to_saved_position():
    """Loads a saved normal position."""
    positions = get_all_positions()
    if not positions:
        rospy.logwarn("No positions saved.")
        return

    print("Saved Positions:")
    for pos in positions:
        print(f"ID: {pos['PositionID']}, Name: {pos['PositionName']}")
    
    try:
        position_id = int(input("Enter the ID of the position to load: "))
        position_data = load_position(position_id)
        if position_data:
            move_joint(position_data['JointPositions'])
        else:
            rospy.logwarn("Position ID not found.")
    except ValueError:
        rospy.logwarn("Invalid input. Please enter a numeric ID.")

def main_menu():
    """Displays the main menu and handles user input."""
    while not rospy.is_shutdown():
        print("\nMain Menu:")
        print("1: Save new position")
        print("2: Load to a saved position")
        print("3: Delete a saved position")
        print("4: Cycle through saved positions")
        print("5: Play Freedrive Sequence")
        print("6: Exit")
        choice = input("Enter your choice (1, 2, 3, 4, 5, or 6): ")
        if choice == '1':
            set_joint_positions()
        elif choice == '2':
            load_to_saved_position()
        elif choice == '3':
            delete_saved_position()
        elif choice == '4':
            cycle_through_positions()
        elif choice == '5':
            play_freedrive_sequence()
        elif choice == '6':
            print("Exiting...")
            break
        else:
            rospy.logwarn("Invalid choice. Please enter 1, 2, 3, 4, 5, or 6.")

def main():
    """Initializes the ROS node and starts the main menu."""
    roscpp_initialize(sys.argv)
    rospy.init_node('move_ur5e_with_moveit', anonymous=True)
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    
    time.sleep(3)
    
    group = MoveGroupCommander("manipulator")
    
    main_menu()
    roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
