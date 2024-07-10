#!/usr/bin/env python

import sys
import os
import time
import rospy
import json
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_commander import roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped

POSITIONS_DIR = 'positions'

def get_position_file(position_id):
    return os.path.join(POSITIONS_DIR, f'position_{position_id}.json')

def move_joint(joint_positions):
    group = MoveGroupCommander("manipulator")
    group.go(joint_positions, wait=True)
    group.stop()

def save_position(position_id, position_data):
    if not os.path.exists(POSITIONS_DIR):
        os.makedirs(POSITIONS_DIR)
    position_file = get_position_file(position_id)
    with open(position_file, 'w') as f:
        json.dump(position_data, f)
    rospy.loginfo(f"Position saved to {position_file}")

def load_position(position_id):
    position_file = get_position_file(position_id)
    try:
        with open(position_file, 'r') as f:
            position_data = json.load(f)
        rospy.loginfo(f"Position loaded from {position_file}")
        return position_data
    except FileNotFoundError:
        rospy.logwarn(f"Position file {position_file} not found.")
        return None

def get_all_positions():
    positions = []
    if os.path.exists(POSITIONS_DIR):
        for filename in os.listdir(POSITIONS_DIR):
            if filename.startswith('position_') and filename.endswith('.json'):
                position_id = int(filename.split('_')[1].split('.')[0])
                position_data = load_position(position_id)
                if position_data:
                    positions.append(position_data)
    return positions

def set_joint_positions():
    positions = get_all_positions()
    while not rospy.is_shutdown():
        user_input = input("Enter joint positions as comma-separated values or 'q' to quit: ")
        if user_input.lower() == 'q':
            break
        try:
            joint_positions = [float(x) for x in user_input.split(',')]
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
        except ValueError:
            rospy.logwarn("Invalid input. Please enter numeric values.")

def load_to_saved_position():
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

def delete_saved_position():
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

def main_menu():
    while not rospy.is_shutdown():
        print("\nMain Menu:")
        print("1: Save new position")
        print("2: Load to a saved position")
        print("3: Delete a saved position")
        print("4: Exit")
        choice = input("Enter your choice (1, 2, 3, or 4): ")
        if choice == '1':
            set_joint_positions()
        elif choice == '2':
            load_to_saved_position()
        elif choice == '3':
            delete_saved_position()
        elif choice == '4':
            print("Exiting...")
            break
        else:
            rospy.logwarn("Invalid choice. Please enter 1, 2, 3, or 4.")

def main():
    roscpp_initialize(sys.argv)
    rospy.init_node('move_ur5e_with_moveit', anonymous=True)
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    
 
    time.sleep(10)
    
    group = MoveGroupCommander("manipulator")
    
    main_menu()
    roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

