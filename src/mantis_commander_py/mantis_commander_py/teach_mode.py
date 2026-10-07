import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import yaml
import math
import os
from mantis_commander_py.transform_quaternion import euler_from_quaternion
from pathlib import Path

class TeachModeNode(Node):
    def __init__(self):
        super().__init__('teach_mode_node')
        # Init tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # IMPORTANT: type here your base link and effector link
        self.base_frame = 'world' 
        self.ee_frame = 'gripper_hand_link'

    def get_current_pose(self):
        try:
            # return actual position of robot
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, 
                self.ee_frame, 
                rclpy.time.Time())
            return trans
        except Exception as e:
            return None
        
def main(args=None):
    rclpy.init(args=args)
    node = TeachModeNode()

    # find main src path
    current_path = os.path.abspath(__file__)
    workspace_dir = None

    # go up until you find src dir
    while current_path != os.path.dirname(current_path):
        current_path = os.path.dirname(current_path)
        if os.path.exists(os.path.join(current_path, 'src')):
            workspace_dir = current_path
            break

    if workspace_dir is None:
        print("ERROR: Automatic finder of src dir path failed. Check if 'src' directory exist in your workspace")
        return

    # Build config path
    waypoints_path = os.path.join(workspace_dir, 'src', 'mantis_commander', 'config', 'waypoints.yaml')

    print(f" Save destination directory: {waypoints_path}")

    # Create new waypoints.yaml file if not exist
    os.makedirs(os.path.dirname(waypoints_path), exist_ok=True)

    rclpy.spin_once(node, timeout_sec=1.0) #spin to get actual pos

    rclpy.spin_once(node, timeout_sec=1.0)

    while rclpy.ok():
        point_name = input("\nProvide name of your new point or type 'q' to exit teach points program:  ")
        if point_name.lower() == 'q':
            print("Teach program closed. Your points succesfully saved in waypoints.yaml file")
            break
            
        # get actual tf position of robot
        rclpy.spin_once(node, timeout_sec=0.1)
        
        trans = node.get_current_pose()
        if trans:
            # 1. Position read
            pos = trans.transform.translation
            position = [round(pos.x, 3), round(pos.y, 3), round(pos.z, 3)]
            
            # 2. quaternion to euler transform
            quat = trans.transform.rotation
            rpy = euler_from_quaternion.euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
            
            # 3. Yaml file structure
            new_waypoint = {
                'name': point_name,
                'position': position,
                'rpy': rpy
            }
            
            # 4. Check if waypoint already exist in file
            data = {'waypoints': []}
            if os.path.exists(waypoints_path):
                with open(waypoints_path, 'r') as f:
                    loaded_data = yaml.safe_load(f)
                    if loaded_data and 'waypoints' in loaded_data:
                        data = loaded_data
                        
            # 5. Saving new waypoint in file
            data['waypoints'].append(new_waypoint)
            with open(waypoints_path, 'w') as f:
                # default_flow_style=None for better read
                yaml.dump(data, f, default_flow_style=None, sort_keys=False)
                
            print(f" SAVED: '{point_name}'")
            print(f"   Position [X, Y, Z] : {position}")
            print(f"   Euler angles [R, P, Y] : {rpy}")
        else:
            print(f" TF ERROR: Cannot find relationship between '{node.base_frame}' and '{node.ee_frame}'.")
            print("Check if robot is launched and all link names are correct in file.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()