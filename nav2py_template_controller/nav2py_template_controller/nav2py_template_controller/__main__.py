import yaml
import nav2py
import nav2py.interfaces
import numpy as np
import rclpy
from rclpy.logging import get_logger
import os
import torch
import math

from .pas_controller import PaSController, set_log_level
from .PaS_CrowdNav.crowd_nav.configs.config import Config

class nav2py_template_controller(nav2py.interfaces.nav2py_costmap_controller):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._register_callback('path', self._path_callback)
        self._register_callback('costmap_pose', self._costmap_pose_callback)
        
        self.logger = get_logger('nav2py_template_controller')
        
        set_log_level(self.logger, 'info')
        
        self.path = None  # Store the latest path
        self.frame_count = 0
        
        # Initialize PaSController
        try:
            self.logger.info("Initializing PaSController...")
            
            config = Config()
            
            # Set model paths: replace with actual model paths

            vae_path = "vae.pth"
            policy_path = "policy.pt"            
            # Check if model files exist
            if not os.path.exists(vae_path) or not os.path.exists(policy_path):
                self.logger.warn(f"Model files not found at {vae_path} or {policy_path}")
                self.logger.warn("Using simple obstacle avoidance instead")
                
                vae_path = None
                policy_path = None
            
            device = "cuda" if torch.cuda.is_available() else "cpu"
            self.logger.info(f"Using device: {device}")
            
            # Create PaSController instance
            self.pas_controller = PaSController(
                vae_path=vae_path,
                policy_path=policy_path,
                device=device,
                config=config
            )
            
            self.logger.info("PaSController initialized successfully")
            
        except Exception as e:
            import traceback
            self.logger.error(f"Error initializing PaSController: {e}")
            self.logger.error(traceback.format_exc())
            
            # Create a backup simple controller if PaSController initialization fails
            self.pas_controller = None
            self.logger.warn("Using simple obstacle avoidance fallback")
        
        self.logger.info("nav2py_template_controller initialized")
        
    def _path_callback(self, path_):
        """
        Process path data from C++ controller
        """
        try:

            if isinstance(path_, list) and len(path_) > 0:
                data_str = path_[0]
                if isinstance(data_str, bytes):
                    data_str = data_str.decode()
                
                self.path = yaml.safe_load(data_str)

        except Exception as e:
            import traceback
            self.logger.error(f"Error processing path data: {e}")
            self.logger.error(traceback.format_exc())

    def _costmap_pose_callback(self, costmap_pose_data):
        """
        Process costmap and pose data from C++ controller
        """
        try:
            self.frame_count += 1
            
            frame_delimiter = "=" * 80
            self.logger.info(f"\n{frame_delimiter}")
            self.logger.info(f"===== PROCESSING FRAME {self.frame_count} =====")
            self.logger.info(f"{frame_delimiter}")
            
            if isinstance(costmap_pose_data, list) and len(costmap_pose_data) > 0:
                data_str = costmap_pose_data[0]
                if isinstance(data_str, bytes):
                    data_str = data_str.decode()
                
                data = yaml.safe_load(data_str)
                self.logger.info(f"Costmap and pose data decoded successfully")
            else:
                if isinstance(costmap_pose_data, bytes):
                    data = yaml.safe_load(costmap_pose_data.decode())
                    self.logger.info(f"Costmap and pose data decoded from bytes")
                else:
                    self.logger.error(f"Unexpected costmap_pose data type: {type(costmap_pose_data)}")
                    return
            
            # Extract costmap info
            costmap_info = data.get('costmap_info', {})
            width = costmap_info.get('width', 0)
            height = costmap_info.get('height', 0)
            resolution = costmap_info.get('resolution', 0.0)
            origin_x = costmap_info.get('origin_x', 0.0)
            origin_y = costmap_info.get('origin_y', 0.0)
            
            self.logger.info(f"Costmap info: {width}x{height}, resolution: {resolution}, origin: ({origin_x}, {origin_y})")
            
            # Extract costmap data
            costmap_data = data.get('costmap_data', [])
            self.logger.info(f"Costmap data length: {len(costmap_data)}")
            
            # Extract robot pose
            robot_pose = data.get('robot_pose', {})
            pose = data 
            position = robot_pose.get('position', {})
            self.logger.info(f"Robot pose: x={position.get('x', 0.0):.2f}, y={position.get('y', 0.0):.2f}")
            
            # Extract velocity if available
            velocity = data.get('robot_velocity', {})
            current_velocity = {
                'linear': velocity.get('linear', {}).get('x', 0.0),
                'angular': velocity.get('angular', {}).get('z', 0.0)
            }
            
            self.logger.info(f"Robot velocity: linear={current_velocity['linear']:.2f}, angular={current_velocity['angular']:.2f}")
            
            goal_pose = None
            if self.path and 'poses' in self.path and len(self.path['poses']) > 0:
                
                # # log all the poses
                # for i, pose in enumerate(self.path['poses']):
                #     self.logger.info(f"Pose {i}: x={pose['pose']['position']['x']:.2f}, y={pose['pose']['position']['y']:.2f}")
                
                last_pose = self.path['poses'][-1]['pose']
                goal_pose = {
                    'x': last_pose['position']['x'],
                    'y': last_pose['position']['y']
                }
                self.logger.info(f"Goal pose (from path end): x={goal_pose['x']:.2f}, y={goal_pose['y']:.2f}")
            else:
                self.logger.warn("No path available, cannot determine goal position")
            
            # Process costmap data with PaSController
            if width > 0 and height > 0 and len(costmap_data) == width * height:
                # Convert to numpy array for processing
                costmap_array = np.array(costmap_data, dtype=np.uint8).reshape(height, width)
                
                # Normalize costmap values to range [0, 1] for the controller
                normalized_costmap = np.zeros((height, width), dtype=np.float32)
                
                # Define thresholds (adjust as needed based on nav2 costmap interpretation)
                free_threshold = 0
                lethal_threshold = 254
                
                # Normalize the costmap
                for i in range(height):
                    for j in range(width):
                        cost = costmap_array[i, j]
                        if cost <= free_threshold:
                            normalized_costmap[i, j] = 0.0  # Free space
                        elif cost >= lethal_threshold:
                            normalized_costmap[i, j] = 1.0  # Occupied space
                        else:
                            # Linear mapping from (free_threshold, lethal_threshold) to (0, 1)
                            normalized_costmap[i, j] = cost / 255.0
                
                # Determine control commands
                if self.pas_controller is not None:
                    try:
                        # Use PaSController to determine velocity commands
                        linear_x, angular_z = self.pas_controller.process_costmap(
                            normalized_costmap, 
                            pose, 
                            resolution, 
                            origin_x, 
                            origin_y,
                            current_velocity,
                            goal_pose
                        )
                        
                        # Check if PaSController returned valid commands
                        if linear_x is None or angular_z is None:
                            # Fallback to simple obstacle avoidance
                            self.logger.info("PaSController returned None, using fallback")
                            linear_x, angular_z = 0.0, 0.0
                        else:
                            self.logger.info(f"PaSController output: linear_x={linear_x:.2f}, angular_z={angular_z:.2f}")
                    except Exception as e:
                        self.logger.error(f"Error in PaSController: {e}")
                        # Fallback to simple obstacle avoidance
                        linear_x, angular_z = 0.0, 0.0
                        self.logger.info(f"Fallback control: linear_x={linear_x:.2f}, angular_z={angular_z:.2f}")
                else:
                    # Use simple obstacle avoidance if PaSController is not available
                    # set the linear_x and angular_z to 0.0
                    linear_x, angular_z = 0.0, 0.0
                    self.logger.info(f"Simple obstacle avoidance: linear_x={linear_x:.2f}, angular_z={angular_z:.2f}")
                
                # Send velocity commands back to the C++ controller
                self._send_cmd_vel(linear_x, angular_z)
                self.logger.info(f"Sent cmd_vel: linear_x={linear_x:.2f}, angular_z={angular_z:.2f}")
            else:
                self.logger.error(f"Invalid costmap dimensions: width={width}, height={height}, data_len={len(costmap_data)}")
                # Send a safe stop command
                self._send_cmd_vel(0.0, 0.0)
                self.logger.info("Sent zero velocity command due to invalid costmap")
                
            # Add closing delimiter
            self.logger.info(f"\n{frame_delimiter}")
            self.logger.info(f"===== FRAME {self.frame_count} COMPLETED =====")
            self.logger.info(f"{frame_delimiter}")
                
        except Exception as e:
            import traceback
            self.logger.error(f"Error processing costmap and pose data: {e}")
            self.logger.error(traceback.format_exc())
            # Send a safe stop command in case of error
            self._send_cmd_vel(0.0, 0.0)
    
        
if __name__ == "__main__":
    nav2py.main(nav2py_template_controller)