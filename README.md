# nav2py_template

This project provides a Python-based controller interface for interacting with `nav2` in ROS2.

## 📌 Installation

### **1. Install ROS2 (Humble) and Navigation 2**
- [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Nav2 Installation Guide](https://docs.nav2.org/getting_started/index.html)

### **2. Clone this repository into your workspace**
```bash
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws/src
git clone git@github.com:zenghjian/nav2py_template.git
```

### **3. Build the workspace**
```bash
cd ~/nav2_ws
source /opt/ros/humble/setup.bash
colcon build
```

### **4. Source the workspace**
```bash
source ~/nav2_ws/install/setup.bash
```

### **5. Run the Nav2 demo launch**
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False params_file:=$(pwd)/src/nav2py_template/my_nav2_params.yaml
```

---

## 🔧 Development

### **Code Structure**
```
nav2py_template/
│── src/
│   ├── nav2py_template/
│   │   ├── nav2py_template_controller/
│   │   │   ├── __init__.py
│   │   │   ├── __main__.py  # Main entry point, calls the planner
│   │   │   ├── planner.py   # Custom planner implementation
│   │   ├── template_controller.cpp  # C++ controller
│   │   ├── template_controller.hpp  # C++ controller header file
│── README.md
│── setup.py
```

### **Development Guide**

#### 1. **Understanding the Template Structure**
The template is designed to help you integrate Python-based navigation algorithms with Nav2. It consists of two main parts:
- C++ Controller (`template_controller.cpp`): Handles ROS2 communication and data transmission
- Python Planner (`planner.py`): Implements your navigation algorithm

#### 2. **Adding Your Custom Planner**

##### 2.1 **Create Your Planner Class**
In `src/nav2py_template/nav2py_template_controller/planner.py`, create your planner class:

```python
from nav2py.interfaces import Controller

class YourCustomPlanner(Controller):
    def __init__(self):
        super().__init__()
        # Initialize your planner's parameters
        self.param1 = None
        self.param2 = None
        
    def configure(self, param1, param2):
        """Configure your planner with parameters"""
        self.param1 = param1
        self.param2 = param2
        
    def compute_velocity_commands(self, current_pose, current_velocity, goal_pose):
        """
        Implement your navigation algorithm here
        
        Args:
            current_pose: Current robot pose (geometry_msgs/Pose)
            current_velocity: Current robot velocity (geometry_msgs/Twist)
            goal_pose: Goal pose (geometry_msgs/Pose)
            
        Returns:
            velocity_command: Desired velocity command (geometry_msgs/Twist)
        """
        # Your navigation logic here
        velocity_command = Twist()
        return velocity_command
```

##### 2.2 **Update Main Entry Point**
Modify `src/nav2py_template/nav2py_template_controller/__main__.py` to create and configure your planner in the controller class:

```python
class nav2py_template_controller(nav2py.interfaces.nav2py_costmap_controller):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        # Create and configure your planner
        self.planner = YourCustomPlanner()
        self.planner.configure(param1=value1, param2=value2)
        
        # Register callbacks
        self._register_callback('data', self._data_callback)
        self._register_callback('path', self._path_callback)
        
        self.logger = get_logger('nav2py_template_controller')
        self.frame_count = 0
        self.path = None
        
        self.logger.info("nav2py_template_controller initialized")
        
    def _data_callback(self, data):
        # Process data and use your planner
        try:
            # ... parse data ...
            
            # Use your planner to compute velocity commands
            velocity_command = self.planner.compute_velocity_commands(
                current_pose=robot_pose,
                current_velocity=velocity,
                goal_pose=goal_pose
            )
            
            # Send velocity commands
            self._send_cmd_vel(
                velocity_command.linear.x,
                velocity_command.angular.z
            )
            
        except Exception as e:
            self.logger.error(f"Error in data callback: {e}")
            self._send_cmd_vel(0.0, 0.0)

```
##### 2.3 **Add Additional Data Transmission (Optional)**
If your planner needs additional sensor data or parameters:

1. Modify `template_controller.hpp` to add new data structures:
```cpp
struct AdditionalData {
    // Add your new data fields here
    sensor_msgs::msg::LaserScan scan_data;
    // ... other data
};
```

2. Update `template_controller.cpp` to handle the new data:
```cpp
void TemplateController::process_additional_data() {
    // Subscribe to new topics
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&TemplateController::scan_callback, this, std::placeholders::_1));
}

void TemplateController::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Process scan data
    additional_data_.scan_data = *msg;
}
```

3. Update the Python interface in `__main__.py`:
```python
def scan_callback(self, scan_data):
    # Process scan data in your planner
    self.planner.process_scan(scan_data)
```




## 📧 Contact
For any questions, please contact [us](huajian.zeng@tum.de)