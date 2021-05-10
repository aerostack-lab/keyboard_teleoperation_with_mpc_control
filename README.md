# Keyboard Teleoperation With MPC Control

![Interface](https://i.ibb.co/9gZ91Ns/keyboard-mpc.png)

# Subscribed topics

- **self_localization/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))      
Current pose of the vehicle

- **self_localization/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))      
Current speed of the vehicle

# Published topics

- **motion_reference/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))  
Pose reference for the controller.

- **actuator_command/flight_action** ([aerostack_msgs/FlightActionCommand](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/7c07e4317e20a1142226d513336a06a2ff585629/msg/FlightActionCommand.msg))      
Flight command specifying a qualitative action of the robot (take off, hover or land). This command can be interpreted by some aerial platforms (e.g. Parrot drones) but it is ignored by others.

---
# Contributors
**Code Maintainer:** Alberto Rodelgo Perales

**Author:** Alberto Rodelgo Perales