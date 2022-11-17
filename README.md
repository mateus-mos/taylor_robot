# Taylor-Robot
Taylor is a differential robot built using ROS2.
![An image of Taylor](/docs/taylor1.jpeg)

# About the project
### ros2_control

*ros2_control* package is used to control the two wheels. The **Arduino Uno** is used as a hardware component that receives a command from controllers and converts it to a PWM signal for the H bridge, through the Arduino Uno the controllers can read/write to these interfaces.

It is the **Raspberry Pi B+** that runs the controllers, which compares the reference value with the measurement output from the encoders and calculates the system's input based on this error (for more details, visit [Control Theory](https://en.wikipedia.org/wiki/Control_theory)).

# Description
# How to setup
