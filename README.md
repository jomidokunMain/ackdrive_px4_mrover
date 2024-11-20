# ackdrive_px4_mrover
   This node is designed to provide a ros2_control hardware interface for an px4 ros2 topics running firmware from `mrover_comms`.
   It is designed to be used with a `diff_drive_controller` from `ros2_control`.
   It is expected to communicate via serial and to have two joints(steering, throttle), each with velocity control and position/velocity feedback.

   It is based on the *CarlikeBot* example from [ros2_control demos](https://github.com/ros-controls/ros2_control_demos/tree/master/example_11).

   For a tutorial on how to develop a hardware interface like this, check out the video below:




## To Do

- [ ] Document changes from earlier versions
- [ ] Document usage and parameters
- [ ] Clean up remaining connections to original demo code
- [ ] Add license etc (in the meantime, do whatever you want with it)
- [ ] Videos tutorial upload 