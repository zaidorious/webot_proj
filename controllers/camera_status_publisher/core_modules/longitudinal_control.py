from controller import Robot


class LongitudinalControl:
    """
    Handles longitudinal control of the Tesla car in Webots by directly accessing the rear-wheel motors.
    """
    def __init__(self, robot, target_speed=5.0):
        """
        Initialize the control module.

        Args:
            robot: Webots Robot instance.
            target_speed: Desired speed in m/s.
        """
        self.robot = robot
        self.target_speed = target_speed

        self.rear_left_wheel = self.robot.getDevice("left_rear_wheel")
        self.rear_right_wheel = self.robot.getDevice("right_rear_wheel")

        if self.rear_left_wheel is None or self.rear_right_wheel is None:
            raise RuntimeError("Rear wheel motors not found. Check device names in the Webots Scene Tree.")

        self.rear_left_wheel.setPosition(float('inf'))
        self.rear_right_wheel.setPosition(float('inf'))

        self.set_speed(0.0)

    def set_speed(self, speed):
        """
        Sets the speed for the rear wheels.

        Args:
            speed: Target speed in m/s.
        """
        self.rear_left_wheel.setVelocity(speed)
        self.rear_right_wheel.setVelocity(speed)

    def apply_longitudinal_control(self, current_time):
        """
        Starts moving the car forward after 5 seconds.

        Args:
            current_time: Current simulation time in seconds.
        """
        if current_time >= 5.0:  
            self.set_speed(self.target_speed)
        else:
            self.set_speed(0.0) 
