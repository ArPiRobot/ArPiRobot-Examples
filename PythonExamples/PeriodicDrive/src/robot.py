from arpirobot.core.robot import BaseRobot
from arpirobot.core.log import Logger
from arpirobot.core.action import ActionManager
from arpirobot.core.network import NetworkTable

from arpirobot.core.drive import ArcadeDriveHelper, CubicAxisTransform, SquareRootAxisTransform

from arpirobot.devices.gamepad import Gamepad
from arpirobot.devices.adafruitmotorhat import AdafruitMotorHatMotor


class Robot(BaseRobot):
    def __init__(self):
        # Do not remove this line
        super().__init__()

        # Create devices and constants as member variables here
        # Only create the devices here. Do not configure them here!

        # Motors
        self.flmotor = AdafruitMotorHatMotor(3)
        self.rlmotor = AdafruitMotorHatMotor(4)
        self.frmotor = AdafruitMotorHatMotor(2)
        self.rrmotor = AdafruitMotorHatMotor(1)

        # Drive helper. Takes a speed and rotation and calculates motor speeds
        # Configure the drive helper to control all four drive motors
        self.drive_helper = ArcadeDriveHelper([self.flmotor, self.rlmotor], [self.frmotor, self.rrmotor])
    
        # Gamepads
        self.gp0 = Gamepad(0)

        # Axis transforms
        self.drive_axis_transform = CubicAxisTransform(0, 0.5)
        self.rotate_axis_transform = SquareRootAxisTransform()

        # Axis numbers
        self.DRIVE_AXIS = 1
        self.ROTATE_AXIS = 2

        # Joystick values beteween -deadband and +deadband are treated as zero
        # A joystick will generally not read exactly zero, so treat anything small 
        # enough as a zero to prevent trying to move the motors very slowly
        self.DEADBAND = 0.1

    def robot_started(self):
        # Run once when the robot starts
        
        # Setup axis transforms
        self.gp0.set_axis_transform(self.DRIVE_AXIS, self.drive_axis_transform)
        self.gp0.set_axis_transform(self.ROTATE_AXIS, self.rotate_axis_transform)

        # Fix motor directions (as needed, depends on wiring)
        self.flmotor.set_inverted(True)
        self.frmotor.set_inverted(True)

    def robot_enabled(self):
        # Runs once each time the robot becomes enabled
        
        # Disable brake mode so driving is more natural
        self.flmotor.set_brake_mode(False)
        self.frmotor.set_brake_mode(False)
        self.rlmotor.set_brake_mode(False)
        self.rrmotor.set_brake_mode(False)

    def robot_disabled(self):
        # Runs once each time the robot becomes disabled
        
        # Put motors in brake mode so they resist movement while robot is disabled
        self.flmotor.set_brake_mode(True)
        self.frmotor.set_brake_mode(True)
        self.rlmotor.set_brake_mode(True)
        self.rrmotor.set_brake_mode(True)

    def enabled_periodic(self):
        # Runs periodically while the robot is enabled
        
        # Get gamepad values for speed and rotation
        # Speed is multiplied by -1 because it is easiest to make positive = forward
        # however, most gamepads will have the up direction on a stick negative
        speed = -1 * self.gp0.get_axis(self.DRIVE_AXIS, self.DEADBAND)
        rotation = self.gp0.get_axis(self.ROTATE_AXIS, self.DEADBAND)

        # Use the speed and rotation to move the motors
        # The drive helper takes care of the math to calculate the left and right speeds
        # Then it sets the motor speeds
        self.drive_helper.update(speed, rotation)

    def disabled_periodic(self):
        # Runs periodically while the robot is disabled
        pass

    def periodic(self):
        # Runs periodically (regardless of robot state)

        # Do not remove this line or some devices will be disabled
        self.feed_watchdog()
