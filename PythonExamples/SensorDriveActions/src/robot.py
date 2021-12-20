from actions import DriveDistanceAction, JSDriveAction, RotateDegreesAction, WaitTimeAction
from arpirobot.core.robot import BaseRobot
from arpirobot.core.log import Logger
from arpirobot.core.action import ActionManager, ActionSeries
from arpirobot.core.network import NetworkTable

from arpirobot.core.drive import ArcadeDriveHelper, CubicAxisTransform, SquareRootAxisTransform

from arpirobot.devices.gamepad import Gamepad, ButtonPressedTrigger
from arpirobot.devices.adafruitmotorhat import AdafruitMotorHatMotor
from arpirobot.devices.gpio import StatusLED

from arpirobot.arduino.iface import ArduinoUartInterface
from arpirobot.arduino.sensor import VoltageMonitor, SingleEncoder, Mpu6050Imu

class Robot(BaseRobot):
    def __init__(self):
        # Do not remove this line
        super().__init__()

        # Create devices and constants as member variables here
        # Only create the devices here. Do not configure them here!
    
        ########################################################################
        # Devices
        ########################################################################
        # Motors
        self.flmotor = AdafruitMotorHatMotor(3)
        self.rlmotor = AdafruitMotorHatMotor(4)
        self.frmotor = AdafruitMotorHatMotor(2)
        self.rrmotor = AdafruitMotorHatMotor(1)

        # Drive helper
        self.drive_helper = ArcadeDriveHelper([self.flmotor, self.rlmotor], [self.frmotor, self.rrmotor])

        # Arduino sensors
        self.arduino = ArduinoUartInterface("/dev/ttyUSB0", 57600)
        self.vmon = VoltageMonitor("A0", 5.00, 30000, 7500)
        self.lencoder = SingleEncoder(2, False)
        self.rencoder = SingleEncoder(3, False)
        self.imu = Mpu6050Imu()

        # Status LED connected to raspberry pi GPIO #12
        self.status_led = StatusLED(12)

        ########################################################################
        # Actions
        ########################################################################

        # Action instances
        self.js_drive_action = JSDriveAction()
        self.drive_two_feet_action = DriveDistanceAction(0.6, 100, 3000)
        self.rotate_pos_90_action = RotateDegreesAction(0.8, 90, 5000)
        self.rotate_neg_270_action = RotateDegreesAction(0.8, -270, 5000)
        self.wait_250_action = WaitTimeAction(250)

        # Action series (sequences of actions to run)
        self.auto_sequence = ActionSeries(
            # This is a list of actions to run sequentially
            [
                self.drive_two_feet_action,         # Drive 2 ft forward
                self.wait_250_action,               # Wait a short amount of time so brake mode works
                self.rotate_pos_90_action,          # Rotate 90 degrees
                self.wait_250_action,               # Wait a short amount of time so brake mode works
                self.drive_two_feet_action,         # Drive 2 ft forward
                self.wait_250_action,               # Wait a short amount of time so brake mode works
                self.rotate_neg_270_action,         # Rotate -270 degrees
                self.wait_250_action,               # Wait a short amount of time so brake mode works
                self.drive_two_feet_action,         # Drive 2 ft forward
                self.wait_250_action                # Wait a short amount of time so brake mode works
            ],

            # This action will run after the action series finishes
            self.js_drive_action                    # When done, drive with joysticks again
        )

        ########################################################################
        # Gamepad and controls
        ########################################################################

        # Axis numbers
        self.DRIVE_AXIS = 1
        self.ROTATE_AXIS = 2
        self.AUTO_BUTTON = 0

        # Joystick values between -deadband and +deadband are treated as zero
        self.DEADBAND = 0.1

        # Gamepads
        self.gp0 = Gamepad(0)

        # Axis transforms
        self.drive_axis_transform = CubicAxisTransform(0, 0.5)
        self.rotate_axis_transform = SquareRootAxisTransform()

        # Gamepad action triggers
        self.auto_trigger = ButtonPressedTrigger(self.gp0, self.AUTO_BUTTON, self.auto_sequence)

    def robot_started(self):
        # Run once when the robot starts
        # Configure devices here
        
        # Setup axis transforms
        self.gp0.set_axis_transform(self.DRIVE_AXIS, self.drive_axis_transform)
        self.gp0.set_axis_transform(self.ROTATE_AXIS, self.rotate_axis_transform)

        # Fix motor directions (as needed, depends on wiring)
        self.flmotor.set_inverted(True)
        self.frmotor.set_inverted(True)

        # Setup sensors and start using arduino to acquire sensor data
        # Each sensor must be added to an arduino interface
        self.arduino.add_device(self.vmon)
        self.arduino.add_device(self.lencoder)
        self.arduino.add_device(self.rencoder)
        self.arduino.add_device(self.imu)

        # Once begin is called, no more devices can be added
        self.arduino.begin()

        # Show motor battery voltage in DS
        self.vmon.make_main_vmon()

    def robot_enabled(self):
        # Runs once each time the robot becomes enabled
        
        # Allow auto sequence to be triggered when the robot is enabled
        ActionManager.add_trigger(self.auto_trigger)

        # Enable joystick drive (by starting the action) when the robot is enabled
        ActionManager.start_action(self.js_drive_action)

    def robot_disabled(self):
        # Runs once each time the robot becomes disabled
        
        # Don't allow auto sequence to be triggered when the robot is disabled
        ActionManager.remove_trigger(self.auto_trigger)

        # Disabling the robot also stops the auto routine
        # Forcefully stopping an ActionSeries will prevent it from running it's finish action
        if(self.auto_sequence.is_running()):
            ActionManager.stop_action(self.auto_sequence)
        
        if(self.js_drive_action.is_running()):
            ActionManager.stop_action(self.js_drive_action)
        
        # Brake mode when the robot becomes disabled
        self.set_brake_mode(True)

    def enabled_periodic(self):
        # Runs periodically while the robot is enabled
        pass

    def disabled_periodic(self):
        # Runs periodically while the robot is disabled
        pass

    def periodic(self):
        # Runs periodically (regardless of robot state)

        # Put sensor data on the network table so it is easy to see any issues
        NetworkTable.set("Gyro Z", str(self.imu.get_gyro_z()))
        NetworkTable.set("L Pos", str(self.lencoder.get_position()))
        NetworkTable.set("R Pos", str(self.rencoder.get_position()))

        # Do not remove this line or some devices will be disabled
        self.feed_watchdog()
    
    def set_brake_mode(self, brake_mode: bool):
        # Custom function to set brake mode on all four motors
        self.flmotor.set_brake_mode(brake_mode)
        self.frmotor.set_brake_mode(brake_mode)
        self.rlmotor.set_brake_mode(brake_mode)
        self.rrmotor.set_brake_mode(brake_mode)
