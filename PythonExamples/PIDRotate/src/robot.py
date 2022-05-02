from arpirobot.core.robot import BaseRobot
from arpirobot.core.log import Logger
from arpirobot.core.action import ActionManager, ActionSeries
from arpirobot.core.network import NetworkTable

# Import devices and other things here
from arpirobot.arduino.sensor import Mpu6050Imu
from arpirobot.arduino.iface import ArduinoUartInterface
from arpirobot.devices.adafruitmotorhat import AdafruitMotorHatMotor
from arpirobot.core.drive import ArcadeDriveHelper, SquareRootAxisTransform, CubicAxisTransform
from arpirobot.devices.gamepad import Gamepad, ButtonPressedTrigger
from arpirobot.core.control import PID

# Import actions here
from actions import JSDriveAction, RotateAction


class Robot(BaseRobot):
    def __init__(self):
        # Do not remove this line
        super().__init__()

        # Create devices and constants as member variables here
        # Only create the devices here. Do not configure them here!

        # Sensors and arduino interface
        self.imu = Mpu6050Imu()
        self.arduino = ArduinoUartInterface("/dev/ttyUSB0", 57600)

        # Motors
        self.flmotor = AdafruitMotorHatMotor(3)
        self.rlmotor = AdafruitMotorHatMotor(4)
        self.frmotor = AdafruitMotorHatMotor(2)
        self.rrmotor = AdafruitMotorHatMotor(1)

        # Drive helper. Takes a speed and rotation and calculates motor speeds.
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

        # Button numbers
        self.ROTATE_BTN = 0

        # Joystick values beteween -deadband and +deadband are treated as zero
        # A joystick will generally not read exactly zero, so treat anything small 
        # enough as a zero to prevent trying to move the motors very slowly
        self.DEADBAND = 0.1

        # Action instances
        self.js_drive_act = JSDriveAction()
        self.rotate_act = RotateAction(90)

        # Action series
        self.rotate_ser = ActionSeries([self.rotate_act], self.js_drive_act)

        # Button triggers to run actions on button presses
        self.rotate_trigger = ButtonPressedTrigger(self.gp0, self.ROTATE_BTN, self.rotate_ser)

        # Netwrok table keys
        self.ROTATE_KP_KEY = "Rotate kP"
        self.ROTATE_KI_KEY = "Rotate kI"
        self.ROTATE_KD_KEY = "Rotate kD"
        self.IMU_Z_ANGLE_KEY = "Current Angle"
        self.ROTATE_TARGET_KEY = "Target Angle"
        self.ROTATE_ACTIVE_KEY = "Rotate Active"

        # PID used when rotating the robot (by action)
        self.rotate_pid = PID(0.1, 0.0001, 0.01)

    def robot_started(self):
        # Run once when the robot starts
        # Configure devices here
        
        # Setup arduino and IMU
        self.arduino.add_device(self.imu)
        self.arduino.begin()
        self.imu.calibrate(10)

        # Setup axis transforms
        self.gp0.set_axis_transform(self.DRIVE_AXIS, self.drive_axis_transform)
        self.gp0.set_axis_transform(self.ROTATE_AXIS, self.rotate_axis_transform)

        # Fix motor directions (as needed, depends on wiring)
        self.flmotor.set_inverted(True)
        self.frmotor.set_inverted(True)

        # Run slightly faster for better PID performance
        self.rotate_act.set_process_period_ms(20)

        # Populate initial PID parameters on network table
        # These can be edited at runtime in the drive station
        NetworkTable.set(self.ROTATE_KP_KEY, str(self.rotate_pid.get_kp()))
        NetworkTable.set(self.ROTATE_KI_KEY, str(self.rotate_pid.get_ki()))
        NetworkTable.set(self.ROTATE_KD_KEY, str(self.rotate_pid.get_kd()))

        # When button pressed rotate 90 degrees
        # The action will exit once rotation is done then
        # The rotate action kills js drive by locking the 
        # same devices. When rotate is done, js drive is 
        # restarted to allow more driving
        ActionManager.add_trigger(self.rotate_trigger)


    def robot_enabled(self):
        # Runs once each time the robot becomes enabled
        
        if not self.js_drive_act.is_running():
            ActionManager.start_action(self.js_drive_act)

    def robot_disabled(self):
        # Runs once each time the robot becomes disabled
        
        ActionManager.stop_action(self.rotate_act)
        ActionManager.stop_action(self.js_drive_act)

    def enabled_periodic(self):
        # Runs periodically while the robot is enabled
        pass

    def disabled_periodic(self):
        # Runs periodically while the robot is disabled
        pass

    def periodic(self):
        # Runs periodically (regardless of robot state)

        # Do not remove this line or some devices will be disabled
        self.feed_watchdog()

        # Update IMu angle periodically
        NetworkTable.set(self.IMU_Z_ANGLE_KEY, str(self.imu.get_gyro_z()))

        # Update target angle periodically (keep up to date)
        NetworkTable.set(self.ROTATE_TARGET_KEY, str(self.rotate_pid.get_setpoint()))

        # Indicate to user when rotate PID is active (via action)
        if self.rotate_act.is_running():
            NetworkTable.set(self.ROTATE_ACTIVE_KEY, "true")
        else:
            NetworkTable.set(self.ROTATE_ACTIVE_KEY, "false")
        
        # Update PID values if changed by drive station
        # If changed to an invalid value (not a number)
        # The value will be overwritten with the old PID setting
        if NetworkTable.changed(self.ROTATE_KP_KEY):
            try:
                self.rotate_pid.set_kp(float(NetworkTable.get(self.ROTATE_KP_KEY)))
            except:
                NetworkTable.set(self.ROTATE_KP_KEY, str(self.rotate_pid.get_kp()))
        if NetworkTable.changed(self.ROTATE_KI_KEY):
            try:
                self.rotate_pid.set_ki(float(NetworkTable.get(self.ROTATE_KI_KEY)))
            except:
                NetworkTable.set(self.ROTATE_KI_KEY, str(self.rotate_pid.get_ki()))
        if NetworkTable.changed(self.ROTATE_KD_KEY):
            try:
                self.rotate_pid.set_kd(float(NetworkTable.get(self.ROTATE_KD_KEY)))
            except:
                NetworkTable.set(self.ROTATE_KD_KEY, str(self.rotate_pid.get_kd()))

    def set_brake_mode(self, brake_mode: bool):
        self.flmotor.set_brake_mode(brake_mode)
        self.frmotor.set_brake_mode(brake_mode)
        self.rlmotor.set_brake_mode(brake_mode)
        self.rrmotor.set_brake_mode(brake_mode)
