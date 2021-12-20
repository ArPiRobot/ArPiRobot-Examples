from arpirobot.core.action import Action
from arpirobot.core.log import Logger
import main
import time


# Create actions here
# Each action must have 4 functions. Use the template below.
# You can refer to the current instance of your Robot class using main.robot


class JSDriveAction(Action):
    """
    Action to drive the robot using gamepad input
    This action will never terminate on its own. It will only terminate if 
    interrupted either by using ActionManager::stopAction or if another action is started that takes
    control of a device that this action used
    """
    def begin(self):
        # Run when the action is started
        # If an action needs exclusive control of devices, lock them here
        
        # This action needs exclusive control of motors
        self.lock_devices([main.robot.flmotor, main.robot.frmotor, 
                main.robot.flmotor, main.robot.rrmotor])
        
        # Driving is more natural in coast mode
        main.robot.set_brake_mode(False)

    def process(self):
        # Run periodically while the action is running
        
        # Get gamepad values for speed and rotation
        speed = -1 * main.robot.gp0.get_axis(main.robot.DRIVE_AXIS, main.robot.DEADBAND)
        rotation = main.robot.gp0.get_axis(main.robot.ROTATE_AXIS, main.robot.DEADBAND)

        # Use the speed and rotation to move the motors
        main.robot.drive_helper.update(speed, rotation)

    def finish(self, was_interrupted: bool):
        # Run when the action is stopped
        # was_interrupted will be True if the action did not stop on its own (see should_continue)
        
        # If this action is stopped, make sure the motors stop too
        main.robot.drive_helper.update(0, 0)
    
    def should_continue(self) -> bool:
        # The action will stop itself when this returns False
        # This is run after each time process() runs to determine if the action should stop
        
        # This action will not stop itself
        return True


class DriveDistanceAction(Action):
    """
    Action to drive at a certain speed for a certain number of encoder ticks
    Also includes a timeout
    """

    def __init__(self, drive_speed: float, encoder_ticks: int, timeout_ms: int):
        super().__init__()
        self.drive_speed = drive_speed
        self.encoder_ticks = encoder_ticks
        self.timeout_ms = timeout_ms
        self.start_time = 0

    def begin(self):
        # Run when the action is started
        # If an action needs exclusive control of devices, lock them here
        
        # This action needs exclusive control of motors
        self.lock_devices([main.robot.flmotor, main.robot.frmotor, 
                main.robot.flmotor, main.robot.rrmotor])

        # Store the time this action started
        self.start_time = time.time()

        # Make the current encoder position zero
        main.robot.lencoder.set_position(0)
        main.robot.rencoder.set_position(0)

        # Brake mode is better here because the robot will roll less after stopping
        main.robot.set_brake_mode(True)

        # Start driving
        main.robot.drive_helper.update(self.drive_speed, 0)

    def process(self):
        # Run periodically while the action is running
        pass

    def finish(self, was_interrupted: bool):
        # Run when the action is stopped
        # was_interrupted will be True if the action did not stop on its own (see should_continue)
        
        # When action is stopped, stop motors
        main.robot.drive_helper.update(0, 0)
    
    def should_continue(self) -> bool:
        # The action will stop itself when this returns False
        # This is run after each time process() runs to determine if the action should stop

        # If this action has run too long, stop this action
        if((time.time() - self.start_time) * 1000 >= self.timeout_ms):
            return False
        
        # If the robot has driven far enough, stop this action
        avg_enc_pos = \
            (main.robot.lencoder.get_position() + main.robot.rencoder.get_position()) / 2.0
        if(avg_enc_pos > self.encoder_ticks):
            return False
        
        # Otherwise, keep driving
        return True


class WaitTimeAction(Action):
    """
    Action to wait some amount of time
    """

    def __init__(self, time_ms: int):
        super().__init__()
        self.time_ms = time_ms
        self.start_time = 0


    def begin(self):
        # Run when the action is started
        # If an action needs exclusive control of devices, lock them here
        
        self.start_time = time.time()

    def process(self):
        # Run periodically while the action is running
        pass

    def finish(self, was_interrupted: bool):
        # Run when the action is stopped
        # was_interrupted will be True if the action did not stop on its own (see should_continue)
        pass
    
    def should_continue(self) -> bool:
        # The action will stop itself when this returns False
        # This is run after each time process() runs to determine if the action should stop
        return (time.time() - self.start_time) * 1000 < self.time_ms

class RotateDegreesAction(Action):
    """
    Action to rotate a certain number of degrees based on the robot's IMU
    This action will rotate the given number of degrees from the starting angle
    """

    def __init__(self, rotation_speed: float, degrees: float, timeout_ms: int):
        super().__init__()
        self.rotation_speed = rotation_speed
        self.degrees = degrees
        self.timeout_ms = timeout_ms
        self.start_time = 0
        self.start_degrees = 0.0

    def begin(self):
        # Run when the action is started
        # If an action needs exclusive control of devices, lock them here
        
        # This action needs exclusive control of motors
        self.lock_devices([main.robot.flmotor, main.robot.frmotor, 
                main.robot.flmotor, main.robot.rrmotor])

        # Store the time and angle
        self.start_time = time.time()
        self.start_degrees = main.robot.imu.get_gyro_z()

        # Brake mode is better here because the robot will roll less after stopping
        main.robot.set_brake_mode(True)

        # Rotate the correct direction
        if(self.degrees >= 0):
            main.robot.drive_helper.update(0, -self.rotation_speed)
        else:
            main.robot.drive_helper.update(0, self.rotation_speed)

    def process(self):
        # Run periodically while the action is running
        pass

    def finish(self, was_interrupted: bool):
        # Run when the action is stopped
        # was_interrupted will be True if the action did not stop on its own (see should_continue)
        
        # Make sure motors stop when this action stops
        main.robot.drive_helper.update(0, 0)
    
    def should_continue(self) -> bool:
        # The action will stop itself when this returns False
        # This is run after each time process() runs to determine if the action should stop
        
        # If this action has run too long, stop this action
        if((time.time() - self.start_time) * 1000 >= self.timeout_ms):
            return False
        
        # If the robot has rotated far enough, stop this action
        if(self.degrees >= 0 and main.robot.imu.get_gyro_z() - self.start_degrees > self.degrees):
            return False
        elif(self.degrees < 0 and main.robot.imu.get_gyro_z() - self.start_degrees < self.degrees):
            return False
        
        # Otherwise, keep driving
        return True
