from arpirobot.core.action import Action
from arpirobot.core.log import Logger
import main
import time


# Action to drive the robot using gamepad input
# This action will never terminate on its own. It will only terminate if 
# interrupted either by using ActionManager::stopAction or if another action is started that takes
# control of a device that this action used
class JSDriveAction(Action):
    def begin(self):
        # Only one action can lock a device at a time
        # The newest action keeps control of the device. 
        # Any other action that had previously locked the device will be stopped.
        # This ensures that only one action will ever attempt to control the motors at any given time
        self.lock_devices([main.robot.flmotor, main.robot.frmotor, main.robot.rlmotor, main.robot.rrmotor])

        # Coast mode is more natural for human driving
        main.robot.set_brake_mode(False)
    
    def process(self):
        # Get gamepad values for speed and rotation
        # Speed is multiplied by -1 because it is easiest to make positive = forward
        # however, most gamepads will have the up direction on a stick negative
        speed = -1 * main.robot.gp0.get_axis(main.robot.DRIVE_AXIS, main.robot.DEADBAND)
        rotation = main.robot.gp0.get_axis(main.robot.ROTATE_AXIS, main.robot.DEADBAND)

        # Use the speed and rotation to move the motors
        # The drive helper takes care of the math to calculate the left and right speeds
        # Then it sets the motor speeds
        main.robot.drive_helper.update(speed, rotation)
    
    def finish(self, was_interrupted: bool):
        # If this action is stopped, make sure the motors stop too
        main.robot.drive_helper.update(0, 0)
    
    def should_continue(self) -> bool:
        # This action will not stop on its own
        return True


# Action to rotate the robot a specified number of degrees using the IMU
# This action will terminate once rotation is done
class RotateAction(Action):
    def __init__(self, degrees: float):
        super().__init__()
        self.degrees = degrees
        self.correct_counter = 0

    def begin(self):
        # Only one action can lock a device at a time
        # The newest action keeps control of the device. 
        # Any other action that had previously locked the device will be stopped.
        # This ensures that only one action will ever attempt to control the motors at any given time
        self.lock_devices([main.robot.flmotor, main.robot.frmotor, main.robot.rlmotor, main.robot.rrmotor])

        # Reset correct counter
        self.correct_counter = 0

        # Brake mode helps prevent oscillations
        main.robot.set_brake_mode(True)

        # Configure rotate PID
        main.robot.rotate_pid.reset()
        main.robot.rotate_pid.set_setpoint(main.robot.imu.get_gyro_z() + self.degrees)

        # No forward / reverse motion during this action
        main.robot.drive_helper.update_speed(0)

    
    def process(self):
        # Calculate current output power given current angle
        power = main.robot.rotate_pid.get_output(main.robot.imu.get_gyro_z())

        # Update rotation based on pid output power
        main.robot.drive_helper.update_rotation(-power)
    
    def finish(self, was_interrupted: bool):
        # If this action is stopped, make sure the motors stop too
        main.robot.drive_helper.update(0, 0)

        # Wait for 100ms to ensure brake mode has long enough to work
        time.sleep(0.1)
    
    def should_continue(self) -> bool:
        if abs(main.robot.imu.get_gyro_z() - main.robot.rotate_pid.get_setpoint()) <= 3.0:
            # If within 3 degrees of target increment counter
            # 3 degree tolerance works well experimentally
            # Do not use too small of a tolerance or IMU drift may cause values
            self.correct_counter += 1
        else:
            # Reset counter when no longer correct angle
            self.correct_counter = 0
        
        # Stop if the angle has been correct for the past 10 iterations
        # Continue if counter less than 10 (should not stop)
        return self.correct_counter < 10
