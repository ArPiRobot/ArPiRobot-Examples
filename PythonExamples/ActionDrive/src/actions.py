from arpirobot.core.action import Action, LockedDeviceList
from arpirobot.core.log import Logger
import main


## Action to drive the robot using gamepad input
## This action will never terminate on its own. It will only terminate if 
## interrupted either by using ActionManager::stopAction or if another action is started that takes
## control of a device that this action used
class JSDriveAction(Action):
    def locked_devices(self) -> LockedDeviceList:
        # If an action needs exclusive control of devices, lock them here

        # Only one action can lock a device at a time
        # The newest action keeps control of the device. 
        # Any other action that had previously locked the device will be stopped.
        # This ensures that only one action will ever attempt to control the motors at any given time
        return [ main.robot.flmotor, main.robot.frmotor, 
                main.robot.rlmotor, main.robot.rrmotor ]

    def begin(self):
        # Run when the action is started
        
        pass

    def process(self):
        # Run periodically while the action is running
        
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
        # Run when the action is stopped
        # was_interrupted will be True if the action did not stop on its own (see should_continue)
        
        # If this action is stopped, make sure the motors stop too
        main.robot.drive_helper.update(0, 0)
    
    def should_continue(self) -> bool:
        # The action will stop itself when this returns False
        # This is run after each time process() runs to determine if the action should stop
        
        # This action will not stop itself
        return True