from arpirobot.core.action import Action
from arpirobot.core.log import Logger
import main


# Create actions here
# Each action must have 4 functions. Use the template below.
# You can refer to the current instance of your Robot class using main.robot

"""
class MyAction(Action):
    # You can define a custom constructor here (with arguments) if needed
    # If you do so make sure the first line is super().__init__()

    def begin(self):
        # Run when the action is started
        # If an action needs exclusive control of devices, lock them here
        pass

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
        return False
"""