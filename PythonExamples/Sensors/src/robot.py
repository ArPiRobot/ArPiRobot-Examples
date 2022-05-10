from arpirobot.core.robot import BaseRobot
from arpirobot.core.log import Logger
from arpirobot.core.action import ActionManager
from arpirobot.core.network import NetworkTable

from arpirobot.arduino.iface import ArduinoUartInterface
from arpirobot.arduino.sensor import VoltageMonitor, SingleEncoder, Mpu6050Imu, Ultrasonic4Pin, IRReflectorModule


class Robot(BaseRobot):
    def __init__(self):
        # Do not remove this line
        super().__init__()

        # Create devices and constants as member variables here
        # Only create the devices here. Do not configure them here!
    
        # Object used to communicate with an arduino connected to the raspberry pi via UART (serial)
        # First argument is the serial port name. Typically this will either be /dev/ttyUSB0 or /dev/tty/ACM0
        # Occasionally, the number may be different, but usually only if multiple arduinos are connected
        # The second argument is the buad rate. This needs to match the value set in the 
        # arduino firmware flashed to the arduino. By default, this is 57600
        self.arduino = ArduinoUartInterface("/dev/ttyUSB0", 57600)

        # Simple voltage divider used to measure battery voltage.
        # This is used for the cheap voltage "sensors" that can be found online
        # First argument is the pin name. This depends on how you connected it to the arduino
        # Second argument is the voltage of the arduino board (either 5V or 3.3V usually)
        # Third argument is the resistance from power to measurement point (R1)
        # Fourth is the resistance from gnd to the measurement point (R2)
        # These resistances depend on how the voltage divider is constructed
        # For the cheap "sensors" purchased online these are R1 = 30000, R2 = 7500
        self.vmon = VoltageMonitor("A0", 5.00, 30000, 7500)

        # SingleEncoder is used with a single channel encoder.
        # The first argument is the pin the encoder is connected to (depends on wiring to arduino)
        # The second argument indicates if an internall pullup resistor should be used.
        # Generally, this should be false if using a prebuilt module
        self.lencoder = SingleEncoder(2, False)
        self.rencoder = SingleEncoder(3, False)

        # This is one type of IMU that can be used with the arduino
        self.imu = Mpu6050Imu()

        # Ultrasonic sensor with 4 pins (2 power, 2 I/O)
        # The two I/O pins are echo and trigger.
        # First argument is trigger pin number, second is echo pin number. Depends on wiring
        self.usonic = Ultrasonic4Pin(7, 8)

        # IR reflector modules
        # Only argument used here is the pin the digital output of the module is connected to
        # Some modules also have analog outputs, which can optionally be specified as the second argument
        self.ldetector = IRReflectorModule(11)
        self.rdetector = IRReflectorModule(12)

    def robot_started(self):
        # Run once when the robot starts

        # Each sensor is instantiated, but not associated with an arduino yet.
        # Each sensor must be added to exactly one arduino interface.
        # A sensor should not be added to multiple arduinos
        # All sensors must be added before starting the arduino interface        
        self.arduino.add_device(self.vmon)
        self.arduino.add_device(self.lencoder)
        self.arduino.add_device(self.rencoder)
        self.arduino.add_device(self.imu)
        self.arduino.add_device(self.usonic)
        self.arduino.add_device(self.ldetector)
        self.arduino.add_device(self.rdetector)

        # Start the arduino that this interface communicates with
        # Once begin has been called the arduino will start providing sensor data
        # However, no more devices can be added.
        # Before begin is called no sensor will receive any data, thus the 
        # values from the sensors are meaningless
        self.arduino.begin()

        # The main vmon will show the voltage in the drive station's battery indicator
        self.vmon.make_main_vmon()

    def robot_enabled(self):
        # Runs once each time the robot becomes enabled
        pass

    def robot_disabled(self):
        # Runs once each time the robot becomes disabled
        pass

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

        # Put some sensor data in the network table
        # This can be viewed in the drive station
        NetworkTable.set("L Pos", str(self.lencoder.get_position()))
        NetworkTable.set("R Pos", str(self.rencoder.get_position()))
        NetworkTable.set("L Vel", str(self.lencoder.get_velocity()))
        NetworkTable.set("R Vel", str(self.rencoder.get_velocity()))
        NetworkTable.set("Gyro Z", str(self.imu.get_gyro_z()))
        NetworkTable.set("Accel Y", str(self.imu.get_accel_y()))
        NetworkTable.set("US Dist", str(self.usonic.get_distance()))
        NetworkTable.set("L Detect", str(self.ldetector.get_digital_value()))
        NetworkTable.set("R Detect", str(self.rdetector.get_digital_value()))
