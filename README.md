# ArPiRobot-Examples
Example robot projects using the ArPiRobot framework

Most of these examples were designed to run either on the mini or full size clipboard robot builds as documented in the example builds section of the ArPiRobot documentation

All of these examples are complete projects. They can be built (C++) and deployed (C++ or Python) using the same process as any other project.

## Examples

| Name            | Description                             | Links            |
| --------------- | --------------------------------------- | ---------------- |
| PeriodicDrive   | Four wheel drive using arcade style controls. Implemented using the periodic programming model. | [C++](./CPPExamples/PeriodicDrive) <br /> [Python](./PythonExamples/PeriodicDrive) |
| ActionDrive     | Four wheel drive using arcade stype controls. Implemented using the action based programming model. | [C++](./CPPExamples/ActionDrive) <br /> [Python](./PythonExamples/ActionDrive) |
| Sensors         | Using an arduino coprocessor running the ArPiRobot Arduino Firmware to use an IMU, two encoders, an ultrasonic sensor, a voltage monitor, and two IR reflection detectors. The data from these sensors is sent to the drive station using the network table. | [C++](./CPPExamples/Sensors) <br /> [Python](./PythonExamples/Sensors) |
| SensorDriveActions | Uses IMU and encoders (with arduino coprocessor) to run a sequence of actions to perform an autonomous routine when a button is pressed. | [C++](./CPPExamples/SensorDriveActions) <br /> [Python](./PythonExamples/SensorDriveActions) |
| PIDRotate       | Use of a PID controller along with IMU data to rotate the robot to a specified angle. Controlled using gamepad. | [C++](./CPPExamples/PIDRotate/) <br /> [Python](./PythonExamples/PIDRotate/) |
