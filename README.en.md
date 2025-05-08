Robotic Arm Gripper Controller
-----------------------------

1. Initialization:
   - Creates connection to robotic arm at IP "192.168.1.18" on port 8080
   - Sets up communication parameters (Modbus mode, baud rate 115200)
   - Install the RobotiQ gripper at the end of the Rayman robotic arm and communicate via the 485 port at the end. The external device number is 9.
   - Run test.py

2. Main Functions:
   - activate(): Wakes up the gripper (sends initialization commands)
   - goToPosition(): Moves gripper to target position and waits until it arrives
   - getPosition(): Reads current gripper position (0-255 range)
   - stop(): Stops gripper movement


Key Features:
- Blocking movement (waits until position reached)
- Position monitoring
- Safety stop function
- Direct hardware communication via Modbus
