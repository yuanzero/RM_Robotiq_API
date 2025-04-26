#!/usr/bin/env python3
"""
Robotic Arm Gripper Controller
=============================

A structured interface for controlling a robotic arm gripper via Modbus communication.
"""

import sys
import os
import time
from typing import Tuple, Optional

# Set up module paths
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.extend([
    ROOT_DIR,
    os.path.join(ROOT_DIR, "Python"),
    os.path.join(ROOT_DIR, "Python", "Robotic_Arm")
])

from Robotic_Arm.rm_robot_interface import *


class RQGripper:
    """Robotic Arm Gripper Controller Class"""
    
    def __init__(self, ip_address: str = "192.168.1.18", port: int = 8080):
        """
        Initialize the gripper controller.
        
        Args:
            ip_address: IP address of the robotic arm
            port: Port number for communication
        """
        self._arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self._handle = None
        self._ip_address = ip_address
        self._port = port
        
        self._initialize_connection()
        self._setup_communication_parameters()
        
    def _initialize_connection(self) -> None:
        """Establish connection with the robotic arm."""
        try:
            self._handle = self._arm.rm_create_robot_arm(self._ip_address, self._port)
            print(f"Connected to robotic arm at {self._ip_address}:{self._port}")
        except Exception as e:
            raise ConnectionError(f"Failed to connect to robotic arm: {str(e)}")
    
    def _setup_communication_parameters(self) -> None:
        """Configure Modbus communication parameters."""
        try:
            print(self._arm.rm_close_modbus_mode(1))
            print(self._arm.rm_set_modbus_mode(1, 115200, 5))
            
            # Setup read/write parameters
            self._write_params = rm_peripheral_read_write_params_t(1, 1000, 9, 3)
            self._read_params = rm_peripheral_read_write_params_t(1, 2000, 9, 3)
        except Exception as e:
            raise RuntimeError(f"Failed to setup communication parameters: {str(e)}")

    def activate(self) -> bool:
        """
        Activate the gripper.
        
        Returns:
            bool: True if activation was successful
        """
        try:
            result1 = self._arm.rm_write_registers(self._write_params, [0, 0, 0, 0, 0, 0])
            result2 = self._arm.rm_write_registers(self._write_params, [1, 0, 0, 0, 0, 0])
            return all([result1, result2])
        except Exception as e:
            print(f"Activation failed: {str(e)}")
            return False

    def go_to_position(self, position: int, speed: int = 0, force: int = 0, 
                      wait: bool = True, timeout: Optional[float] = None) -> bool:
        """
        Move gripper to specified position.
        
        Args:
            position: Target position (0-255)
            speed: Movement speed
            force: Applied force
            wait: Whether to wait until position is reached
            timeout: Maximum time to wait (seconds)
            
        Returns:
            bool: True if movement was successful
        """
        if not 0 <= position <= 255:
            raise ValueError("Position must be between 0 and 255")
            
        try:
            result = self._arm.rm_write_registers(
                self._write_params, 
                [9, 0, 0, position, speed, force]
            )
            
            if wait:
                start_time = time.time()
                while not self.get_position() == position:
                    if timeout and (time.time() - start_time) > timeout:
                        raise TimeoutError("Movement timed out")
                    time.sleep(0.1)
                    
            return result
        except Exception as e:
            print(f"Movement failed: {str(e)}")
            return False

    def get_position(self) -> int:
        """
        Get current gripper position.
        
        Returns:
            int: Current position (0-255)
        """
        try:
            data = self._arm.rm_read_multiple_input_registers(self._read_params)
            return data[1][4]
        except Exception as e:
            print(f"Failed to read position: {str(e)}")
            return -1

    def get_action_status(self) -> int:
        """
        Get gripper action status.
        
        Returns:
            int: Status bit (gGTO bit)
        """
        try:
            data = self._arm.rm_read_multiple_input_registers(self._read_params)
            byte_value = data[1][0]
            
            if byte_value < 0 or byte_value > 255:
                raise ValueError("Invalid status byte value")
                
            return (byte_value >> 3) & 1
        except Exception as e:
            print(f"Failed to read status: {str(e)}")
            return -1

    def stop(self) -> bool:
        """
        Stop gripper movement immediately.
        
        Returns:
            bool: True if stop command was successful
        """
        try:
            return self._arm.rm_write_registers(
                self._write_params, 
                [1, 0, 0, 0, 0, 0]
            )
        except Exception as e:
            print(f"Stop command failed: {str(e)}")
            return False

    def close(self) -> None:
        """Clean up and close connection."""
        try:
            if self._handle:
                self._arm.rm_delete_robot_arm()
                print("Connection closed")
        except Exception as e:
            print(f"Error during cleanup: {str(e)}")

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()


def main():
    """Example usage of the RQGripper class."""
    try:
        with RQGripper() as gripper:
            # Example movement sequence
            gripper.go_to_position(128, wait=True, timeout=5.0)
            print(f"Current position: {gripper.get_position()}")
            
            gripper.go_to_position(50, wait=True)
            print(f"Current position: {gripper.get_position()}")
            
            # Demonstrate incremental movement
            for pos in range(50, 200, 25):
                gripper.go_to_position(pos)
                print(f"Moved to position: {pos}")
                time.sleep(0.5)
                
    except Exception as e:
        print(f"Error in main execution: {str(e)}")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())