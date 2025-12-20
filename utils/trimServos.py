#!/usr/bin/env python3
"""
Servo Trim Utility
Interactive tool to calibrate servo trim, min, and max PWM values
"""
import serial
import struct
import time
import yaml
import sys
from typing import Dict, Any

class ServoTrimmer:
    START_BYTE1 = 0xFF
    START_BYTE2 = 0xFF
    ACK_BYTE = b'\xff@@'
    TIMEOUT = 0.1
    
    def __init__(self, port: str, baudrate: int = 57600):
        self.serial = serial.Serial(port, baudrate, timeout=self.TIMEOUT)
        time.sleep(2)
        print(f"Connected to {port} at {baudrate} baud")
        print()
    
    def _calculate_crc(self, pitch_pwm: int, yaw_pwm: int) -> int:
        return (pitch_pwm ^ yaw_pwm) & 0xFF
    
    def send_position(self, pitch_pwm: int, yaw_pwm: int) -> bool:
        """Send position to servos and wait for ACK"""
        pitch_pwm = min(max(500, pitch_pwm), 2500)
        yaw_pwm = min(max(500, yaw_pwm), 2500)
        
        crc = self._calculate_crc(pitch_pwm, yaw_pwm)
        packet = struct.pack('BBBBBBB', 
                           self.START_BYTE1, self.START_BYTE2,
                           (pitch_pwm >> 8) & 0xFF, pitch_pwm & 0xFF,
                           (yaw_pwm >> 8) & 0xFF, yaw_pwm & 0xFF,
                           crc & 0xFF)
        
        # Try sending twice
        for attempt in range(2):
            self.serial.write(packet)
            ack = self.serial.read(3)
            if len(ack) == 3 and ack == self.ACK_BYTE:
                return True
        
        return False
    
    def get_pwm_input(self, prompt: str, default: int = 1500) -> int:
        """Get PWM value from user with validation"""
        while True:
            try:
                value = input(f"{prompt} (default: {default}): ").strip()
                if not value:
                    return default
                pwm = int(value)
                if 500 <= pwm <= 2500:
                    return pwm
                else:
                    print("  Error: PWM must be between 500 and 2500")
            except ValueError:
                print("  Error: Please enter a valid number")
    
    def get_angle_input(self, prompt: str, default: float = 0.0) -> float:
        """Get angle value from user"""
        while True:
            try:
                value = input(f"{prompt} (default: {default}): ").strip()
                if not value:
                    return default
                return float(value)
            except ValueError:
                print("  Error: Please enter a valid number")
    
    def confirm(self, prompt: str = "Accept this value? (y/n)") -> bool:
        """Get yes/no confirmation from user"""
        while True:
            response = input(f"{prompt}: ").strip().lower()
            if response in ['y', 'yes']:
                return True
            elif response in ['n', 'no']:
                return False
            else:
                print("  Please enter 'y' or 'n'")
    
    def calibrate_servo(self, servo_name: str, axis: str, other_pwm: int) -> Dict[str, Any]:
        """Calibrate a single servo (trim, min, max)"""
        print(f"\n{'='*60}")
        print(f"Calibrating {servo_name} Servo ({axis} axis)")
        print(f"{'='*60}")
        
        result = {}
        
        # Trim PWM
        print(f"\n--- TRIM Position ---")
        print(f"Set the {servo_name} servo to its neutral/center position")
        while True:
            trim_pwm = self.get_pwm_input(f"Enter TRIM PWM for {servo_name}")
            
            if axis == 'pitch':
                success = self.send_position(trim_pwm, other_pwm)
            else:
                success = self.send_position(other_pwm, trim_pwm)
            
            if not success:
                print("  Warning: Failed to send command to servos")
            
            print(f"Current position: {trim_pwm} µs")
            if self.confirm():
                result['trim_pwm'] = trim_pwm
                break
        
        # Min PWM and Angle
        print(f"\n--- MINIMUM Position ---")
        print(f"Set the {servo_name} servo to its minimum position")
        while True:
            min_pwm = self.get_pwm_input(f"Enter MIN PWM for {servo_name}", default=1000)
            
            if axis == 'pitch':
                success = self.send_position(min_pwm, other_pwm)
            else:
                success = self.send_position(other_pwm, min_pwm)
            
            if not success:
                print("  Warning: Failed to send command to servos")
            
            print(f"Current position: {min_pwm} µs")
            if self.confirm("Accept this PWM value? (y/n)"):
                break
        
        while True:
            min_angle = self.get_angle_input(f"Enter angle at MIN position (degrees)", default=-90.0)
            print(f"MIN angle set to: {min_angle}°")
            if self.confirm():
                result['min_pwm'] = min_pwm
                result['min_angle'] = min_angle
                break
        
        # Max PWM and Angle
        print(f"\n--- MAXIMUM Position ---")
        print(f"Set the {servo_name} servo to its maximum position")
        while True:
            max_pwm = self.get_pwm_input(f"Enter MAX PWM for {servo_name}", default=2000)
            
            if axis == 'pitch':
                success = self.send_position(max_pwm, other_pwm)
            else:
                success = self.send_position(other_pwm, max_pwm)
            
            if not success:
                print("  Warning: Failed to send command to servos")
            
            print(f"Current position: {max_pwm} µs")
            if self.confirm("Accept this PWM value? (y/n)"):
                break
        
        while True:
            max_angle = self.get_angle_input(f"Enter angle at MAX position (degrees)", default=90.0)
            print(f"MAX angle set to: {max_angle}°")
            if self.confirm():
                result['max_pwm'] = max_pwm
                result['max_angle'] = max_angle
                break
        
        # Return to trim
        if axis == 'pitch':
            self.send_position(trim_pwm, other_pwm)
        else:
            self.send_position(other_pwm, trim_pwm)
        
        return result
    
    def run_calibration(self) -> Dict[str, Any]:
        """Run full calibration for both servos"""
        print("\n" + "="*60)
        print("SERVO CALIBRATION UTILITY".center(60))
        print("="*60)
        print("\nThis utility will help you calibrate your pitch and yaw servos.")
        print("You will set the trim, minimum, and maximum positions for each servo.")
        print("\nPress Ctrl+C at any time to exit.")
        print()
        
        # Start with both servos at default center
        self.send_position(1500, 1500)
        time.sleep(0.5)
        
        # Calibrate pitch servo first (keep yaw at trim)
        pitch_config = self.calibrate_servo("PITCH", "pitch", 1500)
        
        # Calibrate yaw servo (keep pitch at its trim)
        yaw_config = self.calibrate_servo("YAW", "yaw", pitch_config['trim_pwm'])
        
        # Final summary
        print("\n" + "="*60)
        print("CALIBRATION COMPLETE".center(60))
        print("="*60)
        print("\nPitch Servo Configuration:")
        print(f"  Trim PWM:  {pitch_config['trim_pwm']} µs")
        print(f"  Min PWM:   {pitch_config['min_pwm']} µs  (Angle: {pitch_config['min_angle']}°)")
        print(f"  Max PWM:   {pitch_config['max_pwm']} µs  (Angle: {pitch_config['max_angle']}°)")
        
        print("\nYaw Servo Configuration:")
        print(f"  Trim PWM:  {yaw_config['trim_pwm']} µs")
        print(f"  Min PWM:   {yaw_config['min_pwm']} µs  (Angle: {yaw_config['min_angle']}°)")
        print(f"  Max PWM:   {yaw_config['max_pwm']} µs  (Angle: {yaw_config['max_angle']}°)")
        
        return {
            'pitch_servo': pitch_config,
            'yaw_servo': yaw_config
        }
    
    def close(self):
        self.serial.close()


def save_to_yaml(config: Dict[str, Any], output_file: str = "servo_config.yaml"):
    """Save calibration results to YAML file"""
    with open(output_file, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    print(f"\nConfiguration saved to: {output_file}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python trimServos.py <serial_port> [output_file.yaml]")
        print("Example: python trimServos.py /dev/ttyUSB0")
        print("Example: python trimServos.py COM3 my_config.yaml")
        sys.exit(1)
    
    serial_port = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else "../servo_config.yaml"
    
    try:
        trimmer = ServoTrimmer(serial_port)
        
        config = trimmer.run_calibration()
        
        print("\n")
        if input("Save configuration to file? (y/n): ").strip().lower() in ['y', 'yes']:
            save_to_yaml(config, output_file)
            print("\nYou can use this configuration in your config.yaml file.")
        
        trimmer.close()
        print("\nCalibration complete!")
        
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {serial_port}")
        print(f"Details: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nCalibration cancelled by user")
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
