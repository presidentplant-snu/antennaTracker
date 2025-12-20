#!/usr/bin/env python3
"""
Servo Debug Utility
Tests serial communication with Arduino servo controller
"""
import serial
import struct
import time
import sys

class ServoDebugger:
    START_BYTE1 = 0xFF
    START_BYTE2 = 0xFF
    ACK_BYTE = b'\xff@@'
    
    def __init__(self, port: str, baudrate: int = 57600):
        print("="*70)
        print("SERVO CONTROLLER DEBUG UTILITY".center(70))
        print("="*70)
        print()
        
        print(f"Attempting to connect to: {port} @ {baudrate} baud")
        try:
            self.serial = serial.Serial(port, baudrate, timeout=0.5)
            print("✓ Serial port opened successfully")
        except Exception as e:
            print(f"✗ Failed to open serial port: {e}")
            sys.exit(1)
        
        print("Waiting 2 seconds for Arduino to initialize...")
        time.sleep(2)
        print("✓ Ready to test\n")
    
    def _calculate_crc(self, pitch_pwm: int, yaw_pwm: int) -> int:
        return (pitch_pwm ^ yaw_pwm) & 0xFF
    
    def send_test_packet(self, pitch_pwm: int, yaw_pwm: int, test_name: str = "Test") -> bool:
        """Send a test packet and analyze the response"""
        print(f"\n{'─'*70}")
        print(f"{test_name}")
        print(f"{'─'*70}")
        
        # Constrain values
        pitch_pwm = min(max(500, pitch_pwm), 2500)
        yaw_pwm = min(max(500, yaw_pwm), 2500)
        
        # Calculate CRC
        crc = self._calculate_crc(pitch_pwm, yaw_pwm)
        
        # Build packet
        packet = struct.pack('BBBBBBB', 
                           self.START_BYTE1, self.START_BYTE2,
                           (pitch_pwm >> 8) & 0xFF, pitch_pwm & 0xFF,
                           (yaw_pwm >> 8) & 0xFF, yaw_pwm & 0xFF,
                           crc & 0xFF)
        
        print(f"Target PWM values:")
        print(f"  Pitch: {pitch_pwm} µs")
        print(f"  Yaw:   {yaw_pwm} µs")
        print(f"  CRC:   0x{crc:02X}")
        print()
        
        print(f"Packet bytes (7 bytes):")
        packet_hex = ' '.join(f'{b:02X}' for b in packet)
        print(f"  HEX: {packet_hex}")
        print(f"  DEC: {' '.join(f'{b:3d}' for b in packet)}")
        print()
        
        # Clear any old data
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        
        # Send packet
        print(f"Sending packet...")
        bytes_written = self.serial.write(packet)
        self.serial.flush()
        print(f"✓ Sent {bytes_written} bytes")
        
        # Wait a bit for Arduino to process
        time.sleep(0.05)
        
        # Check how many bytes are waiting
        bytes_waiting = self.serial.in_waiting
        print(f"Bytes waiting in buffer: {bytes_waiting}")
        
        # Read response
        print(f"\nWaiting for ACK (expecting 3 bytes: FF 40 40)...")
        ack = self.serial.read(3)
        
        if len(ack) == 0:
            print("✗ No response received (timeout)")
            return False
        
        print(f"Received {len(ack)} bytes:")
        ack_hex = ' '.join(f'{b:02X}' for b in ack)
        ack_dec = ' '.join(f'{b:3d}' for b in ack)
        print(f"  HEX: {ack_hex}")
        print(f"  DEC: {ack_dec}")
        
        # Check if it matches expected ACK
        expected = self.ACK_BYTE
        if ack == expected:
            print("✓ ACK VALID - Communication successful!")
            return True
        else:
            print(f"✗ ACK INVALID - Expected: {' '.join(f'{b:02X}' for b in expected)}")
            return False
    
    def listen_mode(self, duration: int = 5):
        """Listen to serial port for any data"""
        print(f"\n{'─'*70}")
        print(f"LISTEN MODE - Monitoring serial port for {duration} seconds")
        print(f"{'─'*70}")
        print("Any data received will be displayed below:")
        print()
        
        self.serial.reset_input_buffer()
        start_time = time.time()
        data_received = False
        
        while (time.time() - start_time) < duration:
            if self.serial.in_waiting > 0:
                data = self.serial.read(self.serial.in_waiting)
                data_received = True
                print(f"[{time.time() - start_time:.2f}s] Received {len(data)} bytes: {' '.join(f'{b:02X}' for b in data)}")
            time.sleep(0.1)
        
        if not data_received:
            print("(No data received)")
    
    def run_tests(self):
        """Run a series of diagnostic tests"""
        tests_passed = 0
        tests_total = 0
        
        # Test 1: Center position
        tests_total += 1
        if self.send_test_packet(1500, 1500, "TEST 1: Center Position (1500, 1500)"):
            tests_passed += 1
        time.sleep(0.5)
        
        # Test 2: Different values
        tests_total += 1
        if self.send_test_packet(1000, 2000, "TEST 2: Different Values (1000, 2000)"):
            tests_passed += 1
        time.sleep(0.5)
        
        # Test 3: Minimum values
        tests_total += 1
        if self.send_test_packet(500, 500, "TEST 3: Minimum Values (500, 500)"):
            tests_passed += 1
        time.sleep(0.5)
        
        # Test 4: Maximum values
        tests_total += 1
        if self.send_test_packet(2500, 2500, "TEST 4: Maximum Values (2500, 2500)"):
            tests_passed += 1
        time.sleep(0.5)
        
        # Return to center
        print(f"\nReturning servos to center position...")
        self.send_test_packet(1500, 1500, "RESET: Center Position")
        
        # Summary
        print(f"\n{'='*70}")
        print(f"TEST SUMMARY".center(70))
        print(f"{'='*70}")
        print(f"Tests Passed: {tests_passed}/{tests_total}")
        
        if tests_passed == tests_total:
            print("✓ ALL TESTS PASSED - Communication is working correctly!")
        elif tests_passed > 0:
            print("⚠ PARTIAL SUCCESS - Some packets are getting through")
        else:
            print("✗ ALL TESTS FAILED - Communication is not working")
        
        print()
        print("Possible issues if tests failed:")
        print("  1. Arduino not running the servo controller sketch")
        print("  2. Wrong serial port selected")
        print("  3. Wrong baud rate (should be 57600)")
        print("  4. Loose USB connection")
        print("  5. Arduino reset during test (watch for 3 LED blinks)")
        print("  6. Serial monitor open in Arduino IDE (close it)")
    
    def close(self):
        self.serial.close()
        print("\nSerial port closed.")


def main():
    if len(sys.argv) < 2:
        print("Usage: python debugServo.py <serial_port> [listen]")
        print()
        print("Examples:")
        print("  python debugServo.py /dev/ttyUSB0")
        print("  python debugServo.py COM3")
        print("  python debugServo.py /dev/ttyUSB0 listen  # Listen mode only")
        print()
        sys.exit(1)
    
    serial_port = sys.argv[1]
    listen_only = len(sys.argv) > 2 and sys.argv[2].lower() == 'listen'
    
    try:
        debugger = ServoDebugger(serial_port)
        
        if listen_only:
            debugger.listen_mode(duration=10)
        else:
            debugger.run_tests()
            
            # Optional: Listen for any unexpected data
            print("\nChecking for any unexpected serial data...")
            debugger.listen_mode(duration=2)
        
        debugger.close()
        
    except serial.SerialException as e:
        print(f"\n✗ Serial Error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
