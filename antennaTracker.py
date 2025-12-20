#!/usr/bin/env python3
"""
Antenna Tracker v0.1.0
Tracks aircraft position using MAVLink and controls servos via Arduino
"""

import serial
import time
import struct

import asyncio

from typing import Optional
from datetime import datetime, timedelta

from mavsdk import System
from mavsdk.telemetry import Position

import math
from pyproj import Geod

from dataclasses import dataclass

import os
import yaml

@dataclass 
class GroundStation:
    latitude: float
    longitude: float
    altitude: float #MSL
    heading: float = 0.0 # degrees

@dataclass
class ServoConfig:
    trim_pwm: int
    max_pwm: int
    min_pwm: int
    max_angle: float # degrees
    min_angle: float # degrees
    max_rate: float = 180.0  # degrees per second (default: no practical limit)

@dataclass
class TrackerConfig:
    gcsConfig: GroundStation
    pitchServoConfig: ServoConfig
    yawServoConfig: ServoConfig
    mavlink_addr: str
    serial_port: str

class ServoController:
    START_BYTE_1 = 0xFF
    START_BYTE_2 = 0xFF
    ACK_BYTES = b'\xff@@' #0xFF4040
    TIMEOUT = 0.05

    def __init__(self, port: str, baudrate: int = 57600) -> None:
        self.serial = serial.Serial(port, baudrate, timeout=self.TIMEOUT)
        time.sleep(2)
        self.resend_count = 0 
        self.timeout_count = 0 
        self.last_resend_time: Optional[datetime] = None
        self.last_timeout_time: Optional[datetime] = None
        self.stats_start_time = datetime.now()

    def _calculate_crc(self, pitch_pwm: int, yaw_pwm: int) -> int:
        return (pitch_pwm ^ yaw_pwm) & 0xFF

    def _send_packet(self, pitch_pwm: int, yaw_pwm: int) -> bool:
        crc = self._calculate_crc(pitch_pwm, yaw_pwm)
        packet = struct.pack('BBBBBBB', 
                             self.START_BYTE_1, self.START_BYTE_2, 
                             (pitch_pwm >> 8) & 0xFF, pitch_pwm & 0xFF,
                             (yaw_pwm >> 8) & 0xFF, yaw_pwm & 0xFF,
                             crc & 0xFF)

        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        self.serial.write(packet)

        self.serial.flush()

        time.sleep(0.05)

        ack = self.serial.read(3)

        if ack == self.ACK_BYTES:
            return True
        return False
    
    def send_position(self, pitch_pwm: int, yaw_pwm: int) -> bool:
        pitch_pwm = min(max(500,pitch_pwm),2500)
        yaw_pwm = min(max(500,yaw_pwm),2500)

        if self._send_packet(pitch_pwm,yaw_pwm):
            return True

        if self._send_packet(pitch_pwm, yaw_pwm):
             self.resend_count += 1 
             self.last_resend_time = datetime.now()
             return True

        self.timeout_count += 1
        self.last_timeout_time = datetime.now()

        return False

    def close(self):
        self.serial.close()

class AntennaTracker:
     
    # Mavlink_addr: udpin://bind_host:bind_port
    def __init__(self, config: TrackerConfig) -> None:
        self.config = config
        self.mavlink_addr = config.mavlink_addr
        self.servoController = ServoController(config.serial_port)

        self.aircraft_position: Optional[Position] = None
        self.last_mavlink_time: Optional[datetime] = None

        self.distance_m: float = 0.0

        self.pan_angle_north: float = 0.0  # relative to north
        self.tilt_servo_angle: float = 0.0  # relative to level

        self.pan_servo_angle: float = 0.0  # relative to tracker heading/trim
        self.tilt_servo_angle: float = 0.0  # relative to trim

        self.current_pan_angle: float = 0.0  # Current smoothed angle
        self.current_tilt_angle: float = 0.0  # Current smoothed angle

        self.pan_pwm: int = config.yawServoConfig.trim_pwm
        self.tilt_pwm: int = config.pitchServoConfig.trim_pwm

        self.geod = Geod(ellps='WGS84')

        self.running = False

    def _calculate_angles(self) -> None:
        """Calculate distance and angles using pyproj"""
        if not self.aircraft_position:
            return

        gcs = self.config.gcsConfig
        ac_lat = self.aircraft_position.latitude_deg
        ac_lon = self.aircraft_position.longitude_deg
        ac_alt = self.aircraft_position.absolute_altitude_m

        # Calculate forward azimuth, back azimuth, and distance
        fwd_azimuth, _, distance = self.geod.inv(
                gcs.longitude, gcs.latitude,
                ac_lon, ac_lat
                )

        self.distance_m = distance

        # Pan angle relative to north (0-360)
        self.pan_angle_north = fwd_azimuth % 360

        # Pan angle relative to tracker heading
        self.pan_servo_angle = (self.pan_angle_north - gcs.heading) % 360
        if self.pan_servo_angle > 180:
            self.pan_servo_angle -= 360

        # Tilt angle (elevation angle relative to level)
        altitude_diff = ac_alt - gcs.altitude
        horizontal_distance = distance
        
        if horizontal_distance > 0:
            self.tilt_servo_angle = math.degrees(math.atan2(altitude_diff, horizontal_distance))
        else:
            self.tilt_servo_angle = 0.0
        
    def _angle_to_pwm(self, angle: float, servo_config: ServoConfig) -> int:
        """Convert angle to PWM value"""
        # Clamp angle to servo limits
        angle = max(servo_config.min_angle, min(servo_config.max_angle, angle))
        
        # Linear interpolation
        angle_range = servo_config.max_angle - servo_config.min_angle
        pwm_range = servo_config.max_pwm - servo_config.min_pwm
        
        if angle_range == 0:
            return servo_config.trim_pwm
        
        pwm = servo_config.trim_pwm + int((angle / angle_range) * pwm_range)
        
        # Clamp PWM to limits
        return max(servo_config.min_pwm, min(servo_config.max_pwm, pwm))

    def _apply_rate_limit(self, current: float, target: float, max_rate: float, dt: float) -> float:
        """Apply rate limiting to angle changes"""
        if dt <= 0:
            return current
        
        max_change = max_rate * dt
        delta = target - current
        
        # Clamp the change to the maximum allowed
        if abs(delta) > max_change:
            return current + math.copysign(max_change, delta)
        else:
            return target

    async def run(self) -> None:
        drone = System()
        
        await drone.connect(system_address = self.mavlink_addr)

        print("Waiting for Drone Connection...")
        
        async for state in drone.core.connection_state():
            if state.is_connected:
                print("Drone Connected!")
                break

        self.running = True

        await asyncio.gather(
                self.update_position(drone),
                self.control_loop(),
                self.print_console()
                )

    async def update_position(self, drone: System) -> None:
        async for position in drone.telemetry.position():
            self.aircraft_position = position
            self.last_mavlink_time = datetime.now()

    async def control_loop(self) -> None:
        while self.running:
            if self.aircraft_position:
                # Calculate angles
                self._calculate_angles()
                
                dt = 0.1              

                self.current_pan_angle = self._apply_rate_limit(
                    self.current_pan_angle,
                    self.pan_servo_angle,
                    self.config.yawServoConfig.max_rate,
                    dt
                )
                
                self.current_tilt_angle = self._apply_rate_limit(
                    self.current_tilt_angle,
                    self.tilt_servo_angle,
                    self.config.pitchServoConfig.max_rate,
                    dt
                )


                # Convert to PWM
                self.tilt_pwm = self._angle_to_pwm(
                        self.current_tilt_angle, 
                        self.config.pitchServoConfig
                        )

                self.pan_pwm = self._angle_to_pwm(
                        self.current_pan_angle,
                        self.config.yawServoConfig
                        )

                # Send to servos
                self.servoController.send_position(self.tilt_pwm, self.pan_pwm)

            await asyncio.sleep(0.1)

    async def print_console(self) -> None:
        while self.running:
            # Clear screen
            os.system('clear' if os.name == 'posix' else 'cls')
            
            # Title
            print("=" * 70)
            print("ANTENNA TRACKER v0.1.0".center(70))
            print("=" * 70)
            print()
            
            # Ground station info
            gcs = self.config.gcsConfig
            print(f"Ground Station Location:")
            print(f"  Latitude:  {gcs.latitude:>12.7f}°")
            print(f"  Longitude: {gcs.longitude:>12.7f}°")
            print(f"  Altitude:  {gcs.altitude:>12.2f} m MSL")
            print(f"  Heading:   {gcs.heading:>12.1f}° (relative to North)")
            print()
            
            # Aircraft info
            if self.aircraft_position and self.last_mavlink_time:
                ac = self.aircraft_position
                time_since = (datetime.now() - self.last_mavlink_time).total_seconds()
                print(f"Current Aircraft Location: (Last msg: {time_since:.1f}s ago)")
                print(f"  Latitude:  {ac.latitude_deg:>12.7f}°")
                print(f"  Longitude: {ac.longitude_deg:>12.7f}°")
                print(f"  Altitude:  {ac.absolute_altitude_m:>12.2f} m MSL")
            else:
                print("Current Aircraft Location: NO DATA")
            print()
            
            # Tracking data
            print(f"Aircraft Relative to Ground Station:")
            print(f"  Distance:       {self.distance_m:>12.2f} m")
            print(f"  Pan (North):    {self.pan_angle_north:>12.1f}° (relative to North)")
            print(f"  Tilt (Level):   {self.tilt_servo_angle:>12.1f}° (relative to Level)")
            print()
            
            # Servo angles
            print(f"Servo Angles:")
            print(f"  Pan Target:     {self.pan_servo_angle:>12.1f}°")
            print(f"  Pan Current:    {self.current_pan_angle:>12.1f}°")
            print(f"  Tilt Target:    {self.tilt_servo_angle:>12.1f}°")
            print(f"  Tilt Current:   {self.current_tilt_angle:>12.1f}°")
            print()

            print(f"Servo PWM Sent (relative to trim):")
            print(f"  Pan PWM:        {self.pan_pwm:>12d} µs")
            print(f"  Tilt PWM:       {self.tilt_pwm:>12d} µs")
            print()
            
            # Servo statistics
            sc = self.servoController
            print(f"Servo Communication Statistics:")
            
            if sc.last_resend_time:
                resend_elapsed = (datetime.now() - sc.last_resend_time).total_seconds()
                print(f"  Last Resend:    {resend_elapsed:>12.1f} s ago")
            else:
                print(f"  Last Resend:    {'Never':>12s}")
            
            if sc.last_timeout_time:
                timeout_elapsed = (datetime.now() - sc.last_timeout_time).total_seconds()
                print(f"  Last Timeout:   {timeout_elapsed:>12.1f} s ago")
            else:
                print(f"  Last Timeout:   {'Never':>12s}")
            
            print(f"  Total Resends:  {sc.resend_count:>12d} times")
            print(f"  Total Timeouts: {sc.timeout_count:>12d} times")
            print()
            
            print("=" * 70)
            
            await asyncio.sleep(0.1)  # 10Hz update


    def stop(self):
        self.running = False 
        self.servoController.close()


def load_config(config_path: str = "config.yaml") -> TrackerConfig:
    """Load configuration from YAML file"""
    with open(config_path, 'r') as f:
        config_data = yaml.safe_load(f)
    
    gcs_config = GroundStation(
        latitude=config_data['ground_station']['latitude'],
        longitude=config_data['ground_station']['longitude'],
        altitude=config_data['ground_station']['altitude'],
        heading=config_data['ground_station'].get('heading', 0.0)
    )
    
    pitch_config = ServoConfig(
        trim_pwm=config_data['pitch_servo']['trim_pwm'],
        max_pwm=config_data['pitch_servo']['max_pwm'],
        min_pwm=config_data['pitch_servo']['min_pwm'],
        max_angle=config_data['pitch_servo']['max_angle'],
        min_angle=config_data['pitch_servo']['min_angle']
    )
    
    yaw_config = ServoConfig(
        trim_pwm=config_data['yaw_servo']['trim_pwm'],
        max_pwm=config_data['yaw_servo']['max_pwm'],
        min_pwm=config_data['yaw_servo']['min_pwm'],
        max_angle=config_data['yaw_servo']['max_angle'],
        min_angle=config_data['yaw_servo']['min_angle']
    )
    
    return TrackerConfig(
        gcsConfig=gcs_config,
        pitchServoConfig=pitch_config,
        yawServoConfig=yaw_config,
        mavlink_addr=config_data['mavlink_addr'],
        serial_port=config_data['serial_port'],
    )

if __name__ == "__main__":
    import sys
    
    config_path = sys.argv[1] if len(sys.argv) > 1 else "config.yaml"
    
    try:
        config = load_config(config_path)
        tracker = AntennaTracker(config)
        asyncio.run(tracker.run())
    except KeyboardInterrupt:
        print("\nShutting down...")
        tracker.stop()
    except FileNotFoundError:
        print(f"Error: Configuration file '{config_path}' not found!")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
