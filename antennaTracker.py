#!/usr/bin/env python3

import serial
import time
import struct
import asyncio
import math
import os
import yaml
import argparse
from typing import Optional
from datetime import datetime
from pymavlink import mavutil
from pyproj import Geod
from dataclasses import dataclass


@dataclass
class GroundStation:
    latitude: float
    longitude: float
    altitude: float  # MSL
    heading: float = 0.0  # degrees


@dataclass
class ServoConfig:
    trim_pwm: int
    max_pwm: int
    min_pwm: int
    max_angle: float  # degrees
    min_angle: float  # degrees
    max_rate: float = 180.0  # degrees per second


@dataclass
class TrackerConfig:
    gcsConfig: GroundStation
    pitchServoConfig: ServoConfig
    yawServoConfig: ServoConfig
    serial_port: str


class ServoController:
    START_BYTE_1 = 0xFF
    START_BYTE_2 = 0xFF
    ACK_BYTES = b'\xff@@'
    TIMEOUT = 0.05

    def __init__(self, port: str, baudrate: int = 57600) -> None:
        self.serial = serial.Serial(port, baudrate, timeout=self.TIMEOUT)
        time.sleep(2)
        self.resend_count = 0
        self.timeout_count = 0
        self.last_resend_time: Optional[datetime] = None
        self.last_timeout_time: Optional[datetime] = None

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
        return ack == self.ACK_BYTES

    def send_position(self, pitch_pwm: int, yaw_pwm: int) -> bool:
        pitch_pwm = min(max(500, pitch_pwm), 2500)
        yaw_pwm = min(max(500, yaw_pwm), 2500)

        if self._send_packet(pitch_pwm, yaw_pwm):
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

    def __init__(self, config: TrackerConfig, port: int) -> None:
        self.config = config
        self.port = port
        self.servoController = ServoController(config.serial_port)

        self.aircraft_lat: Optional[float] = None
        self.aircraft_lon: Optional[float] = None
        self.aircraft_alt: Optional[float] = None
        self.last_mavlink_time: Optional[datetime] = None

        self.distance_m: float = 0.0
        self.pan_angle_north: float = 0.0
        self.pan_servo_angle: float = 0.0
        self.tilt_servo_angle: float = 0.0
        self.current_pan_angle: float = 0.0
        self.current_tilt_angle: float = 0.0
        self.pan_pwm: int = config.yawServoConfig.trim_pwm
        self.tilt_pwm: int = config.pitchServoConfig.trim_pwm

        self.geod = Geod(ellps='WGS84')
        self.running = False

    def _calculate_angles(self) -> None:
        if self.aircraft_lat is None:
            return

        gcs = self.config.gcsConfig
        fwd_azimuth, _, distance = self.geod.inv(
            gcs.longitude, gcs.latitude,
            self.aircraft_lon, self.aircraft_lat
        )

        self.distance_m = distance
        self.pan_angle_north = fwd_azimuth % 360

        self.pan_servo_angle = (self.pan_angle_north - gcs.heading) % 360
        if self.pan_servo_angle > 180:
            self.pan_servo_angle -= 360

        altitude_diff = self.aircraft_alt - gcs.altitude
        if distance > 0:
            self.tilt_servo_angle = math.degrees(math.atan2(altitude_diff, distance))
        else:
            self.tilt_servo_angle = 0.0

    def _angle_to_pwm(self, angle: float, servo_config: ServoConfig) -> int:
        angle = max(servo_config.min_angle, min(servo_config.max_angle, angle))
        angle_range = servo_config.max_angle - servo_config.min_angle
        pwm_range = servo_config.max_pwm - servo_config.min_pwm
        if angle_range == 0:
            return servo_config.trim_pwm
        pwm = servo_config.trim_pwm + int((angle / angle_range) * pwm_range)
        return max(servo_config.min_pwm, min(servo_config.max_pwm, pwm))

    def _apply_rate_limit(self, current: float, target: float, max_rate: float, dt: float) -> float:
        if dt <= 0:
            return current
        max_change = max_rate * dt
        delta = target - current
        if abs(delta) > max_change:
            return current + math.copysign(max_change, delta)
        return target

    async def run(self) -> None:
        loop = asyncio.get_event_loop()
        conn = await loop.run_in_executor(
            None, lambda: mavutil.mavlink_connection(f'udpin:0.0.0.0:{self.port}')
        )
        print(f"Listening for MAVLink on UDP port {self.port}...")

        self.running = True
        await asyncio.gather(
            self.update_position(conn),
            self.control_loop(),
            self.print_console()
        )

    async def update_position(self, conn) -> None:
        loop = asyncio.get_event_loop()
        while self.running:
            msg = await loop.run_in_executor(
                None,
                lambda: conn.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=1.0)
            )
            if msg:
                self.aircraft_lat = msg.lat / 1e7
                self.aircraft_lon = msg.lon / 1e7
                self.aircraft_alt = msg.alt / 1000.0  # mm to m MSL
                self.last_mavlink_time = datetime.now()

    async def control_loop(self) -> None:
        while self.running:
            if self.aircraft_lat is not None:
                self._calculate_angles()
                dt = 0.1

                self.current_pan_angle = self._apply_rate_limit(
                    self.current_pan_angle, self.pan_servo_angle,
                    self.config.yawServoConfig.max_rate, dt
                )
                self.current_tilt_angle = self._apply_rate_limit(
                    self.current_tilt_angle, self.tilt_servo_angle,
                    self.config.pitchServoConfig.max_rate, dt
                )

                self.tilt_pwm = self._angle_to_pwm(self.current_tilt_angle, self.config.pitchServoConfig)
                self.pan_pwm = self._angle_to_pwm(self.current_pan_angle, self.config.yawServoConfig)
                self.servoController.send_position(self.tilt_pwm, self.pan_pwm)

            await asyncio.sleep(0.1)

    async def print_console(self) -> None:
        while self.running:
            os.system('clear' if os.name == 'posix' else 'cls')
            print("=" * 70)
            print("ANTENNA TRACKER".center(70))
            print("=" * 70)
            print()

            gcs = self.config.gcsConfig
            print(f"Ground Station:")
            print(f"  Lat:      {gcs.latitude:>12.7f}°")
            print(f"  Lon:      {gcs.longitude:>12.7f}°")
            print(f"  Alt:      {gcs.altitude:>12.2f} m MSL")
            print(f"  Heading:  {gcs.heading:>12.1f}°")
            print(f"  MAVLink:  UDP :{self.port}")
            print()

            if self.aircraft_lat is not None and self.last_mavlink_time:
                age = (datetime.now() - self.last_mavlink_time).total_seconds()
                print(f"Aircraft: (last msg {age:.1f}s ago)")
                print(f"  Lat:      {self.aircraft_lat:>12.7f}°")
                print(f"  Lon:      {self.aircraft_lon:>12.7f}°")
                print(f"  Alt:      {self.aircraft_alt:>12.2f} m MSL")
            else:
                print("Aircraft: NO DATA")
            print()

            print(f"Tracking:")
            print(f"  Distance:     {self.distance_m:>10.2f} m")
            print(f"  Pan (N):      {self.pan_angle_north:>10.1f}°")
            print(f"  Pan target:   {self.pan_servo_angle:>10.1f}°")
            print(f"  Pan current:  {self.current_pan_angle:>10.1f}°")
            print(f"  Tilt target:  {self.tilt_servo_angle:>10.1f}°")
            print(f"  Tilt current: {self.current_tilt_angle:>10.1f}°")
            print()

            print(f"Servo PWM:")
            print(f"  Pan:  {self.pan_pwm:>6d} µs")
            print(f"  Tilt: {self.tilt_pwm:>6d} µs")
            print()

            sc = self.servoController
            print(f"Serial stats:")
            if sc.last_resend_time:
                print(f"  Last resend:  {(datetime.now() - sc.last_resend_time).total_seconds():>8.1f}s ago")
            else:
                print(f"  Last resend:  {'Never':>12s}")
            if sc.last_timeout_time:
                print(f"  Last timeout: {(datetime.now() - sc.last_timeout_time).total_seconds():>8.1f}s ago")
            else:
                print(f"  Last timeout: {'Never':>12s}")
            print(f"  Resends:  {sc.resend_count:>6d}")
            print(f"  Timeouts: {sc.timeout_count:>6d}")
            print()
            print("=" * 70)

            await asyncio.sleep(0.1)

    def stop(self):
        self.running = False
        self.servoController.close()


def load_config(config_path: str) -> TrackerConfig:
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
        min_angle=config_data['pitch_servo']['min_angle'],
        max_rate=config_data['pitch_servo'].get('max_rate', 180.0)
    )

    yaw_config = ServoConfig(
        trim_pwm=config_data['yaw_servo']['trim_pwm'],
        max_pwm=config_data['yaw_servo']['max_pwm'],
        min_pwm=config_data['yaw_servo']['min_pwm'],
        max_angle=config_data['yaw_servo']['max_angle'],
        min_angle=config_data['yaw_servo']['min_angle'],
        max_rate=config_data['yaw_servo'].get('max_rate', 180.0)
    )

    return TrackerConfig(
        gcsConfig=gcs_config,
        pitchServoConfig=pitch_config,
        yawServoConfig=yaw_config,
        serial_port=config_data['serial_port'],
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Antenna Tracker')
    parser.add_argument('--port', type=int, default=13786,
                        help='UDP port to listen for MAVLink (default: 13786)')
    parser.add_argument('--config', default='config.yaml',
                        help='Path to config file (default: config.yaml)')
    args = parser.parse_args()

    tracker = None
    try:
        config = load_config(args.config)
        tracker = AntennaTracker(config, args.port)
        asyncio.run(tracker.run())
    except KeyboardInterrupt:
        print("\nShutting down...")
        if tracker:
            tracker.stop()
    except FileNotFoundError:
        print(f"Error: config file '{args.config}' not found")
        import sys
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        import sys
        sys.exit(1)
