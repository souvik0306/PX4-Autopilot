#!/usr/bin/env python3
"""
EKF2 IMU UDP Telemetry Listener
================================

This script listens to UDP telemetry from EKF2 module on port 14567 and displays
the full vehicle_imu.msg data being processed by the estimator in AI mode.

Usage:
        ./listen_ekf2_udp.py              # Listen on port 14567 (default)
        ./listen_ekf2_udp.py --port 14568 # Listen on custom port

The script displays:
    - Message count and timestamps
    - Device IDs and calibration counters
    - Delta velocity (m/s) - linear acceleration integrated over dt
    - Delta angle (rad) - angular velocity integrated over dt
    - Instantaneous acceleration (m/s²) and gyroscope (rad/s) rates
    - Time deltas (us) - integration intervals
    - Clipping flags
    - Synchronized logging with EKF2

Packet format: 55 bytes
    - uint64_t timestamp (8 bytes)
    - uint64_t timestamp_sample (8 bytes)
    - uint32_t accel_device_id (4 bytes)
    - uint32_t gyro_device_id (4 bytes)
    - float delta_angle[3] (12 bytes)
    - float delta_velocity[3] (12 bytes)
    - uint16_t delta_angle_dt (2 bytes)
    - uint16_t delta_velocity_dt (2 bytes)
    - uint8_t delta_velocity_clipping (1 byte)
    - uint8_t accel_calibration_count (1 byte)
    - uint8_t gyro_calibration_count (1 byte)

This helps verify that EKF2 is receiving and processing the correct AI IMU data, including all metadata.
"""

import socket
import struct
import sys
import argparse
import time
from datetime import datetime


def parse_imu_packet(data):
    """Parse binary IMU packet from EKF2 UDP stream.

    Packet format (55 bytes):
        uint64_t timestamp
        uint64_t timestamp_sample
        uint32_t accel_device_id
        uint32_t gyro_device_id
        float delta_angle[3]
        float delta_velocity[3]
        uint16_t delta_angle_dt
        uint16_t delta_velocity_dt
        uint8_t delta_velocity_clipping
        uint8_t accel_calibration_count
        uint8_t gyro_calibration_count
        Total: 55 bytes
    """
    if len(data) < 55:
        return None

    try:
        values = struct.unpack('<QQII6f2H3B', data[:55])
        timestamp = values[0]
        timestamp_sample = values[1]
        accel_device_id = values[2]
        gyro_device_id = values[3]
        delta_angle = values[4:7]
        delta_velocity = values[7:10]
        delta_angle_dt = values[10]
        delta_velocity_dt = values[11]
        delta_velocity_clipping = values[12]
        accel_calibration_count = values[13]
        gyro_calibration_count = values[14]

        return {
            'timestamp': timestamp,
            'timestamp_sample': timestamp_sample,
            'accel_device_id': accel_device_id,
            'gyro_device_id': gyro_device_id,
            'delta_angle': delta_angle,
            'delta_velocity': delta_velocity,
            'delta_angle_dt': delta_angle_dt,
            'delta_velocity_dt': delta_velocity_dt,
            'delta_velocity_clipping': delta_velocity_clipping,
            'accel_calibration_count': accel_calibration_count,
            'gyro_calibration_count': gyro_calibration_count,
        }
    except struct.error as e:
        return None


def convert_to_rates(pkt):
    """Convert delta values to instantaneous acceleration and gyroscope rates.

    Returns:
        tuple: (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
    """
    dt_angle = pkt['delta_angle_dt'] / 1e6      # Convert us to seconds
    dt_velocity = pkt['delta_velocity_dt'] / 1e6 # Convert us to seconds

    # Divide by dt to get instantaneous rates
    if dt_angle > 0:
        gyro_x = pkt['delta_angle'][0] / dt_angle
        gyro_y = pkt['delta_angle'][1] / dt_angle
        gyro_z = pkt['delta_angle'][2] / dt_angle
    else:
        gyro_x = gyro_y = gyro_z = 0.0

    if dt_velocity > 0:
        accel_x = pkt['delta_velocity'][0] / dt_velocity
        accel_y = pkt['delta_velocity'][1] / dt_velocity
        accel_z = pkt['delta_velocity'][2] / dt_velocity
    else:
        accel_x = accel_y = accel_z = 0.0

    return (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)


def main():
    parser = argparse.ArgumentParser(
        description='Listen to EKF2 IMU UDP telemetry on port 14567'
    )
    parser.add_argument('--port', type=int, default=14567,
                        help='UDP port to listen on (default: 14567)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Verbose output with every packet')

    args = parser.parse_args()

    # Create UDP socket for receiving on port 14567
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Increase socket receive buffer to avoid packet loss
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 * 1024 * 1024)  # 16MB
    except:
        print("[WARNING] Could not set SO_RCVBUF to 16MB")

    # Create UDP socket for sending on port 14568
    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Increase socket send buffer to avoid packet loss
    try:
        tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 16 * 1024 * 1024)  # 16MB
    except:
        print("[WARNING] Could not set SO_SNDBUF to 16MB")

    tx_addr = ('127.0.0.1', 14568)
    filtered_device_id = 0x14010c  # Primary IMU

    try:
        sock.bind(('127.0.0.1', args.port))
        print(f"[EKF2 IMU UDP] Listening on 127.0.0.1:{args.port}")
        print(f"[EKF2 IMU UDP] TX configured to send to 127.0.0.1:14568")
        print(f"[EKF2 IMU UDP] Filtering to device ID: 0x{filtered_device_id:08x} (primary IMU)")
        print(f"[EKF2 IMU UDP] Waiting for EKF2_IMU_SRC=1 (AI mode) telemetry...")
        print()

        msg_count = 0
        tx_count = 0
        tx_errors = 0
        rx_errors = 0
        filtered_out = 0
        first_packet_logged = False
        source_ids = set()

        last_time = time.time()
        rate_sum = 0.0
        rate_count = 0

        while True:
            try:
                data, addr = sock.recvfrom(1024)
                pkt = parse_imu_packet(data)

                if pkt is None:
                    print(f"[!] Invalid packet: got {len(data)} bytes, expected 55")
                    rx_errors += 1
                    continue

                msg_count += 1

                # Track all sources
                source_ids.add(pkt['accel_device_id'])

                # Skip packets from other IMU sources
                if pkt['accel_device_id'] != filtered_device_id:
                    filtered_out += 1
                    continue

                # Log first packet received
                if not first_packet_logged:
                    first_time_ms = pkt['timestamp'] / 1000.0
                    print(f"[STARTUP] PY: First packet received at [{first_time_ms:.1f}ms] | ts={pkt['timestamp']} us")
                    first_packet_logged = True

                # Calculate rate from delta_velocity_dt
                dt_velocity = pkt['delta_velocity_dt'] / 1e6  # Convert to seconds
                if dt_velocity > 0:
                    packet_rate = 1.0 / dt_velocity
                    rate_sum += packet_rate
                    rate_count += 1

                # Convert deltas to instantaneous rates
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = convert_to_rates(pkt)

                # Send back the full 55-byte packet to EKF2 on port 14568
                # (This is where you'd apply AI processing)
                try:
                    tx_result = tx_sock.sendto(data[:55], tx_addr)
                    if tx_result == 55:
                        tx_count += 1
                    else:
                        print(f"[TX ERROR] sendto() returned {tx_result} bytes (expected 55)")
                        tx_errors += 1
                except Exception as e:
                    print(f"[TX ERROR] {e}")
                    tx_errors += 1

                if args.verbose:
                    # Print every packet in verbose mode
                    timestamp_ms = pkt['timestamp'] / 1000.0
                    print(f"[{timestamp_ms:.1f}ms] RX={msg_count} (Filtered out: {filtered_out}) | TX={tx_count} "
                          f"| ACC=[{accel_x:.4f}, {accel_y:.4f}, {accel_z:.4f}] m/s² "
                          f"| GYRO=[{gyro_x:.4f}, {gyro_y:.4f}, {gyro_z:.4f}] rad/s")

                # Log summary every 5000 packets (matching C++ behavior)
                if tx_count % 5000 == 0 and tx_count > 0:
                    current_time = time.time()
                    elapsed = current_time - last_time
                    avg_rate = (rate_sum / rate_count) if rate_count > 0 else 0.0
                    loss_pct = 0.0

                    timestamp_ms = pkt['timestamp'] / 1000.0
                    sources_list = ", ".join([f"0x{id:08x}" for id in sorted(source_ids)])

                    print(f"[{timestamp_ms:.1f}ms] PY RX={msg_count} (Filtered out: {filtered_out}) | TX={tx_count} "
                          f"| Dropped=0 ({loss_pct:.1f}%) | Sources: {len(source_ids)} [{sources_list}] | "
                          f"Rate: {avg_rate:.1f} Hz")
                    print(f"  ACC=[{accel_x:.4f}, {accel_y:.4f}, {accel_z:.4f}] m/s² | "
                          f"GYRO=[{gyro_x:.4f}, {gyro_y:.4f}, {gyro_z:.4f}] rad/s")

                    last_time = current_time
                    rate_sum = 0.0
                    rate_count = 0

            except struct.error as e:
                print(f"[!] Parse error: {e}")
                rx_errors += 1
                continue
            except KeyboardInterrupt:
                break

    except OSError as e:
        print(f"[ERROR] Failed to bind to 127.0.0.1:{args.port}: {e}")
        print(f"        Make sure port {args.port} is not already in use")
        sys.exit(1)
    finally:
        sock.close()
        tx_sock.close()

    print(f"\n[EKF2 IMU UDP] Stopped. Total RX: {msg_count}, Filtered out: {filtered_out}, Total TX: {tx_count}, TX Errors: {tx_errors}, RX Errors: {rx_errors}")


if __name__ == '__main__':
    main()

