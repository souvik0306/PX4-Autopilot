#!/usr/bin/env python3
"""
EKF2 IMU UDP Telemetry Listener and Bridge
==========================================

This utility forms the AI bridge between EKF2 and an external processor. It
receives raw IMU data from EKF2 on port 14567, applies AI processing, and sends
processed data back to EKF2 on port 14568. The bridge maintains bounded
buffers on both ingress and egress paths to minimise packet loss and provide a
stable flow of data.

Features
--------
* High-capacity RX/TX queues with back-pressure logging
* Non-blocking sockets with large kernel buffers (16 MB)
* Deterministic processing loop with selector based polling
* Timestamp monotonicity checks and packet integrity validation
* Periodic statistics output including queue depths and drop counters
* Optional verbose packet dump for diagnostics

Usage::

    ./listen_ekf2_udp.py [--port 14567] [--send-port 14568] [--bias 10.0]

"""

import argparse
import collections
import selectors
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import List

IMU_PACKET_STRUCT = struct.Struct('<QQII6f2H3B')
PACKET_SIZE = IMU_PACKET_STRUCT.size
DEFAULT_RX_PORT = 14567
DEFAULT_TX_PORT = 14568
MAX_QUEUE_DEPTH = 512
LOG_INTERVAL_S = 2.0
WARNING_INTERVAL_S = 1.0


@dataclass
class ImuPacket:
    """Structured representation of a 55-byte EKF2 IMU packet."""

    timestamp: int
    timestamp_sample: int
    accel_device_id: int
    gyro_device_id: int
    delta_angle: tuple
    delta_velocity: tuple
    delta_angle_dt: int
    delta_velocity_dt: int
    delta_velocity_clipping: int
    accel_calibration_count: int
    gyro_calibration_count: int

    @classmethod
    def from_bytes(cls, data: bytes) -> "ImuPacket":
        if len(data) != PACKET_SIZE:
            raise ValueError(f"Invalid packet size: {len(data)} bytes")

        values = IMU_PACKET_STRUCT.unpack(data)
        delta_angle = values[4:7]
        delta_velocity = values[7:10]
        return cls(
            timestamp=values[0],
            timestamp_sample=values[1],
            accel_device_id=values[2],
            gyro_device_id=values[3],
            delta_angle=delta_angle,
            delta_velocity=delta_velocity,
            delta_angle_dt=values[10],
            delta_velocity_dt=values[11],
            delta_velocity_clipping=values[12],
            accel_calibration_count=values[13],
            gyro_calibration_count=values[14],
        )

    def to_bytes(self) -> bytes:
        return IMU_PACKET_STRUCT.pack(
            self.timestamp,
            self.timestamp_sample,
            self.accel_device_id,
            self.gyro_device_id,
            *self.delta_angle,
            *self.delta_velocity,
            self.delta_angle_dt,
            self.delta_velocity_dt,
            self.delta_velocity_clipping,
            self.accel_calibration_count,
            self.gyro_calibration_count,
        )


class ImuAiBridge:
    """Handles UDP reception, processing, and transmission of EKF2 IMU packets."""

    def __init__(self, listen_port: int, send_port: int, bias: float, verbose: bool) -> None:
        self._listen_port = listen_port
        self._send_port = send_port
        self._bias = bias
        self._verbose = verbose

        self._selector = selectors.DefaultSelector()
        self._rx_socket = self._create_socket()
        self._tx_socket = self._create_socket()
        self._configure_rx_socket()
        self._configure_tx_socket()

        self._rx_queue: collections.deque[ImuPacket] = collections.deque(maxlen=MAX_QUEUE_DEPTH)
        self._tx_queue: collections.deque[bytes] = collections.deque(maxlen=MAX_QUEUE_DEPTH)

        self._rx_count = 0
        self._tx_count = 0
        self._dropped_rx = 0
        self._dropped_tx = 0
        self._last_rx_timestamp = 0

        self._last_log = time.monotonic()
        self._last_warning = 0.0
        self._running = False

    @staticmethod
    def _create_socket() -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        buffer_size = 16 * 1024 * 1024
        for opt in (socket.SO_RCVBUF, socket.SO_SNDBUF):
            try:
                sock.setsockopt(socket.SOL_SOCKET, opt, buffer_size)
            except OSError:
                # Large buffers may not be honoured, continue with defaults
                pass
        return sock

    def _configure_rx_socket(self) -> None:
        self._rx_socket.bind(('127.0.0.1', self._listen_port))
        self._selector.register(self._rx_socket, selectors.EVENT_READ)
        print(f"[AI BRIDGE] Listening for EKF2 raw IMU on 127.0.0.1:{self._listen_port}")

    def _configure_tx_socket(self) -> None:
        self._tx_addr = ('127.0.0.1', self._send_port)
        print(f"[AI BRIDGE] Sending processed IMU to 127.0.0.1:{self._send_port}")

    def close(self) -> None:
        try:
            self._selector.unregister(self._rx_socket)
        except Exception:
            pass
        self._rx_socket.close()
        self._tx_socket.close()
        self._running = False

    def _log_warning(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning >= WARNING_INTERVAL_S:
            print(f"[WARN] {message}")
            self._last_warning = now

    def _receive_packets(self) -> None:
        while True:
            try:
                data, _ = self._rx_socket.recvfrom(PACKET_SIZE)
            except BlockingIOError:
                break
            except OSError as exc:
                self._log_warning(f"RX socket error: {exc}")
                break

            if len(data) != PACKET_SIZE:
                self._dropped_rx += 1
                self._log_warning(f"Discarded packet with unexpected size {len(data)} bytes")
                continue

            try:
                packet = ImuPacket.from_bytes(data)
            except ValueError as exc:
                self._dropped_rx += 1
                self._log_warning(str(exc))
                continue

            if self._rx_queue.maxlen and len(self._rx_queue) == self._rx_queue.maxlen:
                self._dropped_rx += 1
                self._log_warning("RX queue full, dropping oldest packet")

            self._rx_queue.append(packet)
            self._rx_count += 1

            if packet.timestamp_sample <= self._last_rx_timestamp and self._last_rx_timestamp != 0:
                self._log_warning(
                    f"Non-monotonic timestamp detected (new={packet.timestamp_sample} <= last={self._last_rx_timestamp})"
                )

            self._last_rx_timestamp = packet.timestamp_sample

            if self._verbose:
                print(f"[RX] ts={packet.timestamp_sample} us | dv={packet.delta_velocity} | da={packet.delta_angle}")

    def _apply_ai_processing(self, packet: ImuPacket) -> ImuPacket:
        if packet.delta_velocity_dt == 0:
            return packet

        dt_seconds = packet.delta_velocity_dt / 1e6
        delta_bias = self._bias * dt_seconds
        dv = tuple(axis + delta_bias for axis in packet.delta_velocity)
        return ImuPacket(
            timestamp=packet.timestamp,
            timestamp_sample=packet.timestamp_sample,
            accel_device_id=packet.accel_device_id,
            gyro_device_id=packet.gyro_device_id,
            delta_angle=packet.delta_angle,
            delta_velocity=dv,
            delta_angle_dt=packet.delta_angle_dt,
            delta_velocity_dt=packet.delta_velocity_dt,
            delta_velocity_clipping=packet.delta_velocity_clipping,
            accel_calibration_count=packet.accel_calibration_count,
            gyro_calibration_count=packet.gyro_calibration_count,
        )

    def _process_packets(self) -> None:
        while self._rx_queue:
            packet = self._rx_queue.popleft()
            processed = self._apply_ai_processing(packet)
            payload = processed.to_bytes()

            if self._tx_queue.maxlen and len(self._tx_queue) == self._tx_queue.maxlen:
                self._tx_queue.popleft()
                self._dropped_tx += 1
                self._log_warning("TX queue full, dropping oldest packet")

            self._tx_queue.append(payload)

    def _flush_tx(self) -> None:
        while self._tx_queue:
            payload = self._tx_queue[0]
            try:
                sent = self._tx_socket.sendto(payload, self._tx_addr)
            except BlockingIOError:
                break
            except OSError as exc:
                self._log_warning(f"TX socket error: {exc}")
                self._tx_queue.popleft()
                self._dropped_tx += 1
                continue

            if sent != PACKET_SIZE:
                self._log_warning(f"Partial packet sent ({sent} of {PACKET_SIZE} bytes)")
                self._tx_queue.popleft()
                self._dropped_tx += 1
                continue

            self._tx_queue.popleft()
            self._tx_count += 1

            if self._verbose:
                print(f"[TX] count={self._tx_count} | queue={len(self._tx_queue)}")

    def _log_stats(self) -> None:
        now = time.monotonic()
        if now - self._last_log >= LOG_INTERVAL_S:
            print(
                f"[AI BRIDGE] RX={self._rx_count} (drop={self._dropped_rx}) | "
                f"TX={self._tx_count} (drop={self._dropped_tx}) | "
                f"queues: rx={len(self._rx_queue)}/{self._rx_queue.maxlen} tx={len(self._tx_queue)}/{self._tx_queue.maxlen}"
            )
            self._last_log = now

    def run(self) -> None:
        self._running = True
        print("[AI BRIDGE] Bridge running. Press Ctrl+C to stop.")

        try:
            while self._running:
                events = self._selector.select(timeout=0.01)
                if events:
                    self._receive_packets()
                else:
                    # Poll the RX socket to drain any lingering packets
                    self._receive_packets()

                self._process_packets()
                self._flush_tx()
                self._log_stats()
        except KeyboardInterrupt:
            print("\n[AI BRIDGE] Stopping bridge...")
        finally:
            self.close()
            print(
                f"[AI BRIDGE] Final stats -> RX={self._rx_count} (drop={self._dropped_rx}), "
                f"TX={self._tx_count} (drop={self._dropped_tx})"
            )


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="EKF2 IMU AI UDP bridge")
    parser.add_argument('--port', type=int, default=DEFAULT_RX_PORT,
                        help=f'UDP port to listen on (default: {DEFAULT_RX_PORT})')
    parser.add_argument('--send-port', type=int, default=DEFAULT_TX_PORT,
                        help=f'UDP port to send processed packets to (default: {DEFAULT_TX_PORT})')
    parser.add_argument('--bias', type=float, default=10.0,
                        help='Constant acceleration bias to apply during AI processing (m/s^2)')
    parser.add_argument('--verbose', '-v', action='store_true', help='Print every packet as it is processed')
    return parser.parse_args(argv)


def main(argv: List[str]) -> int:
    args = parse_args(argv)

    bridge = ImuAiBridge(listen_port=args.port, send_port=args.send_port,
                         bias=args.bias, verbose=args.verbose)
    bridge.run()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
