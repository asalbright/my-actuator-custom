import socket
import struct
import threading
import queue
import time
from collections import deque
import math
import numpy as np


class CommandReceiver:
    """
    UDP receiver for robot control commands with smoothing and extrapolation.
    Runs in a separate thread and puts received commands in a queue.
    """

    def __init__(self, listen_port=12345, queue_maxsize=6):
        """
        Initialize the UDP receiver.

        Args:
            listen_port: Port to listen on
            queue_maxsize: Maximum size of the command queue (keeps only recent commands)
            derivative_smoothing: EMA smoothing factor for derivative (0-1, higher = less smoothing)
            max_derivative_change: Maximum allowed absolute change in derivative per update
        """
        self.listen_port = listen_port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("", listen_port))  # Listen on all interfaces

        # Queue to store received commands
        self.command_queue = queue.Queue(maxsize=queue_maxsize)

        # Threading control
        self._running = False
        self._thread = None

        # Statistics
        self.packets_received = 0
        self.last_sequence_number = None
        self.dropped_packets = 0

        # Struct format: sequence_number (uint32) + x (1 float)
        self.packet_format = "!I1f"
        self.packet_size = struct.calcsize(self.packet_format)

        # History of received UDP data points (time, value)
        self.udp_history = deque(maxlen=6)

        # Smoothing state
        self._last_command = None
        self._last_command_time = time.time()
        self._last_udp_time = time.time()
        self._initialized = False  # Track if we've received first UDP data
        self._lock = threading.Lock()  # Protect shared state

        self.raw_data = []

    def start(self):
        """Start the receiver thread."""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the receiver thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        self.socket.close()

    def get_latest_command(self):
        """
        Get the latest command from the queue.
        Returns None if no command is available.
        """
        data = list(self.command_queue.queue)
        # print("data_len", len(data))
        if not data:
            return 0

        if len(data) < 2:
            return data[-1][1]
        current_time = time.time()
        last_time = data[-1][0]
        slopes = []
        for i in range(1, len(data)):
            t1, y1 = data[i - 1]
            t2, y2 = data[i]
            if t2 - t1 > 0:
                slope = (y2 - y1) / (t2 - t1)
                slopes.append(slope)
        avg_slope = np.mean(slopes) if slopes else 0.0

        time_delta = math.tanh(current_time - last_time)
        estimated_y = data[-1][1] + avg_slope * time_delta
        return estimated_y

    def get_stats(self) -> dict:
        """Get receiver statistics."""
        with self._lock:
            return {
                "packets_received": self.packets_received,
                "dropped_packets": self.dropped_packets,
                "queue_size": self.command_queue.qsize(),
                "current_derivative": self._smoothed_derivative,
                "udp_history_size": len(self.udp_history),
            }

    def _receive_loop(self):
        """Main receive loop (runs in separate thread)."""
        self.socket.settimeout(0.1)  # Short timeout to check _running flag

        while self._running:
            try:
                # Receive packet
                data, addr = self.socket.recvfrom(1024)

                # Validate packet size
                if len(data) != self.packet_size:
                    print(f"Warning: Received packet of unexpected size: {len(data)}")
                    continue

                # Unpack data
                sequence_number, x = struct.unpack(self.packet_format, data)

                val = time.time(), x
                self.raw_data.append(val)

                # Track statistics
                self.packets_received += 1
                if self.last_sequence_number is not None:
                    expected_seq = (self.last_sequence_number + 1) % 0xFFFFFFFF
                    if sequence_number != expected_seq:
                        missed = (sequence_number - expected_seq) % 0xFFFFFFFF
                        self.dropped_packets += missed

                self.last_sequence_number = sequence_number

                # Put command in queue (non-blocking)
                try:
                    self.command_queue.put_nowait(val)
                except queue.Full:
                    # Queue is full, remove oldest and add new
                    try:
                        self.command_queue.get_nowait()  # Remove oldest
                        self.command_queue.put_nowait(val)  # Add new
                    except queue.Empty:
                        pass  # Race condition, ignore

            except socket.timeout:
                continue  # Check _running flag
            except Exception as e:
                if self._running:  # Only print errors if we're supposed to be running
                    print(f"Error in receive loop: {e}")

    def raw_cg_data(self):
        """
        Get the raw UDP data received so far.
        Returns a list of (timestamp, value) tuples.
        """
        with self._lock:
            return list(self.raw_data)
