#!/usr/bin/env python3
"""
tcp_server.py

ROS 1 TCP server to receive JSON messages (e.g., from Unity)
and forward them into the ROS system using `publish_from_unity`.

Key Features:
- Non-blocking select-based TCP server
- Accepts multiple clients
- Routes incoming messages to `AppRunner`
"""

import json
import socket
import select
import logging
import os
import time
import threading
import datetime

class ThroughputMonitor:
    def __init__(self, logger, log_file=None):
        self.logger = logger
        self.rx_bytes = 0
        self.tx_bytes = 0
        self.last_time = time.time()
        self.running = True
        self._lock = threading.Lock()
        self.log_file = log_file  # optional file path

    def add_rx(self, n):
        with self._lock:
            self.rx_bytes += n

    def add_tx(self, n):
        with self._lock:
            self.tx_bytes += n

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.running:
            time.sleep(1.0)
            with self._lock:
                now = time.time()
                elapsed = now - self.last_time
                if elapsed <= 0:
                    continue

                rx_kbps = (self.rx_bytes * 8 / 1000) / elapsed
                tx_kbps = (self.tx_bytes * 8 / 1000) / elapsed

                # Log to console
                self.logger.debug(f"[Throughput] RX={rx_kbps:.2f} kbps | TX={tx_kbps:.2f} kbps")

                # Also append to file
                if self.log_file:
                    try:
                        with open(self.log_file, "a") as f:
                            entry = {
                                "timestamp": time.time(),
                                "rx_kbps": rx_kbps,
                                "tx_kbps": tx_kbps
                            }
                            f.write(json.dumps(entry) + "\n")
                    except Exception as e:
                        self.logger.debug(f"[Throughput] Failed to write log: {e}")

                # reset counters
                self.rx_bytes = 0
                self.tx_bytes = 0
                self.last_time = now

    def stop(self):
        self.running = False


class TCPServer:
    def __init__(self, ros_node, logger=None, host='127.0.0.1', port=65432):
        """
        Args:
            ros_node: An object with a `publish_from_unity()` method (e.g., AppRunner).
            logger: Optional logger instance.
            host: Interface to bind (e.g., 0.0.0.0 for all).
            port: TCP port to listen on.
        """
        self.ros_node = ros_node
        self.logger = logger or logging.getLogger(__name__)
        self.host = host
        self.port = port

        self.server_socket = None
        self.sockets_list = []
        self.clients = {}
        self.running = False

        self.recv_buffer = {}
        self.throughput = ThroughputMonitor(self.logger)

        throughput_log = os.path.join(
            "/home/migasdrone/ros1_ws/src/ros1_server/logs",
            f"throughput_{datetime.datetime.now():%Y%m%d_%H%M%S}.jsonl"
        )
        self.throughput = ThroughputMonitor(self.logger, log_file=throughput_log)


    def start(self):
        """Starts the TCP server loop."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen()
            self.server_socket.setblocking(False)

            self.sockets_list = [self.server_socket]
            self.running = True

            self.logger.info(f"[TCPServer] Listening on {self.host}:{self.port}")
            self.throughput.start()

            while self.running:
                read_sockets, _, exception_sockets = select.select(
                    self.sockets_list, [], self.sockets_list, 0.1
                )

                for notified_socket in read_sockets:
                    if notified_socket == self.server_socket:
                        self._accept_connection()
                    else:
                        self._receive_message(notified_socket)

                for sock in exception_sockets:
                    self._handle_exception(sock)

        except Exception as e:
            self.logger.exception(f"[TCPServer] Server error: {e}")
        finally:
            self.stop()

    def _accept_connection(self):
        client_socket, client_address = self.server_socket.accept()
        client_socket.setblocking(False)
        self.sockets_list.append(client_socket)
        self.clients[client_socket] = client_address
        self.logger.info(f"[TCPServer] New connection from {client_address}")

    def _receive_message(self, sock):
        try:
            data = sock.recv(4096)
            self.throughput.add_rx(len(data))
            if not data:
                return self._disconnect_client(sock)

            buf = self.recv_buffer.setdefault(sock, b'') + data

            # process all complete messages
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)

                # --- NEW: ignore empty or whitespace-only frames ---
                if not line.strip():
                    continue

                decoded = line.decode('utf-8', 'replace').strip()
                self.logger.debug(f"[TCPServer] Received JSON: {decoded}")
                self.ros_node.publish_from_unity(decoded)

            # save any trailing partial
            self.recv_buffer[sock] = buf

        except Exception as e:
            self.logger.error(f"[TCPServer] Error receiving from {self.clients.get(sock)}: {e}")
            self._disconnect_client(sock)

    def _disconnect_client(self, sock):
        addr = self.clients.get(sock, "Unknown")
        self.logger.info(f"[TCPServer] Disconnected: {addr}")
        if sock in self.sockets_list:
            self.sockets_list.remove(sock)
        self.clients.pop(sock, None)
        sock.close()

        # notify AppRunner that Unity has gone away
        try:
            self.ros_node.handle_unity_disconnect()
        except AttributeError:
            pass

    def _handle_exception(self, sock):
        self.logger.warning(f"[TCPServer] Exception on socket {self.clients.get(sock)}")
        self._disconnect_client(sock)

    def stop(self):
        """Closes all sockets and stops the server."""
        self.running = False
        for sock in self.sockets_list:
            sock.close()
        self.logger.info("[TCPServer] Stopped.")
        self.throughput.stop()
