# tcp/client.py
import socket
import threading
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)


from tcp.server import ThroughputMonitor

class TCPClient:
    """
    A simple TCP client with callbacks for connect/disconnect/data.
    """
    def __init__(self, host, port, 
                 on_connected=None, 
                 on_disconnected=None, 
                 on_data=None, 
                 logger=None):
        self.host = host
        self.port = port
        self.on_connected    = on_connected
        self.on_disconnected = on_disconnected
        self.on_data         = on_data
        self.logger          = logger

        self.sock    = None
        self.running = False
        self.thread  = None
        self.throughput = ThroughputMonitor(self.logger)


    def start(self):
        """Connect and start the receive thread."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))

            self.sock.settimeout(None)
            
            self.running = True
            if self.logger: self.logger.info(f"[TCPClient] Connected to {self.host}:{self.port}")
            if self.on_connected: self.on_connected()
            self.throughput.start()

            self.thread = threading.Thread(target=self._recv_loop, daemon=True)
            self.thread.start()
           

        except Exception as e:
            if self.logger: self.logger.error(f"[TCPClient] Could not connect: {e}")
            self.running = False

    def send(self, data: str):
        """Send a UTF-8 string (no framing)."""
        if not self.running or not self.sock:
            if self.logger: self.logger.warning("[TCPClient] Send called when not running")
            return
        try:
            self.sock.sendall(data.encode("utf-8"))
            self.throughput.add_tx(len(data.encode("utf-8")))
        except Exception as e:
            if self.logger: self.logger.error(f"[TCPClient] Send error: {e}")

    def _recv_loop(self):
        """Continuously read from the socket and fire on_data callbacks."""
        try:
            while self.running:
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
                self.throughput.add_rx(len(chunk))
                text = chunk.decode("utf-8")
                if self.on_data:
                    self.on_data(text)
        except Exception as e:
            if self.logger: self.logger.error(f"[TCPClient] Receive error: {e}")
        finally:
            self.running = False
            try:
                self.sock.close()
            except:
                pass
            if self.logger: self.logger.info("[TCPClient] Disconnected")
            if self.on_disconnected: self.on_disconnected()

    def stop(self):
        """Shut down the client."""
        self.running = False
        try:
            if self.sock: self.sock.shutdown(socket.SHUT_RDWR)
        except:
            pass
        finally:
            if self.sock: self.sock.close()
            self.throughput.stop()
