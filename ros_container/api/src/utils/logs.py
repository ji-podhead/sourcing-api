
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
import subprocess
import asyncio # For running subprocess asynchronously
from typing import List, Dict, Optional, Any
import signal
import psutil # For checking if a process is running
import os
# --- JOURNALCTL LOG STREAMING START ---
class LogConnectionManager:
    def __init__(self,logger):
        self.active_connections: list[WebSocket] = []
        self.journalctl_process: Optional[subprocess.Popen] = None
        self.logger = logger
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        if not self.journalctl_process or self.journalctl_process.poll() is not None:
            # Start journalctl process if not running
            # Using 'journalctl -f -o cat' to follow logs with raw output.
            # This assumes journalctl is available and can capture logs relevant to the FastAPI server.
            # If the FastAPI server is not managed by systemd, its logs might not be captured by default.
            # We use shell=False for security and preexec_fn=os.setsid to manage the process group.
            try:
                self.journalctl_process = subprocess.Popen(
                    ["journalctl", "-f", "-o", "cat"],
                    shell=False,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    bufsize=1, # Line buffered
                    preexec_fn=os.setsid # Create a new process group
                )
                asyncio.create_task(self.stream_logs())
            except FileNotFoundError:
                self.logger.error("journalctl command not found. Ensure it is installed and in PATH.")
                await websocket.send_text("Error: journalctl command not found.")
            except Exception as e:
                self.logger.error(f"Failed to start journalctl process: {e}")
                await websocket.send_text(f"Error starting log stream: {e}")

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
        if not self.active_connections and self.journalctl_process:
            # If no more connections, stop the journalctl process
            try:
                # Send SIGINT to the process group
                os.killpg(os.getpgid(self.journalctl_process.pid), signal.SIGINT)
                self.journalctl_process.wait(timeout=5)
            except Exception as e:
                self.logger.error(f"Error stopping journalctl process: {e}")
            finally:
                self.journalctl_process = None

    async def stream_logs(self):
        """Reads logs from journalctl process and broadcasts them."""
        if not self.journalctl_process or not self.journalctl_process.stdout:
            return

        loop = asyncio.get_event_loop()
        try:
            while True:
                # Check if the process is still running
                if self.journalctl_process.poll() is not None:
                    self.logger.error("journalctl process terminated unexpectedly.")
                    break

                # Read a line from stdout asynchronously
                if self.journalctl_process.stdout.closed:
                    self.logger.error("journalctl stdout pipe closed.")
                    break
                
                line = await loop.run_in_executor(None, self.journalctl_process.stdout.readline)

                if not line: # EOF or process closed stdout
                    self.logger.warning("journalctl stdout returned empty line, possibly EOF.")
                    break
                
                # Broadcast the log line as a JSON object
                await self.broadcast({"log": line.strip()})

        except asyncio.CancelledError:
            self.logger.info("Log streaming task cancelled.")
        except Exception as e:
            self.logger.error(f"Error in log streaming: {e}")
        finally:
            # Ensure the process is terminated if the loop breaks or is cancelled
            if self.journalctl_process and self.journalctl_process.poll() is None:
                try:
                    os.killpg(os.getpgid(self.journalctl_process.pid), signal.SIGINT)
                    self.journalctl_process.wait(timeout=5)
                except Exception as e:
                    self.logger.error(f"Error during final cleanup of journalctl process: {e}")
            self.journalctl_process = None # Clear the process reference

    async def broadcast(self, message: Dict[str, Any]):
        """Broadcasts a message to all active connections."""
        send_tasks = []
        for connection in self.active_connections:
            try:
                send_tasks.append(connection.send_json(message))
            except Exception as e:
                self.logger.error(f"Error sending message to WebSocket: {e}")
                # Consider removing broken connections here if necessary
        if send_tasks:
            await asyncio.gather(*send_tasks, return_exceptions=True)

