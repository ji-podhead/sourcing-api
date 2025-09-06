import os
import subprocess
import signal
import asyncio
import pty
import select
import logging
from fastapi import WebSocket, WebSocketDisconnect
from fastapi.routing import APIRoute

logger = logging.getLogger(__name__)

routes = []

async def websocket_terminal(websocket: WebSocket):
    await websocket.accept()
    
    master_fd, slave_fd = pty.openpty()
    
    shell_command = ["/bin/bash", "-l"]
    process = subprocess.Popen(
        shell_command,
        stdin=slave_fd,
        stdout=slave_fd,
        stderr=slave_fd,
        preexec_fn=os.setsid,
        env=os.environ.copy()
    )
    
    os.close(slave_fd)
    logger.info(f"Terminal WebSocket connected. Shell process PID: {process.pid}")

    async def read_from_pty_and_send_to_ws():
        """Reads from the pseudo-terminal and sends to the WebSocket."""
        loop = asyncio.get_event_loop()
        while True:
            try:
                # Use asyncio to wait for data to be readable
                await loop.run_in_executor(None, lambda: select.select([master_fd], [], []))
                output = os.read(master_fd, 4096).decode(errors='ignore')
                if output:
                    await websocket.send_text(output)
                else: # This means PTY was closed
                    break
            except (WebSocketDisconnect, BrokenPipeError):
                break
            except Exception as e:
                logger.error(f"Error reading from PTY: {e}")
                break

    async def read_from_ws_and_send_to_pty():
        """Reads from the WebSocket and sends to the pseudo-terminal."""
        while True:
            try:
                data = await websocket.receive_text()
                os.write(master_fd, data.encode())
            except WebSocketDisconnect:
                break
            except Exception as e:
                logger.error(f"Error reading from WebSocket: {e}")
                break

    # Run both tasks concurrently
    pty_reader_task = asyncio.create_task(read_from_pty_and_send_to_ws())
    ws_reader_task = asyncio.create_task(read_from_ws_and_send_to_pty())

    done, pending = await asyncio.wait(
        [pty_reader_task, ws_reader_task],
        return_when=asyncio.FIRST_COMPLETED,
    )

    # Clean up pending tasks and resources
    for task in pending:
        task.cancel()
    
    process.terminate()
    process.wait()
    os.close(master_fd)
    logger.info(f"Terminal shell process {process.pid} terminated and PTY closed.")

route = APIRoute("/terminal", endpoint=websocket_terminal, methods=["GET"])
