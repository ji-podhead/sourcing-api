import logging
from fastapi import WebSocket, WebSocketDisconnect
from fastapi.routing import APIRoute
from fastapi.responses import JSONResponse
from utils.logs import LogConnectionManager

logger = logging.getLogger(__name__)
log_manager = LogConnectionManager(logger)

routes = []

@routes.append
async def websocket_logs(websocket: WebSocket):
    logger.info("WebSocket connection established for logs.")
    await log_manager.connect(websocket)
    try:
        logger.info("WebSocket connection for logs is now active.")
        # Keep the connection open and wait for disconnect.
        # The log streaming is handled by the stream_logs task.
        while True:
            await asyncio.sleep(1) # Yield control to the event loop
    except WebSocketDisconnect:
        log_manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"Error in websocket_logs handler: {e}")
        log_manager.disconnect(websocket) # Ensure disconnect is called on error

route = APIRoute("/logs", endpoint=websocket_logs, methods=["GET"])
