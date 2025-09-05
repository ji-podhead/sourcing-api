from fastapi import FastAPI
from fastapi.routing import APIRoute
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*","http://localhost:3000", "http://100.93.237.122:3000", "http://localhost:8000"], # Allow requests from the dashboard and direct API access
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

async def read_root():
    """
    Root endpoint for the ROS Container FastAPI.
    Returns a simple message to confirm the API is running.
    """
    return {"message": "ROS Container FastAPI is running!"}

app.add_api_route("/", read_root, methods=["GET"])
