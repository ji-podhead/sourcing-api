import os
from fastapi import FastAPI
from fastapi.routing import APIRoute
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

# --- CORS Configuration ---
origins = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
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
