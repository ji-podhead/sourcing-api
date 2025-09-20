#!/bin/bash

# camera_control - Sends commands to the GigE camera worker pool.

RUNDIR="/tmp/gige_pool"

# --- Main Logic ---
COMMAND=$(echo "$1" | tr '[:lower:]' '[:upper:]') # Befehl in Großbuchstaben
WORKER_NUM=$2

if [ -z "$COMMAND" ] || [ -z "$WORKER_NUM" ]; then
    echo "Usage: $0 <start|stop> <worker_number> [camera_identifier]"
    exit 1
fi

PIPE_FILE="$RUNDIR/worker_$WORKER_NUM.pipe"

if [ ! -p "$PIPE_FILE" ]; then
    echo "Error: Worker $WORKER_NUM does not seem to be running (pipe not found at $PIPE_FILE)."
    exit 1
fi

case "$COMMAND" in
    START)
        CAMERA_ID="$3"
        if [ -z "$CAMERA_ID" ]; then
            echo "Error: 'start' command requires a camera_identifier."
            echo "Usage: $0 start <worker_number> <camera_identifier>"
            exit 1
        fi
        echo "Sending START command for camera '$CAMERA_ID' to worker_$WORKER_NUM..."
        echo "START $CAMERA_ID" > "$PIPE_FILE"
        ;;
    STOP)
        echo "Sending STOP command to worker_$WORKER_NUM..."
        echo "STOP" > "$PIPE_FILE"
        ;;
    *)
        echo "Error: Unknown command '$1'. Use 'start' or 'stop'."
        exit 1
        ;;
esac

echo "Command sent."