#!/bin/bash

# manager.sh - Manages a pool of GigE camera worker processes.

# --- Configuration ---
NUM_WORKERS=10
WORKER_SCRIPT="gige_camera_node.py"
RUNDIR="/tmp/gige_pool"

# --- Functions ---
start_pool() {
    if [ ! -d "$RUNDIR" ]; then
        echo "Creating runtime directory: $RUNDIR"
        mkdir -p "$RUNDIR"
    else
        echo "Runtime directory already exists. Checking for stale processes..."
        stop_pool > /dev/null
    fi

    echo "Starting $NUM_WORKERS worker daemons..."
    for i in $(seq 1 $NUM_WORKERS); do
        WORKER_ID="worker_$i"
        # Start in background, redirect stdout/stderr to a log file
        python3 "$WORKER_SCRIPT" "$WORKER_ID" > "$RUNDIR/$WORKER_ID.log" 2>&1 &
    done

    echo "Waiting for workers to initialize..."
    sleep 2
    status_pool
}

stop_pool() {
    echo "Stopping worker daemons..."
    for i in $(seq 1 $NUM_WORKERS); do
        WORKER_ID="worker_$i"
        PID_FILE="$RUNDIR/$WORKER_ID.pid"
        PIPE_FILE="$RUNDIR/$WORKER_ID.pipe"

        if [ -f "$PID_FILE" ]; then
            PID=$(cat "$PID_FILE")
            kill -9 "$PID" > /dev/null 2>&1
            echo "Killed $WORKER_ID (PID: $PID)"
            rm -f "$PID_FILE"
        fi
        if [ -p "$PIPE_FILE" ]; then
            rm -f "$PIPE_FILE"
        fi
    done
}

status_pool() {
    echo "--- Worker Pool Status ---"
    if [ ! -d "$RUNDIR" ]; then
        echo "Pool is not running."
        return
    fi

    for i in $(seq 1 $NUM_WORKERS); do
        WORKER_ID="worker_$i"
        PID_FILE="$RUNDIR/$WORKER_ID.pid"
        
        if [ -f "$PID_FILE" ]; then
            PID=$(cat "$PID_FILE")
            # Prüfe, ob der Prozess mit dieser PID noch läuft
            if ps -p "$PID" > /dev/null; then
                echo "✅ $WORKER_ID is RUNNING (PID: $PID)"
            else
                echo "❌ $WORKER_ID is STOPPED (stale PID file)"
            fi
        else
            echo "❓ $WORKER_ID is NOT RUNNING (no PID file)"
        fi
    done
    echo "--------------------------"
}

# --- Main Logic ---
case "$1" in
    start)
        start_pool
        ;;
    stop)
        stop_pool
        ;;
    status)
        status_pool
        ;;
    *)
        echo "Usage: $0 {start|stop|status}"
        exit 1
        ;;
esac
