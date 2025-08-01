#!/bin/bash

# Redirect all ROS2 output to a temporary log file
LOG_FILE=$(mktemp)
exec 3>&1  # Save original stdout
exec >"$LOG_FILE" 2>&1

# Cleanup function
cleanup() {
    # Kill processes
    pkill -f 'mir_restapi_server' 2>/dev/null || true
    pkill -f 'ros2 run mir_restapi' 2>/dev/null || true
    
    # Show success message if we have one
    if [ -f success.flag ]; then
        echo "=== Time synchronization completed successfully ===" >&3
    fi
    
    # Clean up temp files
    rm -f "$LOG_FILE" success.flag 2>/dev/null
    exit 0
}

trap cleanup EXIT

echo "=== Starting MIR Time Synchronization ===" >&3
echo "- Cleaning up any existing nodes..." >&3
pkill -f 'mir_restapi_server' 2>/dev/null || true
pkill -f 'ros2 run mir_restapi' 2>/dev/null || true
sleep 2

echo "- Starting new server instance..." >&3
ros2 run mir_restapi mir_restapi_server --ros-args \
    -p mir_hostname:='<robot-ip>' \
    -p mir_restapi_auth:='Basic <your base64-encoded credentials>' &
SERVER_PID=$!

echo "- Waiting for server to initialize..." >&3
MAX_WAIT=15
for ((i=1; i<=$MAX_WAIT; i++)); do
    if ros2 node list | grep -q '/mir_restapi_server'; then
        break
    fi
    if [ $i -eq $MAX_WAIT ]; then
        echo "ERROR: Server failed to start after $MAX_WAIT seconds" >&3
        exit 1
    fi
    sleep 1
done

echo "- Synchronizing time with MIR 250..." >&3
SERVICE_OUTPUT=$(ros2 service call /mir_250_sync_time std_srvs/Trigger)
echo "$SERVICE_OUTPUT" | grep -A1 'response:' | sed 's/^/  /' >&3

echo "- Waiting for sync to complete (the light on the MIR 250 should turn on)..." >&3
WAIT_FOR_SHUTDOWN=100 
for ((i=1; i<=$WAIT_FOR_SHUTDOWN; i++)); do
    if ! ros2 node list | grep -q '/mir_restapi_server'; then
        touch success.flag
        break
    fi
    if [ $i -eq $WAIT_FOR_SHUTDOWN ]; then
        echo "If the light on the MIR 250 still waiving, restart the time synchronization script." >&3
        echo "Otherwise, the time synchronization was successful." >&3
        touch success.flag
    fi
    sleep 1
done
