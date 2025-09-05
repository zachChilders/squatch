#!/bin/bash

# ESP32-S2 QEMU with ROS2 TCP Integration

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
TCP_PORT=5555

# Cleanup function
cleanup() {
    echo "Cleaning up..."
    if [ ! -z "$QEMU_PID" ]; then
        kill $QEMU_PID 2>/dev/null || true
        echo "Stopped QEMU (PID: $QEMU_PID)"
    fi
}

trap cleanup EXIT

case "${1:-help}" in
    "start")
        cd "$PROJECT_ROOT/locale"
        
        # Check if build exists
        if [ ! -f "build/squatch.bin" ]; then
            echo "Building ESP project first..."
            idf.py build
        fi
        
        # Start QEMU with TCP serial in background
        echo "Starting QEMU with TCP serial on port $TCP_PORT..."
        idf.py qemu --qemu-extra-args="-serial tcp:0.0.0.0:$TCP_PORT,server,nowait" &
        QEMU_PID=$!
        echo "QEMU started (PID: $QEMU_PID)"
        
        # Wait for QEMU to be ready
        sleep 3
        
        echo "✓ ESP QEMU ready on TCP port $TCP_PORT"
        echo "Press Ctrl+C to stop QEMU..."
        wait
        ;;
        
    "stop")
        echo "Stopping QEMU processes..."
        pkill -f "qemu.*tcp.*$TCP_PORT" || true
        echo "Stopped"
        ;;
        
    "status")
        echo "QEMU Process:"
        pgrep -f "qemu.*tcp.*$TCP_PORT" && echo "  ✓ QEMU running on TCP port $TCP_PORT" || echo "  ✗ QEMU not running"
        echo "TCP Port:"
        lsof -i :$TCP_PORT >/dev/null 2>&1 && echo "  ✓ Port $TCP_PORT in use" || echo "  ✗ Port $TCP_PORT not in use"
        ;;
        
    "help"|*)
        echo "ESP32-S2 QEMU ROS2 TCP Integration Commands:"
        echo "  start    - Start QEMU with TCP serial for direct ROS2 access"
        echo "  stop     - Stop QEMU processes"  
        echo "  status   - Check if QEMU is running and TCP port is available"
        echo ""
        ;;
esac