#!/bin/bash

# ESP32-S2 QEMU with ROS2 UART Bridge
# Replaces: idf.py qemu monitor
# Provides: QEMU + TCP→PTY bridge for ROS2 container access

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
ESP_PTY="/tmp/esp_uart"
TCP_PORT=5555

# Cleanup function
cleanup() {
    echo "Cleaning up..."
    if [ ! -z "$SOCAT_PID" ]; then
        kill $SOCAT_PID 2>/dev/null || true
        echo "Stopped socat bridge (PID: $SOCAT_PID)"
    fi
    if [ ! -z "$QEMU_PID" ]; then
        kill $QEMU_PID 2>/dev/null || true
        echo "Stopped QEMU (PID: $QEMU_PID)"
    fi
    rm -f "$ESP_PTY"
}

trap cleanup EXIT

case "${1:-help}" in
    "start")
        echo "Starting ESP32-S2 QEMU with ROS2 UART bridge..."
        
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
        
        # Start socat bridge TCP→PTY
        echo "Starting TCP→PTY bridge: localhost:$TCP_PORT → $ESP_PTY"
        socat pty,link="$ESP_PTY",raw tcp:localhost:$TCP_PORT &
        SOCAT_PID=$!
        echo "Bridge started (PID: $SOCAT_PID)"
        
        # Wait for PTY to be created
        timeout=10
        while [ ! -e "$ESP_PTY" ] && [ $timeout -gt 0 ]; do
            sleep 1
            ((timeout--))
        done
        
        if [ -e "$ESP_PTY" ]; then
            echo "✓ ESP UART bridge ready at: $ESP_PTY"
            echo "✓ ROS2 container will access via: /dev/esp_uart"
            echo ""
            echo "ROS2 Integration Commands:"
            echo "  cd brain && ./scripts/dev.sh build-ws"
            echo "  ./scripts/dev.sh esp-bridge"
            echo ""
            echo "Monitor ESP output:"
            echo "  cat $ESP_PTY"
            echo ""
            echo "Press Ctrl+C to stop QEMU and bridge..."
            wait
        else
            echo "✗ Failed to create PTY at $ESP_PTY"
            exit 1
        fi
        ;;
        
    "stop")
        echo "Stopping QEMU and bridge processes..."
        pkill -f "qemu.*tcp::$TCP_PORT" || true
        pkill -f "socat.*$ESP_PTY" || true
        rm -f "$ESP_PTY"
        echo "Stopped"
        ;;
        
    "status")
        echo "QEMU Process:"
        pgrep -f "qemu.*tcp::$TCP_PORT" && echo "  ✓ QEMU running" || echo "  ✗ QEMU not running"
        echo "Socat Bridge:"
        pgrep -f "socat.*$ESP_PTY" && echo "  ✓ Bridge running" || echo "  ✗ Bridge not running"
        echo "PTY Device:"
        [ -e "$ESP_PTY" ] && echo "  ✓ $ESP_PTY exists" || echo "  ✗ $ESP_PTY missing"
        ;;
        
    "monitor")
        echo "Monitoring ESP output from $ESP_PTY..."
        if [ -e "$ESP_PTY" ]; then
            cat "$ESP_PTY"
        else
            echo "PTY not available. Run 'start' first."
        fi
        ;;
        
    "help"|*)
        echo "ESP32-S2 QEMU ROS2 Bridge Commands:"
        echo "  start    - Start QEMU with TCP→PTY bridge for ROS2"
        echo "  stop     - Stop QEMU and bridge processes"  
        echo "  status   - Check if QEMU and bridge are running"
        echo "  monitor  - Monitor ESP output directly"
        echo ""
        echo "Integration Workflow:"
        echo "  1. ./scripts/qemu-ros2.sh start"
        echo "  2. cd brain && ./scripts/dev.sh up"
        echo "  3. ./scripts/dev.sh build-ws"
        echo "  4. ./scripts/dev.sh esp-bridge"
        echo ""
        echo "Replaces: idf.py qemu monitor"
        echo "Provides: QEMU + ROS2 UART bridge via $ESP_PTY"
        ;;
esac