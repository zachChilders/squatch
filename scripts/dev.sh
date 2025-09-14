#!/bin/bash

# Consolidated ESP-QEMU-ROS2 Development Script
# Combines functionality from scripts/qemu.sh and brain/scripts/dev.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
TCP_PORT=5555

# Default to mock mode for development
MOCK_IMU=true
MOCK_GNSS=true

# Parse mock/hardware arguments
parse_mock_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --hardware)
                MOCK_IMU=false
                MOCK_GNSS=false
                shift
                ;;
            --real-imu)
                MOCK_IMU=false
                shift
                ;;
            --real-gnss)
                MOCK_GNSS=false
                shift
                ;;
            --mock-all)
                MOCK_IMU=true
                MOCK_GNSS=true
                shift
                ;;
            --mock-imu)
                MOCK_IMU=true
                shift
                ;;
            --mock-gnss)
                MOCK_GNSS=true
                shift
                ;;
            *)
                shift
                ;;
        esac
    done
}

# Global cleanup function
cleanup() {
    echo "Cleaning up all systems..."
    
    # Cleanup QEMU
    if [ ! -z "$QEMU_PID" ]; then
        kill $QEMU_PID 2>/dev/null || true
        echo "Stopped QEMU (PID: $QEMU_PID)"
    fi
    pkill -f "qemu.*tcp.*$TCP_PORT" 2>/dev/null || true
    
    # Cleanup ROS2 containers
    cd "$PROJECT_ROOT/brain"
    docker-compose down 2>/dev/null || true
    cd "$PROJECT_ROOT"
}

trap cleanup EXIT

# ESP-QEMU Management Functions
esp_start() {
    cd "$PROJECT_ROOT/locale"

    # Set environment variables for mock configuration
    export ENABLE_IMU_MOCK=$MOCK_IMU
    export ENABLE_GNSS_MOCK=$MOCK_GNSS

    if [ ! -f "build/squatch.bin" ]; then
        echo "Building ESP project with mock config (IMU: $MOCK_IMU, GNSS: $MOCK_GNSS)..."
        idf.py build
    fi

    echo "Starting QEMU with TCP serial on port $TCP_PORT..."
    idf.py qemu --qemu-extra-args="-serial tcp:0.0.0.0:$TCP_PORT,server,nowait" &
    QEMU_PID=$!
    echo "QEMU started (PID: $QEMU_PID)"

    # Wait for QEMU to be ready
    sleep 3
    echo "✓ ESP QEMU ready on TCP port $TCP_PORT"

    cd "$PROJECT_ROOT"
}

esp_stop() {
    echo "Stopping QEMU processes..."
    pkill -f "qemu.*tcp.*$TCP_PORT" || true
    echo "Stopped"
}

esp_status() {
    echo "QEMU Process:"
    pgrep -f "qemu.*tcp.*$TCP_PORT" && echo "  ✓ QEMU running on TCP port $TCP_PORT" || echo "  ✗ QEMU not running"
    echo "TCP Port:"
    lsof -i :$TCP_PORT >/dev/null 2>&1 && echo "  ✓ Port $TCP_PORT in use" || echo "  ✗ Port $TCP_PORT not in use"
}

# ROS2 Container Management Functions
ros_build() {
    echo "Building ROS2 brain container..."
    cd "$PROJECT_ROOT/brain"
    docker-compose build
    cd "$PROJECT_ROOT"
}

ros_up() {
    echo "Starting ROS2 brain container..."
    cd "$PROJECT_ROOT/brain"
    docker-compose up -d
    cd "$PROJECT_ROOT"
}

ros_shell() {
    echo "Opening shell in ROS2 brain container..."
    cd "$PROJECT_ROOT/brain"
    docker-compose exec ros2_brain /bin/bash
    cd "$PROJECT_ROOT"
}

ros_build_ws() {
    echo "Building ROS2 workspace in container..."
    cd "$PROJECT_ROOT/brain"
    docker-compose exec ros2_brain bash -c "source /opt/ros/kilted/setup.bash && colcon build"
    cd "$PROJECT_ROOT"
}

ros_clean() {
    echo "Cleaning ROS2 workspace..."
    cd "$PROJECT_ROOT/brain"
    docker-compose exec ros2_brain rm -rf build install log
    cd "$PROJECT_ROOT"
}

ros_down() {
    echo "Stopping ROS2 brain container..."
    cd "$PROJECT_ROOT/brain"
    docker-compose down
    cd "$PROJECT_ROOT"
}

ros_logs() {
    cd "$PROJECT_ROOT/brain"
    docker-compose logs -f ros2_brain
    cd "$PROJECT_ROOT"
}

ros_bridge() {
    echo "Running ESP bridge node..."
    cd "$PROJECT_ROOT/brain"
    docker-compose exec ros2_brain bash -c "source /opt/ros/kilted/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run squatch_nodes esp_bridge_node /dev/esp_uart"
    cd "$PROJECT_ROOT"
}

ros_topics() {
    echo "Listing active ROS2 topics..."
    cd "$PROJECT_ROOT/brain"
    docker-compose exec ros2_brain bash -c "source /opt/ros/kilted/setup.bash && ros2 topic list"
    cd "$PROJECT_ROOT"
}

# Integrated Workflow Commands
start_all() {
    # Parse mock/hardware arguments
    parse_mock_args "$@"

    echo "🚀 Starting complete ESP-QEMU-ROS2 integration..."
    echo "Mock configuration: IMU=$MOCK_IMU, GNSS=$MOCK_GNSS"

    # Step 1: Start QEMU (non-blocking)
    echo "Step 1/4: Starting ESP32 QEMU..."
    esp_start
    
    # Step 2: Start ROS2 container
    echo "Step 2/4: Starting ROS2 container..."
    ros_up
    
    # Wait a moment for container to be ready
    sleep 2
    
    # Step 3: Build ROS2 workspace
    echo "Step 3/4: Building ROS2 workspace..."
    ros_build_ws
    
    # Step 4: Wait for TCP port, then start bridge
    echo "Step 4/4: Starting ESP-ROS2 bridge..."
    echo "Waiting for TCP port $TCP_PORT to be ready..."
    
    # Wait up to 10 seconds for TCP port
    for i in {1..10}; do
        if lsof -i :$TCP_PORT >/dev/null 2>&1; then
            echo "✓ TCP port ready, starting bridge..."
            sleep 1
            ros_bridge &
            break
        fi
        echo "  Waiting for TCP port... ($i/10)"
        sleep 1
    done
    
    echo ""
    echo "✓ ESP32 QEMU running on TCP port $TCP_PORT"
    echo "✓ ROS2 container running"
    echo "✓ ESP bridge connecting to sensor data"
    echo ""
    echo "Monitor with: $0 status"
    echo "View ROS2 topics: $0 ros-topics"
    echo "Press Ctrl+C to stop all systems"
    
    # Keep script running to maintain QEMU process
    wait
}

stop_all() {
    echo "🛑 Stopping all ESP-QEMU-ROS2 systems..."
    
    echo "Stopping ROS2 bridge and container..."
    ros_down
    
    echo "Stopping ESP32 QEMU..."
    esp_stop
    
    echo "✓ All systems stopped"
}

unified_status() {
    echo "📊 ESP-QEMU-ROS2 Integration Status"
    echo "=================================="
    
    echo ""
    echo "ESP32 QEMU:"
    esp_status
    
    echo ""
    echo "ROS2 Container:"
    cd "$PROJECT_ROOT/brain"
    if docker-compose ps | grep -q "Up"; then
        echo "  ✓ ROS2 container running"
    else
        echo "  ✗ ROS2 container not running"
    fi
    cd "$PROJECT_ROOT"
    
    echo ""
    echo "Integration Health:"
    # Check if both systems are running
    if pgrep -f "qemu.*tcp.*$TCP_PORT" >/dev/null && docker-compose -f "$PROJECT_ROOT/brain/docker-compose.yml" ps | grep -q "Up"; then
        echo "  ✓ ESP-QEMU-ROS2 integration ready"
    else
        echo "  ⚠ Integration not fully running"
    fi
}

# Main command dispatcher
case "${1:-help}" in
    # ESP Commands
    "esp-start")
        esp_start
        echo "Press Ctrl+C to stop QEMU..."
        wait
        ;;
    "esp-stop")
        esp_stop
        ;;
    "esp-status")
        esp_status
        ;;
        
    # ROS2 Commands  
    "ros-build")
        ros_build
        ;;
    "ros-up")
        ros_up
        ;;
    "ros-shell")
        ros_shell
        ;;
    "ros-build-ws")
        ros_build_ws
        ;;
    "ros-clean")
        ros_clean
        ;;
    "ros-down")
        ros_down
        ;;
    "ros-logs")
        ros_logs
        ;;
    "ros-bridge")
        ros_bridge
        ;;
    "ros-topics")
        ros_topics
        ;;
        
    # Integrated Commands
    "start-all")
        start_all
        ;;
    "stop-all")
        stop_all
        ;;
    "status")
        unified_status
        ;;
        
    # Help and default
    "help"|*)
        echo "Consolidated ESP-QEMU-ROS2 Development Script"
        echo "============================================="
        echo ""
        echo "🚀 INTEGRATED COMMANDS (recommended):"
        echo "  start-all [flags]  - Start complete ESP-QEMU-ROS2 integration"
        echo "    --hardware       - Use real hardware (disables all mocking)"
        echo "    --real-imu       - Use real IMU, mock GNSS"
        echo "    --real-gnss      - Use real GNSS, mock IMU"
        echo "    --mock-all       - Mock both sensors (default)"
        echo "    --mock-imu       - Force mock IMU"
        echo "    --mock-gnss      - Force mock GNSS"
        echo "  stop-all     - Stop all systems"
        echo "  status       - Show status of all components"
        echo ""
        echo "🔧 ESP32 QEMU COMMANDS:"
        echo "  esp-start    - Start ESP32 QEMU with TCP serial"
        echo "  esp-stop     - Stop QEMU processes"
        echo "  esp-status   - Check QEMU and TCP port status"
        echo ""
        echo "🤖 ROS2 COMMANDS:"
        echo "  ros-build    - Build ROS2 container image"
        echo "  ros-up       - Start ROS2 container"
        echo "  ros-shell    - Open bash shell in container"
        echo "  ros-build-ws - Build ROS2 workspace"
        echo "  ros-clean    - Clean workspace build artifacts"
        echo "  ros-down     - Stop and remove container"
        echo "  ros-logs     - View container logs"
        echo "  ros-bridge   - Run ESP bridge node"
        echo "  ros-topics   - List active ROS2 topics"
        echo ""
        echo "💡 QUICK START:"
        echo "  $0 start-all     # Start everything"
        echo "  # ... develop and test ..."
        echo "  $0 stop-all      # Clean shutdown"
        echo ""
        echo "📋 DEVELOPMENT WORKFLOW:"
        echo "  1. $0 start-all    - Complete system startup"  
        echo "  2. $0 status       - Verify all components"
        echo "  3. $0 ros-topics   - Check ROS2 data flow"
        echo "  4. $0 ros-shell    - Access container for debugging"
        echo "  5. $0 stop-all     - Clean shutdown"
        echo ""
        echo "For individual system control, use esp-* and ros-* commands."
        ;;
esac
