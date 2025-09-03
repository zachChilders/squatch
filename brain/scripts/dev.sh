#!/bin/bash

# ROS2 Brain Container Development Script

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRAIN_DIR="$(dirname "$SCRIPT_DIR")"

cd "$BRAIN_DIR"

case "${1:-help}" in
  "build")
    echo "Building ROS2 brain container..."
    docker-compose build
    ;;
    
  "up")
    echo "Starting ROS2 brain container..."
    docker-compose up -d
    ;;
    
  "shell")
    echo "Opening shell in ROS2 brain container..."
    docker-compose exec ros2_brain /bin/bash
    ;;
    
  "build-ws")
    echo "Building ROS2 workspace in container..."
    docker-compose exec ros2_brain colcon build
    ;;
    
  "clean")
    echo "Cleaning ROS2 workspace..."
    docker-compose exec ros2_brain rm -rf build install log
    ;;
    
  "down")
    echo "Stopping ROS2 brain container..."
    docker-compose down
    ;;
    
  "logs")
    docker-compose logs -f ros2_brain
    ;;
    
  "help"|*)
    echo "ROS2 Brain Development Commands:"
    echo "  build     - Build the container image"
    echo "  up        - Start the container"
    echo "  shell     - Open bash shell in container"
    echo "  build-ws  - Build ROS2 workspace"
    echo "  clean     - Clean workspace build artifacts"
    echo "  down      - Stop and remove container"
    echo "  logs      - View container logs"
    echo ""
    echo "Workflow:"
    echo "  1. ./scripts/dev.sh build"
    echo "  2. ./scripts/dev.sh up"
    echo "  3. Edit nodes in ros2_ws/src/"
    echo "  4. ./scripts/dev.sh build-ws"
    echo "  5. ./scripts/dev.sh shell (to test)"
    ;;
esac