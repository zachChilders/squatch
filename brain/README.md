# Brain

ROS2 container environment for Squatch sensor communication and processing.

## Quick Start

```bash
# Build and start container
./scripts/dev.sh build
./scripts/dev.sh up

# Open shell in container
./scripts/dev.sh shell

# Build ROS2 workspace
./scripts/dev.sh build-ws
```

## Development Workflow

1. Edit ROS2 nodes in `ros2_ws/src/squatch_nodes/`
2. Run `./scripts/dev.sh build-ws` to build in container
3. Use `./scripts/dev.sh shell` to test and run nodes
4. Container has privileged access for UART/device communication

## Structure

- `Dockerfile` - ROS2 Humble with development tools
- `docker-compose.yml` - Privileged container with volume mounts
- `ros2_ws/src/` - Local workspace for node development
- `scripts/dev.sh` - Development helper commands