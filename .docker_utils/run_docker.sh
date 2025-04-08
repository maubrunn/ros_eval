#!/bin/bash

# Container names for both services
CONTAINER_NAME="ros_eval_container"


for arg in "$@"; do
  case $arg in
    --help)
      echo "Usage: ./run_docker.sh [OPTIONS]"
      echo "Options:"
      echo "  --help      Show this help message and exit."
      echo "  --restart   Restarts the container if it is already running."
      echo "  --arm       Run the ARM version of the service."
      exit 0
      ;;
    --restart)
      RESTART_FLAG=1
      ;;
    --arm)
      ARM_FLAG=1
      ;;
    *)
      echo "Unknown option: $arg"
      exit 1
      ;;
  esac
done

if [[ $ARM_FLAG -eq 1 ]]; then
    SERVICE_NAME="ros_eval_arm"
else
    SERVICE_NAME="ros_eval"
fi


# Function to check if the container is running
is_container_running() {
    local container_name=$1
    docker ps --filter "name=$container_name" --format "{{.Names}}"
}

# Start the container if it's not running
start_container() {
    local service=$1
    echo "Starting the container for service $service..."
    docker-compose up -d $service
}

# Restart the container if the flag is set
restart_container() {
    local service=$1
    echo "Restarting the container for service $service..."
    docker-compose down $service && docker-compose up -d $service
}

if [ -z "$(is_container_running $CONTAINER_NAME)" ]; then
    echo "Container for $SERVICE_NAME is not running. Starting it..."
    start_container $SERVICE_NAME
else
    echo "Container for $SERVICE_NAME is already running."
    if [[ $RESTART_FLAG -eq 1 ]]; then
        restart_container $SERVICE_NAME
    fi
fi


