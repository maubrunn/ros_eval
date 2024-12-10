#!/bin/bash

# Container names for both services
CONTAINER_NAME="ros_eval_container"
# Flag for restarting the container
RESTART_FLAG=$1

# Flag to choose ARM service
ARM_FLAG=$2

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
    docker-compose down && docker-compose up -d $service
}

# Check which service to run (x86 or ARM)
if [ "$ARM_FLAG" == "--arm" ]; then
    SERVICE_NAME="ros_eval_arm"
else
    SERVICE_NAME="ros_eval"
fi

# Check if the selected container is running
if [ -z "$(is_container_running $CONTAINER_NAME)" ]; then
    echo "Container for $SERVICE_NAME is not running. Starting it..."
    start_container $SERVICE_NAME
else
    echo "Container for $SERVICE_NAME is already running."
fi

# If the restart flag is set, restart the container
if [ "$RESTART_FLAG" == "--restart" ]; then
    restart_container $SERVICE_NAME
fi

# Run the exec command to get a terminal inside the selected container
echo "Accessing the container terminal for $SERVICE_NAME..."
docker exec -it $CONTAINER_NAME /bin/bash

