#!/bin/bash

echo "Setting up folder structure"
mkdir cache
mkdir plots
mkdir videos

echo "Generating .env file"

echo "USER=$(whoami)" > .env
echo "UID=$(id -u)" >> .env
echo "GID=$(id -g)" >> .env

docker compose build ros_eval