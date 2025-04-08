#!/bin/bash

echo "Setting up folder structure"
mkdir cache
mkdir plots
mkdir videos

echo "Generating .env file"

$USERNAME=$(whoami)
$UID=$(id -u)
$GID=$(id -g)
echo "USER=$USERNAME" > .env
echo "UID=$UID" >> .env
echo "GID=$GID" >> .env

docker compose build