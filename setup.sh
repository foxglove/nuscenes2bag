#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
BASE_DATA_DIR="${SCRIPT_DIR}/data/v1.0-mini"
MAP_EXPANSION_DATA_DIR="${SCRIPT_DIR}/data/maps/expansion"
NUPLAN_MAP_DATA_DIR="${SCRIPT_DIR}/data/maps/sg-one-north"
CAN_DATA_DIR="${SCRIPT_DIR}/data/can_bus"

cd ${SCRIPT_DIR}

echo "Building nuscenes2bag:0.0.1"
docker build -t nuscenes2bag:0.0.1 .

if [ -d "${BASE_DATA_DIR}" ]; then
  echo "${BASE_DATA_DIR} already exists. Skipping base data download."
else
  echo "Downloading base data"
  wget -N https://www.nuscenes.org/data/v1.0-mini.tgz
  echo "Unpacking base data"
  tar -xzf v1.0-mini.tgz -C data
  rm v1.0-mini.tgz
fi

if [ -d "${MAP_EXPANSION_DATA_DIR}" ]; then
  echo "${MAP_EXPANSION_DATA_DIR} already exists. Skipping map expansion data download."
else
  echo "Downloading map expansion data"
  wget -N https://s3.amazonaws.com/data.nuscenes.org/public/v1.0/nuScenes-map-expansion-v1.3.zip
  echo "Unpacking map expansion data"
  tar -xzf nuScenes-map-expansion-v1.3.zip -C data/maps
  rm nuScenes-map-expansion-v1.3.zip
fi

if [ -d "${NUPLAN_MAP_DATA_DIR}" ]; then
  echo "${NUPLAN_MAP_DATA_DIR} already exists. Skipping nuPlan maps data download."
else
  echo "Downloading nuPlan maps data"
  wget -N https://s3.amazonaws.com/data.nuscenes.org/public/nuplan-v0.1/nuplan-maps-v0.1.zip
  echo "Unpacking nuPlan maps data"
  tar -xzf nuplan-maps-v0.1.zip -C data --strip 1
  rm nuplan-maps-v0.1.zip
fi

if [ -d "${CAN_DATA_DIR}" ]; then
  echo "${CAN_DATA_DIR} already exists. Skipping CAN bus data download."
else
  echo "Downloading CAN bus data"
  wget -N https://s3.amazonaws.com/data.nuscenes.org/public/v1.0/can_bus.zip
  echo "Unpacking CAN bus data"
  tar -xzf can_bus.zip -C data
  rm can_bus.zip
fi
