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
  echo "${BASE_DATA_DIR} already exists. Skipping base data."
else
  FILE="v1.0-mini.tgz"
  if [ -f "${FILE}" ]; then
    echo "Unpacking base data (${FILE})"
    tar -xzf "${FILE}" -C data
  else
    echo "Missing base data \"${FILE}\", please download it from nuscenes.org"
    exit 1
  fi
fi

if [ -d "${MAP_EXPANSION_DATA_DIR}" ]; then
  echo "${MAP_EXPANSION_DATA_DIR} already exists. Skipping map expansion data."
else
  FILE="nuScenes-map-expansion-v1.3.zip"
  if [ -f "${FILE}" ]; then
    echo "Unpacking map expansion data (${FILE})"
    unzip "${FILE}" -d data/maps
  else
    echo "Missing map expansion data \"${FILE}\", please download it from nuscenes.org"
    exit 1
  fi
fi

if [ -d "${NUPLAN_MAP_DATA_DIR}" ]; then
  echo "${NUPLAN_MAP_DATA_DIR} already exists. Skipping nuPlan maps data."
else
  FILE="nuplan-maps-v0.1.zip"
  if [ -f "${FILE}" ]; then
    echo "Unpacking nuPlan maps data (${FILE})"
    unzip "${FILE}"
    cp -R nuplan-maps-v0.1/* data/
    rm -rf nuplan-maps-v0.1
  else
    echo "Missing nuPlan maps data \"${FILE}\", please download it from nuscenes.org"
    exit 1
  fi
fi

if [ -d "${CAN_DATA_DIR}" ]; then
  echo "${CAN_DATA_DIR} already exists. Skipping CAN bus data."
else
  FILE="can_bus.zip"
  if [ -f "${FILE}" ]; then
    echo "Unpacking CAN bus data (${FILE})"
    unzip "${FILE}" -d data
  else
    echo "Missing CAN bus data \"${FILE}\", please download it from nuscenes.org"
    exit 1
  fi
fi
