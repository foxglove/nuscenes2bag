#!/bin/bash

docker run --rm -p 8888:8888 -v `pwd`:/notebooks nuscenes2bag:0.0.1
