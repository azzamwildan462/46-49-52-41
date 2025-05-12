#!/bin/bash

colcon build --symlink-install --executor parallel --parallel $(nproc)