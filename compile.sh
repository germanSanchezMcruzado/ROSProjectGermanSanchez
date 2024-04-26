#!/bin/bash

# Execute catkin make command
catkin_make

# Move the file from one local path to another
mv build/planner/planner_node src/planner/planner_node


