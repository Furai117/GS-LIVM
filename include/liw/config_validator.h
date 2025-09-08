#pragma once

#include <ros/ros.h>

// Validate required parameters loaded from YAML files.
// Throws std::runtime_error on the first problem encountered.
void validateParameters(ros::NodeHandle& nh);
