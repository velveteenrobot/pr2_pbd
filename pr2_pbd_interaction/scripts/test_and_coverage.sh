#!/bin/bash
#
# Script to run tests and view coverage results.
rostest pr2_pbd_interaction test_endtoend.test
google-chrome ~/.ros/htmlcov/index.html
