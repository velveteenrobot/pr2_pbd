#!/bin/bash
#
# Script to run tests, view coverage results, and upload to Coveralls.
rostest pr2_pbd_interaction test_endtoend.test
google-chrome ~/.ros/htmlcov/index.html
coveralls --data_file ~/.ros/.coverage
