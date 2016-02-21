# CMake generated Testfile for 
# Source directory: /home/user/Honours/CGP/cgpass1/cgp1-prep/test
# Build directory: /home/user/Honours/CGP/cgpass1/cgp1-prep/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(smoketest "/home/user/Honours/CGP/cgpass1/cgp1-prep/test/tilertest" "-v" "--test=commit")
SET_TESTS_PROPERTIES(smoketest PROPERTIES  WORKING_DIRECTORY "/home/user/Honours/CGP/cgpass1/cgp1-prep")
