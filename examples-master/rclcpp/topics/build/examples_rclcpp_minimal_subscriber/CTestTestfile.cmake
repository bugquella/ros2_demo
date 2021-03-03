# CMake generated Testfile for 
# Source directory: /home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/minimal_subscriber
# Build directory: /home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(copyright "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/test_results/examples_rclcpp_minimal_subscriber/copyright.xunit.xml" "--package-name" "examples_rclcpp_minimal_subscriber" "--output-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/ament_copyright/copyright.txt" "--command" "/opt/ros/dashing/bin/ament_copyright" "--xunit-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/test_results/examples_rclcpp_minimal_subscriber/copyright.xunit.xml")
set_tests_properties(copyright PROPERTIES  LABELS "copyright;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/minimal_subscriber")
add_test(cppcheck "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/test_results/examples_rclcpp_minimal_subscriber/cppcheck.xunit.xml" "--package-name" "examples_rclcpp_minimal_subscriber" "--output-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/dashing/bin/ament_cppcheck" "--xunit-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/test_results/examples_rclcpp_minimal_subscriber/cppcheck.xunit.xml")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/minimal_subscriber")
add_test(cpplint "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/test_results/examples_rclcpp_minimal_subscriber/cpplint.xunit.xml" "--package-name" "examples_rclcpp_minimal_subscriber" "--output-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/ament_cpplint/cpplint.txt" "--command" "/opt/ros/dashing/bin/ament_cpplint" "--xunit-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/test_results/examples_rclcpp_minimal_subscriber/cpplint.xunit.xml")
set_tests_properties(cpplint PROPERTIES  LABELS "cpplint;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/minimal_subscriber")
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/test_results/examples_rclcpp_minimal_subscriber/lint_cmake.xunit.xml" "--package-name" "examples_rclcpp_minimal_subscriber" "--output-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/dashing/bin/ament_lint_cmake" "--xunit-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/test_results/examples_rclcpp_minimal_subscriber/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/minimal_subscriber")
add_test(uncrustify "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/test_results/examples_rclcpp_minimal_subscriber/uncrustify.xunit.xml" "--package-name" "examples_rclcpp_minimal_subscriber" "--output-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/ament_uncrustify/uncrustify.txt" "--command" "/opt/ros/dashing/bin/ament_uncrustify" "--xunit-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/test_results/examples_rclcpp_minimal_subscriber/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/minimal_subscriber")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/test_results/examples_rclcpp_minimal_subscriber/xmllint.xunit.xml" "--package-name" "examples_rclcpp_minimal_subscriber" "--output-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/ament_xmllint/xmllint.txt" "--command" "/opt/ros/dashing/bin/ament_xmllint" "--xunit-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_subscriber/test_results/examples_rclcpp_minimal_subscriber/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/minimal_subscriber")