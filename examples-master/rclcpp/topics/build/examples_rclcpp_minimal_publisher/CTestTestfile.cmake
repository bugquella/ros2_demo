# CMake generated Testfile for 
# Source directory: /home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/minimal_publisher
# Build directory: /home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(copyright "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/test_results/examples_rclcpp_minimal_publisher/copyright.xunit.xml" "--package-name" "examples_rclcpp_minimal_publisher" "--output-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/ament_copyright/copyright.txt" "--command" "/opt/ros/dashing/bin/ament_copyright" "--xunit-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/test_results/examples_rclcpp_minimal_publisher/copyright.xunit.xml")
set_tests_properties(copyright PROPERTIES  LABELS "copyright;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/minimal_publisher")
add_test(cppcheck "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/test_results/examples_rclcpp_minimal_publisher/cppcheck.xunit.xml" "--package-name" "examples_rclcpp_minimal_publisher" "--output-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/dashing/bin/ament_cppcheck" "--xunit-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/test_results/examples_rclcpp_minimal_publisher/cppcheck.xunit.xml")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/minimal_publisher")
add_test(cpplint "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/test_results/examples_rclcpp_minimal_publisher/cpplint.xunit.xml" "--package-name" "examples_rclcpp_minimal_publisher" "--output-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/ament_cpplint/cpplint.txt" "--command" "/opt/ros/dashing/bin/ament_cpplint" "--xunit-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/test_results/examples_rclcpp_minimal_publisher/cpplint.xunit.xml")
set_tests_properties(cpplint PROPERTIES  LABELS "cpplint;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/minimal_publisher")
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/test_results/examples_rclcpp_minimal_publisher/lint_cmake.xunit.xml" "--package-name" "examples_rclcpp_minimal_publisher" "--output-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/dashing/bin/ament_lint_cmake" "--xunit-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/test_results/examples_rclcpp_minimal_publisher/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/minimal_publisher")
add_test(uncrustify "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/test_results/examples_rclcpp_minimal_publisher/uncrustify.xunit.xml" "--package-name" "examples_rclcpp_minimal_publisher" "--output-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/ament_uncrustify/uncrustify.txt" "--command" "/opt/ros/dashing/bin/ament_uncrustify" "--xunit-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/test_results/examples_rclcpp_minimal_publisher/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/minimal_publisher")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/dashing/share/ament_cmake_test/cmake/run_test.py" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/test_results/examples_rclcpp_minimal_publisher/xmllint.xunit.xml" "--package-name" "examples_rclcpp_minimal_publisher" "--output-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/ament_xmllint/xmllint.txt" "--command" "/opt/ros/dashing/bin/ament_xmllint" "--xunit-file" "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/build/examples_rclcpp_minimal_publisher/test_results/examples_rclcpp_minimal_publisher/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/hegaozhi/dftc/test/ros2_demo/examples-master/rclcpp/topics/minimal_publisher")
