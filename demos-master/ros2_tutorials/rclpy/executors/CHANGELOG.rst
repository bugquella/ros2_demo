^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package examples_rclpy_executors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.2 (2019-05-20)
------------------
* Fix deprecation warnings (`#241 <https://github.com/ros2/examples/issues/241>`_)
* Contributors: Jacob Perron

0.7.1 (2019-05-08)
------------------

0.7.0 (2019-04-14)
------------------

0.6.2 (2019-02-08)
------------------

0.6.0 (2018-11-20)
------------------
* No changes.

0.5.1 (2018-06-27)
------------------

0.5.0 (2018-06-26)
------------------
* add pytest markers to linter tests
* set zip_safe to avoid warning during installation (`#205 <https://github.com/ros2/examples/issues/205>`_)
* Contributors: Dirk Thomas, Mikael Arguedas

0.4.0 (2017-12-08)
------------------
* Destroy nodes when the example is done (`#196 <https://github.com/ros2/examples/issues/196>`_)
* wait_for_ready_callbacks returns a tuple now (`#194 <https://github.com/ros2/examples/issues/194>`_)
  `ros2/rclpy#159 <https://github.com/ros2/rclpy/issues/159>`_ changed wait_for_ready_callbacks to manage the generator internally and return just a tuple
* Use logging (`#190 <https://github.com/ros2/examples/issues/190>`_)
* Fix import statement and usage for rclpy.node.Node (`#189 <https://github.com/ros2/examples/issues/189>`_)
* remove test_suite, add pytest as test_requires
* Follow up to executor example comments (`#184 <https://github.com/ros2/examples/issues/184>`_)
* 0.0.3
* remove Listener from the "ThrottledTalkerListener" name given that this is only a throttled talker (`#183 <https://github.com/ros2/examples/issues/183>`_)
* Examples for Executors and callback groups (`#182 <https://github.com/ros2/examples/issues/182>`_)
* Contributors: Dirk Thomas, Mikael Arguedas, Shane Loretz
