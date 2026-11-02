"""Keep suite-wide fixtures independent of ROS and Qt.

Executor-specific guard adaptation lives in test_arm_movement_executor.py so
collecting pure domain tests does not import the execution stack.
"""
