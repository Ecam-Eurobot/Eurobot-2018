# differential-drive
Fork of code.google.com/p/differential-drive
A ROS package of basic tools for differential drive robots.

Authors: jfstepha (original branch), Joseph Duchesne (C++ ports, catkin/ros indigo update, and a few bug fixes)

License: GPLv3

See the ROS wiki page: http://www.ros.org/wiki/differential_drive for documentation. The C++ version of pid_velocity takes the same paramaters as the Python version.

Changelog from Jfstepha's version:
- All ros topic pub/subs have a queue length defined (typically of 1 for speed reasons)
- There is a C++ port of the pid_velocity node (it's 4-5x faster)
- The C++ version has a corner case bug fixed when a rotary encoder update is behind schedule and velocity is negative.
- I switched the package from rosbuild to catkin since it's the current recommended tool.
