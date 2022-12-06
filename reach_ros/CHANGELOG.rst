^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reach_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Install reach_ros headers (`#40 <https://github.com/marip8/reach/issues/40>`_)
* Added Python interface (`#38 <https://github.com/ros-industrial/reach/issues/38>`_)
* Updated to later version of plugin loader (`#39 <https://github.com/ros-industrial/reach/issues/39>`_)
* Refactored repository into ROS-independent core package and ROS-based plugin implementation package (`#37 <https://github.com/ros-industrial/reach/issues/37>`_)
* Contributors: Michael Ripperger, Collin Thornton

0.2.2 (2022-08-24)
------------------

0.2.1 (2022-07-22)
------------------
* Fix setting of jacobian row subset (`#31 <https://github.com/marip8/reach/issues/31>`_)
* Contributors: Michael Ripperger

0.2.0 (2022-07-22)
------------------
* Various Updates (`#29 <https://github.com/marip8/reach/issues/29>`_)
  * Change FLANN search tree to PCL kd-tree; update nearest neighbors search; parallelize with hardware concurrency
  * Update to use non-deprecated IK solve function
  * Use Eigen vectorized array product rather than for loop
  * Support manipulability calculation for subset of dimensions
  * Added plugin for evaluating manipulability ratio
  * Simplified and vectorized joint penalty evaluator
  * Updated README
  * Updated unit test
* Contributors: Michael Ripperger

0.1.1 (2022-07-07)
------------------

0.1.0 (2022-07-05)
------------------
* Add CI, formatting (`#25 <https://github.com/marip8/reach/issues/25>`_)
  * Added formatting files
  * Run clang formatting
  * Run CMake format
  * Updated to c++14
  * Added CI jobs
* Merge pull request `#1 <https://github.com/marip8/reach/issues/1>`_ from marip8/feature/ci
  Continuous Integration
* Reorganized reach_core headers to allow easier install
* Updated packages CMakeLists
* Merge pull request `#10 <https://github.com/marip8/reach/issues/10>`_ from mripperger/feature/license
  License
* Added licenses to files
* Merge pull request `#9 <https://github.com/marip8/reach/issues/9>`_ from mripperger/update/clean-up
  Minor clean-up
* Updated Affine to Isometry
* Added missing headers
* Merge pull request `#8 <https://github.com/marip8/reach/issues/8>`_ from mripperger/feature/plugins
  Plugin Implementation
* Updated README documentation
* Updated IK solver plugins to implement new pure virtual function
* Updated plugins to use base class provided in reach_core package
* Renamed reach_plugins package to reach_ros
* Contributors: Michael Ripperger, Ripperger, Michael A, mripperger
