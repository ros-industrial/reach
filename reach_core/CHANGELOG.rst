^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reach_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Create nested results directories if they do not exist (`#34 <https://github.com/marip8/reach/issues/34>`_)
* Contributors: Michael Ripperger

0.2.1 (2022-07-22)
------------------

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
* Incorporated heat map coloring to rviz markers (`#30 <https://github.com/marip8/reach/issues/30>`_)
  * Incorporated heat map coloring to rviz markers
  * Fixed formatting on heat map rviz marker code
  * Minor updates
  Co-authored-by: David Spielman <david.spielman@swri.org>
  Co-authored-by: Michael Ripperger <michael.ripperger@swri.org>
* Contributors: David Spielman, Michael Ripperger

0.1.1 (2022-07-07)
------------------

0.1.0 (2022-07-05)
------------------
* Reverted reach_core version before releasing packages
* Fixed bug in reach_core/launch/setup.launch when starting the robot_state_publisher node (`#26 <https://github.com/marip8/reach/issues/26>`_)
* Add CI, formatting (`#25 <https://github.com/marip8/reach/issues/25>`_)
  * Added formatting files
  * Run clang formatting
  * Run CMake format
  * Updated to c++14
  * Added CI jobs
* Remove moveit_core dependency (`#20 <https://github.com/marip8/reach/issues/20>`_)
  * Add moveit_core dependency
  * Remove moveit_core dep and header ref
* Plugin Unit Test (`#16 <https://github.com/marip8/reach/issues/16>`_)
  * Added unit test to check loading of plugins
  * Updated to rostest
* Update plugin description with new library name (`#15 <https://github.com/marip8/reach/issues/15>`_)
* Change the fixed frame in rviz (`#14 <https://github.com/marip8/reach/issues/14>`_)
* Revise Point Cloud Loading Service (`#12 <https://github.com/marip8/reach/issues/12>`_)
  * Updated service for loading point clouds
  * Revised and renamed server for loading point clouds
  * Updated handling of point cloud loading service
  * Updated launch file
  * Consolidated libraries
* Added install rule for config directory (`#11 <https://github.com/marip8/reach/issues/11>`_)
* Demo Update (`#8 <https://github.com/marip8/reach/issues/8>`_)
  * Added robot model to demo
  * Added unit test for demo
  * Updated install rules to install launch files in a 'launch' directory
  * Added joint limits file and reorganized configuration files
  * Added test dependencies to package.xml
* Merge pull request `#3 <https://github.com/marip8/reach/issues/3>`_ from marip8/feature/demo
  Reach Study Demo
* Updated start launch file to load YAML files with substitution
* Merge pull request `#2 <https://github.com/marip8/reach/issues/2>`_ from marip8/fix/launch_file
  Launch File Update
* Update to startup launch file argument loading
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
* Updated reach_core launch and config files
* Removed loading of kinematic group parameter in reach study node
* Moved multiplicative factory plugin to new directory
* Updated name of display base class in reach study instantiation of plugins
* Updated reach study components to accommodate updates to reach record definition
* Moved reach record generation utility and fixed for updated reach record message definition
* Added new pure virtual method to IK solver base class to return joint names
* Updated reach_core components to use base class defined within the package
* Moved multiplicative evaluation factory plugin into reach_core package
* Build file clean up for reach_core package
* Moved base class headers into reach_core package
* Updated calls to display update robot pose in reach visualizer
* Updated interactive marker callbacks to use bind
* Updated .rviz file
* Removed results directory
* Updated example reach study object configuration file
* Updated build files
* Updated reach study node to load new parameters
* Implemented plugins and changes to other core components in main reach study library
* Updated study parameters structure
* Revised reach visualizer class and removed functionality relocated to plugin class
* Added method to database class for writing to database message
* Added method to general utilties for creating from from point and normal
* Removed kinematic utilities
* Moved code out of IK helper class to be put into IK plugin; changed neighbor IK search to free functions
* Merge pull request `#5 <https://github.com/marip8/reach/issues/5>`_ from mripperger/feature/seed_states
  Added support for specifying seed states from which to solve IK
* Added support for specifying seed states from which to solve IK
* Merge pull request `#7 <https://github.com/marip8/reach/issues/7>`_ from mripperger/update/cleanup
  Removed deprecated code
* Removed deprecated code
* Merge pull request `#6 <https://github.com/marip8/reach/issues/6>`_ from mripperger/reorganization
  Repository reorganization
* Reorganized into metapackage with reach_core and reach_msgs packages; fixes to code to accommodate changes
* Contributors: AndyZe, Collin Thornton, Michael Ripperger, Nathan Brooks, Ripperger, Michael A, mripperger
