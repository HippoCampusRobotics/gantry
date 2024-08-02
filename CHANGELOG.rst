^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gantry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* emulate tty to get colored output
* added another missing dependency on ament_index_cpp
* added ament_index_cpp to dependencies, since it is used
* fixed formatting
* build motor_component as component. fixed typo
* fully migrated from hippo_msgs to gantry_msgs
* reset waypoint index when stopping
* fix waypoint index in log message
* add handling of last waypoint
* Merge pull request `#1 <https://github.com/HippoCampusRobotics/gantry/issues/1>`_ from HippoCampusRobotics/devel-waypoint-grid-control
  Devel waypoint grid control
* some old bugfixes
* fixed accidental overwrite of reset max speed service
* Regularly clear io buffer to remove any residual data
  that might cause problems when parsing new requests
* add estimation of time needed
* working in theory - now needs to be tested with real gantry
* fix lambda in for-loop for passing extra argument to subscriber callback
* first things working
* added reset services
* first commit
* fixed wrong eigen dependency
* workaround for deceleration only available on newer MCs
* do not stop gantry while homing due to velocity timeouts
* fixed unintended sharing of variables between object instances
* changed velocity computation
* updated defaults to more reasonable values
* added correct unit transformation for accelerations
* added config for default values
* fixed wrong clock source
* added new services
  also compute velocity instead of using the motion controller data
* updated motor config
* added velocity setpoint time
* update path_follower. also added marker publisher
* implemented time out for velocity setpoints
* updated path_follower
* prevent timer callbacks from queueing up
* make sure everything has been transmitted before exiting.
  This should prevent the buffer from filling up if things are written
  faster than they can be transmitted
* protect callbacks by a mutex
  eventsexecutor is only a single thread. but the workaround to not queue
  up timer events is to have a separate timer manager thread for the
  events executor. To avoid parallel execution and possible race
  conditions, every callback needs to be locked by a mutex to manually
  achieve mutually exlusive callbacks.
* consider slow motor response speed in default configs
* added missing move to position command
* Call init services
* added basic services
* updated motor interface
* make update period parametric
* added xyz launch file
* added xyz manually composed node
* added motor config files
* implemented subscriptions/-callbacks and run loop
* removed non-common commands
* add timeout for canonical read via poll
* fixed param typo
* catch errors during parsing
* fixed flags
* non blocking readline
* run timers
* fixed misplaced bracket
* enable canonical mode
* fixed wrong format specifier
* fixed errenously inverted boolean expression
* initial commit
* Contributors: NBauschmann, Nathalie Bauschmann, Thies Lennart Alff
