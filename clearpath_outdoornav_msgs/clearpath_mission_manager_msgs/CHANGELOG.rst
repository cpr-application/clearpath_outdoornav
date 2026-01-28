^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_mission_manager_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.9 (2024-03-14)
------------------
* Message updates for onav-0.12 (`#34 <https://github.com/clearpathrobotics/clearpath_msgs/issues/34>`_)
  * Add the size_exceeded field to NetworkMapState
  * Mod: Update dock messages for new 0.12 docking APIs
  * Mod: Add new autonomy API types to AutonomyStatus.msg
  * Mod: Use float64 version of Vector2D for returning path recording lat-lons
  * Mod: Add header field to RunNetworkGoToPlanner.action feedback
  ---------
  Co-authored-by: Chris Iverach-Brereton <civerachb@clearpathrobotics.com>
* Contributors: stephen-cpr

0.9.8 (2024-03-11)
------------------
* Add additional messages for Network of Paths
* Contributors: Chris Iverach-Brereton <civerachb@clearpathrobotics.com>, Stephen Phillips <stphillips@clearpathrobotics.com>

0.9.7 (2024-02-05)
------------------

0.9.6 (2023-12-21)
------------------
* Add a new service for deleting multiple IDs at once
* Add a new service to import entire mission objects, including their associated tasks & waypoints
* Contributors: Chris Iverach-Brereton

0.9.5 (2023-11-03)
------------------
* [ONAV-1772] Add 'allow_failure' boolean to Task message type
  - When set to True, any missions that execute this task will be allowed to continue if the task fails
* Merge pull request `#18 <https://github.com/clearpathrobotics/clearpath_msgs/issues/18>`_ from stephen-cpr/ONAV-1771/on_start_stop_tasks
  [ONAV-1771] Add support for on_start / on_stop Mission Tasks
* Merge remote-tracking branch 'origin/noetic-devel' into ONAV-1771/on_start_stop_tasks
* [ONAV-1771] Add support for on_start / on_stop Mission Tasks
  - clearpath_navigation_msgs/Mission.msg has been updated to include 'on_start' and 'on_stop' Task arrays which are executed when the mission begins and ends respectively (on_stop Tasks will execute regardless of mission failure or success)
  - The CreateTask service has been updated to include 'assign_on_start' and 'assign_on_stop' fields (similar functionality as the 'assign_to' field)
  - The 'assign_to' field of the CreateTask service has been updated to 'assign_to_wp' to distinguish it from assign_on_start/assign_on_stop
  - Added the following service endpoints to mission_manager (using the AddRemoveById.srv type):
  - ~add_task_to_start & ~add_task_to_stop - Adds a task to the on_start/on_stop array for given mission
  - ~remove_task_from_start & ~remove_task_from_stop - Removes a task from the on_start/on_stop array for given mission
* Contributors: Stephen Phillips, Tony Baltovski

0.9.4 (2023-10-20)
------------------

0.9.3 (2023-10-19)
------------------

0.9.2 (2023-10-17)
------------------
* Use raw JSON strings for import & export; don't bother gzipping & base-64 encoding them
* Fix over & underline length in changelogs
* Contributors: Chris Iverach-Brereton

0.9.1 (2023-08-22)
------------------

0.9.0 (2023-07-05)
------------------
* Initial release
