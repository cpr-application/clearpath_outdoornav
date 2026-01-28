^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_dock_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

0.9.5 (2023-11-03)
------------------
* Merge remote-tracking branch 'origin/noetic-devel' into ONAV-1771/on_start_stop_tasks
* Contributors: Stephen Phillips

0.9.4 (2023-10-20)
------------------

0.9.3 (2023-10-19)
------------------

0.9.2 (2023-10-17)
------------------
* Use raw JSON strings for import & export; don't bother gzipping & base-64 encoding them
* Add import/export services for docking
* Fix over & underline length in changelogs
* Contributors: Chris Iverach-Brereton

0.9.1 (2023-08-22)
------------------
* Add message field to service response
* Added error message to service response
* Contributors: José Mastrangelo

0.9.0 (2023-07-05)
------------------
* Initial release
