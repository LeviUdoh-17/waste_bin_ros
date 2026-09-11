# Maps

Saved occupancy grids for `laptop_nav_waypoints.launch.py`.

Record one with `laptop_slam_nav2.launch.py` (slam_toolbox mapping mode), keeping the
lid closed throughout so the lid-mounted LD19 stays level, then:

    ros2 run nav2_map_server map_saver_cli -f bin_floor

and drop the resulting `bin_floor.yaml` + `bin_floor.pgm` in this directory. That name
is the default for the launch file's `map:=` argument.
