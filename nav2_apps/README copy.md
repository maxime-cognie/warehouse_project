# nav2_apps

Package to interact with the nav2 API in a warehouse environment.

### Simulation

To start the navigation route for the simulation use the following command:

`ros2 launch nav2_apps nav2_apps_sim.launch.py`
`python3 nav2_apps/scripts/move_shelf_to_ship.py`


### Real robot

To start the navigation route for the real robot use the following command:

`ros2 launch nav2_apps nav2_apps_real.launch.py`
`python3 nav2_apps/scripts/move_shelf_to_ship_real.py`