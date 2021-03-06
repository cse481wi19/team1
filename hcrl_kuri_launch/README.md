# hcrl_kuri_launch

Launch files for Kuri.

## Usage

To get a vanilla, controllable interface to the robot running

    roslaunch hcrl_kuri_launch kuri.launch

You can optionally specify a map file like this

    roslaunch hcrl_kuri_launch kuri.launch map_file:=/path/to/map.yaml

### Mapping Mode

Kuri's default navigation stack, OORT, comes with a custom SLAM implementation. To run the mapping process:

    roslaunch hcrl_kuri_launch kuri_mapping.launch

Take a closer look at `kuri_navigation` and the `mapping_controller` node to understand how to map.