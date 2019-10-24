# Subt Comms Visibility Table DAT generation

## World Boundary

Before generating the .dat file, make sure that the tiles in the world are
within the bounds defined by the `kMin` and `kMax` constants in:

    <path_to_subt>/subt-communication/subt_gazebo_los_model/include/subt_gazebo_los_model/VisibilityTable.hh

If not, modify these variables so that they are large enough to cover the
entire world and rebuild.

## Generate the DAT file

Copy the .dot and .world file into `<path_to_subt>/subt_gazebo/worlds`
then rebuild + install.

Important: Make sure you have all the models referred to by the .world file in
your gazebo models path.

Launch world for generating the .dat file:

    roslaunch subt_gazebo_los_model visibility.launch scenario:=<world_name>

This should create a .dat file in the `<install_dir>/share/subt_gazebo/worlds/`
directory.

## Visualize comms

To test the world with dat file, launch subt:

    roslaunch subt_gazebo quickstart.launch scenario:=<world_name>

Publish a msg to enable comms visualization:

    ign service -s /subt/comms_model/visualize --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: "X1"' --force-version 4.0.0

This enables visualization of comms for X1 and you should see colored point
clouds in the tiles around X1, which represent the regions that X1's comms can
reach. Use the cost caculation method below to verify that comms created from
the dat file is correct.

### Cost caculation

Here are the costs along the edges between dfferent type of nodes (tiles) in
the dot graph:

```
s -> s:  1
s -> i:  3
i -> i:  6
```

where `i` is intersection, bend, vertical shaft and `s` is everything else. The
max cost available is 15.

### visualize dot file

    xdot <world_name>.dot

