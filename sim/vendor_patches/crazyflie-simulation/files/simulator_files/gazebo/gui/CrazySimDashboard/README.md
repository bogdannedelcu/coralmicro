# CrazySim Dashboard

Small Gazebo GUI dashboard for CrazySim.

The plugin subscribes to a Gazebo Transport topic carrying `gz.msgs.StringMsg`
and displays the message in a floating panel. By default it listens on:

```text
/crazysim/dashboard
```

## Build

```bash
cd crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/gui/CrazySimDashboard
cmake -S . -B build
cmake --build build
```

Add the build directory to the Gazebo GUI plugin path before launching:

```bash
export GZ_GUI_PLUGIN_PATH=$PWD/build:${GZ_GUI_PLUGIN_PATH}
```

Then launch the GUI with the bundled dashboard config:

```bash
gz sim -g --gui-config $PWD/dashboard_gui.config
```

For the existing CrazySim launch scripts, start the server as usual and replace
the final `gz sim -g` with the command above, or run the command above in a
second terminal after the server has started.

## Load In A Gazebo GUI Config

Add this plugin block to the active Gazebo GUI config or world `<gui>` section:

```xml
<plugin filename="CrazySimDashboard" name="CrazySim dashboard">
  <gz-gui>
    <title>CrazySim</title>
    <property type="bool" key="showTitleBar">false</property>
    <property type="bool" key="resizable">false</property>
    <property type="double" key="height">150</property>
    <property type="double" key="width">300</property>
    <property type="double" key="z">10</property>
    <property type="string" key="state">floating</property>
    <anchors target="3D View">
      <line own="left" target="left"/>
      <line own="top" target="top"/>
    </anchors>
  </gz-gui>
  <topic>/crazysim/dashboard</topic>
</plugin>
```

## Publish Text

From a shell:

```bash
gz topic -t /crazysim/dashboard -m gz.msgs.StringMsg -p 'data: "mode=RPYT\nthrust=22000\npos=(0.00, 0.00, 0.45)"'
```

From C++, publish a `gz::msgs::StringMsg` on `/crazysim/dashboard`. Any external
controller can update it with state, FPS, coordinates, thrust, watchdog status,
or whatever is useful during a test.
