rodi_gazebo
===========
[RoDI] (http://rodibot.com/) (Robot Didáctico Inalámbrico) Gazebo model.

A Gazebo model and plugin for the RoDI educational robot that allows to
control a simulated version of the robot by using its default firmware
web services API.

![rodi_gazebo_image](http://i.imgur.com/Y3JF6CB.jpg)

Dependencies
------------

The RoDI plugin depends on the [libmicrohttpd](https://www.gnu.org/software/libmicrohttpd/)
library, so it has to be installed to build and use the plugin.

Build
-----

The following steps build the Gazebo RoDI plugin:

$ git clone git://github.com/rodibot/rodi_gazebo.git

$ cd rodi_gazebo

$ pushd plugins/rodi

$ mkdir build && cd build

$ cmake ../ && make

$ source /usr/share/gazebo/setup.sh

$ export GAZEBO_PLUGIN_PATH=$(pwd):${GAZEBO_PLUGIN_PATH}

$ popd

$ export GAZEBO_MODEL_PATH=$(pwd)/models:${GAZEBO_MODEL_PATH}

Usage
-----

To start a simultion with a RoDI model and its plugin:

$ gazebo world/model_rodi.world

The TCP port can be changed using the ```<port/>``` tag in
models/rodi/model.sdf.

Author
------

Javier Martinez Canillas ( javier@dowhile0.org )
