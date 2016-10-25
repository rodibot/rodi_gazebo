rodi_gazebo
===========
[RoDI] (http://rodibot.com/) (Robot Didáctico Inalámbrico) Gazebo model.

Allows to control a RoDI from the Gazebo simulator using the robot's default
firmware web services API.

Dependencies
------------

The plugin depends on the libmicrohttpd library so it has to be installed.

Install
-------

To install the plugin follow these steps:

$ git clone git://github.com/martinezjavier/rodi_gazebo.git

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

The port can be chosen by changing the <port/> tag in
models/rodi/model.sdf.

Author
------

Javier Martinez Canillas <javier@dowhile0.org>
