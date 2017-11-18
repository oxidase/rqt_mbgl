# Building and setup

## Build mapbox-gl-native

Required development packages and setting up of Mapbox GL Native is described at https://github.com/mapbox/mapbox-gl-native/blob/master/INSTALL.md

The plugin requires Mapbox Qt SDK that can be built with these steps
```
sudo apt-get install qt5-default
cd $HOME
git clone https://github.com/mapbox/mapbox-gl-native
cd mapbox-gl-native
mkdir build && cd build
cmake .. -DMBGL_PLATFORM=qt
make -j4
```

## Build rqt_mbgl

In the ROS workspace directory
```
cd src
git clone https://github.com/oxidase/rqt_mbgl
cd ..
catkin_make --force-cmake --cmake-args -DMAPBOX_GL_PATH=$HOME/mapbox-gl-native
```


## Setup access to vector tiles

To use the plugin you need to [sign up to Mapbox](https://www.mapbox.com/signup/),
got to your [API access tokens page](https://www.mapbox.com/studio/account/tokens/)
and set the environment variable `MAPBOX_ACCESS_TOKEN` to your [Mapbox access token](https://www.mapbox.com/help/how-access-tokens-work/):

```
export MAPBOX_ACCESS_TOKEN=MYTOKEN
rqt --force-discover
```

Start plugin  `Visualization` → `Mapbox GL Native`
