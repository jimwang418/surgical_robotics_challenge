From `surgical_robotics_challenge` directory, to launch AMBF simulation and CRTK interface:

```
source ~/ambf/build/devel/setup.bash
ambf_simulator --launch_file launch.yaml -l 3,4,16,22 --override_max_comm_freq 120 -p 120 -t 1 --plugins ~/ambf_crtk_plugin/build/libambf_crtk_plugin.so --conf crtk_config.yaml
```

To run the dVRK console:

```
rosrun dvrk_robot dvrk_console_json -j console-SUJ-ECM-MTMR-PSM1-MTML-PSM2-Teleop.json -p 0.005
```

To start the camera:
```
roslaunch dvrk_video jhu_daVinci_video.launch 
```