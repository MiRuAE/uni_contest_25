# 2025 University Autonomous driving challenge.
**Team ChaChaPing**

## How to start?
```bash
git clone https://github.com/miru-ae/uni_contest_25.git && cd uni_contest_25
```

## Build & Launch 
### Build
```bash 
colcon build --symlink-install 
```
### F1Tenth Stack
```
ros2 launch f1tenth_stack bringup_launch.py
```
### Launcher 
```
 ros2 launch launcher_pkg mk_launch.py
```

### Camera Exposure setup
#### Complete bright sunlight setup
v4l2-ctl -d /dev/video0 -c auto_exposure=1,exposure_time_absolute=3,gain=5,white_balance_automatic=0,white_balance_temperature=5500,backlight_compensation=0

