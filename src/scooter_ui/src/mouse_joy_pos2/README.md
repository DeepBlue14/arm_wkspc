
```
ls /dev/input
xinput list
xinput set-prop 9 "Device Enabled" 0 #where 9 is the device ID
rosrun mouse_joy_pos2 traxsys-c
```

```
xinput set-prop 9 "Device Enabled" 1
```