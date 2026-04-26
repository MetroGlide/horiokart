

### Odom offset republisher
If odom could not reset to zero when robot logging mapping rosbag, you can use this node to republish odom with offset. 

```
ros2 bag play --clock <bagfile> --remap /odom:=/odom_raw
```

```
ros2 run horiokart_drivers odom_offset_republisher.py
```

### Gps transform node
This node will transform gps data from latitude, longitude to map coordinate. 

```
```