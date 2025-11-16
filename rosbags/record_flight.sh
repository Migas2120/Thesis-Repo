#!/bin/bash
# record_flight.sh
# Record important MAVROS topics (ROS 1) with pseudo-wildcards

# Timestamped bag filename
BAGNAME="flight_$(date +%Y%m%d_%H%M%S).bag"

# Collect topics using grep (mimics wildcards)
/usr/bin/rostopic list | grep '^/mavros/local_position/' > /tmp/topics.txt
/usr/bin/rostopic list | grep '^/mavros/global_position/' >> /tmp/topics.txt
/usr/bin/rostopic list | grep '^/mavros/imu/' >> /tmp/topics.txt

# Add single topics explicitly
echo "/mavros/battery" >> /tmp/topics.txt
echo "/mavros/battery_status" >> /tmp/topics.txt
echo "/mavros/state" >> /tmp/topics.txt

echo "/mavros/local_position/pose" >> /tmp/topics.txt
echo "/mavros/local_position/velocity_local" >> /tmp/topics.txt
echo "/mavros/odometry/out" >> /tmp/topics.txt

echo "/mavros/global_position/global" >> /tmp/topics.txt
echo "/mavros/global_position/rel_alt" >> /tmp/topics.txt
echo "/mavros/imu/data" >> /tmp/topics.txt
echo "/mavros/imu/mag" >> /tmp/topics.txt   
echo "/mavros/imu/data_raw" >> /tmp/topics.txt

echo "/mavros/trajectory/path" >> /tmp/topics.txt


TOPICS=$(cat /tmp/topics.txt)

if [ -n "$1" ]; then
    echo "Recording MAVROS topics for $1 seconds into $BAGNAME ..."
    rosbag record -O "$BAGNAME" --duration="$1" $TOPICS
else
    echo "Recording MAVROS topics into $BAGNAME (Ctrl+C to stop) ..."
    rosbag record -O "$BAGNAME" $TOPICS
fi
