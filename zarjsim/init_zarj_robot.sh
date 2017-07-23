#!/bin/bash

function is_robot_z() {
    z=`rostopic echo -n 1 /ihmc_ros/valkyrie/output/robot_pose | grep 'z:' | head -1 | sed -e 's/.*z: //'`
    echo "Robot z: $z"
    c=`echo "$z <= $1" | bc`
    if [ "$c" -eq 1 ] ; then
        return 0
    else
        return 1
    fi
}

#
# Running this at the same time as all the other tasks
#  appears to increase the chance of other crashes.
# So let's just chill
echo "Sleeping for 10 seconds"
sleep 10

x=0
echo "Waiting up to 30 seconds for topics to appear"
while [ $x -lt 30 ] ; do
    x=$((x+1))
    rostopic list 2>/dev/null | grep -q '/clock' 
    if [ $? -eq 0 ] ; then
        rostopic list 2>/dev/null | grep -q robot_pose && break
    fi
    sleep 1
done

echo "Waiting up to 30 seconds for the robot to reach 1.10"
y=$((x+30))
while [ $x -lt $y ] ; do
    x=$((x+1))
    is_robot_z 1.10 && break
    sleep 1
done

if [ $x -lt $y ] ; then
    echo "Robot reached 1.10"
else
    echo "Error: Robot never reached 1.10"
fi


echo "Lower harness"
rostopic pub -1 /valkyrie/harness/velocity std_msgs/Float32 '{data: -0.05}'

echo "Waiting up to 30 seconds for the robot to reach 1.03"
y=$((x+30))
while [ $x -lt $y ] ; do
    x=$((x+1))
    is_robot_z 1.03 && break
    sleep 1
done

if [ $x -lt $y ] ; then
    echo "Robot reached 1.03"
else
    echo "Error: Robot never reached 1.03"
fi

echo "Enabling low level control mode 2; detaching"
rostopic pub -1 /ihmc_ros/valkyrie/control/low_level_control_mode ihmc_valkyrie_ros/ValkyrieLowLevelControlModeRosMessage '{requested_control_mode: 2, unique_id: -1}'
sleep 3
rostopic pub -1 /valkyrie/harness/detach std_msgs/Bool true &
sleep 3

echo "Waiting up to 30 seconds for the robot to get back to 0.985"
y=$((x+30))
while [ $x -lt $y ] ; do
    x=$((x+1))
    is_robot_z 0.985 || break
    sleep 1
done

if [ $x -lt $y ] ; then
    echo "Robot made it back to 0.985"
else
    echo "Error: Robot never made it back to 0.985"
fi


echo "Init done; robot ready for commands"
[ -n "$BATCH_SIGNAL_DIR" ] && touch $BATCH_SIGNAL_DIR/simup.signal
exit 0
