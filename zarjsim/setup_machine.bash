#!/bin/bash

# This is probably more controversial, but I found that
#  if I wanted to use both the tip version of srcsim
#  and the 'main' version, that I had to rename the original
if [ ! -d /opt/ros/indigo/share/srcsim.original ] ; then
    echo "Advanced concept:  if you want to be able to run both the 'official' srcsim"
    echo " as well as a version from source, you can do this."
    echo " It gets tricky; you end up needing to delete (and recreate) ~/.gazebo"
    echo "To do this: sudo mv /opt/ros/indigo/share/srcsim /opt/ros/indigo/share/srcsim.original"
    echo "Then the srcsim script will respect a 'original' or 'source' parameter"
    echo "--- or ---"
    echo "Alternate choice: an apt-get purge srcsim will remove srcsim from your system."
    echo "This means you can only run with source, but it does simplify things."
    #sudo mv /opt/ros/indigo/share/srcsim /opt/ros/indigo/share/srcsim.original
elif [ -d /opt/ros/indigo/share/srcsim ] ; then
    echo 'Hmm.  You have both /opt/ros/indigot/share/srcsim and /opt/ros/indigo/share/srcsim.original.'
    echo 'You probably updated your package.  I think it is best if you figure out what to do.'
    echo 'You can probably delete the /opt/ros/indigo/share/srcsim.original, and move the new srcsim'
    echo 'on top of it'.
fi


if [ ! -f /etc/security/limits.d/ros-nofile.conf ] ; then
    echo "Extending file number limits for ros user group.  See issue 134."
    sudo cp ros-nofile.conf /etc/security/limits.d/
fi


# I find it useful to change the launch process to not always delay for 15
#   seconds.  I use the init_robot.sh.loadtime.patch to allow a
#   ZARJ_LOAD_TIME variable to govern that.
grep -q ZARJ_LOAD_TIME /opt/ros/indigo/lib/srcsim/init_robot.sh
if [ $? -ne 0 ] ; then
    echo "I suggest patching init_robot.sh to reduce wait times."
    echo "It can make the slow world startup a little less aggravating."
    echo "The following command will do it."
    echo 'sudo patch -d /opt/ros/indigo/lib/srcsim -p1 < init_robot.sh.loadtime.patch'
    echo 'You can also apply that patch to the srcsim source directory '
fi

# Similarly, a unique.launch.allowgui.patch allows us to run this with nogui
grep -q gui /opt/ros/indigo/share/srcsim/launch/unique.launch
if [ $? -ne 0 ] ; then
    echo "You can patch unique.launch to allow you to disable the gui."
    echo "That's nice for batch runs."
    echo "The following command will do it."
    echo 'sudo patch -d /opt/ros/indigo/share/srcsim/launch -p1 < unique.launch.allowgui.patch'
fi

sudo apt-get install ros-indigo-tf2-geometry-msgs

# The full instructions for setting up srcsim from source, for me, were
#  as follows:
#    1.  clone srcsim to a directory parallel to zarj.
#    2.  cd srcsim
#    3.  hg update tip   (or whatever branch you crave)
#    4.  mkdir build
#    5.  cd build
#    6.  cmake .. && make

#echo 'Replacing valkyrie_sim_gazebo_sync.urdf from package val_description with a newer one.  See Issue 96.'
#echo 'Note: this was done with an update to val_description at the release of srcsim 0.4.0-2.'
#echo 'You probably do not need this...'
#echo 'Press ENTER to proceed, ^C to stop.'
#read 
#set -x
#
##For Issue #96
#if [ ! -f /opt/nasa/indigo/share/val_description/model/urdf/valkyrie_sim_gazebo_sync.urdf.original ] ; then
#    sudo cp /opt/nasa/indigo/share/val_description/model/urdf/valkyrie_sim_gazebo_sync.urdf /opt/nasa/indigo/share/val_description/model/urdf/valkyrie_sim_gazebo_sync.urdf.original
#fi
#sudo cp dependencies/valkyrie_sim_gazebo_sync.urdf /opt/nasa/indigo/share/val_description/model/urdf/valkyrie_sim_gazebo_sync.urdf
#
##For the eye_training worlds to load
#cp -r ../eye_training/models/* ~/.gazebo/models/
