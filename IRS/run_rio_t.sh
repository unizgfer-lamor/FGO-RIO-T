#!/bin/bash


unzip -o Bags.zip

# List of bag files
bags=("gym" "mocap_dark"  "mocap_difficult" "mocap_easy" "mocap_medium")

# Create output folders
mkdir -p RIO-T
mkdir -p RIO-T/dt

for bag in "${bags[@]}"; do
    echo "Processing bag: $bag.bag"

    # Launch ROS node in background
    roslaunch fgo_rio_t irs.launch > RIO-T/${bag}_launch.log 2>&1 &
    LAUNCH_PID=$!

    # Wait 1 second for node initialization
    sleep 1

    # Start recording /odometry in background
    rostopic echo -p /odometry > RIO-T/${bag}.txt &
    RECORD_ODOM_PID=$!

    sleep 1

    # Start recording /dt in background
    rostopic echo -p /dt > RIO-T/dt/${bag}_dt.txt &
    RECORD_DT_PID=$!

    sleep 1

    # Start rosbag play in background
    rosbag play --clock ${bag}.bag > RIO-T/${bag}_rosbag.log 2>&1 &
    BAG_PID=$!

    echo "Running processes for $bag.bag... waiting 250 seconds"
    sleep 260

    echo "Time's up for $bag.bag. Stopping processes..."

    # Safely kill background processes if they are still running
    for pid in $BAG_PID $RECORD_ODOM_PID $RECORD_DT_PID; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid"
        fi
    done

    sleep 3

    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill "$LAUNCH_PID"
    fi

    sleep 5
done

echo "Cleaning data files with clean_files_rio_t.py..."
python3 clean_files_rio_t.py


sleep 2
echo "All bags processed!"

echo "Files that WOULD BE DELETED from the top level of RIO-T (dt/ folder and its contents will NOT be touched):"
find RIO-T/ -maxdepth 1 -type f -not -name "*_rio-t.txt" -print

# To actually delete them:
echo "Deleting specified files from the top level of RIO-T..."
find RIO-T/ -maxdepth 1 -type f -not -name "*_rio-t.txt" -delete



