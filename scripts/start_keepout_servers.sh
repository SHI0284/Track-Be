set -e

source /opt/ros/jazzy/setup.bash

BASE_DIR="$HOME/ros2/Track-Be"
PARAMS_FILE="$BASE_DIR/config/keepout_servers.yaml"

echo "Starting keepout mask server..."
ros2 run nav2_map_server map_server \
  --ros-args \
  -r __node:=filter_mask_server \
  --params-file "$PARAMS_FILE" &

MASK_SERVER_PID=$!

echo "Starting costmap filter info server..."
ros2 run nav2_map_server costmap_filter_info_server \
  --ros-args \
  --params-file "$PARAMS_FILE" &

INFO_SERVER_PID=$!

sleep 2

echo "Configuring lifecycle nodes..."
ros2 lifecycle set /filter_mask_server configure || true
ros2 lifecycle set /costmap_filter_info_server configure || true

sleep 1

echo "Activating lifecycle nodes..."
ros2 lifecycle set /filter_mask_server activate || true
ros2 lifecycle set /costmap_filter_info_server activate || true

echo "Keepout servers are running."
echo "Press Ctrl+C to stop."

trap "kill $MASK_SERVER_PID $INFO_SERVER_PID" SIGINT SIGTERM

wait