#!/bin/bash

# ==== ROS 2 環境セットアップ ====
# source /opt/ros/humble/setup.bash
# 必要ならワークスペースも
# source ~/ros2_ws/install/setup.bash

# ROS_DOMAIN_ID を指定（必要に応じて変更）
# export ROS_DOMAIN_ID=0

# ==== rosbridge_server 起動チェック ====
# 既存プロセス検知
ROSBRIDGE_PID=$(pgrep -f rosbridge_websocket_launch.xml)

# ポート9090が使用中か確認
PORT_BUSY=$(lsof -i :9090 | grep LISTEN)

STARTED_ROSBRIDGE=0

if [ -n "$ROSBRIDGE_PID" ] || [ -n "$PORT_BUSY" ]; then
    echo "rosbridge_server is already running; using existing instance."
    if [ -n "$ROSBRIDGE_PID" ]; then
        echo "Existing PID: $ROSBRIDGE_PID"
    else
        echo "Port 9090 is in use by another process."
    fi
    STARTED_ROSBRIDGE=0
else
    echo "Starting rosbridge_server..."
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
    ROSBRIDGE_PID=$!
    STARTED_ROSBRIDGE=1
fi

# ==== Webサーバ起動（ポート8000） ====
python3 -m http.server 8000 &
HTTP_PID=$!

# ==== アクセスURL表示 ====
SERVER_IP=$(hostname -I | awk '{print $1}')
URL="http://${SERVER_IP}:8000/index.html"

echo "rosbridge_server PID: $ROSBRIDGE_PID"
echo "HTTP server PID: $HTTP_PID"
echo "Access the application at:"
echo "  $URL"
echo "(Open this URL in your browser from a PC that can reach this server.)"

# ==== Ctrl+C で終了処理 ====
trap "
    echo 'Stopping HTTP server...'
    kill $HTTP_PID
    if [ $STARTED_ROSBRIDGE -eq 1 ]; then
        echo 'Stopping rosbridge_server...'
        kill $ROSBRIDGE_PID
    else
        echo 'rosbridge_server was already running; leaving it running.'
    fi
    exit
" SIGINT SIGTERM

# ==== プロセス終了待ち ====
wait $HTTP_PID

