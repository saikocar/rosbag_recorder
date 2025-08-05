#!/bin/bash

# rosbridge_server 起動
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!

# Webサーバ起動（ポート8000）
python3 -m http.server 8000 &
HTTP_PID=$!

URL="http://localhost:8000/index.html"

echo "rosbridge_server PID: $ROSBRIDGE_PID"
echo "HTTP server PID: $HTTP_PID"
echo "Opening browser at $URL"

# ブラウザを自動で開く（Linuxの場合）
if command -v xdg-open > /dev/null; then
    xdg-open "$URL"
# macOSの場合
elif command -v open > /dev/null; then
    open "$URL"
else
    echo "Please open your browser and visit $URL"
fi

# Ctrl+C で両方終了させたい場合の処理
trap "echo 'Stopping servers...'; kill $ROSBRIDGE_PID $HTTP_PID; exit" SIGINT SIGTERM

# プロセス終了待ち
wait $ROSBRIDGE_PID $HTTP_PID

