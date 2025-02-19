#!/bin/bash

# 尝试停止并重启守护进程
if ! ibus-daemon -r -d -x; then
    echo "Error: Failed to restart ibus-daemon" >&2
    exit 1
fi

sleep 2

# 确保服务重启
if ! ibus restart; then
    echo "Error: Failed to restart ibus service" >&2
    exit 1
fi

echo "IBus restarted successfully"
exit 0
