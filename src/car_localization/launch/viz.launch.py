"""開一個 WebSocket 給 Foxglove 連, 這樣就能看地圖 / 雷射 / TF / 位姿。

wildbot 是用 foxglove_bridge (docker-compose_foxglove_bridge.yml) 或
rosbridge_server (docker-compose_rosbridge_server.yml) 做這件事。這個容器目前
**只有 rosbridge_server**, 所以預設走它; 之後如果把 foxglove_bridge 裝進映像檔
(Dockerfile 已經加了那一行, 重 build 就有), 這個 launch 會自動改用它。

    ros2 launch car_localization viz.launch.py

啟動後它會把要填進 Foxglove 的網址印出來。Foxglove Studio 選 "Open connection"
-> Rosbridge (或 Foxglove WebSocket) -> 貼上網址。

要看什麼:
    /map                        OccupancyGrid, 地圖 (定位模式載入的那張, 或建圖中的)
    /scan                       LaserScan, 運動補償後的一圈掃描
    /localization/pose          車子現在在哪
    /localization/scan_matched  配準後的點雲, 疊在地圖上看貼不貼
    /tf                         map -> base_link -> sim_lidar / sim_imu
    /cmd_vel                    Foxglove 的 Teleop 面板往這裡發, 就能用滑鼠開車
"""
import socket

from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _has(pkg):
    try:
        get_package_share_directory(pkg)
        return True
    except PackageNotFoundError:
        return False


def _addresses():
    out = []
    try:
        host = socket.gethostname()
        out.append(host)
        for info in socket.getaddrinfo(host, None, socket.AF_INET):
            ip = info[4][0]
            if ip not in out:
                out.append(ip)
    except OSError:
        pass
    return out


def generate_launch_description():
    use_foxglove = _has('foxglove_bridge')
    port = '8765' if use_foxglove else '9090'
    kind = 'Foxglove WebSocket' if use_foxglove else 'Rosbridge'
    addrs = _addresses()

    lines = [
        '',
        '=' * 68,
        f'  {kind} 已啟動 (port {port})',
        '  Foxglove Studio -> Open connection -> ' + kind + ' -> 貼上其中一個:',
    ]
    lines += [f'      ws://{a}:{port}' for a in addrs]
    lines += [
        '',
        '  連不上的話: run_isaac_gui.sh 沒有做 port mapping, 所以要嘛用上面的',
        '  容器 IP 直連 (Linux 上可以), 要嘛在 docker run 加 -p ' + port + ':' + port,
        '=' * 68,
        '',
    ]
    if not use_foxglove:
        lines.insert(-1, '  (裝了 foxglove_bridge 之後這個 launch 會自動改用它, 效能好很多)')

    args = [
        DeclareLaunchArgument('port', default_value=port),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    if use_foxglove:
        bridge = Node(
            package='foxglove_bridge', executable='foxglove_bridge',
            name='foxglove_bridge', output='screen',
            parameters=[{'port': LaunchConfiguration('port'),
                         'address': '0.0.0.0',
                         'use_sim_time': use_sim_time}])
    else:
        bridge = Node(
            package='rosbridge_server', executable='rosbridge_websocket',
            name='rosbridge_websocket', output='screen',
            parameters=[{'port': LaunchConfiguration('port'),
                         'address': '0.0.0.0',
                         'use_sim_time': use_sim_time}])

    # rosbridge 需要 rosapi 才能讓 Foxglove 列出 topic 清單
    extra = []
    if not use_foxglove and _has('rosapi'):
        extra.append(Node(package='rosapi', executable='rosapi_node', name='rosapi',
                          parameters=[{'use_sim_time': use_sim_time}]))

    return LaunchDescription(args + [LogInfo(msg='\n'.join(lines)), bridge] + extra)
