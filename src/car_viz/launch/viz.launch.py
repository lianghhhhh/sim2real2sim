"""開一個 WebSocket 給 Foxglove 連, 這樣就能看地圖 / 雷射 / TF / 五條定位線。

    ros2 launch car_viz viz.launch.py
    ros2 launch car_viz viz.launch.py bridge:=rosbridge     # 強制用 rosbridge
    ros2 launch car_viz viz.launch.py compress:=false       # 不壓縮 /rgb (bridge 很可能會卡死)

啟動後它會把要填進 Foxglove 的網址印出來。Foxglove Studio 選 "Open connection"
-> Foxglove WebSocket (或 Rosbridge) -> 貼上網址。

bridge:=auto (預設) 的挑法: 有裝 foxglove_bridge 就用它 (port 8765, 點雲效能好
很多), 沒有就退回 rosbridge_server (port 9090, 會順便起 rosapi, Foxglove 才列得
出 topic 清單)。映像檔 `sim2real2sim:v1` 本身只有 rosbridge; Dockerfile 已經加了
foxglove-bridge, 重 build 之後就會自動改用它。

影像會順便壓成 <topic>/compressed (每個 topic 一個 image_compressor: 10 Hz、寬 960 px、
JPEG q70), **原始影像不經過 bridge** —— /rgb 跟 /camera_loc/detections (YOLO 標註圖)
都是 1920x1536 x 最多 50 Hz = 每秒 440 MB, Foxglove 裡點一下 bridge 就卡死。
Foxglove 的 Image 面板選 .../compressed 那個。

這個 package 只負責「開一個門」, 不起任何定位節點 —— 要看的東西自己另外開
(collect_all.launch.py、各條 xxx_loc.launch.py 都可以), 開了什麼就看得到什麼。

(以前這支 launch 放在已經刪掉的 car_localization 裡, 2026-09 搬到這裡。)
"""
import re
import socket

from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_PORT = {'foxglove': '8765', 'rosbridge': '9090'}


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


def _layout_path():
    try:
        return get_package_share_directory('car_viz') + '/config/foxglove_layout.json'
    except PackageNotFoundError:
        return 'src/car_viz/config/foxglove_layout.json'


def _setup(context):
    want = LaunchConfiguration('bridge').perform(context)
    if want == 'auto':
        kind = 'foxglove' if _has('foxglove_bridge') else 'rosbridge'
    elif want in DEFAULT_PORT:
        kind = want
    else:
        raise RuntimeError(f"bridge:={want} 不認得, 只能是 auto | foxglove | rosbridge")

    pkg = 'foxglove_bridge' if kind == 'foxglove' else 'rosbridge_server'
    if not _has(pkg):
        # 不擋: 讓 launch 自己報 package not found, 但先講清楚是哪裡缺
        hint = ('apt install ros-humble-foxglove-bridge' if kind == 'foxglove'
                else 'apt install ros-humble-rosbridge-suite')
        return [LogInfo(msg=f'!! 找不到 {pkg} —— {hint}, 或改用 bridge:=auto')]

    port = LaunchConfiguration('port').perform(context) or DEFAULT_PORT[kind]
    use_sim_time = LaunchConfiguration('use_sim_time')
    compress = LaunchConfiguration('compress').perform(context).lower() in ('true', '1', 'yes')
    image_topics = [t.strip() for t in LaunchConfiguration('image_topics').perform(context).split(',')
                    if t.strip()]
    compress = compress and bool(image_topics)
    label = 'Foxglove WebSocket' if kind == 'foxglove' else 'Rosbridge'

    lines = [
        '',
        '=' * 68,
        f'  {label} 已啟動 (port {port})',
        f'  Foxglove Studio -> Open connection -> {label} -> 貼上其中一個:',
    ]
    lines += [f'      ws://{a}:{port}' for a in _addresses()]
    if compress:
        lines += ['', '  影像: Image 面板選 .../compressed (原始影像太大, 不經過 bridge)']
        lines += [f'      {t} -> {t}/compressed' for t in image_topics]
        if kind == 'rosbridge':
            # rosbridge 沒有黑名單可以擋, 只能靠自己別去點原始的
            lines.append('  !! rosbridge 擋不掉原始影像, 在 Foxglove 裡千萬別點上面左邊那幾個')
    lines += [
        '',
        f'  現成版面: {_layout_path()}',
        '  (Foxglove -> Layout -> Import from file)',
        '',
        '  連不上的話: run_isaac_gui.sh 沒有做 port mapping, 所以要嘛用上面的',
        f'  容器 IP 直連 (Linux 上可以), 要嘛在 docker run 加 -p {port}:{port}',
        '=' * 68,
        '',
    ]
    if kind == 'rosbridge' and want == 'auto':
        lines.insert(-1, '  (裝了 foxglove_bridge 之後這個 launch 會自動改用它, 效能好很多)')

    params = [{'port': int(port), 'address': '0.0.0.0', 'use_sim_time': use_sim_time}]
    if kind == 'foxglove':
        fox = dict(params[0])
        if compress:
            # 白名單是 regex 全字比對 (std::regex, 支援 lookahead): 除了原始影像以外全放行。
            # 只擋這幾個 topic 本身, /rgb/compressed 之類的子 topic 不受影響。
            raw = '|'.join(re.escape(t) for t in image_topics)
            fox['topic_whitelist'] = [f'^(?!({raw})$).*']
        actions = [Node(package='foxglove_bridge', executable='foxglove_bridge',
                        name='foxglove_bridge', output='screen', parameters=[fox])]
    else:
        actions = [Node(package='rosbridge_server', executable='rosbridge_websocket',
                        name='rosbridge_websocket', output='screen', parameters=params)]
        # rosbridge 需要 rosapi 才能讓 Foxglove 列出 topic 清單
        if _has('rosapi'):
            actions.append(Node(package='rosapi', executable='rosapi_node', name='rosapi',
                                parameters=[{'use_sim_time': use_sim_time}]))
    if compress:
        common = {'max_rate': float(LaunchConfiguration('image_rate').perform(context)),
                  'max_width': int(LaunchConfiguration('image_width').perform(context)),
                  'jpeg_quality': int(LaunchConfiguration('jpeg_quality').perform(context)),
                  'use_sim_time': use_sim_time}
        # 每個 topic 一個節點, 各自「有人看才訂」; 節點名字要唯一, 從 topic 名字生
        for t in image_topics:
            actions.append(Node(
                package='car_viz', executable='image_compressor',
                name='image_compressor_' + re.sub(r'\W+', '_', t).strip('_'),
                output='screen', parameters=[dict(common, image_topic=t)]))
    return [LogInfo(msg='\n'.join(lines))] + actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bridge', default_value='auto',
                              description='auto | foxglove | rosbridge'),
        DeclareLaunchArgument('port', default_value='',
                              description='空 = foxglove 8765 / rosbridge 9090'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('compress', default_value='true',
                              description='把 image_topics 壓成 <topic>/compressed, 並把原始的擋在 bridge 外面'),
        DeclareLaunchArgument('image_topics', default_value='/rgb,/camera_loc/detections',
                              description='要壓的影像 topic, 逗號分隔'),
        DeclareLaunchArgument('image_rate', default_value='10.0', description='Hz, <= 0 不限頻'),
        DeclareLaunchArgument('image_width', default_value='960', description='px, <= 0 不縮圖'),
        DeclareLaunchArgument('jpeg_quality', default_value='70', description='1~100'),
        OpaqueFunction(function=_setup),
    ])
