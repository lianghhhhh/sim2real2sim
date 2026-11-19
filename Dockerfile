# sim2real2sim:v1 (ROS/YOLO/定位, CUDA torch) + mpc_isaac_image 的 MPC 工具 (acados, casadi, l4casadi, sklearn)
# + Nav2 規劃器 (nav_mpc_pkg, 2026-11-18 加; 推上去的 v2 還沒有, 要重 build)
#
# build (在 repo 根目錄):
#   docker build -t sim2real2sim:combined .
#   (這份沒有從 build context 複製檔案, BuildKit 不會把 car_run_data 之類的大資料夾傳進去)
# 發布 (run_isaac_mpc.sh / run_car.sh 等預設用這個):
#   docker tag sim2real2sim:combined registry.screamtrumpet.csie.ncku.edu.tw/liang/sim2real2sim:v3
#   docker push registry.screamtrumpet.csie.ncku.edu.tw/liang/sim2real2sim:v3
# BASE 必須留在 v1 (v2 就是從這裡 build 出來的)。
#
# 兩個原 image 都是 Ubuntu 22.04 + ROS humble + Python 3.10, 差別只在 MPC 工具。合併時要處理:
#   - numpy: 這裡沿用 sim2real2sim 的 numpy<2 (cv_bridge 需要), 原 MPC image 是 2.2.6
#   - torch: 沿用 sim2real2sim 的 CUDA torch。CUDA 版 torch 的 CMake 設定要求 CUDA toolkit (nvcc),
#     image 裡沒有, 所以 l4casadi 在第一段用「同版本的 CPU torch」編成 wheel, 再裝到 CUDA torch 旁邊
#     (同版本的 C++ API/ABI 相同)。
#   - casadi 3.7.2 / sklearn 1.7.2 / acados commit 釘在原 MPC image 的版本
#   - pandas 2.3.3 (安裝那行沒鎖版本, numpy<2 的限制下 pip 選到的; 推上去的 v2 就是這版)
#
# old_docker/Dockerfile.old 是 build v1 的那份 (BASE 的來源)。

ARG BASE=registry.screamtrumpet.csie.ncku.edu.tw/screamlab/sim2real2sim:v1

# ---------- stage 1: 編 l4casadi wheel ----------
FROM ${BASE} AS l4casadi-build
ARG THREADS=8
ARG L4CASADI_BRANCH=v2.0.0-release
SHELL ["/bin/bash", "-c"]

RUN TORCH_VER=$(python3 -c "import torch; print(torch.__version__.split('+')[0])") && \
    echo "build l4casadi against torch ${TORCH_VER}+cpu" && \
    pip install --no-cache-dir --force-reinstall --no-deps \
        "torch==${TORCH_VER}+cpu" --index-url https://download.pytorch.org/whl/cpu && \
    pip install --no-cache-dir "cmake>=3.27,<4.0" "scikit-build>=0.17" "ninja>=1.11" && \
    git clone --depth 1 --branch ${L4CASADI_BRANCH} \
        https://github.com/Tim-Salzmann/l4casadi.git /tmp/l4casadi && \
    CMAKE_BUILD_PARALLEL_LEVEL=${THREADS} \
        pip wheel --no-cache-dir --no-deps --no-build-isolation -w /wheels /tmp/l4casadi

# ---------- stage 2: 最終 image ----------
FROM ${BASE}
ARG THREADS=8
ARG ACADOS_COMMIT=374f57234eec1daf517a34c461f25f07faf9f6be
ARG TERA_VERSION=0.2.0
SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install -y --no-install-recommends cmake libblas-dev liblapack-dev && \
    rm -rf /var/lib/apt/lists/* && apt-get clean

# Nav2 的規劃器 + costmap (src/nav_mpc_pkg 用; 控制器是我們的 MPC, 所以不裝整套 navigation2)。
# v1 只有 nav2_map_server / nav2_msgs / nav2_util。實體車的 kros-car-new (Jazzy) 已經有完整 Nav2。
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-nav2-planner ros-humble-nav2-smac-planner ros-humble-nav2-navfn-planner \
        ros-humble-nav2-costmap-2d ros-humble-nav2-lifecycle-manager ros-humble-nav2-map-server && \
    rm -rf /var/lib/apt/lists/* && apt-get clean

# acados (與原 MPC image 同一個 commit)
RUN git clone https://github.com/acados/acados.git /acados && \
    cd /acados && git checkout ${ACADOS_COMMIT} && \
    git submodule update --init --recursive && \
    mkdir -p build && cd build && \
    cmake .. -DACADOS_WITH_OPENMP=ON && \
    make install -j${THREADS}
ENV ACADOS_SOURCE_DIR=/acados
ENV LD_LIBRARY_PATH=/acados/lib

# acados 產生 solver 要用的 tera renderer (原 MPC image 沒裝到, 第一次 build solver 會問要不要下載)
RUN mkdir -p /acados/bin && \
    curl -fsSL -o /acados/bin/t_renderer \
        https://github.com/acados/tera_renderer/releases/download/v${TERA_VERSION}/t_renderer-v${TERA_VERSION}-linux-amd64 && \
    chmod +x /acados/bin/t_renderer

COPY --from=l4casadi-build /wheels /tmp/wheels
RUN pip install --no-cache-dir casadi==3.7.2 scikit-learn==1.7.2 pandas && \
    pip install --no-cache-dir --no-deps /tmp/wheels/l4casadi-*.whl && \
    pip install --no-cache-dir /acados/interfaces/acados_template && \
    pip install --no-cache-dir --force-reinstall "setuptools<70.0.0" && \
    rm -rf /tmp/wheels

ENV MPC_CONFIG_PATH=/workspaces/model/mpc_config.json

WORKDIR /workspaces
ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD ["/bin/bash", "-l"]
