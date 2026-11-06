#!/usr/bin/env bash
# 机载电脑（Jetson + ROS Noetic）首次部署脚本。
# 解决工程从 Windows 拷贝后出现的三类问题：
#   1. src/fuel 相对软链接损坏为普通文本文件；
#   2. .sh/.py 丢失 Linux 可执行权限；
#   3. build/devel 残留旧路径 /home/ubt/uav_explor 的编译产物。
# 用法：先把整个工程拷贝到家目录（如 ~/uav_explor_LTAOM），然后执行
#       bash onboard_setup.sh
set -euo pipefail

project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$project_root"

echo "[1/4] 检查软链接 src/fuel ..."
fuel_link="src/fuel"
fuel_target_rel="../third_party/FUEL/fuel_planner"
if [ -d "$fuel_link" ]; then
    echo "  $fuel_link 有效，跳过"
else
    echo "  $fuel_link 损坏或缺失（Windows 拷贝后常见），重建 -> $fuel_target_rel"
    rm -f "$fuel_link"
    ln -sfn "$fuel_target_rel" "$fuel_link"
fi
if [ ! -f "$fuel_link/plan_env/package.xml" ]; then
    echo "错误：$fuel_link 目标内容缺失（找不到 plan_env/package.xml）。" >&2
    echo "请确认 third_party/FUEL/fuel_planner 已完整拷贝后重试。" >&2
    exit 1
fi

echo "[2/4] 恢复脚本执行权限 ..."
chmod 0755 "$project_root"/*.sh
find src -type f \( -name "*.sh" -o -name "*.py" \) -exec chmod 0755 {} +

echo "[3/4] 清理旧路径编译产物（build/devel/install）..."
rm -rf build devel install

echo "[4/4] 全量编译（catkin_make，并行度 ${FUEL_BUILD_JOBS:-2}）..."
bash compile_fuel.sh

cat <<EOF

============================================================
部署完成。接下来执行：
  source source.sh
  echo \$ROS_PACKAGE_PATH
    # 第一个条目应为：$project_root/src
  bash handheld_map.sh session:=save
============================================================
EOF
