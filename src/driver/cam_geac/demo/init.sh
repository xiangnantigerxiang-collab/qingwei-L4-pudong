#!/bin/bash

# 获取当前目录的父目录
parent_dir=$(dirname "$(pwd)")

# 追加lib/sensorlib到路径中
final_path="${parent_dir}/lib/sensorlib"

# 检查路径是否存在
if [ ! -d "$final_path" ]; then
    echo "Error: Directory $final_path does not exist."
    exit 1
fi

# 检查/etc/ld.so.conf中是否已经有未被注释的相同路径
if grep -q "^${final_path}$" /etc/ld.so.conf; then
    echo "The path $final_path is already correctly added to /etc/ld.so.conf."
    sudo ldconfig
    exit 0
fi

# 从/etc/ld.so.conf中找到所有包含lib/sensorlib的行并注释它们
sudo sed -i "s/^.*lib\/sensorlib.*/# &/g" /etc/ld.so.conf

# 将路径追加到/etc/ld.so.conf中
echo "$final_path" | sudo tee -a /etc/ld.so.conf > /dev/null

# 更新动态链接器的缓存
sudo ldconfig

geac_devices=(
    "/dev/ttyTHS*"
    "/dev/ttysWK*"
    "/dev/ttyAMA*"
    "/dev/ttyc0WK*"
    "/sys/class/tz_gpio/camera_*/value"
    "/sys/class/tz_gpio/dser_*/value"
    "/sys/class/rb_gpio/camera_*/value"
    "/sys/class/rb_gpio/dser_*/value"
    "/dev/gmslcam*"
    "/dev/video*"
)

for pattern in "${geac_devices[@]}"; do
    if ls $pattern 1>/dev/null 2>&1; then
        # 修改权限
        sudo chmod 777 $pattern
        sudo chown -R nvidia:nvidia $pattern
    fi
done

# 输出确认信息
echo "Path $final_path has been added to /etc/ld.so.conf and ldconfig cache updated."
