#!/bin/bash
# 使用方法
# ./serdes_tool.sh 功能编号 bus_num 串化器地址 解串器地址
# 如: ./serdes_tool.sh 1 30 0x40 0x48
# 含义: 检查bus为30上的地址为0x48地址的max9296状态 以及 地址为0x40的串化器状态

#功能1: 监测max9296状态
#功能2: 监测max96712状态


#监测max9296状态
max9296_status_check(){
    max9296_bus=$1
    serial_addr=$2
    max9296_addr=$3
    
    while true; do 
        clear

        echo -e "串化器是否收到数据: bit7"
        echo -n "pipe_x: "
        i2ctransfer -f -y $max9296_bus w2@$serial_addr 0x01 0x02 r1
        echo -n "pipe_y: "
        i2ctransfer -f -y $max9296_bus w2@$serial_addr 0x01 0x0a r1
        echo -n "pipe_z: "
        i2ctransfer -f -y $max9296_bus w2@$serial_addr 0x01 0x12 r1
        echo -n "pipe_u: "
        i2ctransfer -f -y $max9296_bus w2@$serial_addr 0x01 0x1a r1

        echo -e "\nmax9296是否收到数据: bit0"
        echo -n "pipe_x: "
        i2ctransfer -f -y $max9296_bus w2@$max9296_addr 0x01 0xDC r1
        echo -n "pipe_y: "
        i2ctransfer -f -y $max9296_bus w2@$max9296_addr 0x01 0xFC r1
        echo -n "pipe_z: "
        i2ctransfer -f -y $max9296_bus w2@$max9296_addr 0x02 0x1C r1
        echo -n "pipe_u: "
        i2ctransfer -f -y $max9296_bus w2@$max9296_addr 0x02 0x3C r1

        echo -e "\nmax9296是否开流: bit1"
        i2ctransfer -f -y $max9296_bus w2@$max9296_addr 0x03 0x13 r1

        sleep 1
    done
}

#监测max96712状态
max96712_status_check(){
    max96712_bus=$1
    serial_addr=$2
    max96712_addr=$3

    while true; do 
        clear

        echo -e "串化器是否收到数据: bit7"
        echo -n "pipe_x: "
        i2ctransfer -f -y $max96712_bus w2@$serial_addr 0x01 0x02 r1
        echo -n "pipe_y: "
        i2ctransfer -f -y $max96712_bus w2@$serial_addr 0x01 0x0a r1
        echo -n "pipe_z: "
        i2ctransfer -f -y $max96712_bus w2@$serial_addr 0x01 0x12 r1
        echo -n "pipe_u: "
        i2ctransfer -f -y $max96712_bus w2@$serial_addr 0x01 0x1a r1

        echo -e "\nmax96712是否收到数据: bit0"
        echo -n "pipe_x: "
        i2ctransfer -f -y $max96712_bus w2@$max96712_addr 0x01 0xDC r1
        echo -n "pipe_y: "
        i2ctransfer -f -y $max96712_bus w2@$max96712_addr 0x01 0xFC r1
        echo -n "pipe_z: "
        i2ctransfer -f -y $max96712_bus w2@$max96712_addr 0x02 0x1C r1
        echo -n "pipe_u: "
        i2ctransfer -f -y $max96712_bus w2@$max96712_addr 0x02 0x3C r1
        echo -n "pipe_x: "
        i2ctransfer -f -y $max96712_bus w2@$max96712_addr 0x02 0x5C r1
        echo -n "pipe_y: "
        i2ctransfer -f -y $max96712_bus w2@$max96712_addr 0x02 0x7C r1
        echo -n "pipe_z: "
        i2ctransfer -f -y $max96712_bus w2@$max96712_addr 0x02 0x9C r1
        echo -n "pipe_u: "
        i2ctransfer -f -y $max96712_bus w2@$max96712_addr 0x02 0xBC r1

        echo -e "\nmax96712是否开流: bit1"
        i2ctransfer -f -y $max96712_bus w2@$max96712_addr 0x04 0x0B r1
        sleep 1
    done
}

#显示帮助信息
display_helpinfo(){
    echo  -e "使用方法:
    格式: ./serdes_tool.sh \033[31m功能编号 bus_num 串化器地址 解串器地址\033[0m
    例如: ./serdes_tool.sh 1        30       0x40       0x48
    含义: 检查bus为30上的地址为0x48地址的max9296状态 以及 地址为0x40的串化器状态\n"

    echo "功能列表:
    功能1: 监测max9296 serdes状态
    功能2: 监测max96712 serdes状态"
}

bus_num="$2"
ser_addr="$3"
des_addr="$4"

case $1
    in "1")
        #功能1: 监测max9296 serdes状态
        max9296_status_check $bus_num $ser_addr $des_addr
        ;;
    "2")
        #功能3: 监测max96712 serdes状态
        max96712_status_check $bus_num $ser_addr $des_addr
        ;;
    "-h")
        #显示帮助信息
        display_helpinfo
        ;;
    *)
        echo "不支持的功能, 请重新输入!"
        exit
        ;;
esac
