#include "vin_log.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <string.h>
#include <errno.h>
#include <vector>

// 定义I2C操作结构体
typedef struct {
    int bus_num;          // I2C总线号
    int dev_addr;         // 设备地址（十进制）
    int fd;               // I2C设备文件描述符
} I2COperations;

typedef enum {
    DATA_TYPE_RGB = 0,   // 写入RGB寄存器组
    DATA_TYPE_DEPTH_OLD = 1,  // 写入Depth寄存器组（老版本）
    DATA_TYPE_DEPTH_NEW = 2  // 写入Depth寄存器组（新版本）
} DataType;

typedef enum {
    DEVICE_TYPE_D405 = 0,
    DEVICE_TYPE_D415 = 1,
    DEVICE_TYPE_D457 = 2
} DeviceType;

// I2C操作结构体创建/销毁
I2COperations* i2c_ops_create(int bus_num, int dev_addr);
void i2c_ops_destroy(I2COperations* ops);

// I2C设备初始化
int i2c_init(I2COperations* ops);

// 底层I2C读写操作
int i2c_write(I2COperations* ops, const uint8_t* data, int data_len);
int i2c_read(I2COperations* ops, const uint8_t* reg_addr, int reg_len, 
             uint8_t* read_data, int read_len);

// 业务步骤函数
int write_26_bytes(I2COperations* ops);
int write_4_bytes(I2COperations* ops);
int loop_read_2_bytes(I2COperations* ops);
int read_2_bytes_and_convert(I2COperations* ops);
uint8_t* read_specified_length(I2COperations* ops, int length, int* out_len);
// uint8_t* execute_all_operations(I2COperations* ops, int* out_len);
uint8_t* execute_all_operations(I2COperations* ops, DataType data_type, int* out_len);
bool set_high_accuracy_mode(I2COperations* ops, DeviceType device_type);
