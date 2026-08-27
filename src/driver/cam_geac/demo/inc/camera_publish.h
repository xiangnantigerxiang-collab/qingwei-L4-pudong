#pragma once
#include <chrono>
#include <memory>
#include <string>
#include <unistd.h>
#include <bits/stdc++.h>

#if (defined(SUPPORT_ROS1_JPG) || defined(SUPPORT_ROS1_YUV))
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/Bool.h"

/**
 * @class CCameraPublisher
 * @brief ROS1图像发布器类，用于发布压缩图像消息
 *
 * 此类封装了ROS1的压缩图像发布功能，简化了图像数据的发布过程
 */
class CCameraPublisher
{
public:
    /**
     * @brief CCameraPublisher 构造函数
     *
     * 使用给定的话题名称创建图像发布器
     *
     * @param name 话题名称（如"cam0/compressed"）
     */
    CCameraPublisher(std::string name);

    /**
     * @brief CCameraPublisher 类的析构函数
     *
     * 释放资源，清理ROS相关对象
     */
    ~CCameraPublisher();

    /**
     * @brief 发布压缩图像消息
     *
     * 将图像数据封装为ROS消息并发布到指定话题
     *
     * @param stTime 图像时间戳
     * @param pData 图像数据指针
     * @param nDatalen 图像数据长度
     * @return 成功发布返回true，否则返回false
     */
    bool Publisher(struct timespec stTime, unsigned char *pData, int nDatalen);

private:
    char m_szTopic[128];            ///< 话题名称
    ros::Publisher m_Publisher;     ///< ROS发布者对象
    ros::NodeHandle m_nh;           ///< ROS节点句柄
};

/**
 * @class CCameraStatusPublisher
 * @brief ROS1相机状态发布器类，用于发布相机数据状态
 *
 * 此类用于发布相机是否有数据的标志位，方便其他节点订阅判断相机状态
 */
class CCameraStatusPublisher
{
public:
    /**
     * @brief CCameraStatusPublisher 构造函数
     *
     * 使用给定的话题名称创建状态发布器
     *
     * @param name 话题名称（如"cam0/status"）
     */
    CCameraStatusPublisher(std::string name);

    /**
     * @brief CCameraStatusPublisher 类的析构函数
     *
     * 释放资源，清理ROS相关对象
     */
    ~CCameraStatusPublisher();

    /**
     * @brief 发布相机状态消息
     *
     * @param hasData true表示相机有数据，false表示无数据
     * @return 成功发布返回true，否则返回false
     */
    bool PublishStatus(bool hasData);

    /**
     * @brief 更新并发布相机数据状态
     *
     * 在接收到图像数据时调用，更新最后接收时间
     */
    void UpdateDataReceived();

    /**
     * @brief 检查相机是否超时（无数据）
     *
     * 根据最后接收数据的时间判断是否超时
     * @param timeoutMs 超时时间（毫秒），默认1000ms
     * @return true表示超时（无数据），false表示正常
     */
    bool IsTimeout(int timeoutMs = 1000);

private:
    char m_szTopic[128];            ///< 话题名称
    ros::Publisher m_Publisher;     ///< ROS发布者对象
    ros::NodeHandle m_nh;           ///< ROS节点句柄
    ros::Time m_lastDataTime;       ///< 最后接收数据的时间
    bool m_hasData;                 ///< 当前是否有数据标志
};
#endif

#if (defined(SUPPORT_ROS2_JPG) || defined(SUPPORT_ROS2_YUV))
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

class CCameraPublisher : public rclcpp::Node
{
public:
    /**
     * @brief CCameraPublisher 构造函数
     *
     * 使用给定的相机配置和发布器创建 CCameraPublisher 对象。
     *
     * @param camCfg 相机配置
     * @param publisher 发布器指针
     */
    CCameraPublisher(std::string name);

    /**
     * @brief CCameraPublisher 类的析构函数
     *
     * CCameraPublisher 类的析构函数，用于释放资源。
     */
    ~CCameraPublisher();

    bool Publisher(struct timespec stTime, unsigned char *pData, int nDatalen);

private:
    char m_szTopic[128];
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_Publisher;
};
#endif