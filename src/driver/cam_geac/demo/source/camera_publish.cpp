#if (defined(SUPPORT_ROS1_JPG) || defined(SUPPORT_ROS1_YUV))
#include "../inc/camera_publish.h"

/**
 * @brief CCameraPublisher构造函数
 * 
 * 初始化ROS节点并创建图像发布者
 * 
 * @param name 话题名称
 */
CCameraPublisher::CCameraPublisher(std::string name)
{
    // 保存话题名称
    strncpy(m_szTopic, name.c_str(), sizeof(m_szTopic));
    m_szTopic[sizeof(m_szTopic) - 1] = '\0'; // 确保字符串终止
    
    // 创建CompressedImage类型发布者，队列大小为2
    m_Publisher = m_nh.advertise<sensor_msgs::CompressedImage>(name, 2);
}

/**
 * @brief CCameraPublisher析构函数
 * 
 * 清理资源，ROS1会自动管理大部分资源
 */
CCameraPublisher::~CCameraPublisher()
{
    // 析构函数（当前无资源需要显式释放，ROS1资源会自动管理）
}

/**
 * @brief 发布压缩图像消息
 * 
 * 将图像数据封装为ROS消息并发布
 * 
 * @param stTime 图像时间戳
 * @param pData 图像数据指针
 * @param nDatalen 图像数据长度
 * @return 总是返回true表示成功
 */
bool CCameraPublisher::Publisher(struct timespec stTime, unsigned char *pData, int nDatalen)
{
    // 检查数据有效性
    if (pData == nullptr || nDatalen <= 0) {
        // ROS_WARN("Invalid image data: null pointer or zero length");
        return false;
    }
    
    // 创建压缩图像消息对象
    sensor_msgs::CompressedImage image_msg;
    
    // 填充消息头信息
    image_msg.header.stamp.sec = stTime.tv_sec;
    image_msg.header.stamp.nsec = stTime.tv_nsec;
    image_msg.header.frame_id = m_szTopic;

    // 配置图像数据参数
    image_msg.data.resize(nDatalen);
    image_msg.format = "jpeg";

    // 数据填充
    std::memcpy(&image_msg.data[0], pData, nDatalen);
    
    // 发布消息
    m_Publisher.publish(image_msg);
    
    // 输出调试信息
    // ROS_INFO("Published compressed image on %s, size: %d bytes", m_szTopic, nDatalen);

    return true;
}

/**
 * @brief CCameraStatusPublisher构造函数
 * 
 * 初始化ROS节点并创建状态发布者
 * 
 * @param name 话题名称
 */
CCameraStatusPublisher::CCameraStatusPublisher(std::string name)
{
    // 保存话题名称
    strncpy(m_szTopic, name.c_str(), sizeof(m_szTopic));
    m_szTopic[sizeof(m_szTopic) - 1] = '\0'; // 确保字符串终止
    
    // 创建Bool类型发布者，队列大小为1，设置latch（新订阅者能立即收到最后一条状态）
    m_Publisher = m_nh.advertise<std_msgs::Bool>(name, 1, true);
    
    // 初始化状态
    m_hasData = false;
    m_lastDataTime = ros::Time::now();
    
    // 发布初始状态（无数据）
    PublishStatus(false);
}

/**
 * @brief CCameraStatusPublisher析构函数
 * 
 * 清理资源，ROS1会自动管理大部分资源
 */
CCameraStatusPublisher::~CCameraStatusPublisher()
{
    // 析构函数（当前无资源需要显式释放，ROS1资源会自动管理）
}

/**
 * @brief 发布相机状态消息
 * 
 * @param hasData true表示相机有数据，false表示无数据
 * @return 成功发布返回true，否则返回false
 */
bool CCameraStatusPublisher::PublishStatus(bool hasData)
{
    // 创建Bool消息对象
    std_msgs::Bool status_msg;
    status_msg.data = hasData;
    
    // 发布消息
    m_Publisher.publish(status_msg);
    
    return true;
}

/**
 * @brief 更新并发布相机数据状态
 * 
 * 在接收到图像数据时调用，更新最后接收时间并发布状态
 */
void CCameraStatusPublisher::UpdateDataReceived()
{
    m_lastDataTime = ros::Time::now();
    
    // 如果之前状态是无数据，现在更新为有数据并发布
    if (!m_hasData) {
        m_hasData = true;
        PublishStatus(true);
    }
}

/**
 * @brief 检查相机是否超时（无数据）
 * 
 * 根据最后接收数据的时间判断是否超时
 * @param timeoutMs 超时时间（毫秒），默认1000ms
 * @return true表示超时（无数据），false表示正常
 */
bool CCameraStatusPublisher::IsTimeout(int timeoutMs)
{
    ros::Duration elapsed = ros::Time::now() - m_lastDataTime;
    bool isTimeout = elapsed.toSec() * 1000 > timeoutMs;
    
    // 如果超时且之前状态是有数据，更新为无数据并发布
    if (isTimeout && m_hasData) {
        m_hasData = false;
        PublishStatus(false);
    }
    
    return isTimeout;
}
#endif

#if (defined(SUPPORT_ROS2_JPG) || defined(SUPPORT_ROS2_YUV))
#include "../inc/camera_publish.h"

// 构造函数初始化ROS2节点和图像发布者
// 参数name指定发布话题的名称
CCameraPublisher::CCameraPublisher(std::string name)
 :rclcpp::Node("Image_publisher")  // 创建ROS2节点，固定节点名称为"Image_publisher"
{
 m_Publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(name, 2); 
    // 创建CompressedImage类型发布者：
    // name参数指定具体话题名称（如"/camera/image/compressed"）
    // 2表示消息队列缓存深度（缓冲未及时处理的消息）
}

CCameraPublisher::~CCameraPublisher()
{
    // 析构函数（当前无资源需要显式释放，ROS2资源会自动管理）
}

bool CCameraPublisher::Publisher(struct timespec stTime, unsigned char *pData, int nDatalen)
{
    // 创建压缩图像消息对象
    sensor_msgs::msg::CompressedImage image_msg;
    
    // 填充消息头信息
    image_msg.header.stamp.sec = stTime.tv_sec;        // 时间戳秒级部分（来自设备采集时间）
    image_msg.header.stamp.nanosec = stTime.tv_nsec;   // 时间戳纳秒级部分
    image_msg.header.frame_id = m_szTopic;             // 坐标系标识（通常与话题名关联）

    // 配置图像数据参数
    image_msg.data.resize(nDatalen);                   // 预分配数据空间（精确匹配输入数据长度）
    image_msg.format = "jpeg";                         // 指定编码格式（固定为JPEG格式，确保ROS2客户端兼容性）

    // 数据填充与发布
    std::memcpy(image_msg.data.data(), (unsigned char *)pData, image_msg.data.size()); 
        // 直接复制原始压缩数据到消息缓冲区（需确保pData指向有效且长度匹配）
    m_Publisher->publish(image_msg); 
        // 将消息推入发布队列（实际网络传输由ROS2底层异步完成）

    return true; // 成功发布返回状态
}
#endif
