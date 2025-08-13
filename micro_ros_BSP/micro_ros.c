#include "micro_ros.h"

rcl_publisher_t publisher;
rcl_publisher_t publisher_ultrasonic;

std_msgs__msg__Int32 msg;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Float32MultiArray ultrasonic_msg;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist sub_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t timer;
rcl_timer_t timer_ultrasonic;

void fill_imu_message() {
    // 从IMU读取数据（根据你的传感器API）
    MPU6050_Process();

    // 设置时间戳
    int64_t stamp = rmw_uros_epoch_millis();
    imu_msg.header.stamp.sec = stamp * 1e-3;
    imu_msg.header.stamp.nanosec = stamp - imu_msg.header.stamp.sec * 1000;
    micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");  // 设置坐标系
    
    // 填充加速度数据 (m/s^2)
    imu_msg.linear_acceleration.x = MPU6050.Ax;
    imu_msg.linear_acceleration.y = MPU6050.Ay;
    imu_msg.linear_acceleration.z = MPU6050.Az;
    
    // 填充角速度数据 (rad/s)
    imu_msg.angular_velocity.x = MPU6050.Gx;
    imu_msg.angular_velocity.y = MPU6050.Gy;
    imu_msg.angular_velocity.z = MPU6050.Gz;
    
    // 如果没有方向数据，可以设置为0并设置协方差为-1
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;
    
    // 设置协方差矩阵（根据你的传感器特性）
    // 行主序排列
    imu_msg.orientation_covariance[0] = -1;  // 表示方向数据不可用
    imu_msg.angular_velocity_covariance[0] = 0.01;  // 示例值
    imu_msg.linear_acceleration_covariance[0] = 0.01;  // 示例值
    // imu_msg.data.data[0] = MPU6050.Ax;  // ax
    // imu_msg.data.data[1] = MPU6050.Ay;  // ay
    // imu_msg.data.data[2] = MPU6050.Az;  // az
    // imu_msg.data.data[3] = MPU6050.Gx;   // gx
    // imu_msg.data.data[4] = MPU6050.Gy;   // gy
    // imu_msg.data.data[5] = MPU6050.Gz;   // gz
}

void ultrasonic_publish()
{
  if(Ultrasonic_canPublish())
  {
    Ultrasonic_clearFlag();
    ultrasonic_msg.data.data[0] = Ultrasonic_getLeftDistance();
    ultrasonic_msg.data.data[1] = Ultrasonic_getRightDistance();
    rcl_publish(&publisher_ultrasonic, &ultrasonic_msg, NULL);
  }
}

void twist_callback(const void *msg_in)
{
  // 将接收到的消息指针转化为 geometry_msgs__msg__Twist 类型
  const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msg_in;
  // 从 Twist 消息中获取线速度和角速度
  float linear_x = twist_msg->linear.x;
  float angular_z = twist_msg->angular.z;

  if(linear_x>0)
  {
    Motor_Forward();
  }
  else if(linear_x<0)
  {
    Motor_Backward();
  }

  if(angular_z>0)
  {
    Motor_LeftTurn();
  }
  else if(angular_z<0)
  {
    Motor_RightTurn();
  }

  if(linear_x == 0 && angular_z == 0)
  {
    Motor_Stop();
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    fill_imu_message();
    rcl_publish(&publisher, &imu_msg, NULL);
    //rcl_publish(&publisher, &msg, NULL);
    //msg.data++;
  }
}

void Micro_Ros_initial()
{
  rmw_uros_set_custom_transport(
    true,
    (void *) &huart3,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);
 
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;
 
  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__); 
  }
 
  // micro-ROS app
  allocator = rcl_get_default_allocator();
 
  //create init_options
  rclc_support_init(&support, 0, NULL, &allocator);
 
  // create node
  rclc_node_init_default(&node, "cubemx_node", "", &support);
 
  // create publisher
  // rclc_publisher_init_default(
  //   &publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //   "cubemx_publisher");
 
  // msg.data = 0;
 
  // imu数据
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_data");

  // 超声波数据
  rclc_publisher_init_default(
    &publisher_ultrasonic,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "ultrasonic_data"
  );

  ultrasonic_msg.data.data = (float *)malloc(2 * sizeof(float));
  ultrasonic_msg.data.size = 2;

  // 初始化订阅者
  rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel");

  // 创建定时器，16ms发一次
  const unsigned int timer_timeout = 16;
  rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback);
  
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  // 给执行器添加定时器
  rclc_executor_add_timer(&executor, &timer);
  // 设置订阅的回调函数
  rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &twist_callback, ON_NEW_DATA);
}

void Micro_Ros_Process()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // 循环处理数据
    // fill_imu_message();
    // rcl_publish(&publisher, &imu_msg, NULL);
    ultrasonic_publish();
}