/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/float32_multi_array.h>


#include <stdbool.h>
#include "usart.h"

#include "gpio.h"
#include "tim.h"
#include "mpu6050.h"
#include "ultrasonic.h"
#include "motor_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for MicroRosTask */
osThreadId_t MicroRosTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t MicroRosTask_attributes = {
  .name = "MicroRosTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UltrasonicTask */
osThreadId_t UltrasonicTaskHandle;
const osThreadAttr_t UltrasonicTask_attributes = {
  .name = "UltrasonicTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
 
void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

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

void Reboot_Check()
{
  static uint8_t RebootPin_status = GPIO_PIN_RESET;
  if((HAL_GPIO_ReadPin(Reboot_Pin_GPIO_Port,Reboot_Pin_Pin) == GPIO_PIN_SET) && (RebootPin_status == GPIO_PIN_RESET))
  {
    NVIC_SystemReset();
  }
    
  RebootPin_status = HAL_GPIO_ReadPin(Reboot_Pin_GPIO_Port,Reboot_Pin_Pin);
}

/* USER CODE END FunctionPrototypes */

void StartMicroRosTask(void *argument);
void StartUltrasonicTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MicroRosTask */
  MicroRosTaskHandle = osThreadNew(StartMicroRosTask, NULL, &MicroRosTask_attributes);

  /* creation of UltrasonicTask */
  UltrasonicTaskHandle = osThreadNew(StartUltrasonicTask, NULL, &UltrasonicTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMicroRosTask */
/**
  * @brief  Function implementing the MicroRosTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMicroRosTask */
void StartMicroRosTask(void *argument)
{
  /* USER CODE BEGIN StartMicroRosTask */
  /* Infinite loop */
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

  for(;;)
  {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // 循环处理数据
    // fill_imu_message();
    // rcl_publish(&publisher, &imu_msg, NULL);
    ultrasonic_publish();
    osDelay(5);
  }
  /* USER CODE END StartMicroRosTask */
}

/* USER CODE BEGIN Header_StartUltrasonicTask */
/**
* @brief Function implementing the UltrasonicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUltrasonicTask */
void StartUltrasonicTask(void *argument)
{
  /* USER CODE BEGIN StartUltrasonicTask */
  /* Infinite loop */
  for(;;)
  {
    Reboot_Check();
    Ultrasonic_Start();
    osDelay(80);
  }
  /* USER CODE END StartUltrasonicTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

