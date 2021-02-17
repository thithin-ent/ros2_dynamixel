#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "math.h"

#define _USE_MATH_DEFINES

rclcpp::Node::SharedPtr node = nullptr;
using namespace dynamixel;

// Control table address
#define ADDR_OPERTAING_MODE   11
#define ADDR_TORQUE_ENABLE    64              // Torque on/off 제어
#define ADDR_GOAL_VELOCITY    104             // 속도 제어
#define ADDR_GOAL_POSITION    116             // 위치 제어
#define ADDR_PRESENT_POSITION 132             // 현재 위치

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               4               // DXL1 ID
#define DXL2_ID               3               // DXL2 ID
#define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  

#define VALUE                 0
#define OPERATING_sposition   4               // 제어모드, 확장위치제어:4, 위치제어: 3, 속도제어: 1
#define OPERATING_position    3  
#define meter_tick            (48*M_PI/4096)


PortHandler *portHandler;
PacketHandler *packetHandler;

void distance_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  uint8_t dxl_error = 0;
  int move_distance = static_cast<int>(msg->data/meter_tick);
  packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, move_distance ,&dxl_error);   //  -1,048,575 ~ 1,048,575까지 받기, 1당 0.088도
  RCLCPP_INFO(node->get_logger(), "distance: %d", move_distance);                                      //  mm를 토픽으로 받아 절대 위치를 기준으로 해당 거리만큼 이동
}


void position_callback(const std_msgs::msg::Int16::SharedPtr msg)
{
  uint8_t dxl_error = 0;
  packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, msg->data ,&dxl_error);
  RCLCPP_INFO(node->get_logger(), "position: %d", msg->data);
}



int main(int argc, char **argv)  
{
  uint8_t dxl_error = 0;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("Dynamixel");

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if(portHandler->openPort() == false)
  {
    RCLCPP_INFO(node->get_logger(),"Failed to open the port!");
  }
  if(portHandler->setBaudRate(BAUDRATE) == false)
  {
    RCLCPP_INFO(node->get_logger(),"Failed to set the baudrate!");
  }

  packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_OPERTAING_MODE, OPERATING_position, &dxl_error); //  eeprom 입력
  packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_OPERTAING_MODE, OPERATING_sposition, &dxl_error); //    // 
 
  packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);                    //   토크잠금
  packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);                    //     //
  
  RCLCPP_INFO(node->get_logger(),"start setting");
  packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, VALUE, &dxl_error);
  packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, VALUE, &dxl_error);

  auto subscription1 =
    node->create_subscription<std_msgs::msg::Int32>("/move_distance", 10, distance_callback);
  
  auto subscription2 =
    node->create_subscription<std_msgs::msg::Int16>("/move_position", 10, position_callback);
  

  rclcpp::spin(node);
  packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  rclcpp::shutdown();
  
 return 0;
}
