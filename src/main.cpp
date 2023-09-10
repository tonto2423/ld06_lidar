#include <iostream>
#include "cmd_interface_linux.h"
#include <stdio.h>
#include "lipkg.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "tofbf.h"
#include <string>

void publishLoop(
    ros::Publisher &lidar_pub, 
    CmdInterfaceLinux &cmd_port, 
    LiPkg* &lidar, 
    const std::vector<std::string>& serial_port_candidates, 
    const std::string& lidar_frame, 
    const double& range_threshold
) {
  if (!cmd_port.IsOpened()) {
    for (const auto& port_name_const : serial_port_candidates) {
      std::string port_name = port_name_const; // Create a non-const copy to pass to the Open method
      if (cmd_port.Open(port_name)) {
        ROS_INFO("LiDAR_LD06 started successfully");
        cmd_port.SetReadCallback([](const char*, size_t){});
        delete lidar;
        lidar = new LiPkg();
        lidar->SetLidarFrame(lidar_frame);
        lidar->SetRangeThreshold(range_threshold);
        cmd_port.SetReadCallback([&lidar](const char* byte, size_t len) {
          if (lidar->Parse((uint8_t*)byte, len)) {
            lidar->AssemblePacket();
          }
        });
        break;
      } 
      else {
        ROS_ERROR("Can't open %s", port_name.c_str());
      }
    }
  }

  if (!cmd_port.IsOpened()) {
    return;
  }

  if (cmd_port.IsDisconnected()) {
    ROS_ERROR("Disconnected. reconnecting");
    cmd_port.Close();
  }

  if (lidar->IsFrameReady()) {
    lidar_pub.publish(lidar->GetLaserScan());
    lidar->ResetFrameReady();
  }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ld06_lidar");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // LiPkg インスタンスとコマンドポートを初期化
    LiPkg *lidar = new LiPkg;
    CmdInterfaceLinux cmd_port;
    std::string port_name;
    std::string lidar_frame;
    double range_threshold_;
    std::vector<std::string> serial_port_candidates_;

    // パラメータを取得
    nh_private.param<std::string>("lidar_frame", lidar_frame, "lidar_frame");
    nh_private.param<std::string>("serial_port", port_name, std::string());
    nh_private.param("range_threshold", range_threshold_, 0.005);
    nh_private.param("serial_port_candidates", serial_port_candidates_, std::vector<std::string>({"/dev/ttyUSB0"}));

    // LiDAR のフレームと閾値を設定
    lidar->SetLidarFrame(lidar_frame);
    lidar->SetRangeThreshold(range_threshold_);

    // コールバック関数を設定
    cmd_port.SetReadCallback([&lidar](const char *byte, size_t len) {
        if (lidar->Parse((uint8_t*)byte, len)) {
            lidar->AssemblePacket();
        }
    });

    ROS_INFO_STREAM("Using port " << port_name);

    ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>("LiDAR/LD06", 1);

    ros::Rate rate(10); // 100ms = 10Hz
    while (ros::ok()) {
        publishLoop(lidar_pub, cmd_port, lidar, serial_port_candidates_, lidar_frame, range_threshold_);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}