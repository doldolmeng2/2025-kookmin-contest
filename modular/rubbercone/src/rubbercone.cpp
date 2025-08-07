// rubbercone/src/rubbercone.cpp
#include "rubbercone/rubbercone.hpp"
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <cstdlib>
#include <chrono>

LidarViewer::LidarViewer()
: Node("rubbercone"),
  window_size_(800),
  scale_(500.0f),
  OFFSET_GAIN_(300.0f),
  rubber_offset_value_(0),
  rubber_end_value_(0)
{
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&LidarViewer::scanCallback, this, std::placeholders::_1)
  );

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&LidarViewer::imageCallback, this, std::placeholders::_1)
  );

  info_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
    "rubbercone_info", 10
  );

  info_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&LidarViewer::publishInfo, this)
  );

  cv::namedWindow("Lidar View", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Camera View", cv::WINDOW_AUTOSIZE);
}

void LidarViewer::publishInfo() {
  std_msgs::msg::Int32MultiArray msg;
  msg.data.resize(2);
  msg.data[0] = rubber_offset_value_;
  msg.data[1] = rubber_end_value_;
  info_pub_->publish(msg);
}

void LidarViewer::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // 1) 캔버스 준비
  cv::Mat canvas = cv::Mat::zeros(window_size_, window_size_, CV_8UC3);
  cv::Point2f center(window_size_/2, window_size_/2);

  // 2) 10cm 단위 동심원 (10~100cm)
  for (int r_cm = 10; r_cm <= 100; r_cm += 10) {
    float rad_px = r_cm * (scale_ / 100.0f);
    cv::circle(canvas, center, int(rad_px), cv::Scalar(100,100,100), 1);
    cv::putText(canvas,
                std::to_string(r_cm) + "cm",
                {int(center.x + 5), int(center.y - rad_px)},
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 1);
  }

  // 3) 유효 포인트 수집
  const float ANG_MAX = 85.0f * M_PI/180.0f;
  std::vector<cv::Point2f> pts;
  float angle = msg->angle_min;
  for (float range : msg->ranges) {
    if (std::isfinite(range) &&
        range >= 0.18f && range <= 1.20f &&
        angle >= -ANG_MAX && angle <= ANG_MAX) {
      pts.emplace_back(range * std::cos(angle),
                       range * std::sin(angle));
    }
    angle += msg->angle_increment;
  }

  // 4) 라바콘 그룹화
  std::vector<cv::Point2f> left_group, right_group;
  cv::Point2f left_first, right_first;
  float left_min=1e6f, right_min=1e6f;
  bool found_left=false, found_right=false;

  for (auto &pt : pts) {
    float d = std::hypot(pt.x, pt.y);
    if (pt.y>0) {
      if (d<left_min) { left_min=d; left_first=pt; found_left=true; }
    } else {
      if (d<right_min) { right_min=d; right_first=pt; found_right=true; }
    }
  }

  const float CONE_D=0.38f;
  if (found_left) {
    left_group.push_back(left_first);
    cv::Point2f cur=left_first;
    while (left_group.size()<3) {
      cv::Point2f next; float best=1e6f; bool ok=false;
      for (auto &pt:pts) {
        if (pt.y<=0) continue;
        float d0=std::hypot(pt.x,pt.y),
              dc=std::hypot(pt.x-cur.x,pt.y-cur.y);
        if (d0>std::hypot(cur.x,cur.y)+0.1f &&
            std::abs(dc-CONE_D)<0.15f && d0<best) {
          bool close=false;
          for (auto &ex:left_group)
            if (std::hypot(pt.x-ex.x,pt.y-ex.y)<0.1f) close=true;
          if (!close) { best=d0; next=pt; ok=true; }
        }
      }
      if (!ok) break;
      left_group.push_back(next);
      cur=next;
    }
  }

  if (found_right) {
    right_group.push_back(right_first);
    cv::Point2f cur=right_first;
    while (right_group.size()<3) {
      cv::Point2f next; float best=1e6f; bool ok=false;
      for (auto &pt:pts) {
        if (pt.y>=0) continue;
        float d0=std::hypot(pt.x,pt.y),
              dc=std::hypot(pt.x-cur.x,pt.y-cur.y);
        if (d0>std::hypot(cur.x,cur.y)+0.1f &&
            std::abs(dc-CONE_D)<0.15f && d0<best) {
          bool close=false;
          for (auto &ex:right_group)
            if (std::hypot(pt.x-ex.x,pt.y-ex.y)<0.1f) close=true;
          if (!close) { best=d0; next=pt; ok=true; }
        }
      }
      if (!ok) break;
      right_group.push_back(next);
      cur=next;
    }
  }

  // 5) 그리기
  for (size_t i=0;i<left_group.size();++i) {
    int px=int(center.x - left_group[i].y*scale_),
        py=int(center.y - left_group[i].x*scale_);
    cv::circle(canvas,{px,py},4,cv::Scalar(255,0,0),-1);
    cv::putText(canvas,"L"+std::to_string(i+1),{px+5,py-5},
                cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(255,255,255),1);
  }
  for (size_t i=0;i<right_group.size();++i) {
    int px=int(center.x - right_group[i].y*scale_),
        py=int(center.y - right_group[i].x*scale_);
    cv::circle(canvas,{px,py},4,cv::Scalar(0,0,255),-1);
    cv::putText(canvas,"R"+std::to_string(i+1),{px+5,py-5},
                cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(255,255,255),1);
  }

  // 6) 목표점 계산
  bool has_mid=false;
  cv::Point2f target;
  if (left_group.size()>=2 && right_group.size()>=2) {
    cv::Point2f lm{(left_group[0].x+left_group[1].x)/2,
                   (left_group[0].y+left_group[1].y)/2};
    cv::Point2f rm{(right_group[0].x+right_group[1].x)/2,
                   (right_group[0].y+right_group[1].y)/2};
    target={(lm.x+rm.x)/2,(lm.y+rm.y)/2};
    has_mid=true;
  } else if (left_group.size()==1 && right_group.size()>=2) {
    cv::Point2f rm{(right_group[0].x+right_group[1].x)/2,
                   (right_group[0].y+right_group[1].y)/2};
    target={(left_group[0].x+rm.x)/2,(left_group[0].y+rm.y)/2};
    has_mid=true;
  } else if (right_group.size()==1 && left_group.size()>=2) {
    cv::Point2f lm{(left_group[0].x+left_group[1].x)/2,
                   (left_group[0].y+left_group[1].y)/2};
    target={(lm.x+right_group[0].x)/2,(lm.y+right_group[0].y)/2};
    has_mid=true;
  }

  // 7) 표시 및 offset 계산
  if (has_mid) {
    int tx=int(center.x - target.y*scale_),
        ty=int(center.y - target.x*scale_);
    cv::circle(canvas,{tx,ty},8,cv::Scalar(0,255,0),-1);
    float offset = -target.y * OFFSET_GAIN_;
    rubber_offset_value_ = static_cast<int32_t>(offset);
    std::ostringstream ss;
    ss<<std::fixed<<std::setprecision(2)
      <<"Offset: "<<offset
      <<" | L:"<<left_group.size()
      <<" R:"<<right_group.size();
    cv::putText(canvas,ss.str(),{10,30},
                cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(255,255,255),2);
  } else {
    rubber_offset_value_ = 0;
    cv::putText(canvas,"Insufficient cones detected",{10,30},
                cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(255,255,255),2);
  }

  // 8) 화면 출력
  cv::imshow("Lidar View", canvas);
  cv::waitKey(1);
}

void LidarViewer::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  cv::Mat img = cv_bridge::toCvCopy(msg,"bgr8")->image;
  cv::imshow("Camera View", img);
  cv::waitKey(1);
}

int main(int argc, char** argv) {
  setenv("GDK_BACKEND","x11",1);
  setenv("QT_QPA_PLATFORM","xcb",1);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarViewer>());
  rclcpp::shutdown();
  return 0;
}
