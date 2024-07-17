/** =================================================
* @author: MorningXu (morningxu1991@163.com)
* @version v1.0.0
* @date: 2024年07月03日
* @brief:
* @copyright:
* ================================================== */

#pragma once

#include <opencv2/opencv.hpp>
#include <string>

namespace mat_to_image_util{
  std::string mat_type23encoding(int mat_type) {
    switch (mat_type) {
      case CV_8UC1:
        return "mono8";
      case CV_8UC3:
        return "bgr8";
      case CV_16SC1:
        return "mono16";
      case CV_8UC4:
        return "rgba8";
    }
  }
}

