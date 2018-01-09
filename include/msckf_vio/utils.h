/*
 *This file is part of msckf_vio
 *
 *    msckf_vio is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    msckf_vio is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with msckf_vio.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MSCKF_VIO_UTILS_H
#define MSCKF_VIO_UTILS_H

#include <ros/ros.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>

namespace msckf_vio {
/*
 * @brief utilities for msckf
 */
  namespace utils {
    Eigen::Isometry3d get_transform_eigen(const ros::NodeHandle &nh,
                                          const std::string &field);
    cv::Mat get_transform_cv(const ros::NodeHandle &nh,
                             const std::string &field);
    cv::Mat get_vec16_transform(const ros::NodeHandle &nh,
                                const std::string &field);
    cv::Mat get_kalibr_style_transform(const ros::NodeHandle &nh,
                                       const std::string &field);
  }
}
#endif
