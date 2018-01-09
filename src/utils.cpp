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
#include <msckf_vio/utils.h>
#include <vector>

namespace msckf_vio {
  namespace utils {
    Eigen::Isometry3d get_transform_eigen(const ros::NodeHandle &nh,
                                          const std::string &field) {
      Eigen::Isometry3d T;
      cv::Mat c = get_transform_cv(nh, field);
      
      T.linear()(0, 0)   = c.at<double>(0, 0);
      T.linear()(0, 1)   = c.at<double>(0, 1);
      T.linear()(0, 2)   = c.at<double>(0, 2);
      T.linear()(1, 0)   = c.at<double>(1, 0);
      T.linear()(1, 1)   = c.at<double>(1, 1);
      T.linear()(1, 2)   = c.at<double>(1, 2);
      T.linear()(2, 0)   = c.at<double>(2, 0);
      T.linear()(2, 1)   = c.at<double>(2, 1);
      T.linear()(2, 2)   = c.at<double>(2, 2);
      T.translation()(0) = c.at<double>(0, 3);
      T.translation()(1) = c.at<double>(1, 3);
      T.translation()(2) = c.at<double>(2, 3);
      return (T);
    }

    cv::Mat get_transform_cv(const ros::NodeHandle &nh,
                             const std::string &field) {
      cv::Mat T;
      try {
        // first try reading kalibr format
        T = get_kalibr_style_transform(nh, field);
      } catch (std::runtime_error &e) {
        // maybe it's the old style format?
        ROS_WARN_STREAM("cannot read transform " << field
                        << " in kalibr format, trying old one!");
        try {
          T = get_vec16_transform(nh, field);
        } catch (std::runtime_error &e) {
          std::string msg = "cannot read transform " + field + " error: " + e.what();
          ROS_ERROR_STREAM(msg);
          throw std::runtime_error(msg);
        }
      }
      return (T);
    }

    cv::Mat get_vec16_transform(const ros::NodeHandle &nh,
                                const std::string &field) {
      std::vector<double> v;
      nh.getParam(field, v);
      if (v.size() != 16) {
        throw std::runtime_error("invalid vec16!");
      }
      cv::Mat T = cv::Mat(v).clone().reshape(1, 4); // one channel 4 rows
      return (T);
    }

    cv::Mat get_kalibr_style_transform(const ros::NodeHandle &nh,
                                       const std::string &field) {
      cv::Mat T = cv::Mat::eye(4, 4, CV_64FC1);
      XmlRpc::XmlRpcValue lines;
      if (!nh.getParam(field, lines)) {
        throw (std::runtime_error("cannot find transform " + field));
      }
      if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw (std::runtime_error("invalid transform " + field));
      }
      for (int i = 0; i < lines.size(); i++) {
        if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
          throw (std::runtime_error("bad line for transform " + field));
        }
        for (int j = 0; j < lines[i].size(); j++) {
          if (lines[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
            throw (std::runtime_error("bad value for transform " + field));
          } else {
            T.at<double>(i,j) = static_cast<double>(lines[i][j]);
          }
        }
      }
      return (T);
    }
  }
}
