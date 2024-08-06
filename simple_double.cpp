/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/

/**
 * @file simple_double.cpp
 * @author Bence Magyar
 * @date June 2012
 * @version 0.1
 * @brief ROS version of the example named "simple" in the ArUco software package.
 */

#include <iostream>
using namespace std;
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// For transformations
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <Eigen/Dense>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

#include <string>  // For std::string

// For scaling 3d cube
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cameraparameters.h"
#include "cvdrawingutils.h"
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <thread>
#include <mutex>
using namespace cv;

cv::Mat inImage;
aruco::CameraParameters camParam;
bool useRectifiedImages, normalizeImageIllumination;
int dctComponentsToRemove;
aruco::MarkerDetector mDetector;
std::vector<aruco::Marker> markers;
ros::Subscriber cam_info_sub;
bool cam_info_received;
image_transport::Publisher image_pub;
image_transport::Publisher debug_pub;
ros::Publisher pose_pub1;
ros::Publisher pose_pub2;
ros::Publisher poseStamped_pub1;
ros::Publisher poseStamped_pub2;
ros::Publisher image_pub_2;
ros::Subscriber image_sub_2;
std::string child_name1;
std::string parent_name;
std::string child_name2;
// Specify the frames
std::string source_frame;
std::string target_frame;

double marker_size;
int marker_id1;
int marker_id2;

bool initial_marker_pose_set = false;
cv::Mat initial_Rvec, initial_Tvec;

Eigen::Matrix4f cached_transform_matrix;
std::mutex transform_mutex;
bool transform_cached = false;
bool got_error = false;
double count = 0;
// cv::Mat initial_image;
// Eigen::Matrix4f transform_matrix;
// cv::Mat background_image;  // Static image with the fixed grid
// bool background_image_set = false;  // Flag to check if background image is set



void transformPoseToBaseLink(const cv::Mat& Rvec, const cv::Mat& Tvec, const Eigen::Matrix4f& transform_matrix, cv::Mat& transformed_Rvec, cv::Mat& transformed_Tvec){
    // Convert Rvec and Tvec to Eigen
    cv::Mat Rmat;
    cv::Rodrigues(Rvec, Rmat);
    Eigen::Matrix3f R;
    cv::cv2eigen(Rmat, R);
    Eigen::Vector3f T;
    cv::cv2eigen(Tvec, T);

    // Create 4x4 transformation matrix
    Eigen::Matrix4f marker_pose = Eigen::Matrix4f::Identity();
    marker_pose.block<3, 3>(0, 0) = R;
    marker_pose.block<3, 1>(0, 3) = T;

    // Apply the transformation
    Eigen::Matrix4f transformed_pose = transform_matrix * marker_pose;

    // Extract R and T from the transformed pose
    Eigen::Matrix3f transformed_R = transformed_pose.block<3, 3>(0, 0);
    Eigen::Vector3f transformed_T = transformed_pose.block<3, 1>(0, 3);

    // Convert back to cv::Mat
    cv::Mat transformed_Rmat;
    cv::eigen2cv(transformed_R, transformed_Rmat);
    cv::Rodrigues(transformed_Rmat, transformed_Rvec);
    cv::eigen2cv(transformed_T, transformed_Tvec);
}

// Function to get the transformation matrix from source_frame to target_frame
Eigen::Matrix4f getTransformMatrix(const std::string& source_frame, const std::string& target_frame) {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    Eigen::Matrix4f matrix;
    
    try {
        // Wait for the transform to be available
        listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3.0));

        // Get the transformation
        listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);

        // Convert to a 4x4 matrix
        tf::Matrix3x3 rotation_matrix = transform.getBasis();
        tf::Vector3 translation_vector = transform.getOrigin();
        
        Eigen::Matrix3f eigen_rotation;
        eigen_rotation << 
            rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2],
            rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2],
            rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2];

        Eigen::Vector3f eigen_translation(translation_vector.getX(), translation_vector.getY(), translation_vector.getZ());
        
        matrix.setIdentity();
        matrix.block<3, 3>(0, 0) = eigen_rotation;
        matrix.block<3, 1>(0, 3) = eigen_translation;
        // matrix.block<3, 3>(0, 0) = Eigen::Matrix3f(rotation_matrix);
        // matrix.block<3, 1>(0, 3) = Eigen::Vector3f(translation_vector.getX(), translation_vector.getY(), translation_vector.getZ());

        // Optionally, set the bottom row if needed (though it is usually already [0, 0, 0, 1])
        matrix(3, 0) = 0.0f;
        matrix(3, 1) = 0.0f;
        matrix(3, 2) = 0.0f;
        matrix(3, 3) = 1.0f;

        

    } catch (tf::TransformException &ex) {
        got_error = true;
        ROS_ERROR("%s", ex.what());
        // Return an identity matrix in case of an error
        matrix.setIdentity();
        
    }

    return matrix;
}

// Function to validate the transform matrix
bool isValidTransformMatrix(const Eigen::Matrix4f& matrix) {
    // Example validation: check if the matrix is non-zero
    if (matrix.isZero()) {
        return false;
    }
    // Additional checks can be added here
    return true;
}

  // Function to get the transform matrix with caching
Eigen::Matrix4f getCachedTransformMatrix(const std::string& source_frame, const std::string& target_frame) {
      std::lock_guard<std::mutex> lock(transform_mutex);
      if (!transform_cached || !isValidTransformMatrix(cached_transform_matrix)) 
      {
          Eigen::Matrix4f transform_matrix = getTransformMatrix(source_frame, target_frame);
          if (isValidTransformMatrix(transform_matrix)) {
              cached_transform_matrix = transform_matrix;
              transform_cached = true;
          } else {
              // Handle invalid transform matrix case
              ROS_WARN("Computed transform matrix is invalid");
              throw std::runtime_error("Invalid transform matrix");
          }
      }
      return cached_transform_matrix;
  }



void drawDotsOnLine(cv::Mat &Image, cv::Point2f start, cv::Point2f end, float interval, cv::Scalar color, int dotSize) {
    float lineLength = cv::norm(end - start);
    int numDots = static_cast<int>(lineLength / interval);

    for (int i = 1; i <= numDots; ++i) {
        float t = i * interval / lineLength;
        cv::Point2f dot = start * (1.0f - t) + end * t;
        cv::circle(Image, dot, dotSize, color, cv::FILLED);
    }
}

void draw3dCube_scaled(cv::Mat& Image,  const cv::Mat& Rvec, const cv::Mat& Tvec, const aruco::CameraParameters& CP, int lineSize, bool setYperpendicular, double scale)
 {
   cv::Mat objectPoints(8, 3, CV_32FC1);
   float halfSize = 0.05f * 0.5f * scale ;
  
   if (setYperpendicular)
   {
     objectPoints.at<float>(0, 0) = -halfSize;
     objectPoints.at<float>(0, 1) = 0;
     objectPoints.at<float>(0, 2) = -halfSize;
     objectPoints.at<float>(1, 0) = halfSize;
     objectPoints.at<float>(1, 1) = 0;
     objectPoints.at<float>(1, 2) = -halfSize;
     objectPoints.at<float>(2, 0) = halfSize;
     objectPoints.at<float>(2, 1) = 0;
     objectPoints.at<float>(2, 2) = halfSize;
     objectPoints.at<float>(3, 0) = -halfSize;
     objectPoints.at<float>(3, 1) = 0;
     objectPoints.at<float>(3, 2) = halfSize;
  
     objectPoints.at<float>(4, 0) = -halfSize;
     objectPoints.at<float>(4, 1) = 0.05f * scale;
     objectPoints.at<float>(4, 2) = -halfSize;
     objectPoints.at<float>(5, 0) = halfSize;
     objectPoints.at<float>(5, 1) =  0.05f * scale;
     objectPoints.at<float>(5, 2) = -halfSize;
     objectPoints.at<float>(6, 0) = halfSize;
     objectPoints.at<float>(6, 1) =  0.05f * scale;
     objectPoints.at<float>(6, 2) = halfSize;
     objectPoints.at<float>(7, 0) = -halfSize;
     objectPoints.at<float>(7, 1) =  0.05f * scale;
     objectPoints.at<float>(7, 2) = halfSize;
   }
   else
   {
     objectPoints.at<float>(0, 0) = -halfSize;
     objectPoints.at<float>(0, 1) = -halfSize;
     objectPoints.at<float>(0, 2) = 0;
     objectPoints.at<float>(1, 0) = halfSize;
     objectPoints.at<float>(1, 1) = -halfSize;
     objectPoints.at<float>(1, 2) = 0;
     objectPoints.at<float>(2, 0) = halfSize;
     objectPoints.at<float>(2, 1) = halfSize;
     objectPoints.at<float>(2, 2) = 0;
     objectPoints.at<float>(3, 0) = -halfSize;
     objectPoints.at<float>(3, 1) = halfSize;
     objectPoints.at<float>(3, 2) = 0;
  
     objectPoints.at<float>(4, 0) = -halfSize;
     objectPoints.at<float>(4, 1) = -halfSize;
     objectPoints.at<float>(4, 2) =  0.05f * scale;
     objectPoints.at<float>(5, 0) = halfSize;
     objectPoints.at<float>(5, 1) = -halfSize;
     objectPoints.at<float>(5, 2) =  0.05f * scale;
     objectPoints.at<float>(6, 0) = halfSize;
     objectPoints.at<float>(6, 1) = halfSize;
     objectPoints.at<float>(6, 2) =  0.05f * scale;
     objectPoints.at<float>(7, 0) = -halfSize;
     objectPoints.at<float>(7, 1) = halfSize;
     objectPoints.at<float>(7, 2) =  0.05f *scale;
   }

   // Transform the object points using the transformation matrix
    // for (int i = 0; i < objectPoints.rows; ++i)
    // {
    //     cv::Point3f pt(objectPoints.at<float>(i, 0), objectPoints.at<float>(i, 1), objectPoints.at<float>(i, 2));
    //     Eigen::Vector4f pt_homogeneous(pt.x, pt.y, pt.z, 1.0);
    //     Eigen::Vector4f pt_transformed = transform_matrix * pt_homogeneous;
    //     objectPoints.at<float>(i, 0) = pt_transformed(0);
    //     objectPoints.at<float>(i, 1) = pt_transformed(1);
    //     objectPoints.at<float>(i, 2) = pt_transformed(2);
    // }
  
   std::vector<Point2f> imagePoints;
   cv::projectPoints(objectPoints, Rvec, Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);
   // draw lines of different colours
   ROS_INFO("Length of imagePoints is %d", imagePoints.size());
   for (int i = 0; i < 4; i++)
   {
    cv::line(Image, imagePoints[i], imagePoints[(i + 1) % 4], cv::Scalar(0, 0, 255, 255), lineSize);
    // Draw dots along the line
    drawDotsOnLine(Image, imagePoints[i], imagePoints[(i + 1) % 4], 50, cv::Scalar(255, 0, 0, 255), 3);
   }

   for (int i = 0; i < 4; i++)
   {
     cv::line(Image, imagePoints[i + 4], imagePoints[4 + (i + 1) % 4], cv::Scalar(0, 0, 255, 255), lineSize);
   }
   for (int i = 0; i < 4; i++)
   {
     cv::line(Image, imagePoints[i], imagePoints[i + 4], cv::Scalar(0, 0, 255, 255), lineSize);
   }
  //  for (int i=0; i<imagePoints.size(); i++)
  //  {
  //   // ROS_INFO("imagePoints[%d] = (%.2f, %.2f)", i, imagePoints[i].x, imagePoints[i].y);
  //   // ROS_INFO("imagePoints[%d] = (%.2f, %.2f)", i + 4, imagePoints[i+ 4].x, imagePoints[i+ 4].y);
  //  }
 }




void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  double ticksBefore = cv::getTickCount();
  static tf::TransformBroadcaster br;
  if (cam_info_received)
  {
    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage = cv_ptr->image;

      // New
      // cv::Mat camera_image = cv_ptr->image;

      // detection results will go into "markers"
      markers.clear();
      // ok, let's detect
      mDetector.detect(inImage, markers, camParam, marker_size, false);
      // mDetector.detect(camera_image, markers, camParam, marker_size, false);
      // for each marker, draw info and its boundaries in the image

      // only publishing the selected marker
      
      for (unsigned int i = 0; i < markers.size(); ++i)
      {
        
        
        if (markers[i].id == marker_id1)
        {
          // ROS_INFO("This is the markers size %d", markers[i].id);
          tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
          br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name1));
          geometry_msgs::Pose poseMsg;
          tf::poseTFToMsg(transform, poseMsg);
          pose_pub1.publish(poseMsg);

          geometry_msgs::PoseStamped poseStampedMsg;
          poseStampedMsg.header.frame_id = parent_name;
          poseStampedMsg.header.stamp = curr_stamp;
          poseStampedMsg.pose.position.x    = poseMsg.position.x;
          poseStampedMsg.pose.position.y    = poseMsg.position.y;
          poseStampedMsg.pose.position.z    = poseMsg.position.z;
          poseStampedMsg.pose.orientation.x = poseMsg.orientation.x;
          poseStampedMsg.pose.orientation.y = poseMsg.orientation.y;
          poseStampedMsg.pose.orientation.z = poseMsg.orientation.z;
          poseStampedMsg.pose.orientation.w = poseMsg.orientation.w;
          poseStamped_pub1.publish(poseStampedMsg);
        }
        else if (markers[i].id == marker_id2)
        {
          tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
          br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name2));
          geometry_msgs::Pose poseMsg;
          tf::poseTFToMsg(transform, poseMsg);
          pose_pub2.publish(poseMsg);

          geometry_msgs::PoseStamped poseStampedMsg;
          poseStampedMsg.header.frame_id = parent_name;
          poseStampedMsg.header.stamp = curr_stamp;
          poseStampedMsg.pose.position.x    = poseMsg.position.x;
          poseStampedMsg.pose.position.y    = poseMsg.position.y;
          poseStampedMsg.pose.position.z    = poseMsg.position.z;
          poseStampedMsg.pose.orientation.x = poseMsg.orientation.x;
          poseStampedMsg.pose.orientation.y = poseMsg.orientation.y;
          poseStampedMsg.pose.orientation.z = poseMsg.orientation.z;
          poseStampedMsg.pose.orientation.w = poseMsg.orientation.w;
          poseStamped_pub2.publish(poseStampedMsg);
          
        }

        // but drawing all the detected markers
        // The following line was uncommented
        // markers[i].draw(inImage, cv::Scalar(0, 0, 255), 4, false);

        // Convert aruco::Marker to CustomMarker
        // CustomMarker customMarker(markers[i].points);
        // customMarker.draw_scaled(inImage, cv::Scalar(0, 0, 255), 4, false, true, 5.0);
            
      }

      // paint a circle in the center of the image
      //cv::circle(inImage, cv::Point(inImage.cols / 2, inImage.rows / 2), 4, cv::Scalar(0, 255, 0), 1);

      if (markers.size() == 1)
      {
       // In here
        if (markers[0].id == marker_id1){
          geometry_msgs::PoseStamped poseStampedMsg;
          poseStampedMsg.header.frame_id = parent_name;
          poseStampedMsg.header.stamp = curr_stamp;
          poseStampedMsg.pose.position.x = 10;
          poseStampedMsg.pose.position.y = 10;
          poseStampedMsg.pose.position.z = 10;
          poseStampedMsg.pose.orientation.x = 10;
          poseStampedMsg.pose.orientation.y = 10;
          poseStampedMsg.pose.orientation.z = 10;
          poseStampedMsg.pose.orientation.w = 10;
          poseStamped_pub2.publish(poseStampedMsg);
          }
        else if (markers[0].id == marker_id2){
          //ROS_WARN("I came here");
          geometry_msgs::PoseStamped poseStampedMsg;
          poseStampedMsg.header.frame_id = parent_name;
          poseStampedMsg.header.stamp = curr_stamp;
          poseStampedMsg.pose.position.x = 10;
          poseStampedMsg.pose.position.y = 10;
          poseStampedMsg.pose.position.z = 10;
          poseStampedMsg.pose.orientation.x = 10;
          poseStampedMsg.pose.orientation.y = 10;
          poseStampedMsg.pose.orientation.z = 10;
          poseStampedMsg.pose.orientation.w = 10;
          poseStamped_pub1.publish(poseStampedMsg);
        }
        }

      if (markers.size() == 0)
      {    
          geometry_msgs::PoseStamped poseStampedMsg;
          poseStampedMsg.header.frame_id = parent_name;
          poseStampedMsg.header.stamp = curr_stamp;
          poseStampedMsg.pose.position.x = 10;
          poseStampedMsg.pose.position.y = 10;
          poseStampedMsg.pose.position.z = 10;
          poseStampedMsg.pose.orientation.x = 10;
          poseStampedMsg.pose.orientation.y = 10;
          poseStampedMsg.pose.orientation.z = 10;
          poseStampedMsg.pose.orientation.w = 10;
          poseStamped_pub1.publish(poseStampedMsg);
          poseStamped_pub2.publish(poseStampedMsg);
        }
        
      if (markers.size() == 2)
      {
        float x[2], y[2], u[2], v[2];
        for (unsigned int i = 0; i < 2; ++i)
        {
          ROS_DEBUG_STREAM(
              "Marker(" << i << ") at camera coordinates = (" << markers[i].Tvec.at<float>(0,0) << ", " << markers[i].Tvec.at<float>(1,0) << ", " << markers[i].Tvec.at<float>(2,0));
          // normalized coordinates of the marker
          x[i] = markers[i].Tvec.at<float>(0, 0) / markers[i].Tvec.at<float>(2, 0);
          y[i] = markers[i].Tvec.at<float>(1, 0) / markers[i].Tvec.at<float>(2, 0);
          // undistorted pixel
          u[i] = x[i] * camParam.CameraMatrix.at<float>(0, 0) + camParam.CameraMatrix.at<float>(0, 2);
          v[i] = y[i] * camParam.CameraMatrix.at<float>(1, 1) + camParam.CameraMatrix.at<float>(1, 2);
        }

        ROS_DEBUG_STREAM(
            "Mid point between the two markers in the image = (" << (x[0]+x[1])/2 << ", " << (y[0]+y[1])/2 << ")");

//        // paint a circle in the mid point of the normalized coordinates of both markers
//        cv::circle(inImage, cv::Point((u[0] + u[1]) / 2, (v[0] + v[1]) / 2), 3, cv::Scalar(0, 0, 255), cv::FILLED);

        // compute the midpoint in 3D:
        float midPoint3D[3]; // 3D point
        for (unsigned int i = 0; i < 3; ++i)
          midPoint3D[i] = (markers[0].Tvec.at<float>(i, 0) + markers[1].Tvec.at<float>(i, 0)) / 2;
        // now project the 3D mid point to normalized coordinates
        float midPointNormalized[2];
        midPointNormalized[0] = midPoint3D[0] / midPoint3D[2]; //x
        midPointNormalized[1] = midPoint3D[1] / midPoint3D[2]; //y
        u[0] = midPointNormalized[0] * camParam.CameraMatrix.at<float>(0, 0) + camParam.CameraMatrix.at<float>(0, 2);
        v[0] = midPointNormalized[1] * camParam.CameraMatrix.at<float>(1, 1) + camParam.CameraMatrix.at<float>(1, 2);

        ROS_DEBUG_STREAM(
            "3D Mid point between the two markers in undistorted pixel coordinates = (" << u[0] << ", " << v[0] << ")");

        // paint a circle in the mid point of the normalized coordinates of both markers
        cv::circle(inImage, cv::Point(u[0], v[0]), 3, cv::Scalar(0, 0, 255), cv::FILLED);

      }

      

      // draw a 3D cube in each marker if there is 3D info
      if (camParam.isValid() && marker_size != -1)
      {
        
        // Call the function to get the transformation matrix
        // Works. Fixing grid to base_link independent of camera views
        // Eigen::Matrix4f transform_matrix = getTransformMatrix(source_frame, target_frame);

        Eigen::Matrix4f transform_matrix;

          // std::lock_guard<std::mutex> lock(transform_mutex);
          if (!transform_cached)
          {
            transform_matrix = getTransformMatrix(source_frame, target_frame);
            // For some reason only if there is an error in the getTransformMatrix that says 
            // "Could not find a connection between 'base_link' and 'camera_color_optical_frame' because they are not part of the same tree.
            // Tf has two or more unconnected trees" 
            // Only after this the blue grid is being visualised well
            if (got_error)
            {
            cached_transform_matrix = transform_matrix;
            transform_cached = true;
            }
          }
          else
          {
            transform_matrix = cached_transform_matrix;
          }
        

        // std::vector<std::thread> threads;

        for (unsigned int i = 0; i < markers.size(); ++i)
        {
            // threads.emplace_back([&, i]() {
          cv::Mat transformed_Rvec, transformed_Tvec;
          transformPoseToBaseLink(markers[i].Rvec, markers[i].Tvec, transform_matrix, transformed_Rvec, transformed_Tvec);

          draw3dCube_scaled(inImage, transformed_Rvec, transformed_Tvec, camParam, 2, false, 6);
            // });
        }
        // }
        // for (auto& t : threads)
        // {
        //     t.join();
        // }

        // Works. Fixing grid to base_link independent of camera views. For loop and the next 3 lines in it
        // for (unsigned int i = 0; i < markers.size(); ++i)
        // {
        //   cv::Mat transformed_Rvec, transformed_Tvec;
        //   transformPoseToBaseLink(markers[i].Rvec, markers[i].Tvec, transform_matrix, transformed_Rvec, transformed_Tvec);
        //   draw3dCube_scaled(inImage, transformed_Rvec, transformed_Tvec, camParam, 2, false, 6);
        // }
      }

      if (image_pub.getNumSubscribers() > 0)
      {
        // ROS_INFO("In second if");
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage;
        image_pub.publish(out_msg.toImageMsg());
      }

      if (debug_pub.getNumSubscribers() > 0)
      {
        // ROS_INFO("In third if");
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector.getThresholdedImage();
        debug_pub.publish(debug_msg.toImageMsg());
      }

      ROS_DEBUG("runtime: %f ms", 1000 * (cv::getTickCount() - ticksBefore) / cv::getTickFrequency());
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
}

// wait for one camerainfo, then shut down that subscriber
void cam_info_callback(const sensor_msgs::CameraInfo &msg)
{
  camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
  cam_info_received = true;
  cam_info_sub.shutdown();
}

void reconf_callback(aruco_ros::ArucoThresholdConfig &config, std::uint32_t level)
{
  mDetector.setDetectionMode(aruco::DetectionMode(config.detection_mode), config.min_image_size);
  normalizeImageIllumination = config.normalizeImage;
  dctComponentsToRemove = config.dctComponentsToRemove;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_simple");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> server;
  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig>::CallbackType f_;
  f_ = boost::bind(&reconf_callback, _1, _2);
  server.setCallback(f_);

  normalizeImageIllumination = false;
  

  nh.param<bool>("image_is_rectified", useRectifiedImages, true);
  ROS_INFO_STREAM("Image is rectified: " << useRectifiedImages);

  image_transport::Subscriber image_sub = it.subscribe("/image", 1, &image_callback);
  cam_info_sub = nh.subscribe("/camera_info", 1, &cam_info_callback);

  cam_info_received = false;
  image_pub = it.advertise("result", 1);
  debug_pub = it.advertise("debug", 1);
  pose_pub1 = nh.advertise<geometry_msgs::Pose>("pose", 100);
  pose_pub2 = nh.advertise<geometry_msgs::Pose>("pose2", 100);
  // image_pub_2 = nh.advertise<sensor_msgs::Image>("fixed_grid_image", 1);

  poseStamped_pub1 = nh.advertise<geometry_msgs::PoseStamped>("poseStamped", 100);
  poseStamped_pub2 = nh.advertise<geometry_msgs::PoseStamped>("poseStamped2", 100);

  nh.param<double>("marker_size", marker_size, 0.05);
  nh.param<int>("marker_id1", marker_id1, 12);
  nh.param<int>("marker_id2", marker_id2, 8);
  nh.param<bool>("normalizeImage", normalizeImageIllumination, true);
  nh.param<int>("dct_components_to_remove", dctComponentsToRemove, 2);
  if (dctComponentsToRemove == 0)
    normalizeImageIllumination = false;
  nh.param<std::string>("parent_name", parent_name, "");
  nh.param<std::string>("child_name1", child_name1, "");
  nh.param<std::string>("child_name2", child_name2, "");
  // Get parameters from the ROS parameter server with default values
  nh.param<std::string>("source_frame", source_frame, "camera_color_optical_frame");
  nh.param<std::string>("target_frame", target_frame, "base_link");

  // ros::Subscriber image_sub_2 = nh.subscribe("camera/image_raw", 1, image_callback);

  if (parent_name == "" || child_name1 == "" || child_name2 == "")
  {
    ROS_ERROR("parent_name and/or child_name was not set!");
    return -1;
  }

  ROS_INFO("ArUco node started with marker size of %f meters and marker ids to track: %d, %d", marker_size, marker_id1,
           marker_id2);
  ROS_INFO("ArUco node will publish pose to TF with (%s, %s) and (%s, %s) as (parent,child).", parent_name.c_str(),
           child_name1.c_str(), parent_name.c_str(), child_name2.c_str());

  ros::spin();
}
