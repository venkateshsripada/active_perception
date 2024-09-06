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
#include <unordered_map>
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
#include <tuple>
#include <utility>  // For std::pair

// For scaling 3d cube
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cameraparameters.h"
#include "cvdrawingutils.h"
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

// #include <thread>
// #include <mutex>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <moveit_msgs/MoveGroupActionFeedback.h>
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
ros::Publisher pub_y;
ros::Publisher pub_x;
ros::Publisher pub_z;
ros::Publisher pub_GridVisualised;


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
cv::Mat initial_Rvec;
cv::Mat initial_Tvec;

Eigen::Matrix4f cached_transform_matrix;
std::mutex transform_mutex;
cv::Mat cubepoints(8, 3, CV_32F);
cv::Mat vertices_BaseLink_mapping;
bool transform_cached = false;
bool got_error = false;
bool got_new_marker = false;
int prev_marker_id;
aruco::Marker prev_marker;

bool got_TR_inital_marker = false;
cv::Mat marker1_BaseLink_matrix(4,4, CV_32F);
cv::Mat marker2_BaseLink_matrix;
bool correct_transform = false;
bool computed_cube_points = false;
bool got_vertices_map = false;

std::array<int, 4> all_markers = {2,8,5,10};

// Variables to draw cube
int scale = 3;
int numDots = 4;
int numDots_Z = 3;
int dotSize = 6;
float lift_Z = 0.4;

// Box in marker frame
float size_x = 0.5f * 0.42f* scale;
float size_y = -1e-2f* 2.0f * scale ;     // 0.05*0.01*scale
float size_z = 0.5f * 0.08f * scale;

// float init_x = -0.5f* 0.1f * scale ;  
// float init_y = -0.5f* 0.1f * scale ;  
float init_x = 0.0;
float init_y= 0.52;
float z_offset = 0.0;

cv::Scalar orangeColor(255, 128, 0, 255);
cv::Scalar purpleColor(51, 51, 255, 255);
cv::Scalar pinkColor(255, 0, 255, 255);
cv::Scalar yellowColor(255, 255, 0, 255);
cv::Scalar redColor(255, 0, 0, 255);
cv::Scalar greenColor(0, 255, 0, 255);
cv::Scalar blueColor(0, 0, 255, 255);
cv::Scalar blackColor(0, 0, 0, 255);

unordered_map<int, cv::Scalar> z_axis_grids_colours;
cv::Mat vertices_pose_map_y(numDots, numDots, CV_32F);
cv::Mat vertices_pose_map_x(numDots, numDots, CV_32F);
cv::Mat vertices_pose_map_z(numDots, 1, CV_32F);

std::vector<float> vec_x;
std::vector<float> vec_y;
std::vector<float> vec_z;
// cv::Mat vertices_pose_map(numDots, numDots, CV_32FC3, cv::Scalar(0, 0, 0));

using KeyType = std::tuple<int,int>;
using ValueType = std::vector<float, float>;

// struct {
//   double member1;
//   double member2;
// } template_struct;

// template_struct created_template_struct;



// Custom hash function for std::pair<int, int>
namespace std {
    template <>
    struct hash<std::pair<int, int>> {
        size_t operator()(const std::pair<int, int>& p) const noexcept {
            auto hash1 = std::hash<int>{}(p.first);
            auto hash2 = std::hash<int>{}(p.second);
            return hash1 ^ (hash2 << 1); // XOR and bit-shift to combine the two hash values
        }
    };
}

unordered_map<std::pair<int, int>, std::tuple<float, float>, std::hash<std::pair<int, int>>> vertices_pose;


// cv::Mat initial_image;
// Eigen::Matrix4f transform_matrix;
// cv::Mat background_image;  // Static image with the fixed grid
// bool background_image_set = false;  // Flag to check if background image is set

// moveit_msgs::MoveGroupActionFeedback move_group_feedback_data;

// void move_group_callback(const moveit_msgs::MoveGroupActionFeedback::ConstPtr& msg) {
//     move_group_feedback_data = *msg;
// }

Eigen::Matrix4f transformPoseToBaseLink(const cv::Mat& Rvec, const cv::Mat& Tvec, const Eigen::Matrix4f& transform_matrix){
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
    // cv::Mat transformed_Rmat;
    // cv::eigen2cv(transformed_R, transformed_Rmat);
    // cv::Rodrigues(transformed_Rmat, transformed_Rvec);
    // cv::eigen2cv(transformed_T, transformed_Tvec);

    // This is the marker's translation and rotation to be used in Active Perception pipeline                                                    
    for(int i = 0; i < transformed_pose.rows(); ++i) {
        for(int j = 0; j < transformed_pose.cols(); ++j) {
            ROS_INFO("Base link matrix(%d,%d) = %f", i, j, transformed_pose(i,j));
        }
    }

    return transformed_pose;

    
}

// Function to get the transformation matrix from source_frame to target_frame
Eigen::Matrix4f getTransformMatrix(const std::string& source_frame, const std::string& target_frame) {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    
    try {
        // Wait for the transform to be available
        listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3.0));

        // Get the transformation
        listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);

        // Convert to a 4x4 matrix
        tf::Matrix3x3 rotation_matrix = transform.getBasis();
        tf::Vector3 translation_vector = transform.getOrigin();

        for (int i = 0; i < 3; ++i) {
          tf::Vector3 row = rotation_matrix.getRow(i);
          ROS_INFO("rotation_matrix %d: [%f, %f, %f]", i, row.x(), row.y(), row.z());
        }

        ROS_INFO("Translation vector : [%f, %f, %f]", translation_vector.getX(), translation_vector.getY(), translation_vector.getZ());
        
        
        // Eigen::Matrix3f eigen_rotation;
        // eigen_rotation << 
        //     rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2],
        //     rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2],
        //     rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2];

        Eigen::Matrix3f eigen_rotation;
        eigen_rotation << 
            rotation_matrix.getRow(0).x(), rotation_matrix.getRow(0).y(), rotation_matrix.getRow(0).z(),
            rotation_matrix.getRow(1).x(), rotation_matrix.getRow(1).y(), rotation_matrix.getRow(1).z(),
            rotation_matrix.getRow(2).x(), rotation_matrix.getRow(2).y(), rotation_matrix.getRow(2).z();


        Eigen::Vector3f eigen_translation(translation_vector.getX(), translation_vector.getY(), translation_vector.getZ());
        
        // matrix.setIdentity();
        matrix.block<3, 3>(0, 0) = eigen_rotation;
        matrix.block<3, 1>(0, 3) = eigen_translation;
        // matrix.block<3, 3>(0, 0) = Eigen::Matrix3f(rotation_matrix);
        // matrix.block<3, 1>(0, 3) = Eigen::Vector3f(translation_vector.getX(), translation_vector.getY(), translation_vector.getZ());

        // Optionally, set the bottom row if needed (though it is usually already [0, 0, 0, 1])
        // matrix(3, 0) = 0.0f;
        // matrix(3, 1) = 0.0f;
        // matrix(3, 2) = 0.0f;
        // matrix(3, 3) = 1.0f;
        // for(int i = 0; i < matrix.rows(); ++i) {
        //   for (int j = 0; j < matrix.cols(); ++j) {
        //     ROS_INFO("Transform Matrix in function(%d,%d) = %f", i, j, matrix(i,j));
            
        //   }
        // }

        

    } catch (tf::TransformException &ex) {
        got_error = true;
        ROS_ERROR("%s", ex.what());
        // Return an identity matrix in case of an error
        // matrix.setIdentity();
        
    }

    return matrix;
}




cv::Mat get_baseLinkMatrix(cv::Mat& Rvec, cv::Mat& Tvec){

  cv::Mat marker_matrix;
    // Convert Rvec and Tvec to Eigen
  cv::Mat Rmat;
  cv::Rodrigues(Rvec, Rmat);
  Eigen::Matrix3f R;
  cv::cv2eigen(Rmat, R);
  Eigen::Vector3f T;
  cv::cv2eigen(Tvec, T);

  // Create 4x4 transformation matrix
  Eigen::Matrix4f temp_marker_matrix = Eigen::Matrix4f::Identity();
  temp_marker_matrix.block<3, 3>(0, 0) = R;
  temp_marker_matrix.block<3, 1>(0, 3) = T;
  // Convert back to cv::Mat
  cv::eigen2cv(temp_marker_matrix, marker_matrix);
  got_new_marker = false;
  return marker_matrix;

}

// Function to flatten a cv::Mat to a std::vector<float>
std::vector<float> matToVector(const cv::Mat& mat) {
    std::vector<float> vec;
    if (mat.isContinuous()) {
        vec.assign((float*)mat.datastart, (float*)mat.dataend);
    } else {
        for (int i = 0; i < mat.rows; ++i) {
            vec.insert(vec.end(), mat.ptr<float>(i), mat.ptr<float>(i) + mat.cols);
        }
    }
    return vec;
}

void create_gridMapping(float start, float end, int index, bool fixed_x){
    float lineLength = cv::norm(end - start);
    float interval = lineLength / (numDots - 1);  // Calculate interval based on numDots

    for(int j = 0; j < numDots; ++j) {
        
      if (!fixed_x)
        {float val = start - j * interval;
          vertices_pose_map_y.at<float>(index, numDots - 1 -j) = val;
        ROS_INFO("Vertices map pose y (%d, %d) = %f", index, numDots - 1 -j, val);}
      else
        {float val = end + j * interval;
          vertices_pose_map_x.at<float>(j, numDots-1-index) = val;
        ROS_INFO("Vertices map pose x(%d, %d) = %f", j, numDots-1-index, val);}
        
    }
}

void create_gridMapping_Z(float start, float end, float offset){
    float lineLength = cv::norm(end - start);
    float interval = lineLength / (numDots_Z - 1);  // Calculate interval based on numDots

    for(int j = 0; j < numDots_Z; ++j) {
        
        float val = start + j * interval + offset;
        vertices_pose_map_z.at<float>(j, 0) = val;
        ROS_INFO("Vertices map pose z (%d, %d) = %f",j, 0, val);        
    }
}

void createMapping(float init_start, float init_end, float opposite_start, float opposite_end, bool fixed_x){

  // Input is i, (i+1)%4 and (i + 2)%4, (i + 3)%4 
  // i.e first and third line
  float init_lineLength = cv::norm(init_end - init_start);
  float opposite_lineLength = cv::norm(opposite_end - opposite_start);

  float init_interval = init_lineLength / (numDots - 1);  // Calculate interval based on numDots
  float opposite_interval = opposite_lineLength / (numDots - 1);  // Calculate interval based on numDots

  for(int i = 0; i < numDots; ++i) {
      float init_val = init_start + i * init_interval;
      float opposite_val = opposite_end -  i * opposite_interval;

      create_gridMapping(init_val, opposite_val, i, fixed_x);
    }

  // Add to dictionary
  // float lineLength = cv::norm(end - start);
  // float interval = lineLength / (numDots - 1);  // Calculate interval based on numDots
  // for(int i = 0; i < numDots; ++i) {
  //   float distance = i * interval;
  //   float dot = start + distance; // Distance from the start of line segment
  //   vertices_pose_map.at<float>()
  //   if (index % 2 == 0)
  //   {vertices_pose[std::make_pair(index, i)] = std::make_tuple(dot, fixed_coord);}   // Key = coordinates, dot 0,2,4,6 values = [changing x distance, fixed y coordinate]
  //   else{
  //     vertices_pose[std::make_pair(index, i)] = std::make_tuple(fixed_coord, dot);}  // Key = coordinates, dot 1,3,5,7 values = [fixed x coordinate, changing y distance]
  // }

}

cv::Mat calculate_transfoms(int& scale){
    cv::Mat objectPoints(8,3, CV_32F);
  
    // Box in marker frame
    // float size_x = 0.5f * 0.4f* scale;
    // float size_y = 0.4f* 0.01f * scale ;   
    // float size_z = 0.5f * 0.1f * scale;

    // // float init_x = -0.5f* 0.1f * scale ;  
    // // float init_y = -0.5f* 0.1f * scale ;  
    // float init_x = 0.0;
    // float init_y= 0.5;
    // float z_offset = 0.0;
  
    objectPoints.at<float>(0, 0) = init_x; objectPoints.at<float>(0, 1) = init_y; objectPoints.at<float>(0, 2) = z_offset;
    objectPoints.at<float>(1, 0) = size_x; objectPoints.at<float>(1, 1) = init_y; objectPoints.at<float>(1, 2) = z_offset;
    objectPoints.at<float>(2, 0) = size_x; objectPoints.at<float>(2, 1) = -size_y; objectPoints.at<float>(2, 2) = z_offset;
    objectPoints.at<float>(3, 0) = init_x; objectPoints.at<float>(3, 1) = -size_y; objectPoints.at<float>(3, 2) = z_offset;

    objectPoints.at<float>(4, 0) = init_x; objectPoints.at<float>(4, 1) = init_y; objectPoints.at<float>(4, 2) = size_z + z_offset;
    objectPoints.at<float>(5, 0) = size_x; objectPoints.at<float>(5, 1) = init_y; objectPoints.at<float>(5, 2) = size_z + z_offset;
    objectPoints.at<float>(6, 0) = size_x; objectPoints.at<float>(6, 1) = -size_y; objectPoints.at<float>(6, 2) = size_z+ z_offset;
    objectPoints.at<float>(7, 0) = init_x; objectPoints.at<float>(7, 1) = -size_y; objectPoints.at<float>(7, 2) = size_z + z_offset;

  // This is the box around the first marker used as refernce for all to move to
  // cv::Mat inner_boxMatrix_markerLink = cv::Mat::eye(4,4, CV_32F);
  // cv::Mat outer_boxMatrix_markerLink = cv::Mat::eye(4,4, CV_32F);
  cv::Mat boxMatrix_markerLink(8,4, CV_32F);

  // inner_boxMatrix_markerLink.at<float>(0,3) = objectPoints.at<float>(0,0);
  // inner_boxMatrix_markerLink.at<float>(1, 3) = objectPoints.at<float>(0, 1); 

  // outer_boxMatrix_markerLink.at<float>(0,3) = objectPoints.at<float>(4,0);
  // outer_boxMatrix_markerLink.at<float>(1, 3) = objectPoints.at<float>(4, 1); 
  // outer_boxMatrix_markerLink.at<float>(2, 3) = objectPoints.at<float>(4, 2);
  objectPoints.copyTo(boxMatrix_markerLink(cv::Range(0, 8), cv::Range(0, 3)));
  boxMatrix_markerLink.col(3).setTo(cv::Scalar(1.0));
  cv::Mat box_matrixMarker_transposed = boxMatrix_markerLink.t();

  // ROS_INFO("rows are %d and columns are %d", box_matrixMarker_transposed.rows, box_matrixMarker_transposed.cols);
  // for(int i = 0; i < box_matrixMarker_transposed.rows; ++i) {
  //   for (int j = 0; j < box_matrixMarker_transposed.cols; ++j) {
  //     ROS_INFO("box intial Matrix(%d,%d) = %f", i, j, box_matrixMarker_transposed.at<float>(i,j));
  //   }
  // }
  
  // Map vertices to poses
  // box around marker wrt base_link
  cv::Mat box_BaseLink(4,8,CV_32F);
  if (!marker1_BaseLink_matrix.empty() && !got_vertices_map){
    box_BaseLink = marker1_BaseLink_matrix * box_matrixMarker_transposed;
    cv::Mat box_BaseLink_transposed = box_BaseLink.t();
    vertices_BaseLink_mapping = box_BaseLink_transposed(cv::Range::all(), cv::Range(0, 3));

    // Create a new column, here we initialize it to zero but you can fill it with any data
    // cv::Mat new_column = cv::Mat::zeros(8, 1, box_BaseLink_8x3.type());
    // ROS_INFO("Here");
    // // Add the new column to the 8x3 matrix
    // cv::Mat box_BaseLink_8x4;
    // cv::hconcat(box_BaseLink_8x3, new_column, box_BaseLink_8x4);
    // ROS_INFO("Step 2");
    // // Reshape the resulting 8x4 matrix into an 8x2x1 matrix
    // int dims[3] = {8, 2, 2};  // {rows, cols, channels}
    // vertices_BaseLink_mapping = box_BaseLink_8x4.reshape(1, 3, dims);
    // ROS_INFO("Step 3");
    got_vertices_map = true;
    for(int i = 0; i < vertices_BaseLink_mapping.size[0]; ++i) {
      for (int j = 0; j < vertices_BaseLink_mapping.size[1]; ++j) {
        // for (int k = 0; k < vertices_BaseLink_mapping.size[2]; ++k) {
        ROS_INFO("Verices map(%d,%d) = %f", i, j, vertices_BaseLink_mapping.at<float>(i,j));
        }
      }
    // }

    // Inner cube
    for(int i = 0; i < 2; ++i) {
      if (i %2)
        {  bool fixed_x = true;
        createMapping(vertices_BaseLink_mapping.at<float>(i,0), vertices_BaseLink_mapping.at<float>(i+1,0), vertices_BaseLink_mapping.at<float>(i+2,0), vertices_BaseLink_mapping.at<float>(i+2+1,0), fixed_x);}
          // Access and print the values
        //   for (const auto& pair : vertices_pose) {
        //       const auto& key = pair.first;
        //       const auto& value = pair.second;
        //       ROS_INFO("vertices_pose_dictionary[( %d, %d)] = {%f, %f}", key.first, key.second, std::get<0>(value), std::get<1>(value));
        //   }
        // }  // x divided into 4
      else
        { bool fixed_x = false;
          createMapping(vertices_BaseLink_mapping.at<float>(i,1), vertices_BaseLink_mapping.at<float>((i+1)%4,1), vertices_BaseLink_mapping.at<float>(i+2,1), vertices_BaseLink_mapping.at<float>((i+2+1)%4,1), fixed_x);}  // y divided into 4
      }

    // Z axis
    create_gridMapping_Z(vertices_BaseLink_mapping.at<float>(0,2), vertices_BaseLink_mapping.at<float>(4,2), lift_Z);

    // Convert each cv::Mat to std::vector<float>
    vec_x = matToVector(vertices_pose_map_x);
    vec_y = matToVector(vertices_pose_map_y);
    vec_z = matToVector(vertices_pose_map_z);

    // Fill the vertices_pose_map with data
    // // outer cube
    // for(int i = 0; i < 4; ++i) {
    //   if (i % 2 == 0)
    //     {createMapping(vertices_BaseLink_mapping.at<float>(i+4,0), vertices_BaseLink_mapping.at<float>(4+ i+1,0), vertices_BaseLink_mapping.at<float>(i+4, 1), i+4);}  // x divided into 4
    //   else
    //     {createMapping(vertices_BaseLink_mapping.at<float>(i+4,1), vertices_BaseLink_mapping.at<float>(4 + (i+1)%4,1), vertices_BaseLink_mapping.at<float>(i+4,0), i+4);}  // y divided into 4
    //   }

  }// end if

  // objectPoints(cv::Rect(0,4,3,4)).copyTo(outer_boxMatrix_markerLink(cv::Rect(0,0,3,4))); //.rowRange(0, 4).colRange(0, 3); // 4 rows 3 columns
  // outer_boxMatrix_markerLink.col(3).rowRange(0, 4) = 1.0f;  // Making it a homogennous matrix


  // This is the box around the first marker used as refernce for all to move to
  cv::Mat box_init_marker_baseLink(4,8, CV_32F);
  cv::Mat marker2_BaseLink_inverse;
  // cv::Mat outer_box_init_marker_baseLink(4,4, CV_32F);
  if (!marker1_BaseLink_matrix.empty() && !marker2_BaseLink_matrix.empty()){
      if (cv::invert(marker2_BaseLink_matrix, marker2_BaseLink_inverse, cv::DECOMP_SVD)) {
        // Perform matrix multiplication
        box_init_marker_baseLink = marker2_BaseLink_inverse * marker1_BaseLink_matrix * box_matrixMarker_transposed;
      }
      else{
        ROS_INFO("Unable to compute inversion and multiplication");
      }
  

  // Transpose the matrix
  cv::Mat box_init_marker_baseLink_transposed = box_init_marker_baseLink.t();
  // Remove the last column to make it 8x3
  cubepoints = box_init_marker_baseLink_transposed(cv::Range::all(), cv::Range(0, 3));

  // Round values to 2 decimal places
  for (int i = 0; i < cubepoints.rows; ++i) {
    for (int j = 0; j < cubepoints.cols; ++j) {
        float& value = cubepoints.at<float>(i, j);
        value = std::round(value * 100) / 100.0f;
    }
  }

  // for(int i = 0; i < cubepoints.rows; ++i) {
  //   for (int j = 0; j < cubepoints.cols; ++j) {
  //     ROS_INFO("Cube points(%d,%d) = %f", i, j, cubepoints.at<float>(i,j));
      
  //   }
  // }
  computed_cube_points = true;
  } 
  return cubepoints;
}




void drawDotsOnLine(cv::Mat &Image, cv::Point2f start, cv::Point2f end, cv::Scalar color, bool annotate, int index) {
    float lineLength = cv::norm(end - start);

    cv::Point2f direction = (end - start) / lineLength;  // Unit direction vector

    float interval = lineLength / (numDots - 1);  // Calculate interval based on numDots

    for(int i = 0; i < numDots; ++i) {
        float distance = i * interval;
        cv::Point2f dot = start + direction * distance;
        cv::circle(Image, dot, dotSize, color, cv::FILLED);
        if (annotate){
          cv::putText(Image, std::to_string(index) + "; " + std::to_string(numDots - 1- i), dot, cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(118, 185, 0), 2);
          // First, render the `index` in green
          // cv::putText(Image, std::to_string(index), dot, cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(118, 185, 0), 2);

          // // Calculate the width of the `index` text to determine where to start drawing the comma
          // int baseline = 0;
          // cv::Size indexTextSize = cv::getTextSize(std::to_string(index), cv::FONT_HERSHEY_DUPLEX, 1.0, 2, &baseline);
          // cv::Point commaPos(dot.x + indexTextSize.width, dot.y);  // Position the comma after the index

          // // Render the comma `","` in black
          // cv::putText(Image, ",", commaPos, cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 0, 0), 2);

          // // Calculate the width of the comma to determine where to start drawing the next text
          // cv::Size commaTextSize = cv::getTextSize(",", cv::FONT_HERSHEY_DUPLEX, 1.0, 2, &baseline);
          // cv::Point nextTextPos(commaPos.x + commaTextSize.width, commaPos.y);  // Position the next text after the comma

          // // Render the `(numDots - 1 - i)` in yellow
          // cv::putText(Image, std::to_string(numDots - 1 - i), nextTextPos, cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 255, 0), 2);


        }
    }    
}


void drawDotsOnLine_Z(cv::Mat &Image, cv::Point2f start, cv::Point2f end, cv::Scalar color, bool annotate, int index) {
    float lineLength = cv::norm(end - start);

    cv::Point2f direction = (end - start) / lineLength;  // Unit direction vector

    float interval = lineLength / (numDots - 1);  // Calculate interval based on numDots

    for(int i = 0; i < numDots_Z; ++i) {
        float distance = i * interval;
        cv::Point2f dot = start + direction * distance;
        cv::circle(Image, dot, dotSize, color, cv::FILLED);
        if (annotate){
          cv::putText(Image, std::to_string(index) + "," + std::to_string(numDots_Z - 1- i), dot, cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(118, 185, 0), 2);
        }
    }    
}

void drawmesh(cv::Mat &Image, cv::Point2f init_start, cv::Point2f init_end, cv::Point2f opposite_start, cv::Point2f opposite_end, cv::Scalar line_color, cv::Scalar dot_color, int line_size, bool annotate){
    // Input is i, (i+1)%4 and (i + 2)%4, (i + 3)%4 
    // i.e first and third line
    float init_lineLength = cv::norm(init_end - init_start);
    float opposite_lineLength = cv::norm(opposite_end - opposite_start);

    cv::Point2f init_direction = (init_end - init_start) / init_lineLength;  // Unit direction vector
    cv::Point2f opposite_direction = (opposite_end - opposite_start) / opposite_lineLength;  // Unit direction vector

    float init_interval = init_lineLength / (numDots - 1);  // Calculate interval based on numDots
    float opposite_interval = opposite_lineLength / (numDots - 1);  // Calculate interval based on numDots

    for(int i = 0; i < numDots; ++i) {
        float init_distance = i * init_interval;
        float opposite_distance = i * opposite_interval;
        cv::Point2f init_dot = init_start + init_direction * init_distance;
        cv::Point2f opposite_dot = opposite_end - opposite_direction * opposite_distance;

        drawDotsOnLine(Image, init_dot, opposite_dot, dot_color, annotate, i);
        // if (annotate){
        //   cv::putText(Image, std::to_string(i), init_dot, cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(118, 185, 0), 2);
        //   cv::putText(Image, std::to_string(numDots - i), opposite_dot, cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(118, 185, 0), 2);
        // }
        cv::line(Image, init_dot, opposite_dot, line_color, line_size);
      }
}


void drawmesh_Z(cv::Mat &Image, cv::Point2f init_start, cv::Point2f init_end, cv::Point2f opposite_start, cv::Point2f opposite_end, cv::Scalar line_color, cv::Scalar dot_color, int line_size, bool annotate){
    // Input is i, (i+1)%4 and (i + 2)%4, (i + 3)%4 
    // i.e first and third line
    float init_lineLength = cv::norm(init_end - init_start);
    float opposite_lineLength = cv::norm(opposite_end - opposite_start);

    cv::Point2f init_direction = (init_end - init_start) / init_lineLength;  // Unit direction vector
    cv::Point2f opposite_direction = (opposite_end - opposite_start) / opposite_lineLength;  // Unit direction vector

    float init_interval = init_lineLength / (numDots_Z - 1);  // Calculate interval based on numDots
    float opposite_interval = opposite_lineLength / (numDots_Z - 1);  // Calculate interval based on numDots

    for(int i = 0; i < numDots_Z; ++i) {
        float init_distance = i * init_interval;
        float opposite_distance = i * opposite_interval;
        cv::Point2f init_dot = init_start + init_direction * init_distance;
        cv::Point2f opposite_dot = opposite_end - opposite_direction * opposite_distance;

        drawDotsOnLine_Z(Image, init_dot, opposite_dot, dot_color, annotate, i);
        // if (annotate){
        //   cv::putText(Image, std::to_string(i), init_dot, cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(118, 185, 0), 2);
        //   cv::putText(Image, std::to_string(numDots - i), opposite_dot, cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(118, 185, 0), 2);
        // }
        cv::line(Image, init_dot, opposite_dot, line_color, line_size);
      }
}

void drawmesh_alongZ(cv::Mat &Image, cv::Point2f init_start, cv::Point2f init_end, cv::Point2f opposite_start, cv::Point2f opposite_end, cv::Scalar line_color, cv::Scalar dot_color, int line_size, std::vector<cv::Point2f>& z_axis_grids){
    // Input is i, (i+1)%4 and (i + 2)%4, (i + 3)%4 
    // i.e first and third line
    float init_lineLength = cv::norm(init_end - init_start);
    float opposite_lineLength = cv::norm(opposite_end - opposite_start);

    cv::Point2f init_direction = (init_end - init_start) / init_lineLength;  // Unit direction vector
    cv::Point2f opposite_direction = (opposite_end - opposite_start) / opposite_lineLength;  // Unit direction vector

    float init_interval = init_lineLength / (numDots_Z - 1);  // Calculate interval based on numDots
    float opposite_interval = opposite_lineLength / (numDots_Z - 1);  // Calculate interval based on numDots

    for(int i = 0; i < numDots_Z; ++i) {
        float init_distance = i * init_interval;
        float opposite_distance = i * opposite_interval;
        cv::Point2f init_dot = init_start + init_direction * init_distance;
        cv::Point2f opposite_dot = opposite_start + opposite_direction * opposite_distance;

        // drawDotsOnLine(Image, init_dot, opposite_dot, dot_color);
        cv::line(Image, init_dot, opposite_dot, line_color, line_size);
        z_axis_grids.push_back(init_dot);
        z_axis_grids.push_back(opposite_dot);
      }
}

void draw3dCube_scaled(cv::Mat& Image, cv::Mat& cubepoints, const cv::Mat& Rvec, const cv::Mat& Tvec, const aruco::CameraParameters& CP, int lineSize, bool setYperpendicular, double scale)
 {
   cv::Mat objectPoints(8, 3, CV_32F);
   cv::Mat cubePoints(8,3,CV_32F);
  //  float halfSize = 0.05f * 0.5f * scale ;
  

  
    objectPoints.at<float>(0, 0) = init_x; objectPoints.at<float>(0, 1) = init_y; objectPoints.at<float>(0, 2) = z_offset;
    objectPoints.at<float>(1, 0) = size_x; objectPoints.at<float>(1, 1) = init_y; objectPoints.at<float>(1, 2) = z_offset;
    objectPoints.at<float>(2, 0) = size_x; objectPoints.at<float>(2, 1) = -size_y; objectPoints.at<float>(2, 2) = z_offset;
    objectPoints.at<float>(3, 0) = init_x; objectPoints.at<float>(3, 1) = -size_y; objectPoints.at<float>(3, 2) = z_offset;

    objectPoints.at<float>(4, 0) = init_x; objectPoints.at<float>(4, 1) = init_y; objectPoints.at<float>(4, 2) = size_z + z_offset;
    objectPoints.at<float>(5, 0) = size_x; objectPoints.at<float>(5, 1) = init_y; objectPoints.at<float>(5, 2) = size_z + z_offset;
    objectPoints.at<float>(6, 0) = size_x; objectPoints.at<float>(6, 1) = -size_y; objectPoints.at<float>(6, 2) = size_z+ z_offset;
    objectPoints.at<float>(7, 0) = init_x; objectPoints.at<float>(7, 1) = -size_y; objectPoints.at<float>(7, 2) = size_z + z_offset;
    

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
  // std::vector<Point2f> imagePoints;
  // cv::projectPoints(objectPoints, Rvec, Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);
  std::vector<Point2f> imagePoints;
  if (computed_cube_points){
    // ROS_INFO("In cubepoints");
    for(int i = 0; i < cubepoints.rows; ++i) {
    for (int j = 0; j < cubepoints.cols; ++j) {
      cubePoints.at<float>(i,j) = cubepoints.at<float>(i,j);
    }
  }
    cv::projectPoints(cubePoints, Rvec, Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);
  }
  else{
   cv::projectPoints(objectPoints, Rvec, Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);
  }
   // draw lines of different colours
  //  ROS_INFO("Length of imagePoints is %d", imagePoints.size());

  std::vector<cv::Point2f> z_axis_grids;
  z_axis_grids.clear();
   for (int i = 0; i < 4; i++)
   {
     cv::line(Image, imagePoints[i], imagePoints[i + 4], blueColor, lineSize);
      // Draw dots on line taking first and third coordinates
      drawmesh_alongZ(Image, imagePoints[(i+4) % 4], imagePoints[4 + (i + 4) % 4],  imagePoints[(i+1) % 4], imagePoints[(4 + (i+1) % 4)], blueColor, redColor, lineSize, z_axis_grids);
   }

   for (int i = 0; i < numDots_Z*2; ++i){
      bool annotate_text;
      if (i < numDots_Z){annotate_text = true;}
      else {annotate_text = false;}

      drawmesh(Image, z_axis_grids[2*i], z_axis_grids[2*i + 1], z_axis_grids[2*i + (numDots_Z*4)], z_axis_grids[(2*i + 1) + (numDots_Z*4)], z_axis_grids_colours[(2*i) % (numDots_Z*2)], redColor, lineSize, annotate_text);
    }

  //   for (int i=0; i<z_axis_grids.size(); i++)
  //  {
  //   ROS_INFO("gridpoints[%d] = (%.2f, %.2f)", 2*i, z_axis_grids[i].x, z_axis_grids[i].y);
  //   ROS_INFO("gridpoints[%d] = (%.2f, %.2f)", 2*i + 1, z_axis_grids[2*i + 1].x, z_axis_grids[2*i + 1].y);
  //  }

  // Inner square
   for (int i = 0; i < 4; i++)
   {
    cv::line(Image, imagePoints[i], imagePoints[(i + 1) % 4], blueColor, lineSize);
    // Draw dots along the line
    // drawDotsOnLine(Image, imagePoints[i], imagePoints[(i + 1) % 4], 5, cv::Scalar(255, 0, 0, 255), 3);
    if (i<2){
      // Draw dots on line taking first and third coordinates
      drawmesh(Image, imagePoints[i], imagePoints[(i+1) % 4],  imagePoints[i+2], imagePoints[(i+3) % 4], blueColor, redColor, lineSize, false);
    }
   }
  
   // Outer square
   for (int i = 0; i < 4; i++)
   {
     cv::line(Image, imagePoints[i + 4], imagePoints[4 + (i + 1) % 4], greenColor, lineSize);
      if (i<2){
      // Draw dots on line taking first and third coordinates
      drawmesh(Image, imagePoints[i+ 4], imagePoints[4 + (i + 1) % 4],  imagePoints[(i+2) + 4], imagePoints[4 + (i + 3) % 4], greenColor, redColor, lineSize, false);
      }
   }
    
    
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
      // For active perception, there is the dd
      if (camParam.isValid() && marker_size != -1)
      {
        
        
        Eigen::Matrix4f transform_matrix;
        Eigen::Matrix4f transformed_baseLink_matrix;
        const float epsilon = 1e-5f;
        

          // std::lock_guard<std::mutex> lock(transform_mutex);
          if (!transform_cached)
          {
            // if (move_group_feedback_data.feedback.state == "IDLE"){
            //     ROS_INFO("Data is correct");

                transform_matrix = getTransformMatrix(source_frame, target_frame);
                // For some reason only if there is an error in the getTransformMatrix that says 
                // "Could not find a connection between 'base_link' and 'camera_color_optical_frame' because they are not part of the same tree.
                // Tf has two or more unconnected trees" 
                // Only after this the blue grid is being visualised well
                if (got_error)
                {
                transform_matrix = getTransformMatrix(source_frame, target_frame);
                cached_transform_matrix = transform_matrix;
                transform_cached = true;
                prev_marker_id = markers[0].id;
                ROS_INFO("Prev marker ID is %d", prev_marker_id);
                for(int i = 0; i < cached_transform_matrix.rows(); ++i) {
                    for (int j = 0; j < cached_transform_matrix.cols(); ++j) {
                      ROS_INFO("Cached transform Matrix(%d,%d) = %f", i, j, cached_transform_matrix(i,j));
                      
                    }
                  }
                }
            // }
          }
          else
          {
            
            transform_matrix = cached_transform_matrix;
            correct_transform = true;
            if (std::abs(transform_matrix(0,1) - 0.0f) < epsilon)
            { transform_cached = false;
              correct_transform = false;
              // for(int i = 0; i < transform_matrix.rows(); ++i) {
              //     for (int j = 0; j < transform_matrix.cols(); ++j) {
              //       ROS_INFO("transform Matrix(%d,%d) = %f", i, j, transform_matrix(i,j));
                    
              //     }
              // }
            }
            std::vector<int> visible_markers; 
            visible_markers.clear();
            for (unsigned int i = 0; i < markers.size(); ++i)
            {
              visible_markers.push_back(markers[i].id);
            }

            for (unsigned int i = 0; i < markers.size(); ++i)
            {
                // threads.emplace_back([&, i]() {
              cv::Mat transformed_Rvec, transformed_Tvec;
            
              auto it = std::find(all_markers.begin(), all_markers.end(), markers[0].id);
              auto prev_marker_visible = std::find(visible_markers.begin(), visible_markers.end(), prev_marker_id);
              // Check if the detected marker is valid and if it different from previous marker. Also vreify that previous marker is not currently visible
              if (it != all_markers.end() && markers[0].id != prev_marker_id && prev_marker_visible == visible_markers.end()){
                transform_cached = false;
                got_error = false;
                correct_transform = false;
                got_new_marker = true;
                ROS_INFO("New marker is %d", markers[0].id);
              }

              // if (correct_transform){
              // transformed_baseLink_matrix = transformPoseToBaseLink(markers[0].Rvec, markers[0].Tvec, transform_matrix, transformed_Rvec, transformed_Tvec);
              
              // }
              // The translation and rotation matrices of the first marker wrt base_link (ideally marker placed closest to base of robot)
              if (correct_transform && !got_TR_inital_marker && markers[0].id == 8){
                ROS_INFO("Saved intial marker translation and rotation. Marker ID is %d", markers[0].id);
                // initial_Tvec = transformed_Tvec;
                // initial_Rvec = transformed_Rvec;
                // Eigen::Matrix3f transformed_R = transformed_pose.block<3, 3>(0, 0);
                Eigen::Matrix4f eigen_marker1_BaseLink_matrix;
                eigen_marker1_BaseLink_matrix = transformPoseToBaseLink(markers[0].Rvec, markers[0].Tvec, transform_matrix);
                // Copy the data from the Eigen matrix to the cv::Mat
                cv::eigen2cv(eigen_marker1_BaseLink_matrix, marker1_BaseLink_matrix);
                // for (int i = 0; i < 4; ++i) {
                //     for (int j = 0; j < 4; ++j) {
                //         ROS_INFO("After conversion to cv matrix(%d,%d) = %f", i, j,marker1_BaseLink_matrix.at<float>(i, j));
                //     }
                // }
                got_TR_inital_marker = true;
              }
              
              else if (correct_transform && got_new_marker)
              {
                ROS_INFO("Got second matrix as well");
                Eigen::Matrix4f eigen_marker2_BaseLink_matrix;
                eigen_marker2_BaseLink_matrix = transformPoseToBaseLink(markers[0].Rvec, markers[0].Tvec, transform_matrix);
                // marker2_BaseLink_matrix = get_baseLinkMatrix(transformed_Rvec, transformed_Tvec);
                eigen2cv(eigen_marker2_BaseLink_matrix, marker2_BaseLink_matrix);
                got_new_marker = false;
              }

              if (correct_transform){
                cubepoints = calculate_transfoms(scale);
                draw3dCube_scaled(inImage, cubepoints, markers[0].Rvec, markers[0].Tvec, camParam, 2, false, scale);
                std_msgs::Bool grid_visible;
                grid_visible.data = true;
                pub_GridVisualised.publish(grid_visible);
                }
              else{
                std_msgs::Bool grid_visible;
                grid_visible.data = false;
                pub_GridVisualised.publish(grid_visible);
              }
            }
        }
         
        // Create ROS messages for each vertex_pose_matrix
        std_msgs::Float32MultiArray msg_y;
        msg_y.data = vec_y;

        std_msgs::Float32MultiArray msg_x;
        msg_x.data = vec_x;

        std_msgs::Float32MultiArray msg_z;
        msg_z.data = vec_z;

        pub_x.publish(msg_x);
        pub_y.publish(msg_y);                
        pub_z.publish(msg_z);
      }
      else{
        std_msgs::Bool grid_visible;
        grid_visible.data = false;
        pub_GridVisualised.publish(grid_visible);
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

  // ros::Subscriber move_group_feedback = nh.subscribe("/move_group/feedback", 1000, move_group_callback);
  z_axis_grids_colours[0] = blueColor;
  z_axis_grids_colours[2] = blackColor;
  z_axis_grids_colours[4] = redColor;
  z_axis_grids_colours[6] = greenColor;
  z_axis_grids_colours[8] = greenColor;

  cam_info_received = false;
  image_pub = it.advertise("result", 1);
  debug_pub = it.advertise("debug", 1);
  pose_pub1 = nh.advertise<geometry_msgs::Pose>("pose", 100);
  pose_pub2 = nh.advertise<geometry_msgs::Pose>("pose2", 100);
  // image_pub_2 = nh.advertise<sensor_msgs::Image>("fixed_grid_image", 1);

  poseStamped_pub1 = nh.advertise<geometry_msgs::PoseStamped>("poseStamped", 100);
  poseStamped_pub2 = nh.advertise<geometry_msgs::PoseStamped>("poseStamped2", 100);

  pub_y = nh.advertise<std_msgs::Float32MultiArray>("matrix_y", 100);
  pub_x = nh.advertise<std_msgs::Float32MultiArray>("matrix_x", 100);
  pub_z = nh.advertise<std_msgs::Float32MultiArray>("matrix_z", 100);
  pub_GridVisualised = nh.advertise<std_msgs::Bool>("Grid_visualised",100);


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
