/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// MODULE
#include "rotors_gazebo_plugins/gazebo_odometry_plugin.h"

// SYSTEM
#include <chrono>
#include <iostream>

// 3RD PARTY
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// USER
#include <rotors_gazebo_plugins/common.h>
#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"
#include "PoseWithCovarianceStamped.pb.h"
#include "TransformStamped.pb.h"
#include "TransformStampedWithFrameIds.pb.h"
#include "Vector3dStamped.pb.h"

namespace gazebo {

GazeboOdometryPlugin::~GazeboOdometryPlugin() {
}

void GazeboOdometryPlugin::Load(physics::ModelPtr _model,
                                sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  SdfVector3 noise_normal_position;
  SdfVector3 noise_normal_quaternion;
  SdfVector3 noise_normal_linear_velocity;
  SdfVector3 noise_normal_angular_velocity;
  SdfVector3 noise_normal_drift_position;
  SdfVector3 noise_normal_drift_quaternion;

  SdfVector3 noise_uniform_position;
  SdfVector3 noise_uniform_quaternion;
  SdfVector3 noise_uniform_linear_velocity;
  SdfVector3 noise_uniform_angular_velocity;
  SdfVector3 noise_uniform_drift_position;
  SdfVector3 noise_uniform_drift_quaternion;

  const SdfVector3 zeros3(0.0, 0.0, 0.0);

  odometry_queue_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_odometry_plugin] Please specify a robotNamespace.\n";

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  if (_sdf->HasElement("linkName")) {
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  } else {
    gzerr << "[gazebo_odometry_plugin] Please specify a linkName.\n";
  }
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL) {
    gzthrow("[gazebo_odometry_plugin] Couldn't find specified link \""
            << link_name_ << "\".");
  }

  if (_sdf->HasElement("covarianceImage")) {
    std::string image_name =
        _sdf->GetElement("covarianceImage")->Get<std::string>();
    covariance_image_ = cv::imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);
    if (covariance_image_.data == NULL) {
      gzerr << "loading covariance image " << image_name << " failed"
            << std::endl;
    } else {
      gzlog << "loading covariance image " << image_name << " successful"
            << std::endl;
    }
  }

  if (_sdf->HasElement("randomEngineSeed")) {
    random_generator_.seed(
        _sdf->GetElement("randomEngineSeed")->Get<unsigned int>());
  } else {
    random_generator_.seed(
        std::chrono::system_clock::now().time_since_epoch().count());
  }
  getSdfParam<std::string>(_sdf, "poseTopic", pose_pub_topic_, pose_pub_topic_);
  getSdfParam<std::string>(_sdf, "poseWithCovarianceTopic",
                           pose_with_covariance_stamped_pub_topic_,
                           pose_with_covariance_stamped_pub_topic_);
  getSdfParam<std::string>(_sdf, "positionTopic", position_stamped_pub_topic_,
                           position_stamped_pub_topic_);
  getSdfParam<std::string>(_sdf, "transformTopic", transform_stamped_pub_topic_,
                           transform_stamped_pub_topic_);
  getSdfParam<std::string>(_sdf, "odometryTopic", odometry_pub_topic_,
                           odometry_pub_topic_);
  getSdfParam<std::string>(_sdf, "gtOdometryTopic", gt_odometry_pub_topic_,
                           gt_odometry_pub_topic_);
  getSdfParam<std::string>(_sdf, "gtPoseTopic", gt_pose_pub_topic_,
                           gt_pose_pub_topic_);
  getSdfParam<std::string>(_sdf, "driftTopic", drift_pub_topic_, drift_pub_topic_);
  getSdfParam<std::string>(_sdf, "parentFrameId", parent_frame_id_,
                           parent_frame_id_);
  getSdfParam<std::string>(_sdf, "childFrameId", child_frame_id_,
                           child_frame_id_);
  getSdfParam<SdfVector3>(_sdf, "noiseNormalPosition", noise_normal_position,
                          zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseNormalQuaternion",
                          noise_normal_quaternion, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseNormalLinearVelocity",
                          noise_normal_linear_velocity, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseNormalAngularVelocity",
                          noise_normal_angular_velocity, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseNormalDriftPosition", noise_normal_drift_position,
                          zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseNormalDriftQuaternion",
                          noise_normal_drift_quaternion, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseUniformPosition", noise_uniform_position,
                          zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseUniformQuaternion",
                          noise_uniform_quaternion, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseUniformLinearVelocity",
                          noise_uniform_linear_velocity, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseUniformAngularVelocity",
                          noise_uniform_angular_velocity, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseUniformDriftPosition", noise_uniform_drift_position,
                          zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseUniformDriftQuaternion",
                          noise_uniform_drift_quaternion, zeros3);
  getSdfParam<int>(_sdf, "measurementDelay", measurement_delay_,
                   measurement_delay_);
  getSdfParam<int>(_sdf, "measurementDivisor", measurement_divisor_,
                   measurement_divisor_);
  getSdfParam<double>(_sdf, "unknownDelay", unknown_delay_, unknown_delay_);
  getSdfParam<double>(_sdf, "covarianceImageScale", covariance_image_scale_,
                      covariance_image_scale_);

  parent_link_ = world_->EntityByName(parent_frame_id_);
  if (parent_link_ == NULL && parent_frame_id_ != kDefaultParentFrameId) {
    gzthrow("[gazebo_odometry_plugin] Couldn't find specified parent link \""
            << parent_frame_id_ << "\".");
  }

  position_n_[0] = NormalDistribution(0, noise_normal_position.X());
  position_n_[1] = NormalDistribution(0, noise_normal_position.Y());
  position_n_[2] = NormalDistribution(0, noise_normal_position.Z());

  attitude_n_[0] = NormalDistribution(0, noise_normal_quaternion.X());
  attitude_n_[1] = NormalDistribution(0, noise_normal_quaternion.Y());
  attitude_n_[2] = NormalDistribution(0, noise_normal_quaternion.Z());

  drift_position_n_[0] = NormalDistribution(0, noise_normal_drift_position.X());
  drift_position_n_[1] = NormalDistribution(0, noise_normal_drift_position.Y());
  drift_position_n_[2] = NormalDistribution(0, noise_normal_drift_position.Z());

  drift_attitude_n_[0] = NormalDistribution(0, noise_normal_drift_quaternion.X());
  drift_attitude_n_[1] = NormalDistribution(0, noise_normal_drift_quaternion.Y());
  drift_attitude_n_[2] = NormalDistribution(0, noise_normal_drift_quaternion.Z());

  linear_velocity_n_[0] =
      NormalDistribution(0, noise_normal_linear_velocity.X());
  linear_velocity_n_[1] =
      NormalDistribution(0, noise_normal_linear_velocity.Y());
  linear_velocity_n_[2] =
      NormalDistribution(0, noise_normal_linear_velocity.Z());

  angular_velocity_n_[0] =
      NormalDistribution(0, noise_normal_angular_velocity.X());
  angular_velocity_n_[1] =
      NormalDistribution(0, noise_normal_angular_velocity.Y());
  angular_velocity_n_[2] =
      NormalDistribution(0, noise_normal_angular_velocity.Z());

  position_u_[0] = UniformDistribution(-noise_uniform_position.X(),
                                       noise_uniform_position.X());
  position_u_[1] = UniformDistribution(-noise_uniform_position.Y(),
                                       noise_uniform_position.Y());
  position_u_[2] = UniformDistribution(-noise_uniform_position.Z(),
                                       noise_uniform_position.Z());

  attitude_u_[0] = UniformDistribution(-noise_uniform_quaternion.X(),
                                       noise_uniform_quaternion.X());
  attitude_u_[1] = UniformDistribution(-noise_uniform_quaternion.Y(),
                                       noise_uniform_quaternion.Y());
  attitude_u_[2] = UniformDistribution(-noise_uniform_quaternion.Z(),
                                       noise_uniform_quaternion.Z());

  drift_position_u_[0] = UniformDistribution(-noise_uniform_drift_position.X(),
                                             noise_uniform_drift_position.X());
  drift_position_u_[1] = UniformDistribution(-noise_uniform_drift_position.Y(),
                                             noise_uniform_drift_position.Y());
  drift_position_u_[2] = UniformDistribution(-noise_uniform_drift_position.Z(),
                                             noise_uniform_drift_position.Z());

  drift_attitude_u_[0] = UniformDistribution(-noise_uniform_drift_quaternion.X(),
                                             noise_uniform_drift_quaternion.X());
  drift_attitude_u_[1] = UniformDistribution(-noise_uniform_drift_quaternion.Y(),
                                             noise_uniform_drift_quaternion.Y());
  drift_attitude_u_[2] = UniformDistribution(-noise_uniform_drift_quaternion.Z(),
                                             noise_uniform_drift_quaternion.Z());


  linear_velocity_u_[0] = UniformDistribution(
      -noise_uniform_linear_velocity.X(), noise_uniform_linear_velocity.X());
  linear_velocity_u_[1] = UniformDistribution(
      -noise_uniform_linear_velocity.Y(), noise_uniform_linear_velocity.Y());
  linear_velocity_u_[2] = UniformDistribution(
      -noise_uniform_linear_velocity.Z(), noise_uniform_linear_velocity.Z());

  angular_velocity_u_[0] = UniformDistribution(
      -noise_uniform_angular_velocity.X(), noise_uniform_angular_velocity.X());
  angular_velocity_u_[1] = UniformDistribution(
      -noise_uniform_angular_velocity.Y(), noise_uniform_angular_velocity.Y());
  angular_velocity_u_[2] = UniformDistribution(
      -noise_uniform_angular_velocity.Z(), noise_uniform_angular_velocity.Z());

  // Fill in covariance. We omit uniform noise here.
  Eigen::Map<Eigen::Matrix<double, 6, 6> > pose_covariance(
      pose_covariance_matrix_.data());
  Eigen::Matrix<double, 6, 1> pose_covd;

  pose_covd << noise_normal_position.X() * noise_normal_position.X(),
      noise_normal_position.Y() * noise_normal_position.Y(),
      noise_normal_position.Z() * noise_normal_position.Z(),
      noise_normal_quaternion.X() * noise_normal_quaternion.X(),
      noise_normal_quaternion.Y() * noise_normal_quaternion.Y(),
      noise_normal_quaternion.Z() * noise_normal_quaternion.Z();
  pose_covariance = pose_covd.asDiagonal();

  // Fill in covariance. We omit uniform noise here.
  Eigen::Map<Eigen::Matrix<double, 6, 6> > twist_covariance(
      twist_covariance_matrix_.data());
  Eigen::Matrix<double, 6, 1> twist_covd;

  twist_covd << noise_normal_linear_velocity.X() *
                    noise_normal_linear_velocity.X(),
      noise_normal_linear_velocity.Y() * noise_normal_linear_velocity.Y(),
      noise_normal_linear_velocity.Z() * noise_normal_linear_velocity.Z(),
      noise_normal_angular_velocity.X() * noise_normal_angular_velocity.X(),
      noise_normal_angular_velocity.Y() * noise_normal_angular_velocity.Y(),
      noise_normal_angular_velocity.Z() * noise_normal_angular_velocity.Z();
  twist_covariance = twist_covd.asDiagonal();

  Eigen::Map<Eigen::Matrix<double, 6, 6> > drift_covariance(
      drift_covariance_matrix_.data());
  Eigen::Matrix<double, 6, 1> drift_covd;

  drift_covd << noise_normal_drift_position.X() * noise_normal_drift_position.X(),
      noise_normal_drift_position.Y() * noise_normal_drift_position.Y(),
      noise_normal_drift_position.Z() * noise_normal_drift_position.Z(),
      noise_normal_drift_quaternion.X() * noise_normal_drift_quaternion.X(),
      noise_normal_drift_quaternion.Y() * noise_normal_drift_quaternion.Y(),
      noise_normal_drift_quaternion.Z() * noise_normal_drift_quaternion.Z();
  drift_covariance = drift_covd.asDiagonal();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboOdometryPlugin::OnUpdate, this, _1));


  // Initialize previous pose 
  previous_position_ << 0, 0, 0;

  previous_quaternion_.w() = 1.;
  previous_quaternion_.x() = 0;
  previous_quaternion_.y() = 0;
  previous_quaternion_.z() = 0;




  accumulated_drift_position_ << 0, 0, 0;
  accumulated_drift_quaternion_.w() = 1.;
  accumulated_drift_quaternion_.x() = 0;
  accumulated_drift_quaternion_.y() = 0;
  accumulated_drift_quaternion_.z() = 0;


  if (parent_frame_id_ != kDefaultParentFrameId) {
    std::cout << "============================================" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "WARNING: Odometry drift may not work properly "
              << "when the parent frame is not the world frame!" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "============================================" << std::endl;
  }

  initial_pose_ = link_->WorldCoGPose();  
  previous_t_ = world_->SimTime().Double();
  drift_started_ = false;
}

// This gets called by the world update start event.
void GazeboOdometryPlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  // C denotes child frame, P parent frame, and W world frame.
  // Further C_pose_W_P denotes pose of P wrt. W expressed in C.
  ignition::math::Pose3d W_pose_W_C = link_->WorldCoGPose();
  ignition::math::Vector3d C_linear_velocity_W_C = link_->RelativeLinearVel();
  ignition::math::Vector3d C_angular_velocity_W_C = link_->RelativeAngularVel();

  ignition::math::Vector3d gazebo_linear_velocity = C_linear_velocity_W_C;
  ignition::math::Vector3d gazebo_angular_velocity = C_angular_velocity_W_C;
  ignition::math::Pose3d gazebo_pose = W_pose_W_C;

  if (parent_frame_id_ != kDefaultParentFrameId) {
    std::cout << "============================================" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "WARNING: Odometry drift may not work properly "
              << "when the parent frame is not the world frame!" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "============================================" << std::endl;

    /*
    ignition::math::Pose3d W_pose_W_P = parent_link_->WorldPose();
    ignition::math::Vector3d P_linear_velocity_W_P = parent_link_->RelativeLinearVel();
    ignition::math::Vector3d P_angular_velocity_W_P = parent_link_->RelativeAngularVel();
    ignition::math::Pose3d C_pose_P_C_ = W_pose_W_C - W_pose_W_P;
    ignition::math::Vector3d C_linear_velocity_P_C;
    // \prescript{}{C}{\dot{r}}_{PC} = -R_{CP}
    //       \cdot \prescript{}{P}{\omega}_{WP} \cross \prescript{}{P}{r}_{PC}
    //       + \prescript{}{C}{v}_{WC}
    //                                 - R_{CP} \cdot \prescript{}{P}{v}_{WP}
    C_linear_velocity_P_C =
        -C_pose_P_C_.Rot().Inverse() *
            P_angular_velocity_W_P.Cross(C_pose_P_C_.Pos()) +
        C_linear_velocity_W_C -
        C_pose_P_C_.Rot().Inverse() * P_linear_velocity_W_P;

    // \prescript{}{C}{\omega}_{PC} = \prescript{}{C}{\omega}_{WC}
    //       - R_{CP} \cdot \prescript{}{P}{\omega}_{WP}
    gazebo_angular_velocity =
        C_angular_velocity_W_C -
        C_pose_P_C_.Rot().Inverse() * P_angular_velocity_W_P;
    gazebo_linear_velocity = C_linear_velocity_P_C;
    gazebo_pose = C_pose_P_C_;
    */
  }

  // This flag could be set to false in the following code...
  bool publish_odometry = true;

  // First, determine whether we should publish a odometry.
  if (covariance_image_.data != NULL) {
    // We have an image.

    // Image is always centered around the origin:
    int width = covariance_image_.cols;
    int height = covariance_image_.rows;
    int x = static_cast<int>(
                std::floor(gazebo_pose.Pos().X() / covariance_image_scale_)) +
            width / 2;
    int y = static_cast<int>(
                std::floor(gazebo_pose.Pos().Y() / covariance_image_scale_)) +
            height / 2;

    if (x >= 0 && x < width && y >= 0 && y < height) {
      uint8_t pixel_value = covariance_image_.at<uint8_t>(y, x);
      if (pixel_value == 0) {
        publish_odometry = false;
        // TODO: covariance scaling, according to the intensity values could be
        // implemented here.
      }
    }
  }

  if (gazebo_sequence_ % measurement_divisor_ == 0) {


    gz_geometry_msgs::Odometry gt_pose;
    gt_pose.mutable_header()->set_frame_id(parent_frame_id_);
    gt_pose.mutable_header()->mutable_stamp()->set_sec(
        (world_->SimTime()).sec + static_cast<int32_t>(unknown_delay_));
    gt_pose.mutable_header()->mutable_stamp()->set_nsec(
        (world_->SimTime()).nsec + static_cast<int32_t>(unknown_delay_));
    gt_pose.set_child_frame_id(child_frame_id_);

    gt_pose.mutable_pose()->mutable_pose()->mutable_position()->set_x(
        gazebo_pose.Pos().X());
    gt_pose.mutable_pose()->mutable_pose()->mutable_position()->set_y(
        gazebo_pose.Pos().Y());
    gt_pose.mutable_pose()->mutable_pose()->mutable_position()->set_z(
        gazebo_pose.Pos().Z());

    gt_pose.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(
        gazebo_pose.Rot().X());
    gt_pose.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(
        gazebo_pose.Rot().Y());
    gt_pose.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(
        gazebo_pose.Rot().Z());
    gt_pose.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(
        gazebo_pose.Rot().W());

    gt_pose.mutable_twist()->mutable_twist()->mutable_linear()->set_x(
        gazebo_linear_velocity.X());
    gt_pose.mutable_twist()->mutable_twist()->mutable_linear()->set_y(
        gazebo_linear_velocity.Y());
    gt_pose.mutable_twist()->mutable_twist()->mutable_linear()->set_z(
        gazebo_linear_velocity.Z());

    gt_pose.mutable_twist()->mutable_twist()->mutable_angular()->set_x(
        gazebo_angular_velocity.X());
    gt_pose.mutable_twist()->mutable_twist()->mutable_angular()->set_y(
        gazebo_angular_velocity.Y());
    gt_pose.mutable_twist()->mutable_twist()->mutable_angular()->set_z(
        gazebo_angular_velocity.Z());

        
    gazebo_pose = gazebo_pose - initial_pose_;

    gz_geometry_msgs::Odometry odometry;
    odometry.mutable_header()->set_frame_id(child_frame_id_ + "_odometry_frame");
    odometry.mutable_header()->mutable_stamp()->set_sec(
        (world_->SimTime()).sec + static_cast<int32_t>(unknown_delay_));
    odometry.mutable_header()->mutable_stamp()->set_nsec(
        (world_->SimTime()).nsec + static_cast<int32_t>(unknown_delay_));
    std::string frame_name = child_frame_id_ + "_odometry";
    odometry.set_child_frame_id(frame_name);

    odometry.mutable_pose()->mutable_pose()->mutable_position()->set_x(
        gazebo_pose.Pos().X());
    odometry.mutable_pose()->mutable_pose()->mutable_position()->set_y(
        gazebo_pose.Pos().Y());
    odometry.mutable_pose()->mutable_pose()->mutable_position()->set_z(
        gazebo_pose.Pos().Z());

    odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(
        gazebo_pose.Rot().X());
    odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(
        gazebo_pose.Rot().Y());
    odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(
        gazebo_pose.Rot().Z());
    odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(
        gazebo_pose.Rot().W());

    odometry.mutable_twist()->mutable_twist()->mutable_linear()->set_x(
        gazebo_linear_velocity.X());
    odometry.mutable_twist()->mutable_twist()->mutable_linear()->set_y(
        gazebo_linear_velocity.Y());
    odometry.mutable_twist()->mutable_twist()->mutable_linear()->set_z(
        gazebo_linear_velocity.Z());

    odometry.mutable_twist()->mutable_twist()->mutable_angular()->set_x(
        gazebo_angular_velocity.X());
    odometry.mutable_twist()->mutable_twist()->mutable_angular()->set_y(
        gazebo_angular_velocity.Y());
    odometry.mutable_twist()->mutable_twist()->mutable_angular()->set_z(
        gazebo_angular_velocity.Z());

    if (publish_odometry)
      gt_pose_queue_.push_back(
          std::make_pair(gazebo_sequence_ + measurement_delay_, gt_pose));
      odometry_queue_.push_back(
          std::make_pair(gazebo_sequence_ + measurement_delay_, odometry));
  }

  // Is it time to publish the front element?
  if (gazebo_sequence_ == odometry_queue_.front().first) {

    const double t = world_->SimTime().Double(); 
    const double dt = t - previous_t_;
    previous_t_ = t;

    // GT pose handling
    // ---------------------------
    gz_geometry_msgs::Odometry gt_pose_msg(gt_pose_queue_.front().second);
    gt_pose_queue_.pop_front();
    
    if (gt_pose_pub_->HasConnections()) {
      gt_pose_pub_->Publish(gt_pose_msg);
    }

    gazebo::msgs::Vector3d gz_p_gt = gt_pose_msg.pose().pose().position();
    gazebo::msgs::Quaternion gz_q_gt = gt_pose_msg.pose().pose().orientation();
    Eigen::Vector3d p_gt(gz_p_gt.x(), gz_p_gt.y(), gz_p_gt.z()); 
    Eigen::Quaterniond q_gt(gz_q_gt.w(), gz_q_gt.x(), gz_q_gt.y(), gz_q_gt.z()); 



    // Odometry
    // ---------------------------
    // Copy the odometry message that is on the queue
    gz_geometry_msgs::Odometry odometry_msg(odometry_queue_.front().second);
    gz_geometry_msgs::Odometry gt_odometry_msg(odometry_queue_.front().second);


    gz_geometry_msgs::PoseWithCovarianceStamped drift_msg;
    drift_msg.mutable_header()->CopyFrom(odometry_msg.header());

    // Now that we have copied the first element from the queue, remove it.
    odometry_queue_.pop_front();


    gazebo::msgs::Vector3d *gz_p_O_S = gt_odometry_msg.mutable_pose()->mutable_pose()->mutable_position();
    gazebo::msgs::Quaternion *gz_q_O_S = gt_odometry_msg.mutable_pose()->mutable_pose()->mutable_orientation();

    // current pose
    Eigen::Vector3d p_O_S(gz_p_O_S->x(), gz_p_O_S->y(), gz_p_O_S->z()); 
    Eigen::Quaterniond q_O_S(gz_q_O_S->w(), gz_q_O_S->x(), gz_q_O_S->y(), gz_q_O_S->z()); 

    // previous pose
//    Eigen::Vector3d prev_p_O_S(previous_pose_.Pos().X(), previous_pose_.Pos().Y(), previous_pose_.Pos().Z());
//    Eigen::Quaterniond prev_q_O_S(previous_pose_.Rot().W(), previous_pose_.Rot().X(), 
//                                  previous_pose_.Rot().Y(), previous_pose_.Rot().Z());

    
    // delta_pose = (previous_pose)^-1 * current_pose
    Eigen::Quaterniond delta_q; 
    delta_q = previous_quaternion_.inverse() * q_O_S;

    Eigen::Vector3d delta_p;
    delta_p = previous_quaternion_.inverse() * (p_O_S - previous_position_);

   
    // create noisy delta to simulate drift of odometry frame
    Eigen::Vector3d delta_p_noisy;

    delta_p_noisy << delta_p.x() + std::sqrt(dt) * (drift_position_n_[0](random_generator_) + drift_position_u_[0](random_generator_)),
                     delta_p.y() + std::sqrt(dt) * (drift_position_n_[1](random_generator_) + drift_position_u_[1](random_generator_)),
                     delta_p.z() + std::sqrt(dt) * (drift_position_n_[2](random_generator_) + drift_position_u_[2](random_generator_));

    Eigen::Vector3d delta_attitude;
    delta_attitude << std::sqrt(dt) * (drift_attitude_n_[0](random_generator_) + drift_attitude_u_[0](random_generator_)),
                      std::sqrt(dt) * (drift_attitude_n_[1](random_generator_) + drift_attitude_u_[1](random_generator_)),
                      std::sqrt(dt) * (drift_attitude_n_[2](random_generator_) + drift_attitude_u_[2](random_generator_));
    Eigen::Quaterniond delta_q_n = QuaternionFromSmallAngle(delta_attitude);
    delta_q_n.normalize();

    Eigen::Quaterniond delta_q_noisy; 
    delta_q_noisy = delta_q * delta_q_n;

    // update current pose with noisy delta
    if (drift_started_) {
        q_O_S = previous_quaternion_ * delta_q_noisy; 
        p_O_S = previous_quaternion_ * delta_p_noisy + previous_position_;
    }
    
    // new odometry origin
    Eigen::Quaterniond q_W_O = q_gt * q_O_S.inverse();
    Eigen::Vector3d p_W_O = p_gt - q_W_O * p_O_S;
   
    gazebo::msgs::Vector3d* gz_p_W_O =
        drift_msg.mutable_pose_with_covariance()->mutable_pose()->mutable_position();

    gz_p_W_O->set_x(p_W_O.x());
    gz_p_W_O->set_y(p_W_O.y());
    gz_p_W_O->set_z(p_W_O.z());

    gazebo::msgs::Quaternion* gz_q_W_O =
        drift_msg.mutable_pose_with_covariance()->mutable_pose()->mutable_orientation();

    gz_q_W_O->set_w(q_W_O.w());
    gz_q_W_O->set_x(q_W_O.x());
    gz_q_W_O->set_y(q_W_O.y());
    gz_q_W_O->set_z(q_W_O.z());

    drift_msg.mutable_pose_with_covariance()->mutable_covariance()->Clear();
    for (int i = 0; i < drift_covariance_matrix_.size(); i++) {
      drift_msg.mutable_pose_with_covariance()->mutable_covariance()->Add(
          drift_covariance_matrix_[i]);
    }


    // update odometry origin
    initial_pose_.Pos().X() = p_W_O.x();
    initial_pose_.Pos().Y() = p_W_O.y();
    initial_pose_.Pos().Z() = p_W_O.z();

    initial_pose_.Rot().W() = q_W_O.w();
    initial_pose_.Rot().X() = q_W_O.x();
    initial_pose_.Rot().Y() = q_W_O.y();
    initial_pose_.Rot().Z() = q_W_O.z();


    previous_position_ = p_O_S;
    previous_quaternion_ = q_O_S;

    gz_p_O_S->set_x(p_O_S.x());
    gz_p_O_S->set_y(p_O_S.y());
    gz_p_O_S->set_z(p_O_S.z());

    gz_q_O_S->set_w(q_O_S.w());
    gz_q_O_S->set_x(q_O_S.x());
    gz_q_O_S->set_y(q_O_S.y());
    gz_q_O_S->set_z(q_O_S.z());



    // Calculate position distortions.
    Eigen::Vector3d pos_n;
    pos_n << std::sqrt(dt) * (position_n_[0](random_generator_) + position_u_[0](random_generator_)),
             std::sqrt(dt) * (position_n_[1](random_generator_) + position_u_[1](random_generator_)),
             std::sqrt(dt) * (position_n_[2](random_generator_) + position_u_[2](random_generator_));

    // Add additional noise to odometry position
    gazebo::msgs::Vector3d* gz_p_O_S_noisy =
        odometry_msg.mutable_pose()->mutable_pose()->mutable_position();

    gz_p_O_S_noisy->set_x(p_O_S.x() + pos_n.x());
    gz_p_O_S_noisy->set_y(p_O_S.y() + pos_n.y());
    gz_p_O_S_noisy->set_z(p_O_S.z() + pos_n.z());


    // Calculate attitude distortions.
    Eigen::Vector3d theta;
    theta << std::sqrt(dt) * (attitude_n_[0](random_generator_) + attitude_u_[0](random_generator_)),
             std::sqrt(dt) * (attitude_n_[1](random_generator_) + attitude_u_[1](random_generator_)),
             std::sqrt(dt) * (attitude_n_[2](random_generator_) + attitude_u_[2](random_generator_));
    Eigen::Quaterniond q_n = QuaternionFromSmallAngle(theta);
    q_n.normalize();

    gazebo::msgs::Quaternion* gz_q_O_S_noisy =
        odometry_msg.mutable_pose()->mutable_pose()->mutable_orientation();

    Eigen::Quaterniond _q_O_S_noisy(gz_q_O_S_noisy->w(), gz_q_O_S_noisy->x(), gz_q_O_S_noisy->y(), gz_q_O_S_noisy->z());
    _q_O_S_noisy = q_O_S * q_n;
    gz_q_O_S_noisy->set_w(_q_O_S_noisy.w());
    gz_q_O_S_noisy->set_x(_q_O_S_noisy.x());
    gz_q_O_S_noisy->set_y(_q_O_S_noisy.y());
    gz_q_O_S_noisy->set_z(_q_O_S_noisy.z());


    // Calculate linear velocity distortions.
    Eigen::Vector3d linear_velocity_n;
    linear_velocity_n << linear_velocity_n_[0](random_generator_) +
                             linear_velocity_u_[0](random_generator_),
        linear_velocity_n_[1](random_generator_) +
            linear_velocity_u_[1](random_generator_),
        linear_velocity_n_[2](random_generator_) +
            linear_velocity_u_[2](random_generator_);

    gazebo::msgs::Vector3d* linear_velocity =
        odometry_msg.mutable_twist()->mutable_twist()->mutable_linear();

    linear_velocity->set_x(linear_velocity->x() + linear_velocity_n[0]);
    linear_velocity->set_y(linear_velocity->y() + linear_velocity_n[1]);
    linear_velocity->set_z(linear_velocity->z() + linear_velocity_n[2]);

    // Calculate angular velocity distortions.
    Eigen::Vector3d angular_velocity_n;
    angular_velocity_n << angular_velocity_n_[0](random_generator_) +
                              angular_velocity_u_[0](random_generator_),
        angular_velocity_n_[1](random_generator_) +
            angular_velocity_u_[1](random_generator_),
        angular_velocity_n_[2](random_generator_) +
            angular_velocity_u_[2](random_generator_);

    odometry_msg.mutable_pose()->mutable_covariance()->Clear();
    for (int i = 0; i < pose_covariance_matrix_.size(); i++) {
      odometry_msg.mutable_pose()->mutable_covariance()->Add(
          pose_covariance_matrix_[i]);
    }

    odometry_msg.mutable_twist()->mutable_covariance()->Clear();
    for (int i = 0; i < twist_covariance_matrix_.size(); i++) {
      odometry_msg.mutable_twist()->mutable_covariance()->Add(
          twist_covariance_matrix_[i]);
    }

    gt_odometry_msg.mutable_twist()->CopyFrom(odometry_msg.twist());
    gt_odometry_msg.mutable_pose()->mutable_covariance()->CopyFrom(odometry_msg.pose().covariance());


    // ----------------------------------------------------------------
    // Publish all the topics, for which the topic name is specified.
    if (pose_pub_->HasConnections()) {
      pose_pub_->Publish(odometry_msg.pose().pose());
    }

    if (pose_with_covariance_stamped_pub_->HasConnections()) {
      gz_geometry_msgs::PoseWithCovarianceStamped
          pose_with_covariance_stamped_msg;

      pose_with_covariance_stamped_msg.mutable_header()->CopyFrom(
          odometry_msg.header());
      pose_with_covariance_stamped_msg.mutable_pose_with_covariance()->CopyFrom(
          odometry_msg.pose());

      pose_with_covariance_stamped_pub_->Publish(
          pose_with_covariance_stamped_msg);
    }

    if (position_stamped_pub_->HasConnections()) {
      gz_geometry_msgs::Vector3dStamped position_stamped_msg;
      position_stamped_msg.mutable_header()->CopyFrom(odometry_msg.header());
      position_stamped_msg.mutable_position()->CopyFrom(
          odometry_msg.pose().pose().position());

      position_stamped_pub_->Publish(position_stamped_msg);
    }

    if (transform_stamped_pub_->HasConnections()) {
      gz_geometry_msgs::TransformStamped transform_stamped_msg;

      transform_stamped_msg.mutable_header()->CopyFrom(odometry_msg.header());
      transform_stamped_msg.mutable_transform()->mutable_translation()->set_x(
          gz_p_O_S_noisy->x());
      transform_stamped_msg.mutable_transform()->mutable_translation()->set_y(
          gz_p_O_S_noisy->y());
      transform_stamped_msg.mutable_transform()->mutable_translation()->set_z(
          gz_p_O_S_noisy->z());
      transform_stamped_msg.mutable_transform()->mutable_rotation()->CopyFrom(
          *gz_q_O_S_noisy);

      transform_stamped_pub_->Publish(transform_stamped_msg);
    }

    if (odometry_pub_->HasConnections()) {
      odometry_pub_->Publish(odometry_msg);
    }

    if (gt_odometry_pub_->HasConnections()) {
      gt_odometry_pub_->Publish(gt_odometry_msg);
    }

    if (drift_pub_->HasConnections()) {
      drift_pub_->Publish(drift_msg);
    }


    //==============================================//
    //========= BROADCAST TRANSFORM MSG ============//
    //==============================================//

    gz_geometry_msgs::TransformStampedWithFrameIds
        transform_stamped_with_frame_ids_msg;
    transform_stamped_with_frame_ids_msg.mutable_header()->CopyFrom(
        odometry_msg.header());
    transform_stamped_with_frame_ids_msg.mutable_transform()
        ->mutable_translation()
        ->set_x(gz_p_O_S_noisy->x());
    transform_stamped_with_frame_ids_msg.mutable_transform()
        ->mutable_translation()
        ->set_y(gz_p_O_S_noisy->y());
    transform_stamped_with_frame_ids_msg.mutable_transform()
        ->mutable_translation()
        ->set_z(gz_p_O_S_noisy->z());
    transform_stamped_with_frame_ids_msg.mutable_transform()
        ->mutable_rotation()
        ->CopyFrom(*gz_q_O_S_noisy);
    transform_stamped_with_frame_ids_msg.set_parent_frame_id(parent_frame_id_);
    transform_stamped_with_frame_ids_msg.set_child_frame_id(child_frame_id_);

    broadcast_transform_pub_->Publish(transform_stamped_with_frame_ids_msg);

  }  // if (gazebo_sequence_ == odometry_queue_.front().first) {

  ++gazebo_sequence_;
}

void GazeboOdometryPlugin::driftStartedCallback(const boost::shared_ptr<const gz_std_msgs::Bool> & p_boolMsg) {
  if (drift_started_) {
    return;
  }
  std::cout << "------------> start drift " << std::endl;
  drift_started_ = p_boolMsg->data();
}



void GazeboOdometryPlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;


  // Create temporary "ConnectRosToGazeboTopic" publisher and message
  gazebo::transport::PublisherPtr connect_ros_to_gazebo_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
          "~/" + kConnectRosToGazeboSubtopic, 1);

  gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;



  // ============================================ //
  // =============== POSE MSG SETUP ============= //
  // ============================================ //

  pose_pub_ = node_handle_->Advertise<gazebo::msgs::Pose>(
      "~/" + namespace_ + "/" + pose_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   pose_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                pose_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::POSE);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // == POSE WITH COVARIANCE STAMPED MSG SETUP == //
  // ============================================ //

  pose_with_covariance_stamped_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::PoseWithCovarianceStamped>(
          "~/" + namespace_ + "/" + pose_with_covariance_stamped_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic(
      "~/" + namespace_ + "/" + pose_with_covariance_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(
      namespace_ + "/" + pose_with_covariance_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::POSE_WITH_COVARIANCE_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ========= POSITION STAMPED MSG SETUP ======= //
  // ============================================ //

  position_stamped_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::Vector3dStamped>(
          "~/" + namespace_ + "/" + position_stamped_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   position_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                position_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::VECTOR_3D_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ============= ODOMETRY MSG SETUP =========== //
  // ============================================ //

  odometry_pub_ = node_handle_->Advertise<gz_geometry_msgs::Odometry>(
      "~/" + namespace_ + "/" + odometry_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   odometry_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                odometry_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::ODOMETRY);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ============ GT ODOMETRY MSG SETUP ========= //
  // ============================================ //

  gt_odometry_pub_ = node_handle_->Advertise<gz_geometry_msgs::Odometry>(
      "~/" + namespace_ + "/" + gt_odometry_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   gt_odometry_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                gt_odometry_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::GT_ODOMETRY);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ============ GT POSE MSG SETUP ========= //
  // ============================================ //

  gt_pose_pub_ = node_handle_->Advertise<gz_geometry_msgs::Odometry>(
      "~/" + namespace_ + "/" + gt_pose_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   gt_pose_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                gt_pose_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::GT_POSE);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);



  // ============================================ //
  // ============= DRIFT MSG SETUP ============== //
  // ============================================ //

  drift_pub_ = node_handle_->Advertise<gz_geometry_msgs::PoseWithCovarianceStamped>(
      "~/" + namespace_ + "/" + drift_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   drift_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                drift_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::DRIFT);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);



  // ============================================ //
  // ======== TRANSFORM STAMPED MSG SETUP ======= //
  // ============================================ //

  transform_stamped_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::TransformStamped>(
          "~/" + namespace_ + "/" + transform_stamped_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic(
      "~/" + namespace_ + "/" + transform_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                transform_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::TRANSFORM_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ===== "BROADCAST TRANSFORM" MSG SETUP =====  //
  // ============================================ //

  broadcast_transform_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::TransformStampedWithFrameIds>(
          "~/" + kBroadcastTransformSubtopic, 1);




  // ========================================================== //
  // ===== DRAWING STARTED MSG SETUP (ROS->GAZEBO) ============ //
  // ========================================================== //

    drift_started_sub_ = node_handle_->Subscribe("~/" + namespace_ + 
                                                   "/" + drift_started_topic_,
        &GazeboOdometryPlugin::driftStartedCallback, this);

    connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" + drift_started_topic_);
    connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + namespace_ +
                                                     "/" + drift_started_topic_);
    connect_ros_to_gazebo_topic_msg.set_msgtype(
            gz_std_msgs::ConnectRosToGazeboTopic::DRIFT_STARTED);
    connect_ros_to_gazebo_topic_pub->Publish(
            connect_ros_to_gazebo_topic_msg, true);






}

GZ_REGISTER_MODEL_PLUGIN(GazeboOdometryPlugin);

}  // namespace gazebo
