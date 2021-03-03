/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ2_VIEWS_MOVE_PLUGINS__VIEW_CONTROLLERS__ORBIT__ORBIT_VIEW_CONTROLLER_HPP_
#define RVIZ2_VIEWS_MOVE_PLUGINS__VIEW_CONTROLLERS__ORBIT__ORBIT_VIEW_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include <OgreVector3.h>

#include <QCursor>  // NOLINT(build/include_order)

#include "rviz_common/frame_position_tracking_view_controller.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_rendering/objects/shape.hpp"

# include "sensor_msgs/msg/camera_info.hpp"
#include "rviz_common/bit_allocator.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/display_group_visibility_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/queue_size_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"


#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/visibility_control.hpp"



# include <OgreMaterial.h>
# include <OgrePlatform.h>
# include <OgreRenderTargetListener.h>
# include <OgreSharedPtr.h>
# include "rviz_common/ros_topic_display.hpp"
# include "rviz_default_plugins/displays/image/ros_image_texture_iface.hpp"
# include "rviz_default_plugins/visibility_control.hpp"

#include "visibility_control.hpp"


#include <Eigen/Eigen>  
#include <Eigen/Geometry>  
#include <Eigen/Core> 
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

namespace rviz_default_plugins
{
namespace view_controllers
{

/// An orbital camera, controlled by yaw, pitch, distance, and focal point
/**
 * This camera is based on the equation of a sphere in spherical coordinates:
 *
 *   x = distance * cos(yaw) * sin(pitch)
 *   y = distance * cos(pitch)
 *   z = distance * sin(yaw) * sin(pitch)
 *
 * The coordinates are then offset by the focal point
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC MoveOrbitViewController : public
  rviz_common::FramePositionTrackingViewController,public Ogre::RenderTargetListener
{
  Q_OBJECT

public:
  MoveOrbitViewController();
  ~MoveOrbitViewController() override;
  /*
  MoveOrbitViewController(MoveOrbitViewController &&c) = delete;
  MoveOrbitViewController &operator=(MoveOrbitViewController &&c) = delete;
  MoveOrbitViewController(const MoveOrbitViewController &c) = delete;
  MoveOrbitViewController &operator=(const MoveOrbitViewController &c) = delete;
 */

  /// Do subclass-specific initialization.
  /**
   * Called by ViewController::initialize after context_, target_scene_node_,
   * and camera_ are set.
   */
  void onInitialize() override;

  /// Move in/out from the focal point, i.e. adjust distance by amount.
  /**
   * Positive amount moves towards the focal point, negative moves away.
   *
   * \param amount The distance to move.
   */
  void zoom(float amount);

  /// Set the yaw angle.
  void yaw(float angle);

  /// Set the pitch angle.
  void pitch(float angle);

  /// Move the focal point.
  void move(float x, float y, float z);  // NOLINT(build/include_what_you_use): not std::move

  /// Handle incoming mouse events.
  void handleMouseEvent(rviz_common::ViewportMouseEvent & evt) override;

  /// Look at a given location by changing the distance and angles.
  void lookAt(const Ogre::Vector3 & point) override;

  /// Reset the distance and angles to their default values.
  void reset() override;

  /// Configure this view controller to give a similar view to the given source_view.
  /**
   * \param source_view must return a valid `Ogre::Camera *` from `getCamera()`.
   */
  void mimic(ViewController * source_view) override;

  rviz_common::FocalPointStatus getFocalPointStatus() override;

  void update(float dt, float ros_dt) override;
  
  
protected:
  bool setMouseMovementFromEvent(
    const rviz_common::ViewportMouseEvent & event, int32_t & diff_x, int32_t & diff_y);

  void rotateCamera(int32_t diff_x, int32_t diff_y);

  virtual void moveFocalPoint(
    float distance, int32_t diff_x, int32_t diff_y, int32_t last_x, int32_t last_y);

  virtual void handleWheelEvent(rviz_common::ViewportMouseEvent & event, float distance);

  virtual void handleRightClick(
    rviz_common::ViewportMouseEvent & event, float distance, int32_t diff_y);

  virtual void setShiftOrbitStatus();

  void setDefaultOrbitStatus();

  void onTargetFrameChanged(
    const Ogre::Vector3 & old_reference_position,
    const Ogre::Quaternion & old_reference_orientation) override;

  /// Calculate the pitch and yaw values given a new position and the current focal point.
  /**
   * \param position the position from which to calculate the pitch/yaw.
   */
  void calculatePitchYawFromPosition(const Ogre::Vector3 & position);

  /// Calculate the focal shape size and update it's geometry.
  void updateFocalShapeSize();

  virtual void updateCamera();

  Ogre::Vector3 mimicTopDownViewController(rviz_common::ViewController * view_controller);

  /// The camera's yaw (rotation around the y-axis), in radians.
  rviz_common::properties::FloatProperty * yaw_property_;
  /// The camera's pitch (rotation around the x-axis), in radians.
  rviz_common::properties::FloatProperty * pitch_property_;
  /// The camera's distance from the focal point.
  rviz_common::properties::FloatProperty * distance_property_;
  /// The point around which the camera "orbits".
  rviz_common::properties::VectorProperty * focal_point_property_;
  /// Whether the focal shape size is fixed or not.
  rviz_common::properties::BoolProperty * focal_shape_fixed_size_property_;
  /// The focal shape size.
  rviz_common::properties::FloatProperty * focal_shape_size_property_;

  std::unique_ptr<rviz_rendering::Shape> focal_shape_;
  
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  rviz_common::properties::RosTopicProperty * topic_property_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;
  void topic_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void unsubscribe()
  {
    subscription_.reset();
  }

    Eigen::Quaterniond euler2Quaternion( double roll,  double pitch,  double yaw);
    Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w);
    Eigen::Matrix3d Quaternion2RotationMatrix(const double x,const double y,const double z,const double w);
    Eigen::Quaterniond rotationMatrix2Quaterniond(Eigen::Matrix3d R);
    Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, const double yaw);
    Eigen::Vector3d RotationMatrix2euler(Eigen::Matrix3d R);



  
private:
  bool dragging_;
};

}  // namespace view_controllers
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__ORBIT__ORBIT_VIEW_CONTROLLER_HPP_
