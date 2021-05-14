#ifndef cnr_cartesian_velocity_controller__20188101642
#define cnr_cartesian_velocity_controller__20188101642

#include <cmath>
#include <Eigen/Core>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <state_space_filters/filtered_values.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <cnr_hardware_interface/veleff_command_interface.h>
//#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>


namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{


/**
 * @brief The CartesianVelocityController class
 */
class CartesianVelocityController:
    public cnr::control::JointCommandController<
                  hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>
{
public:
  CartesianVelocityController();
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);
  void callback(const geometry_msgs::TwistStampedConstPtr& msg);

protected:

  tf::TransformListener listener_;

  std::mutex mtx_;

  rosdyn::VectorXd vel_sp_;
  rosdyn::VectorXd pos_sp_;
  Eigen::Vector6d twist_in_b_;
};


}
}



#endif
