#include <boost/algorithm/string.hpp>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <state_space_filters/filtered_values.h>
#include <eigen_matrix_utils/overloads.h>
#include <cnr_cartesian_velocity_controller/cnr_cartesian_velocity_controller.h>

#include <pluginlib/class_list_macros.h>

namespace eu = eigen_utils;
namespace ect = eigen_control_toolbox;

PLUGINLIB_EXPORT_CLASS(cnr::control::CartesianVelocityController  , controller_interface::ControllerBase)

namespace cnr
{
namespace control
{


/**
 * @brief CartesianVelocityController::CartesianVelocityController
 */
inline CartesianVelocityController::CartesianVelocityController()
{
}

/**
 * @brief CartesianVelocityController::doInit
 * @return
 */
inline bool CartesianVelocityController::doInit()
{
  //INIT PUB/SUB
  std::string setpoint_topic_name;

  if(!this->getControllerNh().getParam("target_twist_topic",setpoint_topic_name))
      CNR_ERROR(this->logger(),"target_twist_topic not set");

  this->template add_subscriber<geometry_msgs::TwistStamped>(
        setpoint_topic_name,5,boost::bind(&CartesianVelocityController::callback,this,_1), false);

  this->setPriority(this->QD_PRIORITY);

  pos_sp_ = this->getPosition();
  vel_sp_ = 0 * this->getVelocity();

  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief CartesianVelocityController::doStarting
 * @param time
 */
inline bool CartesianVelocityController::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Starting Controller");
  pos_sp_ = this->getPosition();
  vel_sp_ = 0 * this->getVelocity();
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief CartesianVelocityController::stopping
 * @param time
 */
inline bool CartesianVelocityController::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Stopping Controller");
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief CartesianVelocityController::doUpdate
 * @param time
 * @param period
 * @return
 */
inline bool CartesianVelocityController::doUpdate(const ros::Time& /*time*/, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  std::stringstream report;

  // std::lock_guard<std::mutex> lock(mtx_);
  // twist in b in qp
  Eigen::Matrix6Xd J_b = this->chainCommand().toolJacobian();
  Eigen::FullPivLU<Eigen::MatrixXd> pinv_J ( J_b );
  pinv_J.setThreshold ( 1e-2 );
  if(pinv_J.rank()<6)
  {
    CNR_FATAL(this->logger(),"rank: "<<pinv_J.rank()<<"\nJacobian\n"<<J_b);
  }
  CNR_TRACE_THROTTLE(this->logger(),0.5,"Compute IK");

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_b, Eigen::ComputeThinU | Eigen::ComputeThinV);
  CNR_WARN_COND_THROTTLE(this->logger(),
    (svd.singularValues()(svd.cols()-1)==0) || (svd.singularValues()(0)/svd.singularValues()(svd.cols()-1) > 1e2),
    2, "SINGULARITY POINT" );

  vel_sp_ = svd.solve(twist_in_b_);
  pos_sp_ = this->getCommandPosition() + vel_sp_ * period.toSec();
  CNR_TRACE_THROTTLE(this->logger(),0.5,"Compute position: " << pos_sp_.transpose());
  if(rosdyn::saturatePosition(this->chainNonConst(), pos_sp_, &report))
  {
    CNR_WARN_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
  }
  vel_sp_ = (pos_sp_ - this->getCommandPosition())/period.toSec();
  this->setCommandPosition( pos_sp_ );
  this->setCommandVelocity( vel_sp_ );

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

/**
 * @brief CartesianVelocityController::callback
 * @param msg
 */
inline void CartesianVelocityController::callback(const geometry_msgs::TwistStampedConstPtr &msg)
{
{
  size_t l =__LINE__;
  std::string base_link = this->chain().getLinksName().front();

  try
  {
    CNR_DEBUG_THROTTLE(this->logger(), 2, "[ " << this->getControllerNamespace() << " ] >>>>>>>>>> TWIST TARGET TARGET RECEIVED!");

    Eigen::Vector6d twist = Eigen::Vector6d::Zero( );
    twist (0) = msg->twist.linear.x;
    twist (1) = msg->twist.linear.y;
    twist (2) = msg->twist.linear.z;
    twist (3) = msg->twist.angular.x;
    twist (4) = msg->twist.angular.y;
    twist (5) = msg->twist.angular.z;

    if(std::isnan(twist.norm()))
    {
      CNR_WARN_THROTTLE( this->logger(), 2, "[ " << this->getControllerNamespace() <<" ] SAFETY CHECK - Received a Twist with nan values... superimposed to zero!" );
      twist = Eigen::Vector6d::Zero();
    }

    CNR_WARN_THROTTLE( this->logger(), 2, "[ " << this->getControllerNamespace() <<" ] Reference Twist {" << msg->header.frame_id << "}     : " << twist.transpose() );

    l =__LINE__;
    std::string frame_id = boost::to_lower_copy( msg->header.frame_id);
    l =__LINE__;

    Eigen::Affine3d Tbt = this->chainCommand().toolPose();

    if ( frame_id == "tool" )
    {
      l =__LINE__;
      twist_in_b_ = rosdyn::spatialRotation( twist, Tbt.rotation());
    }
    else if ( frame_id == "base" )
    {
      l =__LINE__;
      twist_in_b_ = twist;
    }
    else
    {
      l =__LINE__;
      tf::StampedTransform TF_T_bf;
      CNR_DEBUG_THROTTLE(this->logger(), 2, "[ " << this->getControllerNamespace() << " ] listening to transform between "<<base_link<<" and "<<msg->header.frame_id);
      listener_.waitForTransform ( base_link, msg->header.frame_id, ros::Time(0), ros::Duration ( 10.0 ) );
      listener_.lookupTransform  ( base_link, msg->header.frame_id, ros::Time(0), TF_T_bf);
      Eigen::Affine3d T_bf;
      tf::transformTFToEigen(TF_T_bf, T_bf);

      twist_in_b_ = rosdyn::spatialRotation( twist, T_bf.rotation());
      l =__LINE__;
    }
    ROS_DEBUG_STREAM_THROTTLE(2,"[ " << this->getControllerNamespace() << " ] Reference Twist {base}     : " << twist_in_b_.transpose() );
  }
  catch(tf::TransformException& e)
  {
    CNR_WARN(this->logger(), "[ " << this->getControllerNamespace() << " ] something wrong in Getting the data from tf at line " << l);
    CNR_WARN(this->logger(), "[ " << this->getControllerNamespace() << " ] Listening to transform between "<<base_link<<" and "<<msg->header.frame_id <<" failed" );
    twist_in_b_ = Eigen::Vector6d::Zero();
  }
  catch(std::exception& e)
  {
    CNR_WARN(this->logger(), "[ " << this->getControllerNamespace() << " ] something wrong in Getting the data from tf at line " << l);
    CNR_WARN(this->logger(), "[ " << this->getControllerNamespace() << " ]Exception "<< e.what() );
    twist_in_b_ = Eigen::Vector6d::Zero();
  }
  catch(...)
  {
    CNR_WARN(this->logger(), "[ " << this->getControllerNamespace() << " ] unhandled excpetion.. something wrong in getting the data from tf at line " << l);
    twist_in_b_ = Eigen::Vector6d::Zero();
  }
  CNR_DEBUG_THROTTLE(this->logger(), 2, "[ " << this->getControllerNamespace() << " ] <<<<<<<<< TWIST TARGET TARGET RECEIVED!"  );
  return;
}

}
}
}




