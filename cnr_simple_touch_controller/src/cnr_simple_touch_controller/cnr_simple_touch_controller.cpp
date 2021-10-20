#include <locale>
#include <cnr_simple_touch_controller/cnr_simple_touch_controller.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <boost/algorithm/string.hpp>


/**
 * @brief PLUGINLIB_EXPORT_CLASS
 */
PLUGINLIB_EXPORT_CLASS(cnr::control::SimpleTouchController, controller_interface::ControllerBase);


namespace std
{
inline std::string to_string( const std::vector<std::string>& vals )
{
  std::string ret = "< ";
  for( auto const & val : vals ) ret += val + ", ";
  ret += " >";
  return ret;
}

inline std::string to_string( const std::string& val )
{
  return val;
}

inline std::string to_string( const bool& val )
{
  return val ? "TRUE" : "FALSE";
}

}



#define GET_AND_RETURN( nh, param, value )\
  if (!nh.getParam(param,value) )\
  {\
    ROS_ERROR("[ %s ] The param '%s/%s' is not defined", nh.getNamespace().c_str(), nh.getNamespace().c_str(), std::string( param ).c_str() );\
    return false;\
  }



#define GET_AND_DEFAULT( nh, param, value, def )\
  if (!nh.getParam(param,value) )\
  {\
    ROS_WARN("[ %s ] The param '%s/%s' is not defined", nh.getNamespace().c_str(), nh.getNamespace().c_str(), std::string( param ).c_str() );\
    ROS_WARN("[ %s ] Default value '%s' superimposed. ", nh.getNamespace().c_str(), std::to_string( def ).c_str() );\
    value=def;\
  }


namespace cnr
{
namespace control
{


//bool SimpleTouchController::init( hardware_interface::ForceTorqueSensorInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh )
bool SimpleTouchController::doInit()
{
  //** PARAMS UNDER CONTROLLER PARAM NAMESPACE*********************************************
  std::string output_twist_name;

  GET_AND_RETURN ( this->getControllerNh(), "sensor_name"   , m_ft_resource_name );
  GET_AND_DEFAULT( this->getControllerNh(), "output_twist_ns" , output_twist_name, this->getControllerNamespace()+"/target_cart_twist" );

  m_goal_wrench_norm = -1;
  m_wrench               .setZero( );
  m_initial_wrench       .setZero( );
  m_target_twist         .setZero( );
  m_goal_twist           .setZero( );

  m_ft_h = this->m_hw->getHandle( m_ft_resource_name );


  m_target_twist_pub = this->template add_publisher<geometry_msgs::TwistStamped>(output_twist_name,1000);

  CNR_RETURN_TRUE(this->logger());
}


bool SimpleTouchController::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Starting controller.");
  CNR_INFO(this->logger(),"[ "<<this->getControllerNamespace()<<" ] Starting controller" );

  m_touched = false;

  Eigen::Vector3d force ( m_ft_h.getForce( ) );
  Eigen::Vector3d torque( m_ft_h.getForce( ) );

  m_target_twist.setZero( );


  m_controller_nh_callback_queue.callAvailable();

  m_as.reset(new actionlib::ActionServer<simple_touch_controller_msgs::simpleTouchAction>(this->getControllerNh(), "simple_touch",
                                                                      boost::bind(&SimpleTouchController::actionGoalCallback,    this,  _1), 
                                                                      boost::bind(&SimpleTouchController::actionCancelCallback,  this,  _1), 
                                                                      false));
  m_as->start();

  CNR_INFO(this->logger(),"[ "<<this->getControllerNamespace()<<" ] Controller Started" );
  CNR_RETURN_TRUE(this->logger());

}


bool SimpleTouchController::doStopping(const ros::Time& /*time*/)
{
  CNR_INFO(this->logger(),"[ "<<this->getControllerNamespace()<<" ] Stopping Controller");
  m_target_twist.setZero();

  {
    geometry_msgs::TwistStamped tw;

    tw.twist.linear .x = m_target_twist(0,0);
    tw.twist.linear .y = m_target_twist(1,0);
    tw.twist.linear .z = m_target_twist(2,0);
    tw.twist.angular.x = m_target_twist(3,0);
    tw.twist.angular.y = m_target_twist(4,0);
    tw.twist.angular.z = m_target_twist(5,0);

    tw.header.frame_id = m_goal_twist_frame;
    tw.header.stamp    = ros::Time::now();

    this->publish(m_target_twist_pub, tw );

  }

  m_controller_nh_callback_queue.callAvailable();
  if (m_gh)
  {
    m_gh->setCanceled();
  }
  m_stop_thread = true;
  if (m_as_thread.joinable())
  {
    m_as_thread.join();
  }
  m_gh.reset();
  
  CNR_INFO(this->logger(),"[ "<<this->getControllerNamespace()<<" ] Controller Succesfully Stopped");

  CNR_RETURN_TRUE(this->logger());
}



bool SimpleTouchController::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  
  m_wrench.block(0,0,3,1) = Eigen::Vector3d( m_ft_h.getForce( ) );
  m_wrench.block(3,0,3,1) = Eigen::Vector3d( m_ft_h.getTorque( ) );

  // ROS_DEBUG_STREAM_THROTTLE(2, "[ " << m_controller_nh.getNamespace() <<  "] Measured Wrench {" << m_sensor_frame<<"}: " << m_wrench_s.transpose() );
  m_controller_nh_callback_queue.callAvailable();

  if(m_touched)
  {
    ROS_DEBUG_STREAM_THROTTLE(2, "[ " << this->getControllerNamespace() <<  "] Touched! Set to zero the output twist" );
    m_target_twist.setZero();
  }

  ROS_DEBUG_STREAM_THROTTLE(2, "[ " << this->getControllerNamespace() <<  "] Output twist: " << m_target_twist.transpose() );

  geometry_msgs::TwistStamped tw;

  tw.twist.linear .x = m_target_twist(0,0);
  tw.twist.linear .y = m_target_twist(1,0);
  tw.twist.linear .z = m_target_twist(2,0);
  tw.twist.angular.x = m_target_twist(3,0);
  tw.twist.angular.y = m_target_twist(4,0);
  tw.twist.angular.z = m_target_twist(5,0);
  tw.header.frame_id = m_goal_twist_frame;
  tw.header.stamp    = ros::Time::now();


  this->publish(m_target_twist_pub, tw );

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());

}

void SimpleTouchController::actionGoalCallback(actionlib::ActionServer< simple_touch_controller_msgs::simpleTouchAction>::GoalHandle gh)
{
  try
  {
    auto goal = gh.getGoal();

    std::shared_ptr<actionlib::ActionServer<simple_touch_controller_msgs::simpleTouchAction>::GoalHandle> current_gh;

    current_gh.reset(new actionlib::ActionServer<simple_touch_controller_msgs::simpleTouchAction>::GoalHandle(gh));
    m_gh = current_gh;

    CNR_INFO(this->logger(),"[ "<<this->getControllerNamespace()<<" ] New Goal Received, action start!");
    m_gh->setAccepted();


    if (m_as_thread.joinable())
    {
      m_as_thread.join();
    }



    m_goal_twist        = Eigen::Vector6d( goal->goal_twist.data() );
    m_goal_wrench_norm  = std::abs( goal->target_force);
    m_goal_twist_frame  = goal->goal_twist_frame;
    if (goal->relative_target)
    {
      m_initial_wrench = m_wrench;
    }
    else
    {
      m_initial_wrench.setZero();
    }

    m_release_condition = goal->release_condition;
    if (m_release_condition==simple_touch_controller_msgs::simpleTouchGoal::POSITION)
    {
      m_release_time     = goal->release/m_goal_twist.head(3).norm();
      m_release_force    = 0.0;
    }
    else if (m_release_condition==simple_touch_controller_msgs::simpleTouchGoal::FORCE)
    {
      m_release_time     = 0.0;
      m_release_force    = goal->release;
    }
    else// if (m_release_condition==simple_touch_controller_msgs::simpleTouchGoal::NONE)
    {
      m_release_time     = 0.0;
      m_release_force    = 0.0;
      m_release_condition=simple_touch_controller_msgs::simpleTouchGoal::NONE;
    }


    ROS_DEBUG_STREAM( "[ " << this->getControllerNamespace() <<  "] Goal goal twist           {" << m_goal_twist_frame  <<"} " << m_goal_twist          .transpose() );

    m_touched      = false;
    m_stop_thread  = false;
    m_automa_state = simple_touch_controller_msgs::simpleTouchFeedback::SEEK_CONTACT;
    m_as_thread    = std::thread(&SimpleTouchController::actionThreadFunction,this);
  }
  catch( std::exception& e )
  {
    ROS_ERROR_STREAM("Exception. what: " << e.what() );
    simple_touch_controller_msgs::simpleTouchResult result;
    result.error_code   = -1;
    result.error_string = std::string("exception: ")+e.what();
    m_target_twist.setZero( );
    m_gh->setAborted(result);
  }
  catch( ... )
  {
    ROS_ERROR_STREAM("Generalized Exception.");
    simple_touch_controller_msgs::simpleTouchResult result;
    result.error_code   = -1;
    result.error_string = "goal exception";
    m_target_twist.setZero( );
    m_gh->setAborted(result);
  }
  
}

void SimpleTouchController::actionCancelCallback(actionlib::ActionServer< simple_touch_controller_msgs::simpleTouchAction >::GoalHandle /*gh*/)
{
  ROS_DEBUG("[ %s ] Triggered the Cancel of the Action",  this->getControllerNamespace().c_str());
  if (m_gh)
  {
    m_gh->setCanceled();
    m_stop_thread = true;
    if (m_as_thread.joinable())
    {
      m_as_thread.join();
    }
    m_gh.reset();
    m_target_twist.setZero( );
  }
  else
  {
    ROS_WARN("[ %s ] Triggered the Cancel of the Action but none Goal is active.",  this->getControllerNamespace().c_str());
  }
  ROS_DEBUG("[ %s ] Action Succesfully Cancelled",  this->getControllerNamespace().c_str());
}

void SimpleTouchController::actionThreadFunction()
{
  ROS_DEBUG("[ %s ] START ACTION GOAL LOOP",  this->getControllerNamespace().c_str());
  ros::WallRate lp(100);

  ros::Time leaving_timer;

  while (this->getControllerNh().ok())
  {
    lp.sleep();
    if (!m_gh)
    {
      ROS_ERROR("[ %s ] Goal handle is not initialized",  this->getControllerNamespace().c_str());
      break;
    }

    if( m_stop_thread )
    {
      ROS_ERROR("[ %s ] Triggered an external stop. Break the action loop.",  this->getControllerNamespace().c_str());
      break;
    }

    Eigen::Vector6d wrench = m_wrench-m_initial_wrench;

    double current_force=wrench.head(3).norm();

    // transitions
    if (m_automa_state==simple_touch_controller_msgs::simpleTouchFeedback::SEEK_CONTACT)
    {
      if (current_force>m_goal_wrench_norm)
      {
        m_contact_time = ros::Time::now();
        if (m_release_condition == simple_touch_controller_msgs::simpleTouchGoal::NONE)
          m_automa_state=simple_touch_controller_msgs::simpleTouchFeedback::DONE;
        else
          m_automa_state=simple_touch_controller_msgs::simpleTouchFeedback::RELEASING;
      }
    }
    else if (m_automa_state==simple_touch_controller_msgs::simpleTouchFeedback::RELEASING)
    {
      double current_time=(ros::Time::now()-m_contact_time).toSec();
      if (m_release_condition == simple_touch_controller_msgs::simpleTouchGoal::FORCE)
      {
        if (current_force<=m_release_force)
        {
          m_automa_state=simple_touch_controller_msgs::simpleTouchFeedback::DONE;
        }
      }
      else if (m_release_condition == simple_touch_controller_msgs::simpleTouchGoal::POSITION)
      {
        if (current_time>=m_release_time)
        {
          m_automa_state=simple_touch_controller_msgs::simpleTouchFeedback::DONE;
        }
      }
    }

    simple_touch_controller_msgs::simpleTouchFeedback fb;
    fb.state=m_automa_state;
    m_gh->publishFeedback(fb);


    fb.state=m_automa_state;
    simple_touch_controller_msgs::simpleTouchResult result;
    // states
    if (m_automa_state==simple_touch_controller_msgs::simpleTouchFeedback::SEEK_CONTACT)
    {
      m_target_twist=m_goal_twist;
    }
    else if (m_automa_state==simple_touch_controller_msgs::simpleTouchFeedback::RELEASING)
    {
      m_target_twist=-m_goal_twist;
    }
    else if (m_automa_state==simple_touch_controller_msgs::simpleTouchFeedback::DONE)
    {
      result.error_code   = 0;
      result.error_string = "finished";
      m_gh->setSucceeded(result);
      m_target_twist.setZero();
      break;
    }
    else if (m_automa_state==simple_touch_controller_msgs::simpleTouchFeedback::FAIL)
    {
      result.error_code   = -1;
      result.error_string = ".............";
      m_gh->setAborted(result);
      m_target_twist.setZero();
      break;
    }
  }
  m_gh.reset();
}


}
}
