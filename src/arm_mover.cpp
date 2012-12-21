#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper{
private:
  GripperClient* gripper_client_;  

public:
  //Action client initialization
  Gripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);
    
    //wait for the gripper action server to come up 
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
    }
  }

  ~Gripper(){
    delete gripper_client_;
  }

  //Open the gripper
  void open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)
    
    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper opened!");
    else
      ROS_INFO("The gripper failed to open.");
  }

  //Close the gripper
  void close(){
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.0;
    squeeze.command.max_effort = 50.0;  // Close gently
    
    ROS_INFO("Sending squeeze goal");
    gripper_client_->sendGoal(squeeze);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper closed!");
    else
      ROS_INFO("The gripper failed to close.");
  }
};

ros::Publisher pose_pub;
boost::shared_ptr<Gripper> gripper;
float z_position = 0.45;
bool go_down = false;

void place()
{
          geometry_msgs::PoseStamped pose;
          pose.header.stamp = ros::Time::now();
          pose.header.frame_id = "/base_link";
          pose.pose.position.x = 0.25;
          pose.pose.position.y = -0.55;
          pose.pose.position.z = 1.1; //0.98
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.707;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 0.707;
    
          pose_pub.publish(pose);
          
          ros::Duration(4.5).sleep();
          
          while(pose.pose.position.z > 0.98)
          {
                pose.pose.position.z -= 0.005;
                pose.header.stamp = ros::Time::now();
                pose_pub.publish(pose);
                ros::Duration(0.1).sleep();
          }
          
          ROS_INFO("Placeing object");
          gripper->open();

          ros::Duration(1.0).sleep();
          
           while(pose.pose.position.z < 1.1)
          {
                pose.pose.position.z += 0.005;
                pose.header.stamp = ros::Time::now();
                pose_pub.publish(pose);
                ros::Duration(0.1).sleep();
          }
}

void timer_callback(const ros::TimerEvent& event)
{
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "/rotating_objects_18978_0";
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.04;
        pose.pose.position.z = z_position;
        if(go_down && z_position > 0.32)
                z_position -= 0.005;
        else if(!go_down && z_position < 0.45)
                z_position += 0.005;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.707;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 0.707;
    
        pose_pub.publish(pose);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_pick_and_place");
  
  ros::NodeHandle nh;
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/r_cart/command_pose",1);
  
  ros::Timer timer_pose = nh.createTimer(ros::Duration(0.1), &timer_callback);

  gripper.reset(new Gripper());

  gripper->open();
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  char a;
  std::cin >> a;
  ROS_INFO("Going down");
  go_down = true;
  std::cin >> a;
  gripper->close();
  std::cin >> a;
  ROS_INFO("Going up");
  go_down = false;
/*          std::cin >> a;
          ROS_INFO("Going down");
          go_down = true;
          std::cin >> a;
          gripper.open();
          std::cin >> a;
          ROS_INFO("Going up");
          go_down = false; */
  std::cin >> a;
  ROS_INFO("stopped rotation");
  timer_pose.stop();
  ros::Duration(1.5).sleep();
  place();
 
  return 0;
}
