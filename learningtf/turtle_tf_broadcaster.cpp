#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg)
{

  //create a transform broadcaster object, provided by tf/transform_broadcaster.h
  static tf::TransformBroadcaster br;

  //create a transform object and set initial values (x, y, rotation) to those provided in msg
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0)); //set x and y values
  tf::Quaternion q; //create quaternion object
  q.setRPY(0, 0, msg->theta); //set roll, pitch, and yaw of quaternion
  transform.setRotation(q); //set rotation value of transform to previously defined quaternion object

  //broadcast the transform with timestamp set to current time, parent frame "world", and child frame of the same name as the turtle
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2)
  {
    ROS_ERROR("need turtle name as argument");
    return 1;
  }

  turtle_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
}
