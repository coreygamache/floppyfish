#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10);

  while(node.ok())
  {

    //set the translation of the new transform (offset 2 m left of parent frame [turtle1 in this case, as seen below])
    //edit: changed frame to change over time
    transform.setOrigin(tf::Vector3(2.0*sin(ros::Time::now().toSec()), 2.0*cos(ros::Time::now().toSec()), 0.0));

    //set the rotation of the new transform (x, yi, zj, wk)
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));

    //broadcast new transform from frame turtle1 (parent) to carrot1 (child)
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "turtle1", "carrot1"));

    rate.sleep();
  }

  return 0;
}
