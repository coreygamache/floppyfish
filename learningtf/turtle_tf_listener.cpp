#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;

  //wait for initial turtle to be spawned, then send service call to spawn another
  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  //create publisher to publish turtle_vel messages
  ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  //create listener to receive tf transformations from TransformBroadcaster
  //listener should be scoped to persist (i.e. as the member of a class), otherwise its cache won't fill and most queries will fail
  tf::TransformListener listener;

  ros::Rate rate (10);

  while (node.ok())
  {

    //create new transform with timestamp
    tf::StampedTransform transform;

    try
    {

      //query the listener for a specific transform:
      //arg 1 & 2) specify we want a transform from frame /turtle1 to frame /turtle2
      //arg 3) the timestamp of the transform (or ros::Time(0) for newest available)
      //arg 4) the transform object in which to store the result
      //listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
      //listener.lookupTransform("/turtle2", "/turtle1", ros::Time::now(), transform);

      //using waitForTransform with time = now
      /*ros::Time now = ros::Time::now();
      listener.waitForTransform("/turtle2", "/turtle1", now, ros::Duration(3.0));
      listener.lookupTransform("/turtle2", "/turtle1", now, transform);*/

      //using waitForTransform with time = 5 seconds ago
      //this gets a transform of turtle1's pose relative to turtle2's pose 5 seconds ago
      //this transform relates turtle1's pose 5 seconds ago with turtle2's pose 5 seconds ago; not where it is currently
      /*ros::Time past = ros::Time::now() - ros::Duration(5.0);
      listener.waitForTransform("/turtle2", "/turtle1", past, ros::Duration(1.0));
      listener.lookupTransform("/turtle2", "/turtle1", past, transform);*/

      //gets transform of turtle1's pose 5 seconds ago relative to turtle2's pose now
      //world is a fixed reference frame that doesn't change over time
      //this looks at turtle1's pose relative to world 5 seconds ago and turtle2's pose relative to world now to calculate transform
      ros::Time now = ros::Time::now();
      ros::Time past = now - ros::Duration(5.0);
      listener.waitForTransform("/turtle2", now, "/turtle1", past, "/world", ros::Duration(1.0));
      listener.lookupTransform("turtle2", now, "/turtle1", past, "/world", transform);

    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("error: %s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    //create new variable for new velocity values to be sent to turtle2
    geometry_msgs::Twist vel_msg;

    //calculate turtle2 velocity values based on its distance and angle to turtle1
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
}
