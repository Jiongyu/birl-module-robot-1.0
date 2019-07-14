#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

void set_marker(visualization_msgs::Marker& marker,\
                double position_x,double position_y,\
                double position_z,double orientation_x,\
                double orientation_y,double orientation_z,\
                double orientation_w, int ID);

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_array;

  while (ros::ok())
  {
    set_marker(marker,0,0,1.2,0,0,0,1,0);
    marker_array.markers.push_back(marker);
    //set_marker(marker,0,-1.5,1.2,0,0,0,1,1);
    //marker_array.markers.push_back(marker);

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker_array);

    r.sleep();
  }
}

void set_marker(visualization_msgs::Marker& marker,\
                double position_x,double position_y,\
                double position_z,double orientation_x,\
                double orientation_y,double orientation_z,\
                double orientation_w, int ID){

    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "pole";
    marker.id = ID;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = position_x;
    marker.pose.position.y = position_y;
    marker.pose.position.z = position_z;
    marker.pose.orientation.x = orientation_x;
    marker.pose.orientation.y = orientation_y;
    marker.pose.orientation.z = orientation_z;
    marker.pose.orientation.w = orientation_w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
}