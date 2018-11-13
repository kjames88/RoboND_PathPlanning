#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

ros::Subscriber odometry_sub;

int num_goals = 2;
float locations[2][2] = { {-4.0, 3.0}, {7.0, -5.8} };
int goal_mux = 0;
std::vector<float> dist_history;
int dist_history_ptr = 0;
float marker_location[2] = {0.0, 0.0};
unsigned marker_state = 0;

float sum(const std::vector<float>& vec) {
    float s = 0.0;
    for (int i=0; i<vec.size(); i++) {
        s += vec[i];
    }
    return s;
}

void set_marker_location(unsigned mux) {
    if (mux < 3) {
        marker_location[0] = locations[mux][0];
        marker_location[1] = locations[mux][1];
    } else {
        marker_location[0] = 0.0;
        marker_location[1] = 0.0;
    }
}

void odometry_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    float p_x = odom_msg->pose.pose.position.x;
    float p_y = odom_msg->pose.pose.position.y;
    float g_x = 0.0;
    float g_y = 0.0;
    if (goal_mux < num_goals) {
        g_x = locations[goal_mux][0];
        g_y = locations[goal_mux][1];
        float dist = sqrt((g_x - p_x) * (g_x - p_x) + (g_y - p_y) * (g_y - p_y));
        if (dist_history.size() < 1000) {
            dist_history.push_back(dist);
        } else {
            dist_history.at(dist_history_ptr) = dist;
            dist_history_ptr = (dist_history_ptr + 1) % dist_history.size();
            float error = 0.0;
            for (size_t i=0; i<dist_history.size(); i++) {
                error += dist_history.at(i);
            }
            // average distance over preceding second
            error /= (float) dist_history.size();
            float travel = fabs(error - dist);
            if (travel < 0.01) {
                // distance is fairly stable, probably navigation has stopped
                // Note:  final position is sometimes off more than ideal so if robot
                //   isn't moving then a bit more distance is allowed to make sure goal is detected
                if (dist < 0.5 || (travel < 0.0001 && dist < 1.5)) {
                    // distance to goal is small and stable, looks like we've reached a goal
                    ROS_INFO("Goal reached!  pos %f,%f  goal %f,%f  dist %f, travel %f", p_x, p_y, g_x, g_y, dist,
                        travel);
                    if (goal_mux == 0) {
                        marker_state = 2;  // erase the current marker
                    } else {
                        set_marker_location(goal_mux);
                        marker_state = 1;  // draw new marker
                    }
                    goal_mux++;
                }
            }
        }
    }
}

void setup_marker(visualization_msgs::Marker* marker, unsigned char action) {
    uint32_t shape = visualization_msgs::Marker::CUBE;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker->header.frame_id = "/map";
    marker->header.stamp = ros::Time::now();

    // Set the namespace and id for this marker->  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker->ns = "add_markers";
    marker->id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker->type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker->action = action;

    // Set the pose of the marker->  This is a full 6DOF pose relative to the frame/time specified in the header
    marker->pose.position.x = marker_location[0];
    marker->pose.position.y = marker_location[1];
    marker->pose.position.z = 0;
    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;
    marker->pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker->scale.x = 0.2;
    marker->scale.y = 0.2;
    marker->scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker->color.r = 0.0f;
    marker->color.g = 0.0f;
    marker->color.b = 1.0f;
    marker->color.a = 1.0;

    marker->lifetime = ros::Duration();
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  bool standalone = false;
  if (ros::param::get("/standalone", standalone)) {
    std::cout << "/standalone = " << standalone << std::endl;
  } else {
    std::cout << "param not found" << std::endl;
  }
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  if (standalone == false) {
    odometry_sub = n.subscribe("odom", 1000, odometry_callback);
  }

  goal_mux = 0;
  set_marker_location(goal_mux);
  marker_state = 1;
  visualization_msgs::Marker marker;
  ros::Time marker_time = ros::Time::now();
  ros::Duration d;

  while (ros::ok()) {
      ros::spinOnce();
      usleep(10000);
      switch (marker_state) {
          case 1:
              // draw marker
              setup_marker(&marker, visualization_msgs::Marker::ADD);
              // Publish the marker
              marker_pub.publish(marker);
              // make sure rviz is subscribed before we stop publishing ADD
              if (marker_pub.getNumSubscribers() >= 1) {
                  marker_state = (standalone && goal_mux == 0) ? 4 : 0;
              }
              break;
          case 2:
              // erase previous marker
              setup_marker(&marker, visualization_msgs::Marker::DELETE);
              // Publish the marker
              marker_pub.publish(marker);
              if (standalone) {
                goal_mux++;
                set_marker_location(goal_mux);  // dropoff location
                marker_state = 3;
              } else {
                marker_state = 0;
              }
              break;
          case 3:
              marker_time = ros::Time::now();
              marker_state = 5;
              break;
          case 4:
              marker_time = ros::Time::now();
              marker_state = 6;
              break;
          case 5:
              // delay then draw
              d = ros::Time::now() - marker_time;
              if (d.sec >= 5) {
                  marker_state = 1;
              }
              break;
          case 6:
              // delay then erase
              d = ros::Time::now() - marker_time;
              if (d.sec >= 5) {
                  marker_state = 2;
              }
              break;
          case 0:
          default:
              // NOP
              break;
      }
  }

}
