#include <katana_gazebo_plugins/gazebo_ros_katana_gripper_action_interface.h>

namespace katana_gazebo_plugins
{

void IGazeboRosKatanaGripperAction::setCurrentPoint(double pos, double vel)
{
  GRKAPoint point = {pos, vel};
  setCurrentPoint(point);
}

}
