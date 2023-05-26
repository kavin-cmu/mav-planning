#include <mav_planning/global_planner/global_planner_ros.h>

int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "mav_planner_node");
    ros::NodeHandle nh_priv("~"), nh;

    mav_planning::PlanManagerROS manager(nh_priv, nh);
    ros::spin();
}