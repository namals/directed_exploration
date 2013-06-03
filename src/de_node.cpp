#include <directed_exploration/de_planner.h>

#include <iostream>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "de_planner");   
    DE_Planner planner;
    ros::spin();
    return 0;
}
