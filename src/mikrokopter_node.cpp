#include <ros/ros.h>
#include "mikrokopter_node/mikrokopter.h"

using namespace mikrokopter;

int main(int argc, char **argv) {

    ros::init(argc, argv, "mikrokopter");
    ros::NodeHandle n("~");

    Mikrokopter mikrokopter(n);
    if (mikrokopter.isReady()) {
        ros::spin();
    }
    return 0;
}
