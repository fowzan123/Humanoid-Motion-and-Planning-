#include<ros/ros.h>
#include<chapter5_tutorials/odom_arduino.h>

int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "dummy_odom");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<chapter5_tutorials::odom_arduino>("odom_data",10);
    ros::Rate loop_rate(20);
    chapter5_tutorials::odom_arduino ticks;
    double test=0;
    while (ros::ok())
    {
        /* code for loop body */
        for(int i=0;i<10;i++)
        {   test++;
            ticks.vel_right=test;
            ticks.vel_left=test;
        }

        pub.publish(ticks);
        loop_rate.sleep();
    }
    
    return 0;
}
