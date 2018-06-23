// STL
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

ros::Publisher pub;
sensor_msgs::Joy joy_msg;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mouse_joy");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::Joy>("/joy", 1);
    
    int fd, bytes;
    unsigned char data[3];

    const char* pDevice = "/dev/input/mouse1";

    // Open Mouse
    fd = open(pDevice, O_RDWR);
    if(fd == -1)
    {
        printf("ERROR Opening %s\n", pDevice);
        return -1;
    }
    
    ros::Rate loop_rate(10);

    int left, middle, right;
    signed char xChar, yChar;
    //int prevX, prevY;
    int x, y;
    while(1)
    {
        // Read Mouse     
        bytes = read(fd, data, sizeof(data) );

        if(bytes > 0)
        {
            left = data[0] & 0x1;
            right = data[0] & 0x2;
            middle = data[0] & 0x4;

            xChar = data[1];
            yChar = data[2];
            x = (int)xChar;
            y = (int)yChar;
            //printf("x=%d, y=%d, left=%d, middle=%d, right=%d\n", xChar, yChar, left, middle, right);
            //printf("x=%d, y=%d\n", x, y);
            if(x > 0)
                printf("=>\n");
            else if(x < 0)
                printf("<=\n");
            
            if(y > 0)
                printf("^\n");
            else if(y < 0)
                printf("v\n");
         
            joy_msg = sensor_msgs::Joy();
            joy_msg.header.stamp = ros::Time::now();
            joy_msg.header.frame_id = "mouse_joystick";
            
            joy_msg.axes.resize(2);
            joy_msg.axes[0] = x;
            joy_msg.axes[1] = y;
            
            joy_msg.buttons.resize(3);
            joy_msg.buttons[0] = left;
            joy_msg.buttons[1] = middle;
            joy_msg.buttons[2] = right;
            
            pub.publish(joy_msg);
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
    return 0; 
}
