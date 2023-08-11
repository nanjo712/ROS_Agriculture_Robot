#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
enum STATE{
    START=0,
    GO_TO_POINT,
    DETECT_AND_CALIB_VASE,
    REPORT_AND_WATER,
    STOP
};

const std::string topic_list[3]={"/empty_topic","/ctrl_vel","/calib_vel"};

class FSM
{
private:
    STATE current;
    ros::NodeHandle nh;
    ros::Subscriber vel_sub;
    ros::Publisher vel_pub;
    int vel_topic;
public:
    FSM(ros::NodeHandle &handle)
    {
        nh=handle;
        vel_topic=0;
        vel_sub=nh.subscribe(topic_list[0],1,&FSM::velMsgCallback,this);
        vel_pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }
    void velMsgCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        if (msg->linear.z==1)
        {
            FSM_next_state();
        }
        vel_pub.publish(*msg);
    }
    void FSM_next_state()
    {
        if (current==START)
        {
            current=GO_TO_POINT;
        }
        else if (current==GO_TO_POINT)
        {
            //TODO: check if the robot has reached the end
            current=STOP;

            // In the case we have not reached the end
            current=DETECT_AND_CALIB_VASE;
        }
        else if (current==DETECT_AND_CALIB_VASE)
        {
            current=GO_TO_POINT;
        }
    }
    void FSM_exec()
    {
        switch (current)
        {   
            case START:
            {
                std::vector<std::string> nodes;
                ros::master::getNodes(nodes);
                ROS_INFO("Running nodes:");
                for (const std::string& node : nodes)
                {
                    ROS_INFO("- %s", node.c_str());
                }
                //TODO: check if all the necessary node has been activated
                FSM_next_state();
                break;
            }
            case GO_TO_POINT:
            {
                vel_topic=1;   
                vel_sub=nh.subscribe(topic_list[vel_topic],1,&FSM::velMsgCallback,this);
                break;
            }
            case DETECT_AND_CALIB_VASE:
            {
                vel_topic=2;
                vel_sub=nh.subscribe(topic_list[vel_topic],1,&FSM::velMsgCallback,this);
                break;
            }
            case REPORT_AND_WATER:    
            {
                vel_topic=0;
                // TODO: Report and Water
                break;
            }
            case STOP:
            {
                vel_topic=0;
                //TODO: STOP
                break;
            }
        }
    }
};
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "Auto_FSM");
    ros::NodeHandle nh;

    return 0;
}