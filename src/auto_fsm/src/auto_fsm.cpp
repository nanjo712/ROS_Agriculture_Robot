#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int8.h>

enum STATE{
    START=0,
    GO_TO_POINT,
    DETECT_AND_CALIB_VASE,
    REPORT_AND_WATER,
    STOP
};

enum DROUGHT_LEVEL{
    SLIGHT=0,
    NORMAL,
    SEVERE
};

const std::string topic_list[3]={"/empty_topic","/ctrl_vel","/calib_vel"};

DROUGHT_LEVEL drought_list[3][6]={
    {NORMAL,NORMAL,NORMAL,NORMAL,NORMAL,NORMAL},
    {NORMAL,NORMAL,NORMAL,NORMAL,NORMAL,NORMAL},
    {NORMAL,NORMAL,NORMAL,NORMAL,NORMAL,NORMAL},
};

std::pair<double,double> point_list[4][8]={
    {std::make_pair(0.0,0.0),std::make_pair(0.6,0.0),std::make_pair(1.2,0.0),std::make_pair(1.8,0.0),
    std::make_pair(2.4,0.0),std::make_pair(3.0,0.0),std::make_pair(3.6,0.0),std::make_pair(4.2,0.0)},
    
    {std::make_pair(4.2,1.1),std::make_pair(3.6,1.1),std::make_pair(3.0,1.1),std::make_pair(2.4,1.1),
    std::make_pair(1.8,1.1),std::make_pair(1.2,1.1),std::make_pair(0.6,1.1),std::make_pair(0.0,1.1)},

    {std::make_pair(0.0,2.2),std::make_pair(0.6,2.2),std::make_pair(1.2,2.2),std::make_pair(1.8,2.2),
    std::make_pair(2.4,2.2),std::make_pair(3.0,2.2),std::make_pair(3.6,2.2),std::make_pair(4.2,2.2)},

    {std::make_pair(4.2,3.4),std::make_pair(3.6,3.4),std::make_pair(3.0,3.4),std::make_pair(2.4,3.4),
    std::make_pair(1.8,3.4),std::make_pair(1.2,3.4),std::make_pair(0.6,3.4),std::make_pair(0.0,3.4)},
};



class FSM
{
private:
    STATE current;
    ros::NodeHandle nh;
    ros::Subscriber vel_sub;
    ros::Publisher vel_pub;
    ros::Publisher point_pub;
    ros::Publisher state_pub;
    geometry_msgs::PointStamped nextPoint;
    int curField,pointIndex;
    int vel_topic;
public:
    FSM(ros::NodeHandle &handle)
    {
        nh=handle;
        current=START;
        vel_topic=0;
        vel_sub=nh.subscribe(topic_list[0],1,&FSM::velMsgCallback,this);
        vel_pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        point_pub=nh.advertise<geometry_msgs::PointStamped>("/clicked_point",10);
        state_pub=nh.advertise<std_msgs::Int8>("/FSM_state",10);
        curField=0,pointIndex=0;
    }
    void velMsgCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        if (msg->linear.z==1)
        {
            FSM_next_state();
            ROS_INFO("Next State %d",current);
        }
        vel_pub.publish(*msg);
    }
    void find_next_point()
    {
        pointIndex++;
        if (pointIndex==8)  // which means we need to switch to next field;
        {
            curField++;
            pointIndex=0;
        }
        nextPoint.header.frame_id="map";
        nextPoint.point.x=point_list[curField][pointIndex].first;
        nextPoint.point.y=point_list[curField][pointIndex].second;
        point_pub.publish(nextPoint);
    }
    void FSM_next_state()
    {
        if (current==START)
        {
            current=GO_TO_POINT;
            vel_topic=1;   
            vel_sub=nh.subscribe(topic_list[vel_topic],1,&FSM::velMsgCallback,this);
            find_next_point();
        }
        else if (current==GO_TO_POINT)
        {
            if (curField==4) 
            {
                current=STOP;
                vel_topic=0;
                vel_sub=nh.subscribe(topic_list[vel_topic],1,&FSM::velMsgCallback,this);
                return ;
            }
            else 
            {
                // In the case we have not reached the end
                current=DETECT_AND_CALIB_VASE;
                vel_topic=2;
                vel_sub=nh.subscribe(topic_list[vel_topic],1,&FSM::velMsgCallback,this);
            }
        }
        else if (current==DETECT_AND_CALIB_VASE)
        {
            if (1<=pointIndex&&pointIndex<=6)
            {
                current=REPORT_AND_WATER;
                vel_topic=0;
                vel_sub=nh.subscribe(topic_list[vel_topic],1,&FSM::velMsgCallback,this);
            }
            else
            {
                current=GO_TO_POINT;
                vel_topic=1;   
                vel_sub=nh.subscribe(topic_list[vel_topic],1,&FSM::velMsgCallback,this);
                find_next_point();
            }
        }
        else if (current==REPORT_AND_WATER)
        {
            current=GO_TO_POINT;
            vel_topic=1;   
            vel_sub=nh.subscribe(topic_list[vel_topic],1,&FSM::velMsgCallback,this);
            find_next_point();
        }

        std_msgs::Int8 tmp;
        tmp.data=current;
        state_pub.publish(tmp);
    }
    void FSM_exec()
    {
        // ROS_INFO("Current State %d",current);
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
                break;
            }
            case DETECT_AND_CALIB_VASE:
            {
                break;
            }
            case REPORT_AND_WATER:    
            {
                vel_topic=0;

                geometry_msgs::Twist msg;
                msg.linear.x=0;
                msg.linear.y=drought_list[curField][pointIndex-1];
                msg.linear.z=0;
                msg.angular.x=0;
                msg.angular.y=0;
                msg.angular.z=0;

                for (int i=0;i<drought_list[curField][pointIndex-1];i++)
                {
                    vel_pub.publish(msg);
                    ros::Duration(1.5).sleep();
                }   
                FSM_next_state();
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
    FSM fsm(nh);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        fsm.FSM_exec();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}