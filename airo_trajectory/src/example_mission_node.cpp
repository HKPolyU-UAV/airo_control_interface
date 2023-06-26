#include <airo_trajectory/airo_trajectory_server.h>

int i = 0;
 
enum State{
    TAKEOFF,
    POSE_YAW,
    POSE_NO_YAW,
    POSE_TWIST,
    TRAJ_FILE,
    LAND
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_mission_node");
    ros::NodeHandle nh;
    ros::Rate rate(40.0);
    State state = TAKEOFF;

    AIRO_TRAJECTORY_SERVER airo_trajectory_server(nh);

    std::vector<geometry_msgs::Point> target_points;
    geometry_msgs::Twist target_twist;
    std::vector<double> target_yaw;
                
    target_points.resize(3);
    target_yaw.resize(2);

    target_points[0].x = 0;
    target_points[0].y = -2.5;
    target_points[0].z = 2.5;
    target_yaw[0] = M_PI/4;

    target_points[1].x = -1.5;
    target_points[1].y = 2.5;
    target_points[1].z = 1.0;
    target_yaw[1] = -M_PI/4;

    target_points[2].x = 0.0;
    target_points[2].y = 0.0;
    target_points[2].z = 1.0;

    target_twist.linear.x = 0.5;
    target_twist.linear.y = 0.5;
    target_twist.linear.z = 0.5;

    while(ros::ok()){
        switch(state){
            case TAKEOFF:{
                if (airo_trajectory_server.takeoff()){
                    state = POSE_YAW;
                }
                break;
            }

            case POSE_YAW:{
                airo_trajectory_server.pose_cmd(target_points[i],target_yaw[i]);
                if(airo_trajectory_server.target_reached(target_points[i])){
                    i += 1;
                    if(i == target_yaw.size()){
                        state = POSE_NO_YAW;
                    }
                }
                break;
            }

            case POSE_NO_YAW:{
                airo_trajectory_server.pose_cmd(target_points[2]);
                if(airo_trajectory_server.target_reached(target_points[2])){
                    state = POSE_TWIST;
                }
                break;
            }

            case POSE_TWIST:{
                airo_trajectory_server.pose_cmd(target_points[0],target_twist,target_yaw[0]);
                if(airo_trajectory_server.target_reached(target_points[0])){
                    state = LAND;
                }
                break;
            }

            case TRAJ_FILE:{
                
            }

            case LAND:{
                airo_trajectory_server.land();
                if (airo_trajectory_server.land()){
                    return 0;
                }
                break;
            }
        }

        ros::spinOnce();
        ros::Duration(rate).sleep();
    }

    return 0;
}