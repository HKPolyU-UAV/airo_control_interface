#include "airo_trajectory_utils.hpp"


bool AIRO_TRAJECTORY_UTILS::target_reached(const geometry_msgs::Point& msg, const geometry_msgs::PoseStamped::ConstPtr& current_pose){
    return sqrt(pow(msg.x - current_pose->pose.position.x,2)+pow(msg.y - current_pose->pose.position.y,2)
    +pow(msg.z - current_pose->pose.position.z,2)) < 1.0;
}

geometry_msgs::Quaternion AIRO_TRAJECTORY_UTILS::yaw_to_quaternion(double yaw){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
    return quaternion;
}

int AIRO_TRAJECTORY_UTILS::readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data){
    std::ifstream file(fileName);
    std::string line;
    int number_of_lines = 0;

    if (file.is_open())
    {
        while(getline(file, line)){
            number_of_lines++;
            std::istringstream linestream( line );
            std::vector<double> linedata;
            double number;

            while( linestream >> number ){
                linedata.push_back( number );
            }
            data.push_back( linedata );
        }

        file.close();
    }
    else
    {
        return 0;
    }

    return number_of_lines;
}

void AIRO_TRAJECTORY_UTILS::takeoff(){
    if(fsm_info.is_landed == true){
        while(ros::ok()){
            ROS_INFO_THROTTLE(1.0, "[AIRo TRAJECTORY] Sending takeoff trigger.");
            takeoff_land_trigger.takeoff_land_trigger = true; // Takeoff
            takeoff_land_trigger.header.stamp = ros::Time::now();
            takeoff_land_pub.publish(takeoff_land_trigger);
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            if(fsm_info.is_waiting_for_command){
                state = COMMAND;
                break;
            }
        }
    }
}

// void ref_cb(int line_to_read)
//         {
//             if (QUADROTOR_N+line_to_read+1 <= number_of_steps)  // All ref points within the file
//             {
//                 for (unsigned int i = 0; i <= QUADROTOR_N; i++)  // Fill all horizon with file data
//                 {
//                     for (unsigned int j = 0; j <= QUADROTOR_NY; j++)
//                     {
//                         acados_in.yref[i][j] = trajectory[i+line_to_read][j];
//                     }
//                 }
//             }
//             else if(line_to_read < number_of_steps)    // Part of ref points within the file
//             {
//                 for (unsigned int i = 0; i < number_of_steps-line_to_read; i++)    // Fill part of horizon with file data
//                 {
//                     for (unsigned int j = 0; j <= QUADROTOR_NY; j++)
//                     {
//                         acados_in.yref[i][j] = trajectory[i+line_to_read][j];
//                     }
//                 }

//                 for (unsigned int i = number_of_steps-line_to_read; i <= QUADROTOR_N; i++)  // Fill the rest horizon with the last point
//                 {
//                     for (unsigned int j = 0; j <= QUADROTOR_NY; j++)
//                     {
//                         acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
//                     }
//                 }
//             }
//             else    // none of ref points within the file
//             {
//                 for (unsigned int i = 0; i <= QUADROTOR_N; i++)  // Fill all horizon with the last point
//                 {
//                     for (unsigned int j = 0; j <= QUADROTOR_NY; j++)
//                     {
//                         acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
//                     }
//                 }
//             }
//         }