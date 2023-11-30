#include <ros/ros.h>
#include <std_msgs/LaserScan.h>
#include <vector>
#include <tuple>
//ranges includes the measurements

int measuments_queue = 0;
std::vector<std::tuple<float,float>> positions;
std::vector<std::tuple<float,float>> euclidean_positions;
// each entry is a vector for person i
std::vector<std::vector<std::tuple<float,float>>> instant_positions;
std::vector<std::vector<std::tuple<float,float>>> detected_positions;

void scanCallback(const std_msgs::LaserScan::ConstPtr& msg)
{
    float[] ranges = msg->ranges;
    //*** Utility variables to distinguish people
    int non_zero_seq = 0;
    int zero_seq = 0;
    // Conversion cycle
    for(int i=0; i<ranges.size(); i++){
        if(ranges[i] == std::numeric_limits<float>::infinity()){
            zero_seq++;
            continue;
        }
        else{
            //Convert to euclidean

            non_zero_seq++;
            //If non_zero_seq is 1 it's a new person
            if (non_zero_seq == 1){
                instant_positions.push_back(std::vector<std::tuple<float,float>>());
                //Push
            }   
            //If the zero sequence is small and we have some
            // stored measuments, it's the other feet so add to the same person

            //If the zero sequence is big, it's a new person
            if(zero_seq >= 5){
                non_zero_seq = 1;
                instant_positions.push_back(std::vector<std::tuple<float,float>>());
                //Push
            }
            zero_seq = 0;
        }
    }

    // Instaneous position cycle
    for(int k=0; k<instant_positions.size(); k++){
        //If the vector is empty, initialize it
        if(detected_position.empty()){
            //Initialize with k empty vectors
            for(int i=0; i<k; i++){
                detected_positions.push_back(std::vector<std::tuple<float,float>>());
            }
        }

        float x_mean = 0;
        float y_mean = 0;
        for(int i=0; i<instant_positions[k].size(); i++){
            x_mean += std::get<0>(instant_positions[k][i]);
            y_mean += std::get<1>(instant_positions[k][i]);
        }
        x_mean = x_mean/instant_positions[k].size();
        y_mean = y_mean/instant_positions[k].size();
        detected_positions[k].push_back(std::make_tuple(x_mean,y_mean));
        instant_positions.clear();
    }

    // Increase the counter for the stored measurements
    measuments_queue++;
    // Mean measurement printing
    if (measuments_queue==10){
        //Do the mean and print the positions
        for(int j=0; j<detected_positions.size(); j++){
            float x_mean = 0;
            float y_mean = 0;
            for(int i=0; i<detected_positions[j].size(); i++){
                x_mean += std::get<0>(detected_positions[j][i]);
                y_mean += std::get<1>(detected_positions[j][i]);
            }
            x_mean = x_mean/detected_positions[j].size();
            y_mean = y_mean/detected_positions[j].size();
            ROS_INFO("Person %d is at (%f,%f)", j+1, x_mean, y_mean);
        }
        // Reset the variables
        measuments_queue = 0;
        detected_positions.clear();
    }
}