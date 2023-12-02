#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <tuple>
#include <limits>

/*
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
            double x = ranges[i] * cos(angle_min + i * angle_increment);
            double y = ranges[i] * sin(angle_min + i * angle_increment);
            non_zero_seq++;
            //If non_zero_seq is 1 it's a new person
            if (non_zero_seq == 1){
                instant_positions.push_back(std::vector<std::tuple<float,float>>());
                //Push the coordinates in the vector
                instant_positions[instant_positions.size()-1].push_back(std::make_tuple(x,y));
            }   
            //If the zero sequence is small and we have some
            // stored measuments, it's the other feet so add to the same person
            else if(zero_seq < 5 && instant_positions.size() > 0 && non_zero_seq > 1){
                instant_positions[instant_positions.size()-1].push_back(std::make_tuple(x,y));
                zero_seq = 0;
            }
            //If the zero sequence is big, it's a new person
            else if(zero_seq >= 5){
                non_zero_seq = 1;
                instant_positions.push_back(std::vector<std::tuple<float,float>>());
                //Push
                instant_positions[instant_positions.size()-1].push_back(std::make_tuple(x,y));
            }
            zero_seq = 0;
        }
    }

    for (int k = 0; k < instant_positions.size(); k++) {
        // If the vector is empty, initialize it
        if (detected_positions.size() <= k) {
            detected_positions.push_back(std::vector<std::tuple<float, float>>());
        }

        float x_mean = 0;
        float y_mean = 0;
        for (int i = 0; i < instant_positions[k].size(); i++) {
            x_mean += std::get<0>(instant_positions[k][i]);
            y_mean += std::get<1>(instant_positions[k][i]);
        }
        x_mean = x_mean / instant_positions[k].size();
        y_mean = y_mean / instant_positions[k].size();
        detected_positions[k].push_back(std::make_tuple(x_mean, y_mean));
    }

    // Increase the counter for the stored measurements
    measuments_queue++;

    // Mean measurement printing
    if (measuments_queue == 10) {
        // Do the mean and print the positions
        for (int j = 0; j < detected_positions.size(); j++) {
            ROS_INFO("Detected positions size: %d", detected_positions[j].size());
            //Print detected positions size
            if (!detected_positions[j].empty()) {
                float x_mean = 0;
                float y_mean = 0;
                for (int i = 0; i < detected_positions[j].size(); i++) {
                    x_mean += std::get<0>(detected_positions[j][i]);
                    y_mean += std::get<1>(detected_positions[j][i]);
                }
                x_mean = x_mean / detected_positions[j].size();
                y_mean = y_mean / detected_positions[j].size();
                ROS_INFO("Person %d is at (%f,%f)", j + 1, x_mean, y_mean);
            }
        }

        // Reset the variables
        measuments_queue = 0;
        detected_positions.clear();

*/

//*** Global variables
const double angle_min = 0.0;
const double angle_max = 6.8;
const double angle_increment = 0.0087;
const double range_min = 0.12;
const double range_max = 3.5;
const int num_scans_for_clustering = 20;
int scan_count = 0;

std::vector<float> ranges_concat;
std::vector<std::tuple<float, float>> euclidean_positions;
std::vector<std::tuple<float, float>> centroids;

// Function to convert polar coordinates to Cartesian coordinates
std::tuple<float, float> polarToCartesian(float range, float angle) {
    float x = range * cos(angle);
    float y = range * sin(angle);
    return std::make_tuple(x, y);
}

// Function to perform clustering
void clusterPoints(const std::vector<float>& ranges, float clustering_threshold) {
    std::vector<std::vector<float>> clustered_points;

    // Iterate through the ranges
    for (int i = 0; i < ranges.size(); ++i) {
        if (ranges[i] != std::numeric_limits<float>::infinity()) {
            // Convert polar coordinates to Cartesian coordinates
            std::tuple<float, float> current_point = polarToCartesian(ranges[i], i * angle_increment);

            // Check if the current point is close to any existing centroid
            bool found_cluster = false;
            for (int j = 0; j < centroids.size(); ++j) {
                float distance = sqrt(pow(std::get<0>(current_point) - std::get<0>(centroids[j]), 2) +
                                      pow(std::get<1>(current_point) - std::get<1>(centroids[j]), 2));
                if (distance < clustering_threshold) {
                    found_cluster = true;
                    // Update the centroid of the existing cluster
                    centroids[j] = std::make_tuple(
                        (std::get<0>(centroids[j]) + std::get<0>(current_point)) / 2,
                        (std::get<1>(centroids[j]) + std::get<1>(current_point)) / 2);
                    // Add the point to the existing cluster (optional)
                    clustered_points[j].push_back(ranges[i]);
                    break;
                }
            }

            // If the point doesn't belong to any existing cluster, create a new cluster
            if (!found_cluster) {
                clustered_points.push_back({ranges[i]});
                centroids.push_back(current_point);
            }
        }
    }
}

// Callback function for the laser scan
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Extract ranges from the message
    std::vector<float> ranges = msg->ranges;

    //Add to the concatenation
    ranges_concat.insert(ranges_concat.end(), ranges.begin(), ranges.end());
    // Set a clustering threshold (adjust as needed)
    float clustering_threshold = 0.7;


    // Increment the scan count
    scan_count++;

    // If the required number of scans is reached, print the centroids
    if (scan_count == num_scans_for_clustering) {
        // Perform clustering
        clusterPoints(ranges_concat, clustering_threshold);

        ROS_INFO("Euclidean positions after %d scans:", num_scans_for_clustering);
        for (const auto& position : centroids) {
            ROS_INFO("(%f, %f)", std::get<0>(position), std::get<1>(position));
        }

        // Reset scan_count and euclidean_positions
        scan_count = 0;
        centroids.clear();
        ranges_concat.clear();
    }
}

//*** Main
int main(int argc, char** argv){
    //initialize the node
    ros::init(argc, argv, "laser_scanner");
    ros::NodeHandle nh;

    //subscribe to the topic /scan
    ros::Subscriber sub = nh.subscribe("/scan", 1000, scanCallback);
    
    //spin at 5 Hz
    ros::Rate rate(5);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}