#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <tuple>
#include <limits>
#include <cmath>

//*** Global variables
    // Robot parameters
const double angle_min = 0.0;
const double angle_max = 6.28;
const double angle_increment = 0.0087;
const double range_min = 0.12;
const double range_max = 3.5;
    // Accumulation before clustering
const int num_scans_for_clustering = 20;
    // Number of expected points in a scan
const int vector_size = std::floor((angle_max - angle_min) / angle_increment);
    // Scan counter
int scan_count = 0;
    // Vectors for storing the data
std::vector<float> ranges_concat;
std::vector<std::tuple<float, float>> euclidean_positions;
std::vector<std::tuple<float, float>> centroids;

//*** Functions declaration

/// @brief Transforms (r, theta) to (x, y)
/// @param range r
/// @param angle theta
/// @return (x,y)
std::tuple<float, float> polarToCartesian(float range, float angle);

/// @brief Clusters the points in ranges, centroids can be found in the global variable centroids
/// @param ranges concatenation of points
/// @param clustering_threshold distance threshold for merging
void clusterPoints(const std::vector<float>& ranges, float clustering_threshold);

/// @brief Callback for the /scan topic
/// @param msg LaserScan message
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

//*** Main
int main(int argc, char** argv){
    // Initialize the node
    ros::init(argc, argv, "laser_scanner");
    ros::NodeHandle nh;

    // Subscribe to the topic /scan
    ros::Subscriber sub = nh.subscribe("/scan", 1000, scanCallback);
    
    // Spin at 5 Hz
    ros::Rate rate(5);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}

//*** Functions implementation

std::tuple<float, float> polarToCartesian(float range, float angle) {
    float x = range * cos(angle);
    float y = range * sin(angle);
    return std::make_tuple(x, y);
}

void clusterPoints(const std::vector<float>& ranges, float clustering_threshold) {
    // For each point
    for (int i = 0; i < ranges.size(); ++i) {
        // If not infinity, convert and cluster
        if (ranges[i] != std::numeric_limits<float>::infinity()) {
            //since ranges is a concatenation of all the scans, each 720 points we have a new scan
            //print vector size and the real size of the vector
            std::tuple<float, float> current_point = polarToCartesian(ranges[i], angle_min + (i % vector_size) * angle_increment);

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
                    break;
                }
            }

            // If the point isn't close to a cluster, create a new one
            if (!found_cluster) {
                centroids.push_back(current_point);
            }
        }
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Extract ranges from the message
    std::vector<float> ranges = msg->ranges;

    //Add to the concatenation
    ranges_concat.insert(ranges_concat.end(), ranges.begin(), ranges.end());

    // Set a clustering threshold
    float clustering_threshold = 0.7;

    // Increment the scan count
    scan_count++;

    // When we accumulate enough points, cluster
    if (scan_count == num_scans_for_clustering) {

        clusterPoints(ranges_concat, clustering_threshold);

        ROS_INFO("Euclidean positions after %d scans:", num_scans_for_clustering);
        for (const auto& position : centroids) {
            ROS_INFO("(%f, %f)", std::get<0>(position), std::get<1>(position));
        }

        // Reset the variables
        scan_count = 0;
        centroids.clear();
        ranges_concat.clear();
    }
}


/* Non clustering version
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