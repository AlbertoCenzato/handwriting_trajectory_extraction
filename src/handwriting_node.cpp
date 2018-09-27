//
// Created by alberto on 04/04/18.
//

#include <iostream>
#include <thread>
#include <chrono>

#include <ros/ros.h>

#include "handwriting.hpp"

int main(int argc, char** argv) {

    std::cout << "Starting handwriting..." << std::endl;

    ros::init(argc, argv, "handwriting_node");

    ros::NodeHandlePtr nh(new ros::NodeHandle);


    // extract hand trajectory from video
    bool showProcessedFrames = true;
    hw::Handwriting handwriting(nh, showProcessedFrames);

    geometry_msgs::PoseArray waypointCamFrame, waypointTagFrame;
    handwriting.getTrajectories(waypointCamFrame, waypointTagFrame);

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // advertise trajectory topics
    std::string camFrameTopic, tagFrameTopic;
    nh->param("/handwriting_node/waypoints_camera_frame", camFrameTopic, std::string("/handwriting/waypointsCameraFrame"));
    nh->param("/handwriting_node/waypoints_tag_frame",    tagFrameTopic, std::string("/handwriting/waypointsTagFrame"));
    auto trajectoryCamFramePub = nh->advertise<geometry_msgs::PoseArray>(camFrameTopic, 100);
    auto trajectoryTagFramePub = nh->advertise<geometry_msgs::PoseArray>(tagFrameTopic,    100);

    std::cout << "Publishing trajectory on topics: " << camFrameTopic << " and " << tagFrameTopic << std::endl;

    // publish trajectory points
    uint32_t seqNumber = 0;
    while (ros::ok()) {
        auto time = ros::Time::now();
        waypointCamFrame.header.stamp = time;
        waypointTagFrame.header.stamp = time;

        waypointCamFrame.header.seq = seqNumber;
        waypointTagFrame.header.seq = seqNumber++;

        trajectoryTagFramePub.publish(waypointTagFrame);
        trajectoryCamFramePub.publish(waypointCamFrame);

        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    return 0;
}