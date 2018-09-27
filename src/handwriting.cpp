//
// Created by alberto on 04/04/18.
//

#include "handwriting.hpp"

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "trajectory_filters.hpp"

namespace hw {

    // ----------------- Utility functions -----------------------

    cv::Point2d getRightHandCentroid(const openpose_ros_msgs::OpenPoseHumanListConstPtr &humanListMsg)
    {
        cv::Point2d rightHandCentroid(0.f, 0.f);
        const auto &points = humanListMsg->human_list[0].right_hand_key_points_with_prob;
        for (const auto &point : points) {
            rightHandCentroid.x += point.x;
            rightHandCentroid.y += point.y;
        }
        rightHandCentroid.x /= points.size();
        rightHandCentroid.y /= points.size();

        return rightHandCentroid;
    }

    /**
     * Gets the depth of point from depthImage averaging over
     * a square of roiSize^2 pixels centered in point
     * @param point
     * @param depthImage
     * @param roiSize
     * @return
     */
    double getDepth(const cv::Point &point, const cv::Mat &depthImage, int roiSize)
    {
        cv::Rect ROI(point.x - roiSize/2, point.y - roiSize/2, roiSize, roiSize);

        double depth = 0;
        for (auto i = ROI.x; i < ROI.x + ROI.width; ++i)
            for (auto j = ROI.y; j < ROI.y + ROI.height; ++j)
                depth += depthImage.at<uint16_t>(j,i);

        return depth / (roiSize*roiSize);
    }


    bool drawTrajectory(const std::vector<tf2::Vector3> &trajectory, int timeout, std::string windowName="Trajectory")
    {
        if (trajectory.size() < 5)
            return false;

        double xMax = -std::numeric_limits<double>::max();
        double yMax = -std::numeric_limits<double>::max();
        double xMin = std::numeric_limits<double>::max();
        double yMin = std::numeric_limits<double>::max();

        for (const auto &point : trajectory) {
            if (point.x() < xMin)
                xMin = point.x();
            if (point.x() > xMax)
                xMax = point.x();
            if (point.y() < yMin)
                yMin = point.y();
            if (point.y() > yMax)
                yMax = point.y();
        }

        auto width  = int(std::abs(xMax - xMin));
        auto height = int(std::abs(yMax - yMin));

        cv::Mat image = cv::Mat::zeros(height, width, CV_8UC1);
        for (const auto &point : trajectory) {
            cv::Point handPosition(point.x() - xMin, point.y() - yMin);
            cv::circle(image, handPosition, 5, cv::Scalar(255,255,255), 2);
        }

        cv::imshow(windowName, image);
        cv::waitKey(timeout);

        return true;
    }

    void vectorToPose(const tf2::Vector3 &vec, geometry_msgs::Pose &pose)
    {
        pose.position.x = vec.x();
        pose.position.y = vec.y();
        pose.position.z = vec.z();

        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
    }


    // -------------------------------------------------------------
    // -------------------- Handwriting ----------------------------
    // -------------------------------------------------------------


    const ros::Duration Handwriting::REFERENCE_FRAMES_TIMEOUT    = ros::Duration(10);
    const ros::Duration Handwriting::COMPLETE_TRAJECTORY_TIMEOUT = ros::Duration(5);
    const std::string   Handwriting::TRAJECTORY_WINDOW           = "Trajectory";
    const std::string   Handwriting::IMAGE_WINDOW                = "RGB image";


    Handwriting::Handwriting(ros::NodeHandlePtr nh, bool showProcessedFrames) :
            nh(nh), receivingFrames(true), synchronizer(MESSAGE_QUEUE_LENGTH), showProcessedFrames(showProcessedFrames)
    {
        // gets topic names from node parameters
        nh->param("/handwriting_node/tag_reference_frame",    tagReferenceFrame, std::string("tag"));
        nh->param("/handwriting_node/camera_reference_frame", camReferenceFrame, std::string("kinect2_head_rgb_optical_frame"));
        nh->param("/handwriting_node/rgb_camera_info",        rgbCamInfoTopic,   std::string("/kinect2_head/rgb_lowres/camera_info"));

        nh->param("/handwriting_node/image_depth", depthImageTopic, std::string("/kinect2_head/depth_lowres/image"));
        nh->param("/handwriting_node/image_rgb",   rgbImageTopic,   std::string("/kinect2_head/rgb_lowres/image"));
        nh->param("/handwriting_node/human_list",  humanListTopic,  std::string("/openpose_ros/human_list"));
        nh->param("/handwriting_node/tag_tf",      tagPoseTopic,    std::string("/tf"));

        if (showProcessedFrames) {
            cv::namedWindow(TRAJECTORY_WINDOW, CV_WINDOW_NORMAL);
            cv::namedWindow(IMAGE_WINDOW, CV_WINDOW_NORMAL);
        }
    }


    Handwriting::~Handwriting()
    {
        if (showProcessedFrames) {
            cv::destroyWindow(TRAJECTORY_WINDOW);
            cv::destroyWindow(IMAGE_WINDOW);
        }
    }

    void Handwriting::getTrajectories(geometry_msgs::PoseArray &waypointCamFrame,
                                      geometry_msgs::PoseArray &waypointTagFrame)
    {
        retrieveReferenceFramesInfo();

        subscribeTopics();

        // stop receiving frames after some time no frame is received on the subscribed topics
        completeTrajectoryTimer = nh->createTimer(COMPLETE_TRAJECTORY_TIMEOUT, &Handwriting::timerCallback, this);

        // continue receiving hand positions until timer ends
        while (receivingFrames)
            ros::spinOnce();

        unsubscribeTopics();

        for (const auto &point : trajectoryCamFrame)        // transform the trajectory from camera to tag frame
            trajectoryTagFrame.push_back(cam2tag(point));

        cleanupTrajectory();

        if (!drawTrajectory(trajectoryTagFrame, 2000, TRAJECTORY_WINDOW))   // display complete trajectory
            std::cout << "WARNING: trajectory contains very few points!" << std::endl;

        for (const auto point : trajectoryCamFrame) {
            geometry_msgs::Pose pose;
            vectorToPose(point, pose);
            waypointCamFrame.poses.push_back(pose);
        }
        for (const auto point : trajectoryTagFrame) {
            geometry_msgs::Pose pose;
            vectorToPose(point, pose);
            pose.position.z = 0;
            waypointTagFrame.poses.push_back(pose);
        }

        waypointCamFrame.header.frame_id = camReferenceFrame;
        waypointTagFrame.header.frame_id = tagReferenceFrame;
    }



    // ------------------- private member functions ----------------------

    void Handwriting::retrieveReferenceFramesInfo()
    {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener listener(tfBuffer);
        cam2tagReceived = false;

        const auto startTime = ros::Time::now();
        while (ros::Time::now() - startTime < REFERENCE_FRAMES_TIMEOUT && !cam2tagReceived) {
            try {
                auto transformMsg = tfBuffer.lookupTransform(tagReferenceFrame, camReferenceFrame, ros::Time(0)).transform;
                tf2::convert(transformMsg, cam2tag);
            }
            catch (tf2::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
                std::cout << "Retring..." << std::endl;
                ros::Duration(1.0).sleep();
                continue;
            }
            cam2tag = cam2tag.inverse();
            cam2tagReceived = true;
            std::cout << "Ok." << std::endl;
        }

        rgbCamInfoSub = nh->subscribe<sensor_msgs::CameraInfo>(rgbCamInfoTopic, MESSAGE_QUEUE_LENGTH,
                                                               &Handwriting::rgbCamInfoCallback, this);
    }

    void Handwriting::subscribeTopics()
    {
        humanListSub .subscribe(*nh, humanListTopic,  MESSAGE_QUEUE_LENGTH);
        depthImageSub.subscribe(*nh, depthImageTopic, MESSAGE_QUEUE_LENGTH);

        // NOTE: this subscription is optional and has the only purpose
        //       of show on screen which frame is being processed
        rgbImageSub.subscribe(*nh, rgbImageTopic,   MESSAGE_QUEUE_LENGTH);

        synchronizer.connectInput(humanListSub, depthImageSub, rgbImageSub);
        synchronizer.registerCallback(boost::bind(&Handwriting::trajectoryExtractionCallback, this, _1, _2, _3));
    }

    void Handwriting::unsubscribeTopics()
    {
        humanListSub .unsubscribe();
        depthImageSub.unsubscribe();
        rgbImageSub  .unsubscribe();
    }

    void Handwriting::trajectoryExtractionCallback(const openpose_ros_msgs::OpenPoseHumanListConstPtr &humanListMsg,
                                                   const sensor_msgs::ImageConstPtr &depthImageMsg,
                                                   const sensor_msgs::ImageConstPtr &rgbImageMsg)
    {
        if (!cam2tagReceived || !rgbCamInfoReceived) {
            ros::Duration(0.5).sleep();
            return;
        }

        auto handInImageFrame = getRightHandCentroid(humanListMsg);

        auto depthImage = cv_bridge::toCvShare(depthImageMsg, depthImageMsg->encoding)->image;
        auto rgbImage   = cv_bridge::toCvCopy (rgbImageMsg,   rgbImageMsg->encoding  )->image;

        if (depthImage.empty() || rgbImage.empty())
            return;

        auto handDepth = getDepth(handInImageFrame, depthImage, 10);

        // from image to camera frame
        tf2::Vector3 handInCameraFrame(handDepth * (handInImageFrame.x - cx)/fx,
                                       handDepth * (handInImageFrame.y - cy)/fy,
                                       handDepth);

        trajectoryCamFrame.push_back(handInCameraFrame);

        if (showProcessedFrames) {
            cv::circle(rgbImage, handInImageFrame, 5, cv::Scalar(255, 0, 0), 2);
            cv::imshow(IMAGE_WINDOW, rgbImage);
            cv::waitKey(33);

            drawTrajectory(trajectoryCamFrame, 33, TRAJECTORY_WINDOW);
        }

        // reset timer
        completeTrajectoryTimer.setPeriod(COMPLETE_TRAJECTORY_TIMEOUT, true);
    }

    void Handwriting::rgbCamInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        const auto cols = 3;

        fx = msg->K[cols*0 + 0];
        fy = msg->K[cols*1 + 1];
        cx = msg->K[cols*0 + 2];
        cy = msg->K[cols*1 + 2];

        rgbCamInfoReceived = true;
        rgbCamInfoSub.shutdown();
    }


    void Handwriting::timerCallback(const ros::TimerEvent& event)
    {
        receivingFrames = false;
    }


    void Handwriting::cleanupTrajectory()
    {
        filters::filterByZ       (trajectoryCamFrame, trajectoryTagFrame, planeDistanceThreshold);
        filters::filterByVelocity(trajectoryCamFrame, trajectoryTagFrame);
        filters::filterByCentroid(trajectoryCamFrame, trajectoryTagFrame);
    }

} // namespace hw