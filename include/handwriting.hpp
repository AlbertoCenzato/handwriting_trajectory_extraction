//
// Created by alberto on 04/04/18.
//

#ifndef HW_HANDWRITING_HPP
#define HW_HANDWRITING_HPP

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Transform.h>

#include <openpose_ros_msgs/OpenPoseHumanList.h>


namespace hw {

    class Handwriting {

    public:

        static const int MESSAGE_QUEUE_LENGTH = 5;
        static const ros::Duration REFERENCE_FRAMES_TIMEOUT;    // max waiting time for camera info and tf
        static const ros::Duration COMPLETE_TRAJECTORY_TIMEOUT; // waiting time after openpose messages end
        static const std::string TRAJECTORY_WINDOW;
        static const std::string IMAGE_WINDOW;


        /**
         * @brief Class constructor.
         * @param nh: pointer to ROS NodeHandle
         * @param showProcessedFrames: set this to true to visualize the video while it is processed
         */
        Handwriting(ros::NodeHandlePtr nh, bool showProcessedFrames=false);

        ~Handwriting();

        /**
         * Subscribes to openpose and apriltag topics, retrieves the camera infos,
         * collects right hand trajectory points and returns when openpose messages stop.
         * @param waypointCamFrame: output hand trajectory in camera frame
         * @param waypointTagFrame: output hand trakectory in apriltag frame
         */
        void getTrajectories(geometry_msgs::PoseArray &waypointCamFrame, geometry_msgs::PoseArray &waypointTagFrame);

    private:

        using TimeSynchronizerT = message_filters::TimeSynchronizer<openpose_ros_msgs::OpenPoseHumanList,
                sensor_msgs::Image, sensor_msgs::Image>;

        ros::NodeHandlePtr nh;

        bool receivingFrames;

        message_filters::Subscriber<openpose_ros_msgs::OpenPoseHumanList> humanListSub;
        message_filters::Subscriber<sensor_msgs::Image> depthImageSub;
        message_filters::Subscriber<sensor_msgs::Image> rgbImageSub;
        TimeSynchronizerT synchronizer;

        ros::Subscriber rgbCamInfoSub;

        tf2::Transform cam2tag;
        bool cam2tagReceived    = false;
        bool rgbCamInfoReceived = false;

        std::vector<tf2::Vector3> trajectoryCamFrame, trajectoryTagFrame;

        ros::Timer completeTrajectoryTimer;

        std::string tagReferenceFrame, camReferenceFrame, rgbCamInfoTopic;
        std::string depthImageTopic, rgbImageTopic, humanListTopic, tagPoseTopic;

        double fx, fy, cx, cy; // rgb camera intrinsics

        float planeDistanceThreshold = 650;

        bool showProcessedFrames;


        /**
         * If apriltag tf and rgb camera info are available retrieves the right hand position
         * from humanListMsg and depthImageMsg and saves it in trajectoryCamFrame and trajectoryTagFrame
         * @param humanListMsg
         * @param depthImageMsg
         * @param rgbImageMsg
         */
        void trajectoryExtractionCallback(const openpose_ros_msgs::OpenPoseHumanListConstPtr &humanListMsg,
                                          const sensor_msgs::ImageConstPtr &depthImageMsg,
                                          const sensor_msgs::ImageConstPtr &rgbImageMsg);


        /**
         * Subscribes to depthImageTopic, rgbImageTopic, humanListTopic
         */
        void subscribeTopics();

        /**
         * Unsubscribes from to depthImageTopic, rgbImageTopic, humanListTopic
         */
        void unsubscribeTopics();

        /**
         * Retrieves cam2tag transform and subscribes to rgbCamInfoTopic
         */
        void retrieveReferenceFramesInfo();

        /**
         * Retrieves fx, fy, cx, cy
         * @param msg
         */
        void rgbCamInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);


        /**
         * Called when no frame processed by openpose is received for more than COMPLETE_TRAJECTORY_TIMEOUT seconds.
         * Sets receivingFrames to false.
         * @param event
         */
        void timerCallback(const ros::TimerEvent& event);


        /**
         * Removes outliers from the complete trajectory
         */
        void cleanupTrajectory();

    };

} // namespace hw
#endif //HW_HANDWRITING_HPP
