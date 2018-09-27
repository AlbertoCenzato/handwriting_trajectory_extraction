//
// Created by alberto on 29/06/18.
//
#include <vector>
#include <cmath>

#include "trajectory_filters.hpp"

namespace hw {
    namespace filters {

        float distance(const tf2::Vector3 &point1, const tf2::Vector3 &point2)
        {
            return std::sqrt(std::pow(point1.x() - point2.x(), 2) +
                             std::pow(point1.y() - point2.y(), 2) +
                             std::pow(point1.z() - point2.z(), 2));
        }

        void filterByZ(std::vector<tf2::Vector3> &trajectoryCamFrame,
                       std::vector<tf2::Vector3> &trajectoryTagFrame, float threshold)
        {
            if (trajectoryCamFrame.empty() || trajectoryTagFrame.empty())
                return;

            assert(trajectoryCamFrame.size() == trajectoryTagFrame.size() &&
                   "Trajectory in camera reference frame and trajectory in tag reference frame\n"
                   "should have the same number of points!");

            std::vector<tf2::Vector3> filteredCamFrame, filteredTagFrame;
            for (auto i = 0; i < trajectoryTagFrame.size(); ++i) {
                const auto &point = trajectoryTagFrame[i];
                if (std::abs(point.z()) < threshold) {
                    filteredTagFrame.push_back(point);
                    filteredCamFrame.push_back(trajectoryCamFrame[i]);
                }
            }

            trajectoryCamFrame = std::move(filteredCamFrame);
            trajectoryTagFrame = std::move(filteredTagFrame);
        }


        void filterByVelocity(std::vector<tf2::Vector3> &trajectoryCamFrame,
                              std::vector<tf2::Vector3> &trajectoryTagFrame)
        {
            if (trajectoryCamFrame.empty() || trajectoryTagFrame.empty())
                return;

            assert(trajectoryCamFrame.size() == trajectoryTagFrame.size() &&
                   "Trajectory in camera reference frame and trajectory in tag reference frame\n"
                   "should have the same number of points!");

            std::vector <tf2::Vector3> filteredCamFrame, filteredTagFrame;
            float meanVelocity = 0;
            auto prevPoint = trajectoryTagFrame[0];
            for (const auto point : trajectoryTagFrame) {
                meanVelocity += distance(point, prevPoint);
                prevPoint = point;
            }

            meanVelocity /= trajectoryTagFrame.size();
            prevPoint = trajectoryTagFrame[0];
            for (auto i = 0; i < trajectoryTagFrame.size(); ++i) {
                const auto &point = trajectoryTagFrame[i];
                if (distance(point, prevPoint) < 2 * meanVelocity) {
                    filteredTagFrame.push_back(point);
                    filteredCamFrame.push_back(trajectoryCamFrame[i]);
                }
                prevPoint = point;
            }

            trajectoryCamFrame = std::move(filteredCamFrame);
            trajectoryTagFrame = std::move(filteredTagFrame);
        }


        void filterByCentroid(std::vector<tf2::Vector3> &trajectoryCamFrame,
                              std::vector<tf2::Vector3> &trajectoryTagFrame)
        {
            if (trajectoryCamFrame.empty() || trajectoryTagFrame.empty())
                return;

            assert(trajectoryCamFrame.size() == trajectoryTagFrame.size() &&
                   "Trajectory in camera reference frame and trajectory in tag reference frame\n"
                   "should have the same number of points!");

            tf2::Vector3 centroid(0, 0, 0);
            for (const auto point : trajectoryTagFrame) {
                centroid += point;
            }
            centroid /= trajectoryTagFrame.size();

            float meanDistance = 0;
            for (const auto point : trajectoryTagFrame) {
                meanDistance += distance(centroid, point);
            }
            meanDistance /= trajectoryTagFrame.size();

            std::vector <tf2::Vector3> filteredCamFrame, filteredTagFrame;
            for (auto i = 0; i < trajectoryTagFrame.size(); ++i) {
                const auto &point = trajectoryTagFrame[i];
                if (distance(centroid, point) < 2 * meanDistance) {
                    filteredTagFrame.push_back(point);
                    filteredCamFrame.push_back(trajectoryCamFrame[i]);
                }
            }

            trajectoryCamFrame = std::move(filteredCamFrame);
            trajectoryTagFrame = std::move(filteredTagFrame);
        }

    } //namespace filters
} // namespace hw