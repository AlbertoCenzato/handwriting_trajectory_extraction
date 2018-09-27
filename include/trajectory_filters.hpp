//
// Created by alberto on 29/06/18.
//

#ifndef HW_FILTERS_TRAJECTORY_FILTERS_HPP
#define HW_FILTERS_TRAJECTORY_FILTERS_HPP

#include <tf2/LinearMath/Vector3.h>

namespace hw {
    namespace filters {

        void filterByZ(std::vector<tf2::Vector3> &trajectoryCamFrame,
                       std::vector<tf2::Vector3> &trajectoryTagFrame, float threshold);

        void filterByVelocity(std::vector<tf2::Vector3> &trajectoryCamFrame,
                              std::vector<tf2::Vector3> &trajectoryTagFrame);

        void filterByCentroid(std::vector<tf2::Vector3> &trajectoryCamFrame,
                              std::vector<tf2::Vector3> &trajectoryTagFrame);

    }
}

#endif //HW_FILTERS_TRAJECTORY_FILTERS_HPP
