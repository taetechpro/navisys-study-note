/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * Selected helpers ported from ov_init/src/utils/helper.h. Only the
 * gram_schmidt routine is taken — the rest of ov_init (dynamic init,
 * Ceres-based MLE, sim) stays out of this repo.
 */

#ifndef OV_MSCKF_INITIALIZER_HELPER_H
#define OV_MSCKF_INITIALIZER_HELPER_H

#include <Eigen/Core>
#include <cmath>

namespace ov_init {

class InitializerHelper {
public:
    /**
     * @brief Build R_GtoI such that body z-axis aligns with the body-frame
     *        gravity vector (in OpenVINS' Global convention the gravity is
     *        the +z direction, so R_GtoI's 3rd column equals the supplied
     *        body-frame "up" direction = normalized mean accelerometer
     *        reading at rest).
     *
     * Source: ov_init/src/utils/helper.h gram_schmidt (verbatim).
     */
    static void gram_schmidt(const Eigen::Vector3d &gravity_inI,
                             Eigen::Matrix3d &R_GtoI) {
        Eigen::Vector3d z_axis = gravity_inI / gravity_inI.norm();
        Eigen::Vector3d x_axis, y_axis;
        Eigen::Vector3d e_1(1.0, 0.0, 0.0);
        Eigen::Vector3d e_2(0.0, 1.0, 0.0);
        double inner1 = e_1.dot(z_axis) / z_axis.norm();
        double inner2 = e_2.dot(z_axis) / z_axis.norm();
        if (std::fabs(inner1) < std::fabs(inner2)) {
            x_axis = z_axis.cross(e_1);
            x_axis = x_axis / x_axis.norm();
            y_axis = z_axis.cross(x_axis);
            y_axis = y_axis / y_axis.norm();
        } else {
            x_axis = z_axis.cross(e_2);
            x_axis = x_axis / x_axis.norm();
            y_axis = z_axis.cross(x_axis);
            y_axis = y_axis / y_axis.norm();
        }
        R_GtoI.block(0, 0, 3, 1) = x_axis;
        R_GtoI.block(0, 1, 3, 1) = y_axis;
        R_GtoI.block(0, 2, 3, 1) = z_axis;
    }
};

} // namespace ov_init

#endif // OV_MSCKF_INITIALIZER_HELPER_H
