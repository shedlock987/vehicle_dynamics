/**
 * MIT License
 *
 * Copyright (c) 2020 Ryan Shedlock
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file veh_dyn_observer.h
 * @brief Base class Vehicle Dynamics Observer
 * @author Ryan Shedlock <rmshedlock@gmail.com>
 * @version 1.0
 */
#ifndef VEH_DYN_OBSERVER_H_
#define VEH_DYN_OBSERVER_H_

/// System
#include <stddef.h>
#include <string.h>
#include <vector>
#include <cassert>
#include <cmath>

/// Library
#include <Eigen/Dense>

namespace vehicle_dynamics{
class Veh_Dyn_Observer{
    public:

    /**
     * Typedef for vector type
     */
    typedef Eigen::VectorXd vec_t;
    /**
     * Typedef for matrix type
     */
    typedef Eigen::MatrixXd mat_t;

    /**
     * @brief      Constructs a new instance.
     *
     * @param[in]  _csf_slope   Front Lumped Cornering Stiffness per Slip Angle (N/rad)
     * @param[in]  _csr_slope   Rear Lumped Cornering Stiffness per Slip Angle (N/rad)
     * @param[in]  _csf_init    Front Lumped Cornering Stiffness Inital Condition (N/rad)
     * @param[in]  _csr_init    Rear Lumped Cornering Stiffness Inital Condition (N/rad)
     * @param[in]  _a           x Distance between the 2D CG and the Front Axle
     * @param[in]  _b           x Distance between the 2D CG and the Rear Axle
     * @param[in]  _mass        Mass of the Vehicle
     * @param[in]  _inertia     Z-axis Moment of Inertia About the CG
     * @param[in]  _fmax        Tire Force Saturaion with Surface Mu of 1.0 (N)
     * @param[in]  _dt          Execution Time (loop period)
     */
    Veh_Dyn_Observer(const double& _csr_slope,
                     const double& _csf_slope,
                     const double& _csr_init,
                     const double& _csf_init,
                     const double& _a,
                     const double& _b,
                     const double& _mass,
                     const double& _inertia,
                     const double _dt);

    /** 
     * @brief      Destroys the object.
     */
     ~Veh_Dyn_Observer();

     /** 
     * @brief      Initializes the Observer
     */
     void Init();

    /** 
     * @brief      Runs the Observer
     * 
     * @param[in]  _Vx          Vehicle Longitudinal Velocity (m/s)
     * @param[in]  _rwa         Road Wheel Steer Angle (rad)
     * @param[in]  _meas_Omega  Measured Yaw Rate (rad/s)
     * @param[in]  _meas_Vy_dot Measured Lat Acceleration (m/s^2)
     * @param[in]  _mu          Estimate of Surface Friction
     * 
     * @return     The State
     */
     virtual vec_t Step(double _Vx, 
          double _rwa,
          double _meas_Omega,
          double _meas_Vy_Dot,
          double _mu);

    /**
     * @brief      Predicts Vy_dot and Omega_dot
     *
     * @return     The state derivative
     */
     void Prediction();

    /**
     * @brief      Corrects and Updates the States (Vy and Omega) 
     * 
     * @param[in]  _Vx          Vehicle Longitudinal Velocity (m/s)
     * 
     * @return     The State   
     */
     void Correction();

     private:
     vec_t x_; 
     vec_t x_dot_;
     mat_t A_; 
     vec_t B_;
     double meas_Omega_;
     double meas_Vy_dot_;
     double a_;
     double b_;
     double csr_slope_;
     double csf_slope_;
     double csr_init_;
     double csf_init_;
     double fmax_;
     double mu_;
     double alpha_front_;
     double alpha_rear_;
     double rwa_;
     double Vx_;
     double dt_;
     double mass_;
     double inertia_;
     bool is_initialized_;
     int exec_cnt_;

     /// Unit Delay (1/z) Storage 
     vec_t unit_delay_x_;

};
} // namespace vehicle_dyamics

#endif /* VEH_DYN_OBSERVER_H_ */