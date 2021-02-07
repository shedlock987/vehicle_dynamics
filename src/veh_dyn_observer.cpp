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
 * @file veh_dyn_observer.c
 * @brief Base Class Vehicle Dynamics Observer
 * @author Ryan Shedlock <rmshedlock@gmail.com>
 * @version 1.0
 */


#include "veh_dyn_observer/veh_dyn_observer.h"

namespace vehicle_dynamics {
Veh_Dyn_Observer::Veh_Dyn_Observer(const double& _csr_slope,
                                   const double& _csf_slope,
                                   const double& _csr_init,
                                   const double& _csf_init,
                                   const double& _a,
                                   const double& _b,
                                   const double& _mass,
                                   const double& _inertia,
                                   const double _dt) {
        csr_slope_ = _csr_slope;
        csf_slope_ = _csf_slope;
        csr_init_ = _csr_init;
        csf_init_ = _csf_init;
        a_ = _a;
        b_ = _b;
        mass_ = _mass;
        inertia_ = _inertia;
        dt_ = _dt;
        is_initialized_ = false;
        exec_cnt_ = 0;

        unit_delay_x_ << 0, 0;
    }

    bool Veh_Dyn_Observer::Init() {
        exec_cnt_++;

        /// Calculate Initial State Matrix
        A_(0,0) = -1.0 * (csf_init_ + csr_init_) / (mass_ * Vx_);
        A_(0,1) = (-1.0 * (a_ * csf_init_ - b_ * csr_init_) / (mass_ * Vx_)) - Vx_;
        A_(1,0) = (a_ * csf_init_ - b_ * csr_init_)/(inertia_ * Vx_);
        A_(1,1) = -1.0 * (pow(a_, 2) * csf_init_ - pow(b_, 2) * csr_init_) / (mass_ * Vx_);

        /// Calculate Input Vector
        B_(0) = csf_init_ / mass_;
        B_(1) = csf_init_ * a_ / inertia_;

        /// Compute Initial States x(t)
        x_(0) =  (meas_Vy_dot_ - A_(0,1) * meas_Omega_ - B_(0) * rwa_) / A_(0,0);
        x_(1) = meas_Omega_;

        /// Compute Initial Output x(t)_dot
        x_dot_(0) = meas_Vy_dot_;
        x_dot_(1) = A_(1,0) * meas_Omega_ + A_(1,1) * x_(0) +B_(1) * rwa_;

        /// Update Unit Delay
        unit_delay_x_ = x_;

        /// Calculate Slip Angles
        alpha_front_ = cos(rwa_) - atan((x_(0)*a_ + x_(1) ) / Vx_);
        alpha_rear_ = -1*atan((x_(0)*b_ + x_(1) ) / Vx_);

        /// Housekeeping
        if(exec_cnt_ <= 1) {
            unit_delay_x_ = x_;
        }
        else
        {
            is_initialized_ = true;
        }

        return is_initialized_;
    }

    void Veh_Dyn_Observer::Prediction() {

        if(is_initialized_) {
        
        /// Adjust parameters based on surface friction
        auto adjusted_fmax = mu_ * fmax_;
        auto adjusted_alpha_f = abs(alpha_front_ / mu_);
        auto adjusted_alpha_r = abs(alpha_rear_ / mu_);
        auto alpha_f_sign = -1 * std::signbit(alpha_front_);
        auto alpha_r_sign = -1 * std::signbit(alpha_front_);
        
        /// Here we use the hyperbolic tangent 
        /// function as an approximation/regression
        /// of cornering stiffness (aka the lat tire 
        /// force to slip angle ratio)

        /// Compute Rear Lateral Tire Force
        auto fyr = alpha_r_sign * (adjusted_fmax * tanh(adjusted_alpha_r / csr_slope_));

        /// Compute Front Lateral Tire Force
        auto fyf = alpha_f_sign * (adjusted_fmax * tanh(adjusted_alpha_f / csf_slope_ ));

        /// M(Vy_dot + Omega*Vx) = Fyr + Fyf*cos(rwa)
        /// The sum of the lateral force on the body
        /// is equal to the sum of the lateral force
        /// exerted by the tires
        auto vy_dot = ((fyr + fyf*cos(rwa_)) / mass_) - (Vx_ * x_(0));

        /// Iz * Omega_dot = b*Fyr + a*Fyf*cos(rwa)
        /// The moment about the body is equal to the 
        /// sum of the moments induced by the tires
        auto omega_dot = ((fyr*b_ + fyf*cos(rwa_)*a_) / inertia_);

        x_dot_ << vy_dot, omega_dot;

        }
    }

void Veh_Dyn_Observer::Correction() {

    if(is_initialized_) {

        /// Integrate the output to get the states
        vec_t x_tmp = unit_delay_x_ + (x_ / dt_);
        unit_delay_x_ = x_tmp;

        /// Correct the Yaw Rate with Measurement
        x_tmp(0) += 0.5 * (x_tmp(0) - meas_Omega_);
        x_ = x_tmp;

        /// Calculate Slip Angles
        alpha_rear_ = -1*atan((x_(0)*b_ + x_(1) ) / Vx_);
        alpha_front_ = cos(rwa_) - atan((x_(0)*a_ + x_(1) ) / Vx_);
    }

}
    
}