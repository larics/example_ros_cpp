/// \file  pid_controller_base.h
///   \brief Contains the definition of the PidControllerBase class.
///

#ifndef PID_CONTROLLER_BASE_H
#define PID_CONTROLLER_BASE_H

#include <limits>
#include <iostream>

/// A class that implements PID control algorithm.
///
/// A class that implements a Proportional-Integral-Derivative (PID) algorithm
/// used to control dynamical systems. Algorithm is computed in its discrete form:
/// u = kp * e + ui_old + ki * e * td + kd * (e - e_old) / td, where
/// u - output of the algorithm
/// kp - proportional gain
/// ki - integral gain
/// kd - derivative gain
/// ui_old - integral value from the previous step
/// e - error of the measured value w.r.t. the reference
/// e_old - error of the measured value from the previous step
/// td - time elapsed from the previous step (in seconds)
///
class PidControllerBase
{
public:

    /// PID controller default constructor.
    ///
    /// Initializes PID gains to zero.
    ///
    PidControllerBase();

    /// PID controller contructor
    ///
    /// \param kp Proportional gain.
    /// \param ki Integral gain.
    /// \param kd Derivative gain.
    ///
    /// Initializes PID gain to the given values.
    ///
    PidControllerBase(double kp, double ki, double kd);

    /// PID controller destructor.
    ///
    /// Does nothing.
    ///
    ~PidControllerBase();

    /// Returns the current proportional gain.
    ///
    double getKp(void);

    /// Returns the current integral gain.
    ///
    double getKi(void);

    /// Returns the current derivative gain.
    ///
    double getKd(void);

    /// Returns the current maximal control value (upper saturation limit).
    ///
    double getUMax(void);

    /// Returns the current minimal control value (lower saturation limit).
    ///
    double getUMin(void);

    /// Sets proportional gain
    /// \param kp The desired value of the proportional gain
    ///
    void setKp(double kp);

    /// Sets integral gain
    /// \param ki The desired value of the integral gain
    ///
    void setKi(double ki);

    /// Sets integral gain
    /// \param kd The desired value of the derivative gain
    ///
    void setKd(double kd);

    /// Sets maximal control value (upper saturation limit).
    /// \param u_max The desired maximal control value.
    ///
    void setUMax(double u_max);

    /// Sets minimal control value (lower saturation limit).
    /// \param u_min The desired minimal control value.
    ///
    void setUMin(double u_min);

    /// Computes PID algorithm.
    /// \param ref Current referent value.
    /// \param meas Current measured value.
    /// \return Output of the PID algorithm (control value).
    ///
    double compute(double ref, double meas);

protected:

    // Proportional gain.
    double kp_;

    // Integral gain.
    double ki_;

    // Derivative gain.
    double kd_;

    // The value of the integral term from the previous step.
    double ui_old_;

    // Error value from the previous step.
    double error_old_;

    // maximal control value (upper saturation limit)
    double u_max_;

    // minimal control value (lower saturation limit)
    double u_min_;

};

#endif // PID_CONTROLLER_BASE_H
