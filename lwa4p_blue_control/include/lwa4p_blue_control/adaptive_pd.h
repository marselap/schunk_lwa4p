/// \file  adaptive_pd.h
///   \brief Contains the definition of the AdaptivePd class.
///

#ifndef ADAPTIVE_PD_H
#define ADAPTIVE_PD_H

#include <limits>
#include <iostream>
#include <time.h>
#include <cmath>

/// \brief A class that implements adaptive control algorithm.
///
/// A class that implements an adaptive PID algorithm
/// used to control dynamical systems. Algorithm is computed in its discrete form:
/// xr(k) = f(k) + kp(k) * e(k) + kd(k) * (e(k) - e(k-1)) / td, with:
/// xr - output of the algorithm
/// f - adaptive auxiliary signal for steady-state error compensation
/// kp - adaptive proportional gain
/// kd - adaptive derivative gain
/// e - error of the measured value w.r.t. the reference
/// td - time elapsed from the previous step (in seconds)
///
class AdaptivePd
{
public:

    /// \brief controller default constructor.
    ///
    /// Initializes gains to zero.
    ///
    AdaptivePd();

    /// controller contructor
    ///
    /// \param a1 Integral gain for auxiliary signal.
    /// \param a2 Proportional gain for auxiliary signal.
    /// \param b1 Integral gain for adaptive proportional gain.
    /// \param b2 Proportional gain for adaptive proportional gain.
    /// \param c1 Integral gain for adaptive derivative gain.
    /// \param c2 Proportional gain for adaptive derivative gain.
    /// \param wp Proportional error weighting factor.
    /// \param xr Initial value for position reference.
    /// \param model_zeta Second-order dynamical error model parameter.
    /// \param model_omega Second-order dynamical error model parameter.
    ///
    /// Initializes PID gains to the given values.
    ///
    AdaptivePd(double a1, double a2, double b1, double b2, double c1, double c2, double wp, double xr, double model_zeta, double model_omega);

    /// \brief PID controller destructor.
    ///
    /// Does nothing.
    ///
    ~AdaptivePd();

    /// Returns the current integral gain for auxiliary signal.
    ///
    double geta1(void);

    /// Returns the current proportional gain for auxiliary signal.
    ///
    double geta2(void);

    /// Returns the current integral gain for adaptive proportional gain.
    ///
    double getb1(void);

    /// Returns the current proportional gain for adaptive proportional gain.
    ///
    double getb2(void);

    /// Returns the current integral gain for adaptive derivative gain.
    ///
    double getc1(void);

    /// Returns the current proportional gain for adaptive proportional gain.
    ///
    double getc2(void);

    /// Returns the current proportional error weighting factor.
    ///
    double getwp(void);

    /// Returns the initial position reference.
    ///
    double getf0(void);

    /// Returns the current maximal control value (upper saturation limit).
    ///
    double getUMax(void);

    /// Returns the current minimal control value (lower saturation limit).
    ///
    double getUMin(void);

    /// Returns the current maximal allowable time step.
    ///
    double getTdMax(void);

    /// Returns the current minimal allowable time step.
    ///
    double getTdMin(void);

    /// Returns the error dynamics model parameter zeta.
    ///
    double getModelZeta(void);

    /// Returns the error dynamics model parameter omega.
    ///
    double getModelOmega(void);

    /// Sets integral gain for auxiliary signal
    /// \param a1 The desired value of the integral gain
    ///
    void seta1(double a1);

    /// Sets proportional gain for auxiliary signal
    /// \param a2 The desired value of the proportional gain
    ///
    void seta2(double a2);

    /// Sets integral gain for adaptive proportional gain
    /// \param b1 The desired value of the integral gain
    ///
    void setb1(double b1);

    /// Sets proportional gain for adaptive proportional gain
    /// \param b2 The desired value of the proportional gain
    ///
    void setb2(double b2);

    /// Sets integral gain for adaptive derivative gain
    /// \param c1 The desired value of the integral gain
    ///
    void setc1(double c1);

    /// Sets proportional gain for adaptive derivative gain
    /// \param c2 The desired value of the proportional gain
    ///
    void setc2(double c2);

    /// Sets proportional error weighting factor
    /// \param wp The desired value of the proportional gain
    ///
    void setwp(double wp);

    /// Sets initial position reference
    /// \param f0 initial position reference
    ///
    void setf0(double f0);

    /// Sets maximal control value (upper saturation limit).
    /// \param u_max The desired maximal control value.
    ///
    void setUMax(double u_max);

    /// Sets minimal control value (lower saturation limit).
    /// \param u_min The desired minimal control value.
    ///
    void setUMin(double u_min);

    /// Sets maximal allowable time step.
    /// \param td_max The desired maximal time step.
    ///
    void setTdMax(double td_max);

    /// Sets minimal allowable time step.
    /// \param td_min The desired minimal time step.
    ///
    void setTdMin(double td_min);

    /// Sets the error dynamics model parameter zeta.
    /// \param model_zeta damping factor of error model
    ///
    void setModelZeta(double model_zeta);

    /// Sets the error dynamics model parameter omega.
    /// \param model_omega frequency for error model
    ///
    void setModelOmega(double model_omega);

    /// Computes one step of error dynamics based on given second order dynamics
    /// /param td Step time
    /// \return Model value of error
    double modelDynamics(double td);

    /// Computes model error deviation for adaptive parameters' tuning
    /// \param e Reference error.
    /// \param em Model error.
    /// \param td Current time step.
    /// \return Weighted measure of error (w_p * (e-em) + (e'-em')).
    ///
    double computeQ(double e, double em, double td);

    /// Computes adaptive auxiliary signal f
    /// \param q Deviation of error from model error.
    /// \param td Current time step.
    /// \return Adaptive auxiliary signal.
    ///
    double computeF(double q, double td);

    /// Computes adaptive proportional gain kp
    /// \param q Deviation of error from model error.
    /// \param e Reference error
    /// \param td Current time step.
    /// \return Adaptive proportional gain.
    ///
    double computeKp(double q, double e, double td);

    /// Computes adaptive derivative gain kp
    /// \param q Deviation of error from model error.
    /// \param e Reference error
    /// \param td Current time step.
    /// \return Adaptive derivative gain.
    ///
    double computeKd(double q, double e, double td);

    /// Computes adaptive PD algorithm.
    /// \param ref Current referent value.
    /// \param meas Current measured value.
    /// \return Output of the PD algorithm (control value).
    ///
    double compute(double ref, double meas, bool contact_force);

    void initController();
    void resetController();


protected:

    /// Integral gain for auxiliary signal.
    double a1_;

    /// Proportional gain for auxiliary signal.
    double a2_;

    /// Integral gain for adaptive proportional gain.
    double b1_;

    /// Proportional gain for adaptive proportional gain.
    double b2_;

    /// Integral gain for adaptive derivative gain.
    double c1_;

    /// Proportional gain for adaptive derivative gain.
    double c2_;

    /// Proportional error weighting factor.
    double wp_;

    /// Initial value for position reference.
    double f0_;

    double f0_prev_;

    /// Reference error value from t = (k-1)td.
    double error_prev_1_;

    /// Reference error value from t = (k-2)td.
    double error_prev_2_;

    /// Model error value from from t = (k-1)td.
    double error_model_1_;

    /// Model error value from from t = (k-2)td.
    double error_model_2_;

    /// Model error ODE solution constant from initial conditions
    double ode_c1_;

    /// Model error ODE solution constant from initial conditions
    double ode_c2_;

    /// model error ODE time constant
    double ode_r1_;

    /// model error ODE time constant
    double ode_r2_;

    /// model error ODE numeric time
    double model_error_time_;

    /// Maximal control value (upper saturation limit)
    double u_max_;

    /// Minimal control value (lower saturation limit)
    double u_min_;

    /// Maximal allowable time step
    double td_max_;

    /// Minimal allowable time step
    double td_min_;

    /// Second-order dynamical error model parameter.
    double model_zeta_;

    /// Second-order dynamical error model parameter.
    double model_omega_;

    /// Previous time step in case discretization time is not uniform
    double td_prev_;

    /// Deviation of reference error from model error in previous step
    double q_prev_;

    /// Adaptive auxiliary signal value from previous step
    double f_;

    /// Adaptive proportional gain value from previous step
    double kp_;

    /// Adaptive derivative gain value from previous step
    double kd_;

    // flag indicates if the algorithm is executed for the first time
    bool is_first_pass_;

    /// Flag if contact force was achieved in previous step
    // Rising edge triggers error model restart
    // Maybe f, kp, kd reset at rising edge??
    // Flag resets at new reference
    bool contact_force_prev_;

    int counter;

    // time of the previous step
    timespec time_old_;

};

#endif // ADAPTIVE_PD_H
