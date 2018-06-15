#include "lwa4p_blue_control/adaptive_pd.h"

AdaptivePd::AdaptivePd()
  : a1_(0.0),
    a2_(0.0),
    b1_(0.0),
    b2_(0.0),
    c1_(0.0),
    c2_(0.0),
    wp_(0.0),
    f0_(0.0),
    //ui_old_(0.0),
    error_prev_1_(0.0),
    error_prev_2_(0.0),
    error_model_1_(0.0),
    error_model_2_(0.0),
    u_max_(std::numeric_limits<double>::infinity()),
    u_min_(-std::numeric_limits<double>::infinity()),
    td_min_(std::numeric_limits<double>::min()),
    td_max_(std::numeric_limits<double>::max()),
    model_zeta_(0.0),
    model_omega_(0.0),
    td_prev_(td_min_),
    q_prev_(0.0),
    f_(0.0),
    kp_(0.0),
    kd_(0.0),
    is_first_pass_(true),
    contact_force_prev_(false)
{

}

AdaptivePd::AdaptivePd(double a1, double a2, double b1, double b2, double c1, double c2, double wp, double xr, double model_zeta, double model_omega)
  : a1_(a1),
    a2_(a2),
    b1_(b1),
    b2_(b2),
    c1_(c1),
    c2_(c2),
    wp_(wp),
    f0_(xr), // position trajectory of free space motion
    //ui_old_(0.0),
    error_prev_1_(0.0),
    error_prev_2_(0.0),
    error_model_1_(0.0),
    error_model_2_(0.0),
    u_max_(std::numeric_limits<double>::infinity()),
    u_min_(-std::numeric_limits<double>::infinity()),
    td_min_(std::numeric_limits<double>::min()),
    td_max_(std::numeric_limits<double>::max()),
    model_zeta_(model_zeta),
    model_omega_(model_omega),
    td_prev_(td_min_),
    q_prev_(0.0),
    f_(xr),
    kp_(0.0),
    kd_(0.0),
    is_first_pass_(true),
    contact_force_prev_(false)
{

}

AdaptivePd::~AdaptivePd()
{

}

double AdaptivePd::geta1(void)
{
  return a1_;
}

double AdaptivePd::geta2(void)
{
  return a2_;
}

double AdaptivePd::getb1(void)
{
  return b1_;
}

double AdaptivePd::getb2(void)
{
  return b2_;
}

double AdaptivePd::getc1(void)
{
  return c1_;
}

double AdaptivePd::getc2(void)
{
  return c2_;
}

double AdaptivePd::getwp(void)
{
  return wp_;
}

double AdaptivePd::getf0(void)
{
  return f0_;
}

double AdaptivePd::getUMax(void)
{
    return u_max_;
}

double AdaptivePd::getUMin(void)
{
  return u_min_;
}

double AdaptivePd::getTdMax(void)
{
    return td_max_;
}

double AdaptivePd::getTdMin(void)
{
  return td_min_;
}

double AdaptivePd::getModelZeta(void)
{
  return model_zeta_;
}

double AdaptivePd::getModelOmega(void)
{
  return model_omega_;
}

void AdaptivePd::seta1(double a1)
{
  a1_ = a1;
}

void AdaptivePd::seta2(double a2)
{
  a2_ = a2;
}

void AdaptivePd::setb1(double b1)
{
  b1_ = b1;
}

void AdaptivePd::setb2(double b2)
{
  b2_ = b2;
}

void AdaptivePd::setc1(double c1)
{
  c1_ = c1;
}

void AdaptivePd::setc2(double c2)
{
  c2_ = c2;
}

void AdaptivePd::setwp(double wp)
{
  wp_ = wp;
}

void AdaptivePd::setf0(double f0)
{
  f0_prev_ = f0_;
  f0_ = f0;
}

void AdaptivePd::setUMax(double u_max)
{
  u_max_ = u_max;
}

void AdaptivePd::setUMin(double u_min)
{
  u_min_ = u_min;
}

void AdaptivePd::setTdMax(double td_max)
{
  td_max_ = td_max;
}

void AdaptivePd::setTdMin(double td_min)
{
  td_min_ = td_min;
}

void AdaptivePd::setModelZeta(double model_zeta)
{
  model_zeta_ = model_zeta;
}

void AdaptivePd::setModelOmega(double model_omega)
{
  model_omega_ = model_omega;
}

double AdaptivePd::modelDynamics(double td)
{

    double error_model;

    model_error_time_ = model_error_time_ + td;
    error_model = ode_c1_ * std::exp(ode_r1_ * model_error_time_) + ode_c2_ * std::exp(ode_r2_ * model_error_time_);
    if (fabs(error_model) < 1e-3) {
        error_model = 0.0;
    }
    return error_model;
}

double AdaptivePd::computeQ(double e, double em, double td){
    double q;
    double de, dem;
    de = (e - error_prev_1_) / td;
    dem = (em - error_model_1_) / td;
    q = wp_ * (e - em) + (de - dem);
    return q;
}

double AdaptivePd::computeF(double q, double td)
{
    double f;
    f = f_ - a1_ * q - a2_ * (q - q_prev_) / td - f0_prev_ + f0_;
    f0_prev_ = f0_;
    return f;
}

double AdaptivePd::computeKp(double q, double e, double td)
{
    double kp;
    double temp1, temp2;
    double pk, pk1;
    temp1 = (b1_ * td + 2 * b2_)/2;
    pk = e * q;
    temp2 = (b1_ * td - 2 * b2_)/2;
    pk1 = error_prev_1_ * q_prev_;
    kp = kp_ + temp1 * pk + temp2 * pk1;
    return kp;
}

double AdaptivePd::computeKd(double q, double e, double td)
{
    double kd;
    double temp1, temp2;
    double pk, pk1;
    temp1 = (c1_ * td + 2 * c2_)/2;
    pk = (e - error_prev_1_) * q / td;
    temp2 = (c1_ * td - 2 * c2_)/2;
    pk1 = (error_prev_1_ - error_prev_2_) * q_prev_ / td;
    kd = kd_ + temp1 * pk + temp2 * pk1;
    return kd;
}

void AdaptivePd::initController()
{
    f_ = 0;
    kp_ = 0;
    kd_ = 0;
    contact_force_prev_ = false;
}

void AdaptivePd::resetController()
{
    contact_force_prev_ = false;
}

double AdaptivePd::compute(double ref, double meas, bool contact_force)
{
    double error, error_model, u, up, ui, ud, td;
    timespec time_current;
    double q, f, kp, kd;

    double e0, de0;
    if (is_first_pass_ == true)
    {
        clock_gettime(CLOCK_REALTIME, &time_old_);
        is_first_pass_ = false;
        error_model_1_ = ref - meas;
        error_model_2_ = 0.0;
        f0_prev_ = f0_;
        f_ = f0_;
        u = f0_;
        counter = 0;
    }
    else
    {
        clock_gettime(CLOCK_REALTIME, &time_current);
        td = (time_current.tv_sec - time_old_.tv_sec) +
            (time_current.tv_nsec - time_old_.tv_nsec) / 1000000000.0;

        // check if time step is too small
        if (td < td_min_)
        {
        // to avoid division by zero, set td to the minimal allowable value
            td = td_min_;
            std::cout << "Warning - time step smaller than " << td_min_ << std::endl;
        }
        else if (td > td_max_)
        {
            // just print warning
            std::cout << "Warning - time step greater than " << td_max_ << std::endl;
        }

        bool flag_start = false;

        error = ref - meas;
        if (contact_force == true) {
            if (contact_force_prev_ == false) {
                e0 = ref - meas; // initial condition - model error = measured error
                de0 = 0.0; // initial condition - model error rate = 0 at t=0
                ode_r1_ = -model_omega_ * (model_zeta_ - std::sqrt(pow(model_zeta_, 2) - 1.0));
                ode_r2_ = -model_omega_ * (model_zeta_ + std::sqrt(pow(model_zeta_, 2) - 1.0));
                ode_c2_ = (de0 - ode_r1_ * e0) / (ode_r2_ - ode_r1_);
                ode_c1_ = e0 - ode_c2_;
                model_error_time_ = 0;
                f_ = f0_;
                td_prev_ = td;
                error_prev_1_ = error;
            }
            error_model = AdaptivePd::modelDynamics(td);
            q = AdaptivePd::computeQ(error, error_model, td);
            flag_start = true;
        }
        else {
            error_model = error;
            q = 0.0;
            q_prev_ = 0.0;
        }

        f = AdaptivePd::computeF(q, td);
        kp = AdaptivePd::computeKp(q, error, td);
        kd = AdaptivePd::computeKd(q, error, td);

        // proportional term
        up = kp * error;
        // derivative term
        ud = kd * (error - error_prev_1_) / td;
        // total = p + i + d
        u = f + up + ud;

        // saturation and anti-wind up
        if (u > u_max_)
            u = u_max_;
        else if (u < u_min_)
            u = u_min_;

        counter;
        if ((f0_ == 101.0) & (counter < 5000)) {
            std::cout << "q = " << q << "; f = " << f << "; kp = " << kp << "; kd = " << kd << "; u " << u << std::endl;
            std::cout << "ref = " << ref  << " meas = " << meas << std::endl;
            std::cout << "e = " << error << "; de = " << (error - error_prev_1_) / td << std::endl;
            std::cout << "em = " << error_model << "; dem = " << (error_model - error_model_1_) / td << std::endl;
            std::cout << "up = " << up << "; ud = " << ud << std::endl;
        }

        error_prev_2_ = error_prev_1_;
        error_prev_1_ = error;
        time_old_ = time_current;
        //error_model_2_ = error_model_1_;
        error_model_1_ = error_model;
        td_prev_ = td;
        q_prev_ = q;
        f_ = f;
        kp_ = kp;
        kd_ = kd;
        contact_force_prev_ = contact_force;
    }
    if (contact_force)
        return u;
    else
        return f0_;

}
