#include "PidController/pid_controller_base.h"

PidControllerBase::PidControllerBase()
              : kp_(0.0),
                ki_(0.0),
                kd_(0.0),
                ui_old_(0.0),
                error_old_(0.0),
                u_max_(std::numeric_limits<double>::infinity()),
                u_min_(-std::numeric_limits<double>::infinity())
{
    //std::cout << "PidControllerBase default constructor" << std::endl;

}

PidControllerBase::PidControllerBase(double kp, double ki, double kd)
              : kp_(kp),
                ki_(ki),
                kd_(kd),
                ui_old_(0.0),
                error_old_(0.0),
                u_max_(std::numeric_limits<double>::infinity()),
                u_min_(-std::numeric_limits<double>::infinity())
{
    /*
    std::cout << "PidControllerBase constructor, pid params kp, ki, kd = "
             << kp_ << ", "
             << ki_ << ", "
             << kd_ << std::endl;
    */
}

PidControllerBase::~PidControllerBase()
{

}

double PidControllerBase::getKp(void)
{
  return kp_;
}

double PidControllerBase::getKi(void)
{
  return ki_;
}

double PidControllerBase::getKd(void)
{
  return kd_;
}

double PidControllerBase::getUMax(void)
{
    return u_max_;
}

double PidControllerBase::getUMin(void)
{
  return u_min_;
}

void PidControllerBase::setKp(double kp)
{
  kp_ = kp;
}

void PidControllerBase::setKi(double ki)
{
  ki_ = ki;
}

void PidControllerBase::setKd(double kd)
{
  kd_ = kd;
}

void PidControllerBase::setUMax(double u_max)
{
  u_max_ = u_max;
}

void PidControllerBase::setUMin(double u_min)
{
  u_min_ = u_min;
}

double PidControllerBase::compute(double ref, double meas)
{
  double error, u, up, ui, ud;

  // compute algorithm
  // TODO Compute elapsed time since the last algorithm execution

  error = ref - meas;
  //std::cout << "Error " << error << std::endl;

  // proportional term
  up = kp_ * error;

  // integral term
  ui = ui_old_  + ki_ * error;

  // derivative term
  ud = kd_ * (error - error_old_);

  // total = p + i + d
  u = up + ui + ud;


  // saturation and anti-wind up
  if (u > u_max_)
      u = u_max_;
  else if (u < u_min_)
      u = u_min_;
  else
  {
      // Integral value is stored for the next step
      // only if control value is not saturated(anti-wind up)
      ui_old_ = ui;
  }


  error_old_ = error;
  return u;

}
