#ifndef DMP_ORIENTATION_H
#define DMP_ORIENTATION_H

#include <dmp_lib/DMP/WSoG.h>
#include <dmp_lib/CanonicalClock/CanonicalClock.h>
#include <dmp_lib/GatingFunction/GatingFunction.h>

namespace as64_
{

class DMP_orient
{

public:

  /** DMP_orient constructor.
   *  @param[in] N_kernels: 3x3 matrix where each row contains the kernels for position,
                            velocity and acceleration and each row for x, y and z coordinate.
                            If 'N_kernels' is a column vector then every coordinate will have the
                            same number of kernels.
   *  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
   *  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
   *  @param[in] can_clock_ptr: Pointer to a DMP canonical system object.
   *  @param[in] shape_attr_gating_ptr: Pointer to a DMP gating function object.
   */
  DMP_orient(const arma::Mat<int> &N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> &can_clock_ptr,
    std::shared_ptr<GatingFunction> &shape_attr_gating_ptr);

  // DMP_orient(const arma::mat &N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> &can_clock_ptr);
  // DMP_orient(const arma::mat &N_kernels, double a_z, double b_z);

  /** Trains the DMP_orient.
   *  @param[in] train_method: The training method (see dmp_::TRAIN_METHOD enum).
   *  @param[in] Time: 1xN row vector with the timestamps of the training data points.
   *  @param[in] Qd_data: 4xN matrix with desired orientation as unit quaternion at each column for each timestep.
   *  @param[in] vRotd_data: 3xN matrix with desired angular velocity at each column for each timestep.
   *  @param[in] dvRotd_data: 3xN matrix with desired angular acceleration at each column for each timestep.
   *  @param[in] train_error: Optinal pointer to return the training error as norm(F-Fd)/n_data.
   */
  void train(dmp_::TRAIN_METHOD train_method, const arma::rowvec &Time, const arma::mat &Qd_data,
            const arma::mat &vRotd_data, const arma::mat &dvRotd_data, arma::mat *train_err=0);


  /** Returns the angular acceleration for the given input state defined by the timestamp,
   *  the orientation, the angular velocity and acceleration, the initial and target orientation
   *  and an optinal coupling term.
   *  @param[in] x: phase variable.
   *  @param[in] Q: Current orientation as unit quaternion.
   *  @param[in] vRot: Current rotational velocity.
   *  @param[in] Q0: initial orientation as unit quaternion.
   *  @param[in] Qg: Goal orientation as unit quaternion.
   *  @param[in] Z_c: Coupling term. (optional, default=arma::vec().zeros(3))
   *  @return dvRot: Rotational acceleration.
   */
  arma::vec getRotAccel(double x, const arma::vec &Q, const arma::vec &vRot,
              const arma::vec &Q0, const arma::vec &Qg, const arma::vec &Z_c=arma::vec().zeros(3));


  void calcStatesDot(double x, const arma::vec &Q, const arma::vec &vRot, const arma::vec &Q0,
    const arma::vec &Qg, const arma::vec &Y_c=arma::vec().zeros(3), const arma::vec &Z_c=arma::vec().zeros(3));

  /** Returns the time scaling factor.
   *  @return: The time scaling factor.
   */
  double getTau() const;


  /** Sets the time scaling factor.
   *  @param[in] tau: The time scaling factor.
   */
  void setTau(double tau);


  /** Returns the phase variable corresponding to the given time instant.
   *  @param[in] t: The time instant.
   *  @return: The phase variable for time 't'.
   */
  double phase(double t) const;


  /** Returns the derivative of the phase variable.
   *  @param[in] x: The phase variable.
   *  @return: The derivative of the phase variable.
   */
  double phaseDot(double x) const;

  arma::vec getDphi() const { return this->dphi; }
  arma::vec getOmega() const { return this->omega; }
  double getDx() const { return dx; }

private:

  arma::vec Qgd; ///< Trained target orientation as unit quaternion.
  arma::vec Q0d; ///< Trained initial orientation as unit quaternion.
  arma::vec log_Qgd_invQ0d; ///< log(Qgd * inv(Q0d)), precalculated
  double tau_d; ///< Trained time-scaling.

  std::vector<std::shared_ptr<dmp_::WSoG>> eq_f; ///< 3x1 vector of WSoG producing the desired orientation.
  std::vector<std::shared_ptr<dmp_::WSoG>> vRot_f; ///< 3x1 vector of WSoG producing the desired rotational velocity.
  std::vector<std::shared_ptr<dmp_::WSoG>> dvRot_f; ///< 3x1 vector of WSoG producing the desired rotational acceleration.

  double a_z; ///< parameter 'a_z' relating to the spring-damper system
  double b_z; ///< parameter 'b_z' relating to the spring-damper system

  std::shared_ptr<CanonicalClock> can_clock_ptr; ///< handle (pointer) to the canonical clock
  std::shared_ptr<GatingFunction> shape_attr_gating_ptr; ///< pointer to gating function for the shape attractor

  double zero_tol; ///< tolerance value used to avoid divisions with very small numbers

  arma::vec dphi; ///< derivative of 'phi' state
  arma::vec omega; ///< rotational velocity
  double dx; ///< phase variable derivative
};

} // namespace as64_

#endif // DMP_ORIENTATION_H
