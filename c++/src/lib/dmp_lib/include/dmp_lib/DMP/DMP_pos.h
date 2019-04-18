#ifndef DMP_POS_H
#define DMP_POS_H

#include <dmp_lib/DMP/DMP.h>
#include <dmp_lib/CanonicalClock/CanonicalClock.h>
#include <dmp_lib/GatingFunction/GatingFunction.h>

namespace as64_
{

class DMP_pos
{

public:

  /** DMP_pos constructor.
   *  @param[in] N_kernels: 3x3 matrix where each row contains the kernels for position,
                            velocity and acceleration and each row for x, y and z coordinate.
                            If 'N_kernels' is a column vector then every coordinate will have the
                            same number of kernels.
   *  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
   *  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
   *  @param[in] can_clock_ptr: Pointer to a DMP canonical system object.
   *  @param[in] shape_attr_gating_ptr: Pointer to a DMP gating function object.
   */
  DMP_pos(const arma::Col<int> &N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> &can_clock_ptr,
    std::shared_ptr<GatingFunction> &shape_attr_gating_ptr);

  // DMP_pos(const arma::mat &N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> &can_clock_ptr);
  // DMP_pos(const arma::mat &N_kernels, double a_z, double b_z);

  /** Trains the DMP_pos.
   *  @param[in] train_method: The training method (see dmp_::TRAIN_METHOD enum).
   *  @param[in] Time: 1xN row vector with the timestamps of the training data points.
   *  @param[in] Qd_data: 4xN matrix with desired orientation as unit quaternion at each column for each timestep.
   *  @param[in] vRotd_data: 3xN matrix with desired angular velocity at each column for each timestep.
   *  @param[in] dvRotd_data: 3xN matrix with desired angular acceleration at each column for each timestep.
   *  @param[in] train_error: Optinal pointer to return the training error as norm(F-Fd)/n_data.
   */
  void train(dmp_::TRAIN_METHOD train_method, const arma::rowvec &Time, const arma::mat &Pd_data,
                        const arma::mat &dPd_data, const arma::mat &ddPd_data, arma::vec *train_err=0);


  /** Calculates the derivatives of the DMP states. The derivatives can then be
   * retrieved with 'getDx', 'getDy' and 'getDz'.
   * @param[in] x: phase variable.
   * @param[in] Y: 'y' state of the DMP.
   * @param[in] Z: 'z' state of the DMP.
   * @param[in] Y0: Initial position.
   * @param[in] Yg: Goal position.
   * @param[in] Y_c: Coupling term for the dynamical equation of the 'y' state.
   * @param[in] Z_c: Coupling term for the dynamical equation of the 'z' state.
   */
  void calcStatesDot(double x, const arma::vec &Y, const arma::vec &Z, const arma::vec &Y0,
    const arma::vec &Yg, const arma::vec &Y_c=arma::vec().zeros(3), const arma::vec &Z_c=arma::vec().zeros(3));


  /** Returns the acceleration for the given input state defined by the timestamp,
   *  the orientation, the angular velocity and acceleration, the initial and target orientation
   *  and an optinal coupling term.
   *  @param[in] x: Phase variable.
   *  @param[in] P: Current position.
   *  @param[in] dP: Current velocity.
   *  @param[in] P0: Initial position.
   *  @param[in] Pg: Goal position.
   *  @param[in] Z_c: Coupling term. (optional, default=arma::vec().zeros(3))
   *  @return dvRot: Acceleration.
   */
  arma::vec getAccel(double x, const arma::vec &P, const arma::vec &dP,
              const arma::vec &P0, const arma::vec &Pg, const arma::vec &Z_c=arma::vec().zeros(3));


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

  double getDx() const;
  arma::vec getDy() const;
  arma::vec getDz() const;

private:

  std::vector<std::shared_ptr<DMP>> dmp; ///< 3x1 vector of DMP producing the desired trajectory for each x, y and z dimensions.

  std::shared_ptr<CanonicalClock> can_clock_ptr; ///< handle (pointer) to the canonical clock
  std::shared_ptr<GatingFunction> shape_attr_gating_ptr; ///< pointer to gating function for the shape attractor

  double dx;
  arma::vec dY;
  arma::vec dZ;

};

} // namespace as64_

#endif // DMP_POS_H
