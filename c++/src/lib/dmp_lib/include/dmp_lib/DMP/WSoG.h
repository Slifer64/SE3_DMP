#ifndef DMP_WEIGHTED_SUM_OF_GAUSSIANS_H
#define DMP_WEIGHTED_SUM_OF_GAUSSIANS_H

#include <cmath>
#include <vector>
#include <cstring>
#include <memory>
#include <exception>
#include <functional>
#include <armadillo>

#include <dmp_lib/dmp_defs.h>
#include <dmp_lib/trainMethods/LeastSquares.h>
#include <dmp_lib/trainMethods/LWR.h>

namespace as64_
{

namespace dmp_
{

class WSoG
{
public:

  /** Weighted Sum of Gaussians constructor.
   *  @param[in] N_kernels: The number of kernels.
   *  @param[in] shapeAttrGatingFun: Pointer to gating function that ensures the output of WSoG decays
                                     to zero at the end (i.e. when the phase variable reaches 1.0).
   *  @param[in] kernel_std_scaling: Scaling of the kernel's std. (optional, default=1.0)
   */
  WSoG(int N_kernels, std::function<double(double)> shapeAttrGatingFun, double kernel_std_scaling = 1.0);

  /** Returns the number of kernels.
   *  @return The number of kernels.
   */
  int getNumOfKernels() const;

  /** Trains the WSoG.
   *  @param[in] train_method: The training method (see dmp_::TRAIN_METHOD enum).
   *  @param[in] x: Row vector with the timestamps of the training data points. Must take values in [0 1].
   *  @param[in] Fd: Row vector with the desired values.
   *  @param[in] train_error: Optinal pointer to return the training error as norm(F-Fd)/n_data.
   */
  void train(dmp_::TRAIN_METHOD train_method, const arma::rowvec &x, const arma::rowvec &Fd, double *train_error=0);

  /** Returns a column vector with the values of the kernel functions.
   *  @param[in] x: The phase variable.
   *  @return: Column vector with the values of the kernel functions.
   */
  arma::vec kernelFunction(double x) const;


  /** Returns the normalized weighted sum of the Gaussians for the given phase variable (time instant).
   *  @param[in] x: The phase variable.
   *  @param[out] f: The normalized weighted sum of the Gaussians.
   */
  double output(double x) const;

private:

  int N_kernels; ///< number of kernels (basis functions)

  arma::vec w; ///< N_kernels x 1 vector with the kernels' weights
  arma::vec c; ///< N_kernels x 1 vector with the kernels' centers
  arma::vec h; ///< N_kernels x 1 vector with the kernels' inverse width

  std::function<double(double)> shapeAttrGatingFun; ///< Pointer to gating function that ensures the output of WSoG decays to zero at the end (i.e. when the phase variable reaches 1.0)

  long double zero_tol; ///< small value used to avoid divisions with very small numbers
};

} // namespace dmp_

} // namespace as64_

#endif // DMP_WEIGHTED_SUM_OF_GAUSSIANS_H
