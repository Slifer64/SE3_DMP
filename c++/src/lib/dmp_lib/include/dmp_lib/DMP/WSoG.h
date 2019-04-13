#ifndef DMP_WEIGHTED_SUM_OF_GAUSSIANS_H
#define DMP_WEIGHTED_SUM_OF_GAUSSIANS_H

#include <cmath>
#include <vector>
#include <cstring>
#include <memory>
#include <exception>
#include <armadillo>

#include <dmp_lib/GatingFunction/GatingFunction.h>

namespace as64_
{

class WSoG
{
public:

  enum TRAIN_METHOD
  {
    LWR = 201;
    LS = 203;
  };

  /** DMP constructor.
   *  @param[in] N_kernels: the number of kernels
   */
  WSoG(int N_kernels, std::shared_ptr<GatingFunction> &shape_attr_gating_ptr, double kernel_std_scaling = 1.0)
  {
    this->N_kernels = N_kernels;
    this->shapeAttrGatingFun = shapeAttrGatingFun;
    this->kernel_std_scaling = kernel_std_scaling;

    this->zero_tol = 1e-30; // realmin;

    this->w = zeros(this->N_kernels,1);
    this->c = ((1:this->N_kernels)-1)'/(this->N_kernels-1);
    this->h = 1./(kernel_std_scaling*(this->c(2:end)-this->c(1:end-1))).^2;
    this->h = [this->h; this->h(end)];
  }


  int getNumOfKernels()
  {
    return length(this->w);
  }

  /** Trains the WSoG.
   *  @param[in] Time: Row vector with the timestamps of the training data points.
   *  @param[in] Fd_data: Row vector with the desired potition.
   */
  double train(train_method, x, Fd)
  {
    int n_data = x.size();

    arma::rowvec s(n_data);
    Psi = zeros(this->N_kernels, n_data);
    for (int j=0; j<n_data; j++)
    {
      s(j) = this->shapeAttrGatingFun(x(j));
      Psi.col(j) = this->kernelFunction(x(j));
    }

    if (train_method == WSoG.LWR) this->w = LWR(Psi, s, Fd, this->zero_tol);
    else if (train_method == WSoG.LS) this->w = leastSquares(Psi, s, Fd, this->zero_tol);
    else error('[WSoG::train]: Unsopported training method...');

    arma::rowvec F(n_data);
    for (int j=0; j<n_data; j++) F(j) = this->output(x(j));

    double train_error = arma::norm(F-Fd)/n_data;

    return train_error;
  };




  /** Returns a column vector with the values of the kernel functions of the WSoG.
   *  @param[in] x: phase variable
   *  @param[out] psi: column vector with the values of the kernel functions of the DMP
   */
  arma::mat kernelFunction(double x)
  {
    int n = length(x);
    arma::mat psi = zeros(this->N_kernels, n);

    for (int j=0; j<n; j++)
      psi(:,j) = exp(-this->h.*((x(j)-this->c).^2));
    end
  }





  %% Returns the forcing term of the WSoG.
  %  @param[in] x: The phase variable.
  %  @param[out] f: The normalized weighted sum of Gaussians.
  function f = output(this,x)

  Psi = this->kernelFunction(x);

  f = this->shapeAttrGatingFun(x) * dot(Psi,this->w) / (sum(Psi)+this->zero_tol); % add 'zero_tol' to avoid numerical issues

  end

      end

private:

  int N_kernels; ///< number of kernels (basis functions)

  arma::vec w; ///< N_kernels x 1 vector with the kernels' weights
  arma::vec c; ///< N_kernels x 1 vector with the kernels' centers
  arma::vec h; ///< N_kernels x 1 vector with the kernels' inverse width

  double kernel_std_scaling; ///< scales the std of the kernel function

  std::shared_ptr<GatingFunction> shape_attr_gating_ptr; ///< pointer to gating function for the shape attractor

  long double zero_tol; ///< small value used to avoid divisions with very small numbers
};

} // namespace as64_

#endif // DMP_WEIGHTED_SUM_OF_GAUSSIANS_H