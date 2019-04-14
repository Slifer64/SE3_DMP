#include <dmp_lib/DMP/WSoG.h>

namespace as64_
{

namespace dmp_
{

WSoG::WSoG(int N_kernels, std::function<double(double)> shapeAttrGatingFun, double kernel_std_scaling)
{
  this->N_kernels = N_kernels;
  this->shapeAttrGatingFun = shapeAttrGatingFun;

  this->zero_tol = 1e-30; // realmin;

  this->w = arma::vec().zeros(N_kernels);
  this->c = arma::linspace<arma::vec>(0,N_kernels-1, N_kernels)/(N_kernels-1);
  this->h.resize(N_kernels);
  for (int i=0; i<N_kernels-1; i++) this->h(i) = 1 / std::pow(kernel_std_scaling*(this->c(i+1)-this->c(i)),2);
  this->h(N_kernels-1) = this->h(N_kernels-2);
}


int WSoG::getNumOfKernels() const
{
  return this->w.size();
}


void WSoG::train(dmp_::TRAIN_METHOD train_method, const arma::rowvec &x, const arma::rowvec &Fd, double *train_error)
{
  int n_data = x.size();

  arma::rowvec s(n_data);
  arma::mat Psi(this->N_kernels, n_data);
  for (int j=0; j<n_data; j++)
  {
    s(j) = this->shapeAttrGatingFun(x(j));
    Psi.col(j) = this->kernelFunction(x(j));
  }

  if (train_method == dmp_::LWR) this->w = dmp_::localWeightRegress(Psi, s, Fd, this->zero_tol);
  else if (train_method == dmp_::LS) this->w = dmp_::leastSquares(Psi, s, Fd, this->zero_tol);
  else throw std::runtime_error("[WSoG::train]: Unsopported training method...");

  if (train_error)
  {
    arma::rowvec F(n_data);
    for (int j=0; j<n_data; j++) F(j) = this->output(x(j));
    *train_error = arma::norm(F-Fd)/n_data;
  }

};


arma::vec WSoG::kernelFunction(double x) const
{
  arma::vec psi = arma::exp(-this->h % (arma::pow(x-this->c,2)));
  return psi;
}


double WSoG::output(double x) const
{
  arma::vec psi = this->kernelFunction(x);
  double f = this->shapeAttrGatingFun(x) * dot(psi,this->w) / (sum(psi)+this->zero_tol); // add 'zero_tol' to avoid numerical issues
  return f;
}

} // namespace dmp_

} // namespace as64_
