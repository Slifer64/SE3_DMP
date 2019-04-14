#include <dmp_lib/trainMethods/LWR.h>

namespace as64_
{

namespace dmp_
{

arma::mat localWeightRegress(const arma::mat &Psi, const arma::mat &X, const arma::rowvec &Fd, double zero_tol)
{
  int N_kernels = Psi.n_rows;
  int w_dim = X.n_rows;

  arma::mat w(N_kernels, w_dim);
  for (int k=0; k<N_kernels; k++)
  {
      arma::mat X_Psi = X % arma::repmat(Psi.row(k), X.n_rows, 1);
      w.row(k) = (arma::pinv((X_Psi*X.t() + zero_tol)) * X_Psi*Fd.t()).t();
  }

  return w;
}

} // namespace dmp_

} // namespace as64_
