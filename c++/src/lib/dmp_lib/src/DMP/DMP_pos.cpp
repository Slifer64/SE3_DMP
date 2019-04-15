#include <dmp_lib/DMP/DMP_pos.h>

namespace as64_
{

DMP_pos::DMP_pos(const arma::Col<int> &N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> &can_clock_ptr,
  std::shared_ptr<GatingFunction> &shape_attr_gating_ptr)
{
  this->can_clock_ptr = can_clock_ptr;
  this->shape_attr_gating_ptr = shape_attr_gating_ptr;

  this->dmp.resize(3);
  for (int i=0; i<3; i++)
  {
      this->dmp[i].reset(new DMP(N_kernels(i), a_z, b_z, can_clock_ptr, shape_attr_gating_ptr));
  }
}


void DMP_pos::train(dmp_::TRAIN_METHOD train_method, const arma::rowvec &Time, const arma::mat &Pd_data,
                      const arma::mat &dPd_data, const arma::mat &ddPd_data, arma::vec *train_err)
{
    int n_data = Time.size();
    int i_end = n_data-1;

    double tau = Time(i_end);
    this->setTau(tau);

    if (train_err)
    {
      train_err->resize(3);
      for (int i=0; i<3; i++)
      {
          this->dmp[i]->train(train_method, Time, Pd_data.row(i), dPd_data.row(i), ddPd_data.row(i), &(train_err->at(i)));
      }
    }
    else
    {
      for (int i=0; i<3; i++)
      {
          this->dmp[i]->train(train_method, Time, Pd_data.row(i), dPd_data.row(i), ddPd_data.row(i), &(train_err->at(i)));
      }
    }
}


void DMP_pos::calcStatesDot(double x, const arma::vec &Y, const arma::vec &Z, const arma::vec &Y0,
  const arma::vec &Yg, const arma::vec &Y_c, const arma::vec &Z_c)
{
  for (int i=0; i<3; i++) this->dmp[i]->calcStatesDot(x, Y(i), Z(i), Y0(i), Yg(i), Y_c(i), Z_c(i));

  this->dZ.resize(3);
  this->dY.resize(3);
  for (int i=0; i<3; i++)
  {
      this->dY(i) = this->dmp[i]->getDy();
      this->dZ(i) = this->dmp[i]->getDz();
  }
  this->dx = this->phaseDot(x);
}


arma::vec DMP_pos::getAccel(double x, const arma::vec &P, const arma::vec &dP,
            const arma::vec &P0, const arma::vec &Pg, const arma::vec &Z_c)
{

  arma::vec ddP(3);
  for (int i=0; i<3; i++) ddP(i) = this->dmp[i]->getAccel(x, P(i), dP(i), P0(i), Pg(i), 0, Z_c(i));

  return ddP;
}


double DMP_pos::getTau() const
{
    return this->can_clock_ptr->getTau();
}


void DMP_pos::setTau(double tau)
{
    this->can_clock_ptr->setTau(tau);
}


double DMP_pos::phase(double t) const
{
    return this->can_clock_ptr->getPhase(t);
}


double DMP_pos::phaseDot(double x) const
{
    return this->can_clock_ptr->getPhaseDot(x);
}

double DMP_pos::getDx() const { return this->dx; }
arma::vec DMP_pos::getDy() const { return this->dY; }
arma::vec DMP_pos::getdZ() const { return this->dZ; }

} // namespace as64_
