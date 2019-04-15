#include <dmp_lib/DMP/DMP_orient.h>

namespace as64_
{

namespace dmp_
{

  arma::vec quatLog(const arma::vec &quat, double zero_tol=1e-16)
  {
    arma::vec e = quat.subvec(1,3);
    double n = quat(0);

    if (n > 1) n = 1;
    if (n < -1) n = -1;

    arma::vec omega(3);
    double e_norm = arma::norm(e);

    if (e_norm > zero_tol) omega = 2*std::atan2(e_norm,n)*e/e_norm;
    else omega = arma::vec().zeros(3);

    return omega;
  }

  arma::vec quatExp(const arma::vec &v_rot, double zero_tol=1e-16)
  {
    arma::vec quat(4);
    double norm_v_rot = arma::norm(v_rot);
    double theta = norm_v_rot;

   if (norm_v_rot > zero_tol)
   {
      quat(0) = std::cos(theta/2);
      quat.subvec(1,3) = std::sin(theta/2)*v_rot/norm_v_rot;
    }
    else{
      quat << 1 << 0 << 0 << 0;
    }

    return quat;
  }

  arma::vec quatProd(const arma::vec &quat1, const arma::vec &quat2)
  {
    arma::vec quat12(4);

    double n1 = quat1(0);
    arma::vec e1 = quat1.subvec(1,3);

    double n2 = quat2(0);
    arma::vec e2 = quat2.subvec(1,3);

    quat12(0) = n1*n2 - arma::dot(e1,e2);
    quat12.subvec(1,3) = n1*e2 + n2*e1 + arma::cross(e1,e2);

    return quat12;
  }

  arma::vec quatInv(const arma::vec &quat)
  {
    arma::vec quatI(4);

    quatI(0) = quat(0);
    quatI.subvec(1,3) = - quat.subvec(1,3);

    return quatI;
  }

} // namespace dmp_


DMP_orient::DMP_orient(const arma::Mat<int> &N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalClock> &can_clock_ptr,
  std::shared_ptr<GatingFunction> &shape_attr_gating_ptr)
{
  this->zero_tol = 1e-30; // realmin;
  this->a_z = a_z;
  this->b_z = b_z;
  this->can_clock_ptr = can_clock_ptr;
  this->shape_attr_gating_ptr = shape_attr_gating_ptr;

  std::function<double(double)> shapeAttrGatFun_ptr =
  std::bind(static_cast<double(GatingFunction::*)(double)const>(&GatingFunction::getOutput),
            shape_attr_gating_ptr.get(), std::placeholders::_1);

  arma::Mat<int> n_ker(3,3);
  if (N_kernels.n_cols==1)
  {
    for (int i=0; i<3; i++) n_ker(i,0)=n_ker(i,1)=n_ker(i,2)=N_kernels(i);
  }

  this->eq_f.resize(3);
  this->vRot_f.resize(3);
  this->dvRot_f.resize(3);
  for (int i=0; i<3; i++)
  {
      this->eq_f[i].reset(new dmp_::WSoG(n_ker(0,i), shapeAttrGatFun_ptr));
      this->vRot_f[i].reset(new dmp_::WSoG(n_ker(1,i), shapeAttrGatFun_ptr));
      this->dvRot_f[i].reset(new dmp_::WSoG(n_ker(2,i), shapeAttrGatFun_ptr));
  }
}


void DMP_orient::train(dmp_::TRAIN_METHOD train_method, const arma::rowvec &Time, const arma::mat &Qd_data,
                      const arma::mat &vRotd_data, const arma::mat &dvRotd_data, arma::mat *train_err)
{
    int n_data = Time.size();
    int i_end = n_data-1;

    this->Q0d = Qd_data.col(0);
    this->Qgd = Qd_data.col(i_end);
    this->log_Qgd_invQ0d = dmp_::quatLog( dmp_::quatProd(this->Qgd, dmp_::quatInv(this->Q0d) ) );

    arma::mat eqd(3, n_data);
    for (int j=0; j<n_data; j++) eqd.col(j) = dmp_::quatLog(dmp_::quatProd(Qd_data.col(j),dmp_::quatInv(this->Qgd)));

    double tau = Time(i_end);
    this->tau_d = tau;
    this->setTau(tau);

    arma::rowvec x(n_data);
    for (int j=0; j<n_data; j++) x(j) = this->phase(Time(j));

    if (train_err)
    {
      train_err->resize(3,3);
      for (int i=0; i<3; i++)
      {
          this->eq_f[i]->train(train_method, x, eqd.row(i), &(train_err->at(0,i)));
          this->vRot_f[i]->train(train_method, x, vRotd_data.row(i), &(train_err->at(1,i)));
          this->dvRot_f[i]->train(train_method, x, dvRotd_data.row(i), &(train_err->at(2,i)));
      }
    }
    else
    {
      for (int i=0; i<3; i++)
      {
          this->eq_f[i]->train(train_method, x, eqd.row(i));
          this->vRot_f[i]->train(train_method, x, vRotd_data.row(i));
          this->dvRot_f[i]->train(train_method, x, dvRotd_data.row(i));
      }
    }
}


arma::vec DMP_orient::getRotAccel(double x, const arma::vec &Q, const arma::vec &vRot,
            const arma::vec &Q0, const arma::vec &Qg, const arma::vec &Z_c)
{

    arma::vec eqd(3);
    arma::vec vRotd(3);
    arma::vec dvRotd(3);
    for (int i=0; i<3; i++)
    {
        eqd(i) = this->eq_f[i]->output(x);
        vRotd(i) = this->vRot_f[i]->output(x);
        dvRotd(i) = this->dvRot_f[i]->output(x);
    }

    arma::vec Qd = dmp_::quatProd( dmp_::quatExp(eqd), this->Qgd );

    double tau = this->getTau();
    double kt = this->tau_d / tau;
    arma::vec ks = dmp_::quatLog( dmp_::quatProd(Qg, dmp_::quatInv(Q0) ) ) / this->log_Qgd_invQ0d;

    arma::vec QQg = dmp_::quatProd(Q,dmp_::quatInv(Qg));
    arma::vec inv_exp_QdQgd = dmp_::quatInv( dmp_::quatExp( ks % dmp_::quatLog( dmp_::quatProd(Qd,dmp_::quatInv(this->Qgd)) ) ) );

    arma::vec dvRot = std::pow(kt,2)*ks%dvRotd - (this->a_z/tau)*(vRot-kt*ks%vRotd)
            -(this->a_z*this->b_z/std::pow(tau,2)) * dmp_::quatLog ( dmp_::quatProd(QQg, inv_exp_QdQgd)) + Z_c;

    return dvRot;
}


double DMP_orient::getTau() const
{
    return this->can_clock_ptr->getTau();
}


void DMP_orient::setTau(double tau)
{
    this->can_clock_ptr->setTau(tau);
}


double DMP_orient::phase(double t) const
{
    return this->can_clock_ptr->getPhase(t);
}


double DMP_orient::phaseDot(double x) const
{
    return this->can_clock_ptr->getPhaseDot(x);
}

} // namespace as64_
