//Adapt from RTABmap

#include "cartographer/mapping/internal/3d/gravity_factor/gravity_factor.h"
#include "glog/logging.h"

using namespace std;
namespace gtsam {

//***************************************************************************
Vector GravityFactor::attitudeError(const Rot3& nRb,
    OptionalJacobian<2, 3> H) const {
  const double k_cost_value = 1e-5;
  if (H) {
    Matrix23 D_nRef_R;
    Matrix22 D_e_nRef;
    Vector3 r = nRb.xyz();
    Unit3 nRef = Rot3::RzRyRx(r.x(), r.y(), 0).rotate(bRef_, D_nRef_R);
    Vector e = nZ_.error(nRef, D_e_nRef);
    Vector2 k_cost_value_vec(k_cost_value, k_cost_value);
    e += k_cost_value_vec;
    (*H) = D_e_nRef * D_nRef_R;
    return e;
  } else {
    Vector3 r = nRb.xyz();
    Unit3 nRef = Rot3::RzRyRx(r.x(), r.y(), 0) * bRef_;
    Vector e = nZ_.error(nRef);
    Vector2 k_cost_value_vec(k_cost_value, k_cost_value);
    e += k_cost_value_vec;
    return e;
  }
}

//***************************************************************************
void Rot3GravityFactor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << s << "Rot3GravityFactor on " << keyFormatter(this->key()) << "\n";
  nZ_.print("  measured direction in nav frame: ");
  bRef_.print("  reference direction in body frame: ");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool Rot3GravityFactor::equals(const NonlinearFactor& expected,
    double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != NULL && Base::equals(*e, tol) && this->nZ_.equals(e->nZ_, tol)
      && this->bRef_.equals(e->bRef_, tol);
}

//***************************************************************************
void Pose3GravityFactor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << s << "Pose3GravityFactor on " << keyFormatter(this->key()) << "\n";
  nZ_.print("  measured direction in nav frame: ");
  bRef_.print("  reference direction in body frame: ");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool Pose3GravityFactor::equals(const NonlinearFactor& expected,
    double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != NULL && Base::equals(*e, tol) && this->nZ_.equals(e->nZ_, tol)
      && this->bRef_.equals(e->bRef_, tol);
}
}
