#include <cstddef>
#include <array>
#pragma once

#include "geometry.h"

template<int Rows, int Cols>
using Matrix = Eigen::Matrix<float, Rows, Cols>;

typedef struct
{
  float minAngle;
  float maxAngle;
  float gain;
  float offset;
} actuator_parameters;


namespace Robot {

  // Pseudokód: "function T(l)" – translačný vektor pozdĺž Z osi
    static inline Matrix<3,1> T(float l) {
    Matrix<3,1> t;
    t << 0.0f, 0.0f, l;
    return t;
}

template<std::size_t DOF>
class RobotKinematicsBase {
public:

  virtual bool SolveIK(
    const Matrix<3, 1>& pr,
    const Matrix<3, 3>& Rr,
    Matrix<DOF, 1>& theta_,
    float tol_pos,
    float tol_ori,
    float lambda,
    int max_iter)  = 0;


  virtual std::pair<Matrix<3, 1>, Matrix<3, 3>> forwardKinematics(const Matrix<DOF, 1>& theta) const = 0;
};



class JointActuators {
public:
  JointActuators() {}
  virtual bool writeAngles(const float*, std::size_t) = 0;
};

template<std::size_t DOF>
class RobotArm {
public:

  RobotArm(RobotKinematicsBase<DOF>* robot_kinematics, JointActuators* actuators)
    : robot_kinematics_(robot_kinematics), jointAngles_(Matrix<DOF, 1>::Zero()), actuators_(actuators) {
  }

  ~RobotArm() = default;


  bool setJointAngles(const Matrix<DOF, 1>& angles) {

    if (!actuators_->writeAngles(angles.data(), DOF)) return false;
    return true;
  }

  float getJointAngle(std::size_t index) const {
    return jointAngles_(index);
  }


  Matrix<DOF, 1> getJointAngles() const {
    return jointAngles_;
  }

  std::pair<Matrix<3, 1>, Matrix<3, 3>> getCurrentEffectorPositionOrientation() const {
    return robot_kinematics_->forwardKinematics(jointAngles_);
  }

  bool goTo(const Matrix<3, 1>& pr, const Matrix<3, 3>& Rr) {
    Matrix<DOF, 1> theta = jointAngles_;
    bool res = robot_kinematics_->SolveIK(pr, Rr, theta, 1e-5f, 1e-3f, 1e-2, 50);
    
    if (res) {
      res = setJointAngles(theta);
    }
    return res;
  }



protected:
  JointActuators* actuators_;
  Matrix<DOF, 1> jointAngles_;
  RobotKinematicsBase<DOF>* robot_kinematics_;
};

template<std::size_t DOF>
class RobotKinematics : public RobotKinematicsBase<DOF>
{
public:

    RobotKinematics(const std::array<float, DOF>& L_,
                    const std::array<const RotationMatrix*, DOF>& R_)
        : L(L_), R_func(R_) {}

    std::pair<Matrix<3,1>, Matrix<3,3>>
    forwardKinematics(const Matrix<DOF,1>& theta) const override
    {
        // Iterujeme od posledného kĺbu dovnútra smerom k základni
        Matrix<3,1> vekt = Matrix<3,1>::Zero();
        for (int i = (int)DOF - 1; i >= 0; --i)
        {
            vekt = (*R_func[i])(theta(i)) * (T(L[i]) + vekt);
        }

        // Orientácia efektora: R1 * R2 * ... * Rn
        Matrix<3,3> R_ee = Matrix<3,3>::Identity();
        for (std::size_t i = 0; i < DOF; ++i)
            R_ee = R_ee * (*R_func[i])(theta(i));

        return { vekt, R_ee };
    }

    bool SolveIK(
        const Matrix<3,1>&, const Matrix<3,3>&,
        Matrix<DOF,1>&, float, float, float, int) override
    {
        return false;
    }

private:
    std::array<float, DOF> L;
    std::array<const RotationMatrix*, DOF> R_func;
};

}
