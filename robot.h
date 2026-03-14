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

    RobotKinematics(const std::array<Matrix<3, 1>, DOF>& L_,
                    const std::array<const RotationMatrix*, DOF>& R_)
        : L(L_), R_func(R_) {}

    std::pair<Matrix<3,1>, Matrix<3,3>>
    forwardKinematics(const Matrix<DOF,1>& theta) const override
    {
        // Iterujeme od posledného kĺbu dovnútra smerom k základni
        Matrix<3,1> vekt = Matrix<3,1>::Zero();
        for (int i = (int)DOF - 1; i >= 0; --i)
        {
            vekt = (*R_func[i])(theta(i)) * (L[i] + vekt);
        }

        // Orientácia efektora: R1 * R2 * ... * Rn
        Matrix<3,3> R_ee = Matrix<3,3>::Identity();
        for (std::size_t i = 0; i < DOF; ++i)
            R_ee = R_ee * (*R_func[i])(theta(i));

        return { vekt, R_ee };
    }

std::array<Matrix<3, 3>, DOF> getRotDerivatives(const Matrix<DOF, 1>& theta) const {
    std::array<Matrix<3, 3>, DOF> mat_all;         // Pole pre výsledné parciálne derivácie
    int n = DOF;                                   
    Matrix<3, 3> f_mat = Matrix<3, 3>::Identity(); 
    Matrix<3, 3> b_mat = Matrix<3, 3>::Identity();                   

    // Prvý cyklus: Výpočet celkového dopredného súčinu matíc
    // Týmto f_mat naakumuluje súčin R_0 * R_1 * ... * R_{n-2}
    for (int i = n - 2; i >= 0; i--) {
        f_mat = (*R_func[i])(theta(i)) * f_mat; 
    }

    // Druhý cyklus: Výpočet derivácií smerom od konca k začiatku
    for (int j = n - 1; j >= 0; j--) {
        if (j < n - 1) {
            b_mat = (*R_func[j + 1])(theta(j + 1)) * b_mat;
            
            // "Odstraňovanie" aktuálnej matice zľava pomocou transpozície (R^-1 = R^T)
            f_mat = f_mat * (*R_func[j])(theta(j)).transpose();
        }
        
        // Samotný výpočet derivácie pre j-ty kĺb
        // Pre prístup k derivácii voláme metódu d() cez pointer z tvojho poľa R_func
        mat_all[j] = f_mat * R_func[j]->d(theta(j)) * b_mat;
    }

    return mat_all;
}

Matrix<3, DOF> getJacobian(const Matrix<DOF, 1>& theta) const {
    int n = DOF; // Počet kĺbov
    Matrix<3, DOF> jakob_mat = Matrix<3, DOF>::Zero(); // Výsledná Jakobián matica (3 riadky, n stĺpcov)
    
    Matrix<3, 3> matica = Matrix<3, 3>::Identity(); // [1 0 0; 0 1 0; 0 0 1]
    Matrix<3, 1> pom_vekt = L[n - 1];            // Inicializácia T[n] v pseudokóde

    // Prvý cyklus: od i = n-1 do 1 (1-based) -> od i = n-2 po 0 (0-based)
    // Kumuluje doprednú kinematiku (súčin rotácií od n-2 po 0)
    for (int i = n - 2; i >= 0; i--) {
        matica = (*R_func[i])(theta(i)) * matica;
    }

    // Druhý cyklus: od i = n do 1 (1-based) -> od i = n-1 po 0 (0-based)
    for (int i = n - 1; i >= 0; i--) {
        
        Matrix<3, 1> vekt = L[n - 1]; // T[n] z pseudokódu

        if (i < n - 1) { // Podmienka (n > i) z pseudokódu v 0-based svete
            
            // "Odstraňovanie" rotácie pre aktuálny kĺb
            matica = matica * (*R_func[i])(theta(i)).transpose();
            
            // Akumulácia translačných vektorov od konca (efektora) ku kĺbu 'i'
            pom_vekt = L[i] + (*R_func[i + 1])(theta(i + 1)) * pom_vekt;
            
            vekt = pom_vekt;
        }

        // Výpočet samotného stĺpca Jakobiánu
        // matica: (R_1 * R_2 ... * R_{i-1})
        // dR_i: parciálna derivácia i-teho kĺbu
        // vekt: (T_i + R_{i+1} * (T_{i+1} + ...))
        vekt = matica * R_func[i]->d(theta(i)) * vekt;

        // Uloženie výsledného vektora do i-teho stĺpca matice Jakobiánu
        jakob_mat.col(i) = vekt;
    }

    return jakob_mat;
}

    bool SolveIK(
        const Matrix<3,1>&, const Matrix<3,3>&,
        Matrix<DOF,1>&, float, float, float, int) override
    {
        return false;
    }

private:
    std::array<Matrix<3, 1>, DOF> L;
    std::array<const RotationMatrix*, DOF> R_func;
};

}
