/**
 * @file robot.h
 * @brief Robot kinematics and actuator interfaces.
 */
#pragma once
#include <cstddef>
#include <array>
#include <cmath>
#pragma once

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#include "geometry.h"

template<int Rows, int Cols>
using Matrix = Eigen::Matrix<float, Rows, Cols>;

/**
 * @struct actuator_parameters
 * @brief Joint actuator configuration and calibration parameters
 * 
 * Stores calibration and limit parameters for a single joint actuator.
 * Used to convert between joint angle space and PWM control signals.
 */
typedef struct
{
  float minAngle;  ///< Minimum joint angle in radians
  float maxAngle;  ///< Maximum joint angle in radians
  float gain;      ///< Scaling factor for angle-to-PWM conversion
  float offset;    ///< Offset applied to joint angles during conversion
} actuator_parameters;


/**
 * @namespace Robot
 * @brief Core robotics control and kinematics namespace
 * 
 * Contains base classes and implementations for robot arm kinematic control,
 * including forward kinematics, inverse kinematics (IK), and joint actuation.
 */
namespace Robot {

  /**
   * @brief Creates a translation vector along the Z-axis
   * @param l Translation distance in meters
   * @return 3x1 translation vector [0, 0, l]
   * 
   * Used as a helper function for DH parameter transformations.
   */
  static inline Matrix<3,1> T(float l) {
    Matrix<3,1> t;
    t << 0.0f, 0.0f, l;
    return t;
  }

/**
 * @class RobotKinematicsBase
 * @brief Abstract base class for robot kinematics solvers
 * @tparam DOF Degrees of freedom (number of joints)
 * 
 * Defines the interface for forward and inverse kinematics computations.
 * Concrete implementations must solve both FK (joint angles → end-effector pose)
 * and IK (desired pose → joint angles).
 */
template<std::size_t DOF>
class RobotKinematicsBase {
public:

  /**
   * @brief Solves inverse kinematics using numerical optimization
   * @param pr Desired end-effector position [x, y, z] in meters
   * @param Rr Desired end-effector rotation matrix (3x3)
   * @param theta_ [in/out] Joint angles: initialized with starting values,
   *                        updated with IK solution
   * @param tol_pos Position tolerance in meters (convergence criterion)
   * @param tol_ori Orientation tolerance in radians (convergence criterion)
   * @param lambda Damping factor for Levenberg-Marquardt regularization
   * @param max_iter Maximum number of iterations
   * @return true if solution found within tolerance, false otherwise
   */
  virtual bool SolveIK(
    const Matrix<3, 1>& pr,
    const Matrix<3, 3>& Rr,
    Matrix<DOF, 1>& theta_,
    float tol_pos,
    float tol_ori,
    float lambda,
    int max_iter)  = 0;

  /**
   * @brief Computes end-effector position and orientation from joint angles
   * @param theta Joint angles in radians
   * @return Pair of (position vector [x,y,z], rotation matrix 3x3)
   */
  virtual std::pair<Matrix<3, 1>, Matrix<3, 3>> forwardKinematics(const Matrix<DOF, 1>& theta) const = 0;
};



/**
 * @class JointActuators
 * @brief Abstract base class for joint actuation hardware
 * 
 * Defines the interface for sending computed joint angles to physical actuators
 * (servos, motors, etc.). Implementations handle conversion from angles to
 * hardware control signals (PWM, current commands, etc.).
 */
class JointActuators {
public:
  /**
   * @brief Default constructor
   */
  JointActuators() {}

  /**
   * @brief Sends joint angle commands to all actuators
   * @param angles Array of joint angles in radians
   * @param n Number of joints (should match DOF)
   * @return true if all angles were within valid ranges and written successfully,
   *         false if any angle exceeded joint limits
   */
  virtual bool writeAngles(const float*, std::size_t) = 0;
};

/**
 * @class RobotArm
 * @brief High-level robot arm control interface
 * @tparam DOF Degrees of freedom (number of joints)
 * 
 * Coordinates kinematics solvers with hardware actuators to provide
 * intuitive end-effector-centric control. Maintains current joint state
 * and manages both forward kinematics (state monitoring) and inverse
 * kinematics (motion planning).
 */
template<std::size_t DOF>
class RobotArm {
public:

  /**
   * @brief Initializes robot arm with kinematics solver and actuator hardware
   * @param robot_kinematics Pointer to kinematics solver instance
   * @param actuators Pointer to joint actuators instance
   * 
   * Does not take ownership of pointers; caller must manage object lifetimes.
   */
  RobotArm(RobotKinematicsBase<DOF>* robot_kinematics, JointActuators* actuators)
    : robot_kinematics_(robot_kinematics), jointAngles_(Matrix<DOF, 1>::Zero()), actuators_(actuators) {
  }

  /**
   * @brief Destructor
   */
  ~RobotArm() = default;


  /**
   * @brief Sets and commits joint angles to hardware actuators
   * @param angles Target joint angles in radians
   * @return true if angles were within limits and successfully written,
   *         false if any angle exceeded joint constraints
   * 
   * Updates internal state only if actuators successfully write the angles.
   */
  bool setJointAngles(const Matrix<DOF, 1>& angles) {

    if (!actuators_->writeAngles(angles.data(), DOF)) return false;
    jointAngles_ = angles;
    return true;
  }

  /**
   * @brief Gets current angle of a single joint
   * @param index Joint index (0 to DOF-1)
   * @return Joint angle in radians
   */
  float getJointAngle(std::size_t index) const {
    return jointAngles_(index);
  }

  /**
   * @brief Gets current angles of all joints
   * @return Vector of DOF joint angles in radians
   */
  Matrix<DOF, 1> getJointAngles() const {
    return jointAngles_;
  }

  /**
   * @brief Computes current end-effector pose from joint state
   * @return Pair of (end-effector position [x,y,z] in meters,
   *                   end-effector rotation matrix 3x3)
   */
  std::pair<Matrix<3, 1>, Matrix<3, 3>> getCurrentEffectorPositionOrientation() const {
    return robot_kinematics_->forwardKinematics(jointAngles_);
  }

  /**
   * @brief Moves robot end-effector to desired position and orientation
   * @param pr Desired end-effector position [x, y, z] in meters
   * @param Rr Desired end-effector rotation matrix (3x3)
   * @return true if IK solution found and successfully moved to target,
   *         false if IK failed or joint angles exceeded limits
   * 
   * Solves inverse kinematics and updates joint actuators if solution found.
   * Uses predefined convergence tolerances (1e-5 m position, 1e-3 rad orientation)
   * and Levenberg-Marquardt damping factor 1e-2 with maximum 50 iterations.
   */
  bool goTo(const Matrix<3, 1>& pr, const Matrix<3, 3>& Rr) {
    Matrix<DOF, 1> theta = jointAngles_;
    bool res = robot_kinematics_->SolveIK(pr, Rr, theta, 1e-5f, 1e-3f, 1e-2, 50);
    
    if (res) {
      res = setJointAngles(theta);
    }
    return res;
  }



protected:
  JointActuators* actuators_;                ///< Pointer to joint actuator hardware interface
  Matrix<DOF, 1> jointAngles_;               ///< Current joint angles in radians
  RobotKinematicsBase<DOF>* robot_kinematics_; ///< Pointer to kinematics solver
};

/**
 * @class RobotKinematics
 * @brief Concrete kinematics solver using Jacobian-based inverse kinematics
 * @tparam DOF Degrees of freedom (number of joints)
 * 
 * Implements forward and inverse kinematics for a robot arm using:
 * - Forward kinematics: Composition of rotation matrices and translations
 * - Inverse kinematics: Levenberg-Marquardt numerical optimization with
 *   analytical Jacobian computation for both position and orientation errors
 */
template<std::size_t DOF>
class RobotKinematics : public RobotKinematicsBase<DOF>
{
public:

  /**
   * @brief Initializes kinematics solver with robot link parameters
   * @param L_ Array of link translation vectors (DH parameter d)
   * @param R_ Array of rotation function pointers for each joint
   *           Each RotationMatrix* implements R(theta) = rotation matrix for that joint
   */
  RobotKinematics(const std::array<Matrix<3, 1>, DOF>& L_,
                  const std::array<const RotationMatrix*, DOF>& R_)
      : L(L_), R_func(R_) {}

  /**
   * @brief Computes end-effector position and orientation from joint angles
   * @param theta Joint angles in radians
   * @return Pair of (end-effector position [x,y,z], end-effector rotation matrix)
   * 
   * Implements DH-based forward kinematics: p = (R0 * R1 * ... * Rn) * (L0 + L1 + ...)
   */
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

  /**
   * @brief Computes partial derivatives of rotation matrices with respect to joint angles
   * @param theta Current joint angles in radians
   * @return Array of DOF matrices: dR_i/dtheta_i for each joint i
   * 
   * Used internally by inverse kinematics to build the Jacobian for
   * orientation error (last 3 rows). Computed using:
   * dE_i = dR_i/dtheta_i * Rr^T where dR_i is the derivative of i-th rotation matrix
   */
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

  /**
   * @brief Computes 3×DOF geometric Jacobian for end-effector linear velocity
   * @param theta Current joint angles in radians
   * @return 3×DOF Jacobian matrix: dp/dtheta
   * 
   * Relates joint angular velocities to end-effector linear velocity:
   * v_end = J * dtheta/dt
   * 
   * Each column j represents the contribution of joint j's rotation to
   * end-effector velocity.
   */
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

  /**
   * @brief Solves inverse kinematics using Levenberg-Marquardt optimization
   * @param pr Desired end-effector position [x, y, z] in meters
   * @param Rr Desired end-effector rotation matrix (3×3)
   * @param theta [in/out] Starting joint angles (input), IK solution (output)
   * @param tol_pos Position error tolerance in meters
   * @param tol_ori Orientation error tolerance in radians (extracted from R_error)
   * @param lambda Damping factor for Levenberg-Marquardt regularization
   * @param max_iter Maximum iterations before giving up
   * @return true if converged to solution meeting both tolerances,
   *         false if max iterations exceeded without convergence
   * 
   * Algorithm:
   * 1. Compute forward kinematics for current pose
   * 2. Calculate position error: rp = p_current - p_desired
   * 3. Calculate orientation error from rotation matrix: r0 = [R_err(1,0), R_err(2,0), R_err(2,1)]
   * 4. Build 6×DOF Jacobian combining position and orientation derivatives
   * 5. Apply Levenberg-Marquardt damping to Jacobian diagonal
   * 6. Solve for joint angle updates: delta_theta = J^-1 * (-residuals)
   * 7. Update joint angles and repeat until convergence
   */
  bool SolveIK(
      const Matrix<3, 1>& pr, 
      const Matrix<3, 3>& Rr,
      Matrix<DOF, 1>& theta, 
      float tol_pos, 
      float tol_ori, 
      float lambda, 
      int max_iter) override
    {
        for (int i = 0; i < max_iter; i++)
        {
            // 1. Dopredná kinematika (aktuálny stav)
            auto fk_res = forwardKinematics(theta);
            Matrix<3, 1> p0 = fk_res.first;
            Matrix<3, 3> R0 = fk_res.second;

            // 2. Výpočet reziduí (chyby)
            Matrix<3, 1> rp = p0 - pr;
            
            // Výpočet matice chyby orientácie E = R0 * Rr^T
            Matrix<3, 3> E = R0 * Rr.transpose();

            // Extrakcia chyby orientácie podľa tvojho pseudokódu
            Matrix<3, 1> r0;
            r0(0) = E(1, 0); 
            r0(1) = E(2, 0); 
            r0(2) = E(2, 1); 

            // 3. Kontrola ukončenia (ak sme v tolerancii)
            if (rp.norm() < tol_pos && r0.norm() < tol_ori) {
                return true; 
            }

            // 4. Získanie Jakobiánov (tieto funkcie už v robot.h máš)
            Matrix<3, DOF> J_p = getJacobian(theta);
            std::array<Matrix<3, 3>, DOF> dEr_R = getRotDerivatives(theta);

            // Zostavenie plného Jakobiánu 6xDOF
            Matrix<6, DOF> J = Matrix<6, DOF>::Zero();
            
            // Horná časť (poloha)
            J.template block<3, DOF>(0, 0) = J_p;

            // Dolná časť (orientácia - podľa dE_j = dR_j * Rr^T)
            for (std::size_t j = 0; j < DOF; j++) {
                Matrix<3, 3> dE_j = dEr_R[j] * Rr.transpose();
                
                J(3, j) = dE_j(1, 0);
                J(4, j) = dE_j(2, 0);
                J(5, j) = dE_j(2, 1);
            }

            // 5. Regularizácia (Levenberg-Marquardt štýl)
            for (std::size_t k = 0; k < DOF; k++) {
                J(k, k) += lambda;
            }

            // 6. Zostavenie vektora záporných reziduí
            Matrix<6, 1> minus_r;
            minus_r.template block<3, 1>(0, 0) = -rp;
            minus_r.template block<3, 1>(3, 0) = -r0;

            // 7. Výpočet zmeny uhlov (Riešenie sústavy J * delta_theta = minus_r)
            // Pre 6x6 maticu je ColPivHouseholderQR veľmi stabilná voľba
            Matrix<DOF, 1> delta_theta = J.colPivHouseholderQr().solve(minus_r);

            // 8. Update
            theta += delta_theta;
            
            // --- PRIDANÉ: Normalizácia uhlov do intervalu [-pi, pi] na záver iterácie ---
            for (std::size_t j = 0; j < DOF; ++j) {
                // std::remainder vráti zvyšok po delení, centrovaný okolo 0
                theta(j) = std::remainder(theta(j), 2.0f * M_PI);
            }
            // --------------------------------------------------------------------------
        }

        return false; // Nepodarilo sa nájsť riešenie v rámci max_iter
    }
    

private:
  std::array<Matrix<3, 1>, DOF> L;            ///< DH parameter vectors (link translations)
  std::array<const RotationMatrix*, DOF> R_func; ///< Rotation matrix functions for each joint
};

} // namespace Robot
