/**
 * @file geometry.h
 * @brief Rotation matrices and derivatives for robot kinematics.
 */
#pragma once

#include "Eigen/Dense"


/**
 * @brief Type alias for fixed-size Eigen matrix of floats.
 */
template<int Rows, int Cols>
using Matrix = Eigen::Matrix<float, Rows, Cols>;

/**
 * @name Rotation matrix generation and derivatives
 * 
 * Each set of functions provides:
 * - Rotation matrix from sine and cosine values.
 * - Rotation matrix from an angle (radians).
 * - Derivative of rotation matrix w.r.t. angle.
 * 
 * Rotations follow the right-hand rule.
 */
///@{

/**
 * @brief Rotation matrix in the X-axis from pre-computed sine and cosine.
 * @param s_theta Sine of the rotation angle
 * @param c_theta Cosine of the rotation angle
 * @return 3×3 rotation matrix for rotation in X-axis
 * 
 * Avoids redundant trigonometric computations when both sin and cos are available.
 */
Matrix<3,3> rotX(float s_theta, float c_theta)
{
    Matrix<3, 3> R;
    R << 1.0,     0,      0,
         0, c_theta, -s_theta,
         0, s_theta,  c_theta;
    return R;
}

/**
 * @brief Rotation matrix in the X-axis from angle in radians.
 * @param theta Rotation angle in radians
 * @return 3×3 rotation matrix for rotation in X-axis (right-hand rule)
 */
Matrix<3,3> rotX(float theta)
{
    return rotX(sin(theta), cos(theta));
}

/**
 * @brief Derivative of X-axis rotation matrix with respect to angle from pre-computed sin/cos.
 * @param s_theta Sine of the rotation angle
 * @param c_theta Cosine of the rotation angle
 * @return 3×3 derivative matrix: dR_x/dθ
 * 
 * Represents the rate of change of the rotation matrix as the angle changes.
 * Used in Jacobian computations for inverse kinematics.
 */
Matrix<3,3> drotX(float s_theta, float c_theta)
{
    Matrix<3, 3> R;
    R << 0, 0, 0,
         0, -s_theta, -c_theta,
         0,  c_theta, -s_theta;
    return R;
}

/**
 * @brief Derivative of X-axis rotation matrix with respect to angle in radians.
 * @param theta Rotation angle in radians
 * @return 3×3 derivative matrix: dR_x/dθ
 */
Matrix<3,3> drotX(float theta)
{
    return drotX(sin(theta), cos(theta));
}

/**
 * @brief Rotation matrix in the Y-axis from pre-computed sine and cosine.
 * @param s_theta Sine of the rotation angle
 * @param c_theta Cosine of the rotation angle
 * @return 3×3 rotation matrix for rotation in Y-axis
 * 
 * Avoids redundant trigonometric computations when both sin and cos are available.
 */
Matrix<3,3> rotY(float s_theta, float c_theta)
{
    Matrix<3, 3> R;
    R << c_theta, 0, s_theta,
         0,      1.0, 0,
        -s_theta, 0, c_theta;
    return R;
}

/**
 * @brief Rotation matrix in the Y-axis from angle in radians.
 * @param theta Rotation angle in radians
 * @return 3×3 rotation matrix for rotation in Y-axis (right-hand rule)
 */
Matrix<3,3> rotY(float theta)
{
    return rotY(sin(theta), cos(theta));
}

/**
 * @brief Derivative of Y-axis rotation matrix with respect to angle from pre-computed sin/cos.
 * @param s_theta Sine of the rotation angle
 * @param c_theta Cosine of the rotation angle
 * @return 3×3 derivative matrix: dR_y/dθ
 * 
 * Represents the rate of change of the rotation matrix as the angle changes.
 * Used in Jacobian computations for inverse kinematics.
 */
Matrix<3,3> drotY(float s_theta, float c_theta)
{
    Matrix<3, 3> R;
    R << -s_theta, 0, c_theta,
          0,      0, 0,
         -c_theta, 0, -s_theta;
    return R;
}

/**
 * @brief Derivative of Y-axis rotation matrix with respect to angle in radians.
 * @param theta Rotation angle in radians
 * @return 3×3 derivative matrix: dR_y/dθ
 */
Matrix<3,3> drotY(float theta)
{
    return drotY(sin(theta), cos(theta));
}

/**
 * @brief Rotation matrix in the Z-axis from pre-computed sine and cosine.
 * @param s_theta Sine of the rotation angle
 * @param c_theta Cosine of the rotation angle
 * @return 3×3 rotation matrix for rotation in Z-axis
 * 
 * Avoids redundant trigonometric computations when both sin and cos are available.
 */
Matrix<3,3> rotZ(float s_theta, float c_theta)
{
    Matrix<3, 3> R; 
    R << c_theta, -s_theta, 0,
         s_theta,  c_theta, 0,
         0,        0,       1.0;
    return R;   
}

/**
 * @brief Rotation matrix in the Z-axis from angle in radians.
 * @param theta Rotation angle in radians
 * @return 3×3 rotation matrix for rotation in Z-axis (right-hand rule)
 */
Matrix<3,3> rotZ(float theta)
{
    return rotZ(sin(theta), cos(theta));
}

/**
 * @brief Derivative of Z-axis rotation matrix with respect to angle from pre-computed sin/cos.
 * @param s_theta Sine of the rotation angle
 * @param c_theta Cosine of the rotation angle
 * @return 3×3 derivative matrix: dR_z/dθ
 * 
 * Represents the rate of change of the rotation matrix as the angle changes.
 * Used in Jacobian computations for inverse kinematics.
 */
Matrix<3,3> drotZ(float s_theta, float c_theta)
{
    Matrix<3, 3> R;
    R << -s_theta, -c_theta, 0,
          c_theta, -s_theta, 0,
          0,        0,       0;
    return R;                     
}

/**
 * @brief Derivative of Z-axis rotation matrix with respect to angle in radians.
 * @param theta Rotation angle in radians
 * @return 3×3 derivative matrix: dR_z/dθ
 */
Matrix<3,3> drotZ(float theta)
{
    return drotZ(sin(theta), cos(theta));
}


/**
 * @class RotationMatrix
 * @brief Abstract base class for joint rotation matrix generation
 * 
 * Defines an interface for generating rotation matrices in a fixed axis
 * (X, Y, or Z) and computing their derivatives with respect to joint angles.
 * 
 * Implementations are used in robot kinematics solvers to:
 * - Compute cumulative rotation transformations
 * - Calculate Jacobian matrices for inverse kinematics
 * - Perform forward and inverse kinematic analyses
 */
class RotationMatrix {
public:
    /**
     * @brief Virtual destructor
     */
    virtual ~RotationMatrix() = default;

    /**
     * @brief Compute rotation matrix from angle
     * @param theta Joint angle in radians
     * @return 3×3 rotation matrix for this joint
     */
    virtual Matrix<3,3> operator()(float theta) const = 0;

    /**
     * @brief Compute derivative of rotation matrix with respect to angle
     * @param theta Joint angle in radians
     * @return 3×3 derivative matrix: dR/dθ
     */
    virtual Matrix<3,3> d(float theta) const = 0;

    /**
     * @brief Compute both rotation matrix and derivative in one call
     * @param theta Joint angle in radians
     * @param R [out] Computed rotation matrix (passed by value, updated in call)
     * @param dR [out] Computed derivative matrix (passed by reference)
     * 
     * @note The parameter R is intentionally passed by value for API consistency.
     * Only dR is modified within this function. This is an optimization pattern
     * where both computations share sin/cos evaluation.
     */
    virtual void operator()(float theta, Matrix<3,3> R, Matrix<3,3> &dR) const = 0;
};

/**
 * @class RotationX
 * @brief Rotation in X-axis (right-hand rule: thumb along +X)
 * 
 * Concrete implementation of RotationMatrix for rotations in the X-axis.
 * Common for shoulder/base joints in robotic manipulators.
 */
class RotationX : public RotationMatrix {
public:
    /**
     * @brief Compute X-axis rotation matrix
     * @param theta Rotation angle in radians
     * @return 3×3 rotation matrix in X-axis
     */
    Matrix<3,3> operator()(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return rotX(s, c);
    }

    /**
     * @brief Compute derivative of X-axis rotation matrix
     * @param theta Rotation angle in radians
     * @return 3×3 derivative matrix: dR_x/dθ
     */
    Matrix<3,3> d(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return drotX(s, c);
    }

    /**
     * @brief Compute both X-axis rotation matrix and derivative
     * @param theta Rotation angle in radians
     * @param R [out] Computed rotation matrix
     * @param dR [out] Computed derivative matrix
     */
    void operator()(float theta, Matrix<3,3> R, Matrix<3,3> &dR) const override {
        float s = sin(theta), c = cos(theta);
        R = rotX(s, c);
        dR = drotX(s, c);
    }
};

/**
 * @class RotationY
 * @brief Rotation in Y-axis (right-hand rule: thumb along +Y)
 * 
 * Concrete implementation of RotationMatrix for rotations in the Y-axis.
 * Common for elbow and wrist pitch joints in robotic manipulators.
 */
class RotationY : public RotationMatrix {
public:
    /**
     * @brief Compute Y-axis rotation matrix
     * @param theta Rotation angle in radians
     * @return 3×3 rotation matrix in Y-axis
     */
    Matrix<3,3> operator()(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return rotY(s, c);
    }

    /**
     * @brief Compute derivative of Y-axis rotation matrix
     * @param theta Rotation angle in radians
     * @return 3×3 derivative matrix: dR_y/dθ
     */
    Matrix<3,3> d(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return drotY(s, c);
    }

    /**
     * @brief Compute both Y-axis rotation matrix and derivative
     * @param theta Rotation angle in radians
     * @param R [out] Computed rotation matrix
     * @param dR [out] Computed derivative matrix
     */
    void operator()(float theta, Matrix<3,3> R, Matrix<3,3> &dR) const override {
        float s = sin(theta), c = cos(theta);
        R = rotY(s, c);
        dR = drotY(s, c);
    }
};

/**
 * @class RotationZ
 * @brief Rotation in Z-axis (right-hand rule: thumb along +Z)
 * 
 * Concrete implementation of RotationMatrix for rotations in the Z-axis.
 * Common for waist/azimuth joints and wrist yaw in robotic manipulators.
 */
class RotationZ : public RotationMatrix {
public:
    /**
     * @brief Compute Z-axis rotation matrix
     * @param theta Rotation angle in radians
     * @return 3×3 rotation matrix in Z-axis
     */
    Matrix<3,3> operator()(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return rotZ(s, c);
    }

    /**
     * @brief Compute derivative of Z-axis rotation matrix
     * @param theta Rotation angle in radians
     * @return 3×3 derivative matrix: dR_z/dθ
     */
    Matrix<3,3> d(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return drotZ(s, c);
    }

    /**
     * @brief Compute both Z-axis rotation matrix and derivative
     * @param theta Rotation angle in radians
     * @param R [out] Computed rotation matrix
     * @param dR [out] Computed derivative matrix
     */
    void operator()(float theta, Matrix<3,3> R, Matrix<3,3> &dR) const override {
        float s = sin(theta), c = cos(theta);
        R = rotZ(s, c);
        dR = drotZ(s, c);
    }
};




