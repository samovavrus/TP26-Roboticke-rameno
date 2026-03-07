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
 * @brief Rotation matrix about the X-axis from sine and cosine of angle.
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
 * @brief Rotation matrix about the X-axis from angle in radians.
 */
Matrix<3,3> rotX(float theta)
{
    return rotX(sin(theta), cos(theta));
}

/**
 * @brief Derivative of rotation matrix about X-axis w.r.t. angle from sin/cos.
 *
 * This corresponds to d/dθ (rotX(θ)).
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
 * @brief Derivative of rotation matrix about X-axis from angle in radians.
 */
Matrix<3,3> drotX(float theta)
{
    return drotX(sin(theta), cos(theta));
}

/**
 * @brief Rotation matrix about the Y-axis from sine and cosine of angle.
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
 * @brief Rotation matrix about the Y-axis from angle in radians.
 */
Matrix<3,3> rotY(float theta)
{
    return rotY(sin(theta), cos(theta));
}

/**
 * @brief Derivative of rotation matrix about Y-axis w.r.t. angle from sin/cos.
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
 * @brief Derivative of rotation matrix about Y-axis from angle in radians.
 */
Matrix<3,3> drotY(float theta)
{
    return drotY(sin(theta), cos(theta));
}

/**
 * @brief Rotation matrix about the Z-axis from sine and cosine of angle.
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
 * @brief Rotation matrix about the Z-axis from angle in radians.
 */
Matrix<3,3> rotZ(float theta)
{
    return rotZ(sin(theta), cos(theta));
}

/**
 * @brief Derivative of rotation matrix about Z-axis w.r.t. angle from sin/cos.
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
 * @brief Derivative of rotation matrix about Z-axis from angle in radians.
 */
Matrix<3,3> drotZ(float theta)
{
    return drotZ(sin(theta), cos(theta));
}


/**
 * @brief Abstract base class for rotation matrix generation.
 *
 * Provides an interface for:
 * - Computing a rotation matrix from an angle.
 * - Computing its derivative w.r.t. the angle.
 * - Computing both in a single call.
 */
class RotationMatrix {
public:
    virtual ~RotationMatrix() = default;

    /// Compute rotation matrix from angle
    virtual Matrix<3,3> operator()(float theta) const = 0;

    /// Compute derivative of rotation matrix w.r.t. angle
    virtual Matrix<3,3> d(float theta) const = 0;

    /// Compute both rotation matrix and derivative from angle
    virtual void operator()(float theta, Matrix<3,3> R, Matrix<3,3> &dR) const = 0;
};

/**
 * @brief Rotation about X-axis (right-hand rule).
 */
class RotationX : public RotationMatrix {
public:
    Matrix<3,3> operator()(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return rotX(s, c);
    }

    Matrix<3,3> d(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return drotX(s, c);
    }

    void operator()(float theta, Matrix<3,3> R, Matrix<3,3> &dR) const override {
        float s = sin(theta), c = cos(theta);
        R = rotX(s, c);
        dR = drotX(s, c);
    }
};

/**
 * @brief Rotation about Y-axis.
 */
class RotationY : public RotationMatrix {
public:
    Matrix<3,3> operator()(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return rotY(s, c);
    }

    Matrix<3,3> d(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return drotY(s, c);
    }

    void operator()(float theta, Matrix<3,3> R, Matrix<3,3> &dR) const override {
        float s = sin(theta), c = cos(theta);
        R = rotY(s, c);
        dR = drotY(s, c);
    }
};

/**
 * @brief Rotation about Z-axis.
 */
class RotationZ : public RotationMatrix {
public:
    Matrix<3,3> operator()(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return rotZ(s, c);
    }

    Matrix<3,3> d(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return drotZ(s, c);
    }

    void operator()(float theta, Matrix<3,3> R, Matrix<3,3> &dR) const override {
        float s = sin(theta), c = cos(theta);
        R = rotZ(s, c);
        dR = drotZ(s, c);
    }
};




