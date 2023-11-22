#pragma once

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <complex>
#include <vector>

class PushRecoveryControl {
 private:
  static const inline std::complex<double> i{0.0, 1.0};
  // EOM of 3 DOF model
  // Mass in kg, length in meter
  double alpha = 0.7;
  double m1 = 5.9117,
        m2 = 4.2554,
        m3 = 10.19329;

  double lc1 = 0.15149,
        lc2 = 0.24517,
        lc3 = 0.1585;

  double l1 = 0.3715,
        l2 = 0.49478,
        l3 = 0.32662;

  double g = 9.81;
  double I1 = 0.0222,
        I2 = 0.01009,
        I3 = 0.0219;

  // Desired position, velocity and acceleration of location of the com

  double theta1_d = 0,
        theta2_d = 0,
        theta3_d = 0;

  double theta1_dot_d = 0,
        theta2_dot_d = 0,
        theta3_dot_d = 0;

  double theta1_dddot = 0,
        theta2_dddot = 0,
        theta3_dddot = 0;

  double x_com_d = 0,
        x_com_ddot = 0,
        x_com_dddot = 0;

  // Values
  double theta1, theta2, theta3 = 0;
  double theta1_dot, theta2_dot, theta3_dot = 0;
  double X_COM, X_dot_COM = 0;
  Eigen::MatrixXd ModelCalculation();
  Eigen::MatrixXd CalculateCOM();
  template <typename T>
  int sign (const T &val) { return (val > 0) - (val < 0); }
  Eigen::MatrixXd SMCController(Eigen::RowVectorXf cop,
                                Eigen::MatrixXd J_X_COM,
                                Eigen::MatrixXd J_X_COM_dot);
  Eigen::MatrixXd SMCPostureCorrection();
  double constrainAngle(double x);

 public:
  Eigen::MatrixXd GetTorque(double x_cop,
                            std::vector<double> position,
                            std::vector<double> velocity);
};
