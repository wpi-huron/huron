#include <cmath>
#include "huron/control/push_recovery.h"

// Helper function
double PushRecoveryControl::constrainAngle(double x) {
  x = std::fmod(x + atan(1)*4, 2*atan(1)*4);
  if (x < 0)
    x += 2*atan(1)*4;   // pi
  return x - atan(1)*4;
}

Eigen::MatrixXd PushRecoveryControl::ModelCalculation() {
  // Note the assigning values:
  double q1 = theta1;
  double q2 = theta2;
  double q3 = theta3;
  double q_dot1 = theta1_dot;
  double q_dot2 = theta2_dot;
  double q_dot3 = theta3_dot;
  double r1 = lc1;
  double r2 = lc2;
  double r3 = lc3;

  Eigen::MatrixXd mat_m(3, 3);
  mat_m << I1 + I2 + I3 + pow(l1, 2)*m2 + pow(l1, 2)*m3 +
             pow(l2, 2)*m3 + m1*pow(r1, 2) + m2*pow(r2, 2) +
             m3*pow(r3, 2) + 2*l1*m3*r3*cos(q2 + q3) + 2*l1*l2*m3*cos(q2) +
             2*l1*m2*r2*cos(q2) + 2*l2*m3*r3*cos(q3),
    m3*pow(l2, 2) + 2*m3*cos(q3)*l2*r3 + l1*m3*cos(q2)*l2 +
      m2*pow(r2, 2) + l1*m2*cos(q2)*r2 + m3*pow(r3, 2) +
      l1*m3*cos(q2 + q3)*r3 + I2 + I3,
    I3 + m3*pow(r3, 2) + l1*m3*r3*cos(q2 + q3) + l2*m3*r3*cos(q3),
    m3*pow(l2, 2) + 2*m3*cos(q3)*l2*r3 + l1*m3*cos(q2)*l2 +
      m2*pow(r2, 2) + l1*m2*cos(q2)*r2 + m3*pow(r3, 2) +
      l1*m3*cos(q2 + q3)*r3 + I2 + I3,
    m3*pow(l2, 2) + 2*m3*cos(q3)*l2*r3 +
      m2*pow(r2, 2) + m3*pow(r3, 2) + I2 + I3,
    m3*pow(r3, 2) + l2*m3*cos(q3)*r3 + I3,
    I3 + m3*pow(r3, 2) + l1*m3*r3*cos(q2 + q3) + l2*m3*r3*cos(q3),
    m3*pow(r3, 2) + l2*m3*cos(q3)*r3 + I3,
    m3*pow(r3, 2) + I3;


  Eigen::MatrixXd mat_c(3, 1);
  mat_c << - l1*m3*(pow(q_dot2, 2))*r3*sin(q2 + q3)
             - l1*m3*pow(q_dot3, 2)*r3*sin(q2 + q3)
             - l1*l2*m3*pow(q_dot2, 2)*sin(q2)
             - l1*m2*pow(q_dot2, 2)*r2*sin(q2)
             - l2*m3*pow(q_dot3, 2)*r3*sin(q3)
             - 2*l1*m3*q_dot1*q_dot2*r3*sin(q2 + q3)
             - 2*l1*m3*q_dot1*q_dot3*r3*sin(q2 + q3)
             - 2*l1*m3*q_dot2*q_dot3*r3*sin(q2 + q3)
             - 2*l1*l2*m3*q_dot1*q_dot2*sin(q2)
             - 2*l1*m2*q_dot1*q_dot2*r2*sin(q2)
             - 2*l2*m3*q_dot1*q_dot3*r3*sin(q3)
             - 2*l2*m3*q_dot2*q_dot3*r3*sin(q3),
    l1*m3*pow(q_dot1, 2)*r3*sin(q2 + q3)
      + l1*l2*m3*pow(q_dot1, 2)*sin(q2)
      + l1*m2*pow(q_dot1, 2)*r2*sin(q2)
      - l2*m3*pow(q_dot3, 2)*r3*sin(q3)
      - 2*l2*m3*q_dot1*q_dot3*r3*sin(q3)
      - 2*l2*m3*q_dot2*q_dot3*r3*sin(q3),
    l1*m3*pow(q_dot1, 2)*r3*sin(q2 + q3) + l2*m3*pow(q_dot1, 2)*r3*sin(q3)
      + l2*m3*pow(q_dot2, 2)*r3*sin(q3) + 2*l2*m3*q_dot1*q_dot2*r3*sin(q3);

  Eigen::MatrixXd mat_g(3, 1);
  mat_g << - g*l2*m3*sin(q1 + q2) - g*m2*r2*sin(q1 + q2)
             - g*l1*m2*sin(q1) - g*l1*m3*sin(q1)
             - g*m1*r1*sin(q1) - g*m3*r3*sin(q1 + q2 + q3),
    - g*l2*m3*sin(q1 + q2) - g*m2*r2*sin(q1 + q2) - g*m3*r3*sin(q1 + q2 + q3),
    -g*m3*r3*sin(q1 + q2 + q3);

  Eigen::MatrixXd result(3, 5);
  result << mat_m, mat_c, mat_g;
  return result;
}

Eigen::MatrixXd PushRecoveryControl::CalculateCOM() {
  X_COM = -1*((lc1*sin(theta1))*m1
                + (l1*sin(theta1)+lc2*sin(theta1+theta2))*m2
                + (l1*sin(theta1)+l2*sin(theta1+theta2)
                   +lc3*sin(theta1+theta2+theta3))*m3) / (m1+m2+m3);
  std::cout << "X_COM =" << std::endl <<
    X_COM << std::endl;
  X_dot_COM = -1* (m1*(theta1_dot*lc1*cos(theta1))
                    + m2*(theta1_dot*l1*cos(theta1)
                            +(theta1_dot+theta2_dot)*lc2*cos(theta1+theta2))
                    + m3*(theta1_dot*l1*cos(theta1)
                            +(theta1_dot+theta2_dot)*l2*cos(theta1+theta2)
                            + (theta1_dot+theta2_dot+theta3_dot)
                                *lc3*cos(theta1+theta2+theta3)))
              /(m1+m2+m3);

  Eigen::MatrixXd J_X_COM(1, 3);

  J_X_COM << (m3*(l2*cos(theta1 + theta2)
                    + l1*cos(theta1)
                    + lc3*cos(theta1 + theta2 + theta3))
              + m2*(lc2*cos(theta1 + theta2)
                      + l1*cos(theta1))
              + lc1*m1*cos(theta1))/(m1 + m2 + m3),
    (m3*(l2*cos(theta1 + theta2) + lc3*cos(theta1 + theta2 + theta3))
     + lc2*m2*cos(theta1 + theta2))/(m1 + m2 + m3),
    (lc3*m3*cos(theta1 + theta2 + theta3))/(m1 + m2 + m3);

  // Jacobian Matrix of X-COM Jx
  J_X_COM = -1* J_X_COM;

  Eigen::MatrixXd J_X_COM_dot(1, 3);
  J_X_COM_dot << (-m1*theta1_dot*lc1*sin(theta1)
                  + m2*(-l1*theta1_dot*sin(theta1)
                          - (theta1_dot+theta2_dot)*lc2*sin(theta1+theta2))
                  + m3*(- l2 * (theta1_dot+theta2_dot) * sin(theta1+theta2)
                          - l1 * theta1_dot * sin(theta1)
                          - (theta1_dot+theta2_dot+theta3_dot)
                              * lc3*sin(theta1+theta2+theta3)))/(m1+m2+m3),
    (m3*(-l2*(theta1_dot+theta2_dot)*sin(theta1+theta2)
           - lc3*(theta1_dot+theta2_dot+theta3_dot)*sin(theta1+theta2+theta3))
     -lc2*(theta1_dot+theta2_dot)*m2*sin(theta1+theta2))/(m1+m2+m3),
    (-lc3*(theta1_dot+theta2_dot+theta3_dot)*m3
     *sin(theta1+theta2+theta3))/(m1+m2+m3);

  J_X_COM_dot = -1 * J_X_COM_dot;

  Eigen::MatrixXd J_Z_COM(1, 3);
  J_Z_COM << (m3*(-l2*sin(theta1 + theta2)
                    - l1*sin(theta1)
                    - lc3*sin(theta1 + theta2 + theta3))
              + m2*(-lc2*sin(theta1 + theta2)
                      - l1*sin(theta1))
              - lc1*m1*sin(theta1))/(m1 + m2 + m3),
    (m3*(-l2*sin(theta1 + theta2)
           - lc3*sin(theta1 + theta2 + theta3))
     - lc2*m2*sin(theta1 + theta2))
      /(m1 + m2 + m3),
    (-lc3*m3*sin(theta1 + theta2 + theta3))
      /(m1 + m2 + m3);

  Eigen::MatrixXd J_COM(2, 3);
  // Linear part of Jacobian matrix of COM
  J_COM << J_X_COM, J_Z_COM;



  Eigen::MatrixXd J_W_COM(1, 3);
  J_W_COM << 1, 1, 1;
  Eigen::MatrixXd J_total_COM(2, 3);
  J_total_COM << J_X_COM, J_W_COM;
  // linear and angular part of Jacobian of COM
  Eigen::MatrixXd J_total_COM_dot(2, 3), J_total_COM_pseduo(3, 2);
  J_total_COM_dot << J_X_COM_dot, 0, 0, 0;
  // time derivative of total Jacobian of COM
  J_total_COM_pseduo =
    J_total_COM.completeOrthogonalDecomposition().pseudoInverse();
  // pseduo inverse of the total Jacobian of COM

  Eigen::MatrixXd result(2, 3);
  result << J_X_COM, J_X_COM_dot;
  return result;
}

Eigen::MatrixXd PushRecoveryControl::SMCController(
  const Eigen::Vector2d& cop,
  const Eigen::MatrixXd& J_X_COM,
  const Eigen::MatrixXd& J_X_COM_dot) {
  // SMC for Linear motion
  // rate of change of linear momentum

  double error_in_x = X_COM;
  double error_dot_in_x = X_dot_COM;

  Eigen::MatrixXd theta_dot(3, 1);
  theta_dot << theta1_dot, theta2_dot, theta3_dot;

  double L11 = 1.9, k11 = -1, p11 = 1, c11 = 7,
        Q11 = 0.001, a11 = 2, z11 = 2.5;
  double s_linear_motion = error_dot_in_x + L11*error_in_x;

  double f11 = c11 * (1 - exp(k11*pow(std::abs(s_linear_motion), p11)));
  double s_linear_motion_dot = -Q11
                                * pow(std::abs(s_linear_motion), f11)
                                * sign(s_linear_motion)
                              - z11 * pow(std::abs(s_linear_motion), a11)
                                  * s_linear_motion;

  // Angular Momentum control part
  Eigen::MatrixXd A_theta(1, 3);
  A_theta << I1 + I2 + I3 - pow(m3*(l2*sin(theta1 + theta2)
                                      + l1*sin(theta1)
                                      + lc3*sin(theta1 + theta2 + theta3))
                                  + m2*(lc2*sin(theta1 + theta2)
                                          + l1*sin(theta1))
                                  + lc1*m1*sin(theta1), 2)
                              /(m1 + m2 + m3)
               + pow(l1, 2)*m2 + pow(l1, 2)*m3
               + pow(l2, 2)*m3 + pow(lc1, 2)*m1
               + pow(lc2, 2)*m2 + pow(lc3, 2)*m3
               - pow(m3*(l2*cos(theta1 + theta2)
                           + l1*cos(theta1)
                           + lc3*cos(theta1 + theta2 + theta3))
                       + m2*(lc2*cos(theta1 + theta2)
                               + l1*cos(theta1))
                       + lc1*m1*cos(theta1), 2)/(m1 + m2 + m3)
               + 2*l1*lc3*m3*cos(theta2 + theta3) + 2*l1*l2*m3*cos(theta2)
               + 2*l1*lc2*m2*cos(theta2) + 2*l2*lc3*m3*cos(theta3),

    I2 + I3 + pow(l2,  2)*m3
      + pow(lc2, 2)*m2 + pow(lc3, 2)*m3
      - ((m3*(l2*sin(theta1 + theta2)
                + l1*sin(theta1)
                + lc3*sin(theta1 + theta2 + theta3))
          + m2*(lc2*sin(theta1 + theta2)
                  + l1*sin(theta1))
          + lc1*m1*sin(theta1))
         *(lc3*m3*sin(theta1 + theta2 + theta3)
            + l2*m3*sin(theta1 + theta2)
            + lc2*m2*sin(theta1 + theta2)))
          /(m1 + m2 + m3)
      - ((lc3*m3*cos(theta1 + theta2 + theta3)
          + l2*m3*cos(theta1 + theta2)
          + lc2*m2*cos(theta1 + theta2))
         *(m3*(l2*cos(theta1 + theta2)
                  + l1*cos(theta1)
                  + lc3*cos(theta1 + theta2 + theta3))
            + m2*(lc2*cos(theta1 + theta2)
                    + l1*cos(theta1))
            + lc1*m1*cos(theta1)))
          /(m1 + m2 + m3)
    + l1*lc3*m3*cos(theta2 + theta3)
      + l1*l2*m3*cos(theta2)
      + l1*lc2*m2*cos(theta2)
      + 2*l2*lc3*m3*cos(theta3),

    I3 + pow(lc3, 2)*m3
      + l1*lc3*m3*cos(theta2 + theta3)
      + l2*lc3*m3*cos(theta3)
      - (lc3*m3*cos(theta1 + theta2 + theta3)
         *(m3*(l2*cos(theta1 + theta2)
                  + l1*cos(theta1)
                  + lc3*cos(theta1 + theta2 + theta3))
            + m2*(lc2*cos(theta1 + theta2)
                    + l1*cos(theta1))
            + lc1*m1*cos(theta1)))
          /(m1 + m2 + m3)
      - (lc3*m3*sin(theta1 + theta2 + theta3)
         *(m3*(l2*sin(theta1 + theta2)
                  + l1*sin(theta1)
                  + lc3*sin(theta1 + theta2 + theta3))
            + m2*(lc2*sin(theta1 + theta2) + l1*sin(theta1))
            + lc1*m1*sin(theta1)))/(m1 + m2 + m3);

  Eigen::MatrixXd A_theta_pseudo(3, 1);
  A_theta_pseudo << (I1*m1 + I1*m2 + I2*m1 + I1*m3 + I2*m2
                     + I3*m1 + I2*m3 + I3*m2 + I3*m3
                     + pow(l1, 2)*m1*m2 + pow(l1, 2)*m1*m3
                     + pow(l2, 2)*m1*m3 + pow(l2, 2)*m2*m3
                     + pow(lc1, 2)*m1*m2 + pow(lc1, 2)*m1*m3
                     + pow(lc2, 2)*m1*m2 + pow(lc2, 2)*m2*m3
                     + pow(lc3, 2)*m1*m3 + pow(lc3, 2)*m2*m3
                     - 2*l1*lc1*m1*m2 - 2*l1*lc1*m1*m3 - 2*l2*lc2*m2*m3
                     + 2*l1*lc3*m1*m3*cos(theta2 + theta3)
                     - 2*lc1*lc3*m1*m3*cos(theta2 + theta3)
                     + 2*l1*l2*m1*m3*cos(theta2) + 2*l1*lc2*m1*m2*cos(theta2)
                     - 2*l2*lc1*m1*m3*cos(theta2)
                     + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3)
                     - 2*lc1*lc2*m1*m2*cos(theta2)
                     - 2*lc2*lc3*m2*m3*cos(theta3))/((m1 + m2 + m3)
                       * (pow(I1*m1 + I1*m2 + I2*m1 + I1*m3
                                + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3
                                + pow(l1, 2)*m1*m2 + pow(l1, 2)*m1*m3
                                + pow(l2, 2)*m1*m3 + pow(l2, 2)*m2*m3
                                + pow(lc1, 2)*m1*m2 + pow(lc1, 2)*m1*m3
                                + pow(lc2, 2)*m1*m2 + pow(lc2, 2)*m2*m3
                                + pow(lc3, 2)*m1*m3 + pow(lc3, 2)*m2*m3
                                - 2*l1*lc1*m1*m2 - 2*l1*lc1*m1*m3
                                - 2*l2*lc2*m2*m3
                                + 2*l1*lc3*m1*m3*cos(theta2 + theta3)
                                - 2*lc1*lc3*m1*m3*cos(theta2 + theta3)
                                + 2*l1*l2*m1*m3*cos(theta2)
                                + 2*l1*lc2*m1*m2*cos(theta2)
                                - 2*l2*lc1*m1*m3*cos(theta2)
                                + 2*l2*lc3*m1*m3*cos(theta3)
                                + 2*l2*lc3*m2*m3*cos(theta3)
                                - 2*lc1*lc2*m1*m2*cos(theta2)
                                - 2*lc2*lc3*m2*m3*cos(theta3), 2)
                            /pow((m1 + m2 + m3), 2)
                          + pow(I3*m1 + I3*m2 + I3*m3
                                  + pow(lc3, 2)*m1*m3
                                  + pow(lc3, 2)*m2*m3
                                  + l1*lc3*m1*m3*cos(theta2 + theta3)
                                  - lc1*lc3*m1*m3*cos(theta2 + theta3)
                                  + l2*lc3*m1*m3*cos(theta3)
                                  + l2*lc3*m2*m3*cos(theta3)
                                  - lc2*lc3*m2*m3*cos(theta3), 2)
                              / pow((m1 + m2 + m3), 2) + pow(I2*m1
                                + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3
                                + pow(l2, 2)*m1*m3
                                + pow(l2, 2)*m2*m3
                                + pow(lc2, 2)*m1*m2
                                + pow(lc2, 2)*m2*m3
                                + pow(lc3, 2)*m1*m3
                                + pow(lc3, 2)*m2*m3
                                - 2*l2*lc2*m2*m3
                                + l1*lc3*m1*m3*cos(theta2 + theta3)
                                - lc1*lc3*m1*m3*cos(theta2 + theta3)
                                + l1*l2*m1*m3*cos(theta2)
                                + l1*lc2*m1*m2*cos(theta2)
                                - l2*lc1*m1*m3*cos(theta2)
                                + 2*l2*lc3*m1*m3*cos(theta3)
                                + 2*l2*lc3*m2*m3*cos(theta3)
                                - lc1*lc2*m1*m2*cos(theta2)
                                - 2*lc2*lc3*m2*m3*cos(theta3), 2)
                            /pow((m1 + m2 + m3), 2))),
    (I2*m1 + I2*m2 + I3*m1 + I2*m3 + I3*m2
     + I3*m3 + pow(l2, 2)*m1*m3 + pow(l2, 2)*m2*m3
     + pow(lc2, 2)*m1*m2 + pow(lc2,  2)*m2*m3
     + pow(lc3,  2)*m1*m3 + pow(lc3, 2)*m2*m3
     - 2*l2*lc2*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3)
     - lc1*lc3*m1*m3*cos(theta2 + theta3)
     + l1*l2*m1*m3*cos(theta2)
     + l1*lc2*m1*m2*cos(theta2)
     - l2*lc1*m1*m3*cos(theta2)
     + 2*l2*lc3*m1*m3*cos(theta3)
     + 2*l2*lc3*m2*m3*cos(theta3)
     - lc1*lc2*m1*m2*cos(theta2)
     - 2*lc2*lc3*m2*m3*cos(theta3))/((m1 + m2 + m3)
         *pow(I1*m1 + I1*m2 + I2*m1 + I1*m3 + I2*m2
                 + I3*m1 + I2*m3 + I3*m2 + I3*m3
                 + pow(l1, 2)*m1*m2 + pow(l1, 2)*m1*m3
                 + pow(l2, 2)*m1*m3 + pow(l2, 2)*m2*m3
                 + pow(lc1, 2)*m1*m2 + pow(lc1, 2)*m1*m3
                 + pow(lc2, 2)*m1*m2 + pow(lc2, 2)*m2*m3
                 + pow(lc3, 2)*m1*m3 + pow(lc3, 2)*m2*m3
                 - 2*l1*lc1*m1*m2
                 - 2*l1*lc1*m1*m3
                 - 2*l2*lc2*m2*m3
                 + 2*l1*lc3*m1*m3*cos(theta2 + theta3)
                 - 2*lc1*lc3*m1*m3*cos(theta2 + theta3)
                 + 2*l1*l2*m1*m3*cos(theta2)
                 + 2*l1*lc2*m1*m2*cos(theta2)
                 - 2*l2*lc1*m1*m3*cos(theta2)
                 + 2*l2*lc3*m1*m3*cos(theta3)
                 + 2*l2*lc3*m2*m3*cos(theta3)
                 - 2*lc1*lc2*m1*m2*cos(theta2)
                 - 2*lc2*lc3*m2*m3*cos(theta3), 2)
         /pow((m1 + m2 + m3), 2)
       + pow(I3*m1 + I3*m2 + I3*m3
               + pow(lc3, 2)*m1*m3
               + pow(lc3, 2)*m2*m3
               + l1*lc3*m1*m3*cos(theta2 + theta3)
               - lc1*lc3*m1*m3*cos(theta2 + theta3)
               + l2*lc3*m1*m3*cos(theta3)
               + l2*lc3*m2*m3*cos(theta3)
               - lc2*lc3*m2*m3*cos(theta3), 2)
           /pow((m1 + m2 + m3), 2) +
       pow(I2*m1 + I2*m2 + I3*m1
             + I2*m3 + I3*m2 + I3*m3
             + pow(l2, 2)*m1*m3 + pow(l2, 2)*m2*m3
             + pow(lc2, 2)*m1*m2 + pow(lc2, 2)*m2*m3
             + pow(lc3, 2)*m1*m3 + pow(lc3, 2)*m2*m3
             - 2*l2*lc2*m2*m3
             + l1*lc3*m1*m3*cos(theta2 + theta3)
             - lc1*lc3*m1*m3*cos(theta2 + theta3)
             + l1*l2*m1*m3*cos(theta2)
             + l1*lc2*m1*m2*cos(theta2)
             - l2*lc1*m1*m3*cos(theta2)
             + 2*l2*lc3*m1*m3*cos(theta3)
             + 2*l2*lc3*m2*m3*cos(theta3)
             - lc1*lc2*m1*m2*cos(theta2)
             - 2*lc2*lc3*m2*m3*cos(theta3), 2)
         /pow((m1 + m2 + m3), 2)),
    (I3*m1 + I3*m2 + I3*m3
     + pow(lc3, 2)*m1*m3
     + pow(lc3, 2)*m2*m3
     + l1*lc3*m1*m3*cos(theta2 + theta3)
     - lc1*lc3*m1*m3*cos(theta2 + theta3)
     + l2*lc3*m1*m3*cos(theta3)
     + l2*lc3*m2*m3*cos(theta3)
     - lc2*lc3*m2*m3*cos(theta3))
      /((m1 + m2 + m3)*(pow(I1*m1
                                 + I1*m2 + I2*m1 + I1*m3 + I2*m2 + I3*m1
                                 + I2*m3 + I3*m2 + I3*m3
                                 + pow(l1, 2)*m1*m2
                                 + pow(l1, 2)*m1*m3
                                 + pow(l2, 2)*m1*m3
                                 + pow(l2, 2)*m2*m3
                                 + pow(lc1, 2)*m1*m2
                                 + pow(lc1, 2)*m1*m3
                                 + pow(lc2, 2)*m1*m2
                                 + pow(lc2, 2)*m2*m3
                                 + pow(lc3, 2)*m1*m3
                                 + pow(lc3, 2)*m2*m3
                                 - 2*l1*lc1*m1*m2
                                 - 2*l1*lc1*m1*m3
                                 - 2*l2*lc2*m2*m3
                                 + 2*l1*lc3*m1*m3*cos(theta2 + theta3)
                                 - 2*lc1*lc3*m1*m3*cos(theta2 + theta3)
                                 + 2*l1*l2*m1*m3*cos(theta2)
                                 + 2*l1*lc2*m1*m2*cos(theta2)
                                 - 2*l2*lc1*m1*m3*cos(theta2)
                                 + 2*l2*lc3*m1*m3*cos(theta3)
                                 + 2*l2*lc3*m2*m3*cos(theta3)
                                 - 2*lc1*lc2*m1*m2*cos(theta2)
                                 - 2*lc2*lc3*m2*m3*cos(theta3), 2)
                             /pow((m1 + m2 + m3), 2)
                           + pow(I3*m1 + I3*m2 + I3*m3
                                   + pow(lc3, 2)*m1*m3
                                   + pow(lc3, 2)*m2*m3
                                   + l1*lc3*m1*m3*cos(theta2 + theta3)
                                   - lc1*lc3*m1*m3*cos(theta2 + theta3)
                                   + l2*lc3*m1*m3*cos(theta3)
                                   + l2*lc3*m2*m3*cos(theta3)
                                   - lc2*lc3*m2*m3*cos(theta3), 2)
                               /pow((m1 + m2 + m3), 2)
                           + pow(I2*m1 + I2*m2 + I3*m1
                                   + I2*m3 + I3*m2 + I3*m3
                                   + pow(l2, 2)*m1*m3
                                   + pow(l2, 2)*m2*m3
                                   + pow(lc2, 2)*m1*m2
                                   + pow(lc2, 2)*m2*m3
                                   + pow(lc3, 2)*m1*m3
                                   + pow(lc3, 2)*m2*m3
                                   - 2*l2*lc2*m2*m3
                                   + l1*lc3*m1*m3*cos(theta2 + theta3)
                                   - lc1*lc3*m1*m3*cos(theta2 + theta3)
                                   + l1*l2*m1*m3*cos(theta2)
                                   + l1*lc2*m1*m2*cos(theta2)
                                   - l2*lc1*m1*m3*cos(theta2)
                                   + 2*l2*lc3*m1*m3*cos(theta3)
                                   +2*l2*lc3*m2*m3*cos(theta3)
                                   - lc1*lc2*m1*m2*cos(theta2)
                                   - 2*lc2*lc3*m2*m3*cos(theta3), 2)
                               /pow((m1 + m2 + m3), 2)));

  Eigen::MatrixXd A_theta_dot(1, 3);
  A_theta_dot << -(2*l1*lc3*m1*m3*theta2_dot*sin(theta2 + theta3)
                   + 2*l1*lc3*m1*m3*theta3_dot*sin(theta2 + theta3)
                   - 2*lc1*lc3*m1*m3*theta2_dot*sin(theta2 + theta3)
                   - 2*lc1*lc3*m1*m3*theta3_dot*sin(theta2 + theta3)
                   + 2*l1*l2*m1*m3*theta2_dot*sin(theta2)
                   + 2*l1*lc2*m1*m2*theta2_dot*sin(theta2)
                   - 2*l2*lc1*m1*m3*theta2_dot*sin(theta2)
                   + 2*l2*lc3*m1*m3*theta3_dot*sin(theta3)
                   + 2*l2*lc3*m2*m3*theta3_dot*sin(theta3)
                   - 2*lc1*lc2*m1*m2*theta2_dot*sin(theta2)
                   - 2*lc2*lc3*m2*m3*theta3_dot*sin(theta3))/(m1 + m2 + m3),
    -(l1*lc3*m1*m3*theta2_dot*sin(theta2 + theta3)
      + l1*lc3*m1*m3*theta3_dot*sin(theta2 + theta3)
      - lc1*lc3*m1*m3*theta2_dot*sin(theta2 + theta3)
      - lc1*lc3*m1*m3*theta3_dot*sin(theta2 + theta3)
      + l1*l2*m1*m3*theta2_dot*sin(theta2)
      + l1*lc2*m1*m2*theta2_dot*sin(theta2)
      - l2*lc1*m1*m3*theta2_dot*sin(theta2)
      + 2*l2*lc3*m1*m3*theta3_dot*sin(theta3)
      + 2*l2*lc3*m2*m3*theta3_dot*sin(theta3)
      - lc1*lc2*m1*m2*theta2_dot*sin(theta2)
      - 2*lc2*lc3*m2*m3*theta3_dot*sin(theta3))/(m1 + m2 + m3),
    -(lc3*m3*(l1*m1*theta2_dot*sin(theta2 + theta3)
                  + l1*m1*theta3_dot*sin(theta2 + theta3)
                  - lc1*m1*theta2_dot*sin(theta2 + theta3)
                  - lc1*m1*theta3_dot*sin(theta2 + theta3)
                  + l2*m1*theta3_dot*sin(theta3)
                  + l2*m2*theta3_dot*sin(theta3)
                  - lc2*m2*theta3_dot*sin(theta3)))/(m1 + m2 + m3);

  double k_tunning = 2;  // For second option
  double Desired_Angular_Momentum =
    k_tunning * ((cop(0)) - X_COM) *  (1);

  // Desired accleration to achieve both linear and angular tasks without LPF
  Eigen::MatrixXd mat(2, 3);
  mat << J_X_COM, A_theta;
  Eigen::MatrixXd mat_pinv(3, 2);
  mat_pinv = mat.completeOrthogonalDecomposition().pseudoInverse();


  Eigen::MatrixXd mat2(2, 1);
  Eigen::MatrixXd mult_temp(1, 1);
  mult_temp << J_X_COM_dot*theta_dot;
  double mult = mult_temp(0);
  Eigen::MatrixXd mult2_temp(1, 1);
  mult2_temp << A_theta_dot * theta_dot;
  double mult2 = mult2_temp(0);

  mat2 << (s_linear_motion_dot - L11 * error_dot_in_x - mult),
    (Desired_Angular_Momentum - (mult2));
  Eigen::MatrixXd Desired_Theta_ddot(3, 3);
  Desired_Theta_ddot =  mat_pinv *  mat2;
  Eigen::MatrixXd mat_smc(3, 1);
  mat_smc << Desired_Theta_ddot(0, 0),
    Desired_Theta_ddot(1, 0), Desired_Theta_ddot(2, 0);
  return mat_smc;
}

Eigen::MatrixXd PushRecoveryControl::SMCPostureCorrection() {
  Eigen::MatrixXd theta(3, 1), error_in_q(3, 1);
  theta << theta1, theta2, theta3;

  Eigen::MatrixXd theta_dot(3, 1), errordot_in_q(3, 1);
  theta_dot << theta1_dot, theta2_dot, theta3_dot;

  error_in_q = theta;
  errordot_in_q = theta_dot;

  // For above 80 N
  Eigen::MatrixXd lamda_of_q(3, 3), k_of_q(3, 3);
  lamda_of_q << 2.8, 0, 0,
    0, 2.8, 0,
    0, 0, 2.8;
  k_of_q << 3, 0, 0,
    0, 3, 0,
    0, 0, 3;

  double w1 = 1, w2 = 1, w3 = 1;

  Eigen::MatrixXd s_of_q(3, 1);
  s_of_q = errordot_in_q + lamda_of_q * error_in_q;

  double phi1 = 0.002, phi2 = 0.02, phi3 = 0.02;
  double sat1 = 0, sat2 = 0, sat3 = 0;

  if (std::abs(s_of_q(0, 0)) >= phi1) {  // Note: norm was here
    sat1 = sign(s_of_q(0, 0));
  } else {
    sat1 = s_of_q(0, 0)/ phi1;
  }


  if (std::abs(s_of_q(1, 0)) >= phi2) {
    sat2 = sign(s_of_q(1, 0));
  } else {
    sat2 = s_of_q(1, 0) / phi2;
  }

  if (std::abs(s_of_q(2, 0)) >= phi3) {
    sat3 = sign(s_of_q(2, 0));
  } else {
    sat3 = s_of_q(2, 0) / phi3;
  }

  Eigen::MatrixXd saturation_function(3, 1);
  saturation_function <<  sat1 , sat2 , sat3;

  // Constant power rate reaching law

  double q_double_dot1 = (-k_of_q(0, 0)
                         * (pow(std::abs(s_of_q(0, 0)), w1)) * sat1)
                        - (lamda_of_q(0, 0) * errordot_in_q(0, 0));
  double q_double_dot2 = (-k_of_q(1, 1)
                         * (pow(std::abs(s_of_q(1, 0)), w2)) * sat2)
                        - (lamda_of_q(1, 0) * errordot_in_q(1, 0));
  double q_double_dot3 = (-k_of_q(2, 2)
                         * (pow(std::abs(s_of_q(2, 0)), w3)) * sat3)
                        - (lamda_of_q(2, 0) * errordot_in_q(2, 0));

  Eigen::MatrixXd q_double_dot(3, 1);
  q_double_dot <<  q_double_dot1 , q_double_dot2 , q_double_dot3;

  return q_double_dot;
}

Eigen::MatrixXd PushRecoveryControl::GetTorque(
  const Eigen::Vector2d& cop,
  const Eigen::VectorXd& position,
  const Eigen::VectorXd& velocity) {
  theta1 = constrainAngle(position(4 + 7));  // ankle_pitch_theta
  theta2 = constrainAngle(position(3 + 7));  // knee_pitch_theta
  theta3 = constrainAngle(position(2 + 7));  // hip_pitch_theta

  theta1_dot = constrainAngle(velocity(4 + 6));
  theta2_dot = constrainAngle(velocity(3 + 6));
  theta3_dot = constrainAngle(velocity(2 + 6));

  Eigen::MatrixXd mat_m(3, 3);
  Eigen::MatrixXd mat_c(3, 1);
  Eigen::MatrixXd mat_g(3, 1);
  Eigen::MatrixXd mat_n(3, 1);

  Eigen::MatrixXd result(3, 5);

  result = ModelCalculation();

  // Assigning matrices values
  mat_m.col(0) = result.col(0);
  mat_m.col(1) = result.col(1);
  mat_m.col(2) = result.col(2);
  mat_c.col(0) = result.col(3);
  mat_g.col(0) = result.col(4);

  mat_n = mat_c + mat_g;

  Eigen::MatrixXd result_COM(2, 3);
  result_COM = CalculateCOM();
  Eigen::MatrixXd J_X_COM(1, 3), J_X_COM_dot(1, 3);
  J_X_COM << result_COM.row(0);
  J_X_COM_dot << result_COM.row(1);
  Eigen::MatrixXd Torque_SMC_Linear_plus_angular_compensation(3, 1);
  Eigen::MatrixXd mat_smc(3, 1);
  mat_smc << SMCController(cop,  J_X_COM, J_X_COM_dot);
  Torque_SMC_Linear_plus_angular_compensation << mat_m * mat_smc  +  mat_n;

  Eigen::MatrixXd q_double_dot(3, 1);
  q_double_dot = SMCPostureCorrection();

  Eigen::MatrixXd Pseudo_J_X_COM(3, 1);
  Pseudo_J_X_COM = J_X_COM
                     .completeOrthogonalDecomposition().pseudoInverse();
  // Pseudo Inverse of J_X_COM
  Eigen::MatrixXd Phi_N_of_q(3, 1), eye(3, 3),
    T_posture_of_q(3, 1), T(3, 1);
  eye<< 1, 0, 0,
    0, 1, 0,
    0, 0, 1;
  Phi_N_of_q = (eye - Pseudo_J_X_COM*J_X_COM)
               * q_double_dot;   // Second approach for angular momentum
  T_posture_of_q = mat_m* Phi_N_of_q;
  T = Torque_SMC_Linear_plus_angular_compensation + T_posture_of_q;
  return T;
}
