#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include<eigen3/Eigen/Dense>
#include <complex>

using namespace std;
using namespace Eigen;

using Eigen::MatrixXd;
using Eigen::VectorXd;
std::complex<double> i(0.0,1.0);

class PushRecoveryControl{
 private:
// EOM of 3 DOF model
// Mass in kg, length in meter
  float alpha = 0.7;
  float m1 = 5.9117,
        m2 = 4.2554,
        m3 = 10.19329;

  float lc1 = 0.15149,
        lc2 = 0.24517,
        lc3 = 0.1585;

  float l1 = 0.3715,
        l2 = 0.49478,
        l3 = 0.32662;

  float g = 9.81;
  float I1 = 0.0222,
        I2 = 0.01009,
        I3 = 0.0219;

  // Desired position, velocity and acceleration of location of the com

  float theta1_d = 0,
        theta2_d = 0,
        theta3_d = 0;

  float theta1_dot_d = 0,
        theta2_dot_d = 0,
        theta3_dot_d = 0;

  float theta1_dddot = 0,
        theta2_dddot = 0,
        theta3_dddot = 0;

  float x_com_d = 0,
        x_com_ddot = 0,
        x_com_dddot = 0;

  // Values
  float theta1, theta2, theta3 = 0;
  float theta1_dot, theta2_dot, theta3_dot = 0;
  float X_COM, X_dot_COM = 0;

 public:
  float CalculateXCOP(float r1_ft_torque[], float l1_ft_torque[],
                      float r1_ft_force[], float l1_ft_force[]) {
    /**
     * Outputs:
     * cop_x: x coordinate of COP
     */

    // Vertical distance from the load cell to bottom of the foot
    float d = 0.0983224252792114;
    // Position of left sensor
    RowVectorXf p1(3);
    p1 << 0, 0.0775, d;
    // Position of right sensor
    RowVectorXf p2(3);
    p2 << 0, -0.0775, d;

    RowVectorXf tau_right(3);
    tau_right << r1_ft_torque[0], r1_ft_torque[1], r1_ft_torque[2];
    tau_right = tau_right.transpose();//Convert from Row to column vector

    RowVectorXf tau_left(3);
    tau_left << l1_ft_torque[0], l1_ft_torque[1], l1_ft_torque[2];
    tau_left = tau_left.transpose();

    RowVectorXf f_right(3);
    f_right << r1_ft_force[0], r1_ft_force[1], r1_ft_force[2];
    f_right = f_right.transpose();

    RowVectorXf f_left(3);
    f_left << l1_ft_force[0], l1_ft_force[1], l1_ft_force[2];
    f_left = f_left.transpose();

    return (tau_left(1) + d*f_left(2) + tau_right(1) + d*f_right(2)) / (f_left(0) + f_right(0));
   }

  Eigen::MatrixXf ModelCalculation(){
    // Note the assigning values:
    float q1 = theta1;
    float q2 = theta2;
    float q3 = theta3;
    float q_dot1 = theta1_dot;
    float q_dot2 = theta2_dot;
    float q_dot3 = theta3_dot;
    float r1 = lc1;
    float r2 = lc2;
    float r3 = lc3;

    MatrixXf mat_m(3,3);
    mat_m << I1 + I2 + I3 + pow(l1,2)*m2 + pow(l1,2)*m3 + pow(l2,2)*m3 + m1*pow(r1,2) + m2*pow(r2,2) + m3*pow(r3,2) + 2*l1*m3*r3*cos(q2 + q3) + 2*l1*l2*m3*cos(q2) + 2*l1*m2*r2*cos(q2) + 2*l2*m3*r3*cos(q3),
      m3*pow(l2,2) + 2*m3*cos(q3)*l2*r3 + l1*m3*cos(q2)*l2 + m2*pow(r2,2) + l1*m2*cos(q2)*r2 + m3*pow(r3,2) + l1*m3*cos(q2 + q3)*r3 + I2 + I3,
      I3 + m3*pow(r3,2) + l1*m3*r3*cos(q2 + q3) + l2*m3*r3*cos(q3),
      m3*pow(l2,2) + 2*m3*cos(q3)*l2*r3 + l1*m3*cos(q2)*l2 + m2*pow(r2,2) + l1*m2*cos(q2)*r2 + m3*pow(r3,2) + l1*m3*cos(q2 + q3)*r3 + I2 + I3,
      m3*pow(l2,2) + 2*m3*cos(q3)*l2*r3 + m2*pow(r2,2) + m3*pow(r3,2) + I2 + I3,
      m3*pow(r3,2) + l2*m3*cos(q3)*r3 + I3,
      I3 + m3*pow(r3,2) + l1*m3*r3*cos(q2 + q3) + l2*m3*r3*cos(q3),
      m3*pow(r3,2) + l2*m3*cos(q3)*r3 + I3,
      m3*pow(r3,2) + I3;


    MatrixXf mat_c(3,1);
    mat_c << - l1*m3*(pow(q_dot2,2))*r3*sin(q2 + q3) - l1*m3*pow(q_dot3,2)*r3*sin(q2 + q3) - l1*l2*m3*pow(q_dot2,2)*sin(q2) - l1*m2*pow(q_dot2,2)*r2*sin(q2) - l2*m3*pow(q_dot3,2)*r3*sin(q3) - 2*l1*m3*q_dot1*q_dot2*r3*sin(q2 + q3) - 2*l1*m3*q_dot1*q_dot3*r3*sin(q2 + q3) - 2*l1*m3*q_dot2*q_dot3*r3*sin(q2 + q3) - 2*l1*l2*m3*q_dot1*q_dot2*sin(q2) - 2*l1*m2*q_dot1*q_dot2*r2*sin(q2) - 2*l2*m3*q_dot1*q_dot3*r3*sin(q3) - 2*l2*m3*q_dot2*q_dot3*r3*sin(q3),
      l1*m3*pow(q_dot1,2)*r3*sin(q2 + q3) + l1*l2*m3*pow(q_dot1,2)*sin(q2) + l1*m2*pow(q_dot1,2)*r2*sin(q2) - l2*m3*pow(q_dot3,2)*r3*sin(q3) - 2*l2*m3*q_dot1*q_dot3*r3*sin(q3) - 2*l2*m3*q_dot2*q_dot3*r3*sin(q3),
      l1*m3*pow(q_dot1,2)*r3*sin(q2 + q3) + l2*m3*pow(q_dot1,2)*r3*sin(q3) + l2*m3*pow(q_dot2,2)*r3*sin(q3) + 2*l2*m3*q_dot1*q_dot2*r3*sin(q3);

    MatrixXf mat_g(3,1);
    mat_g << - g*l2*m3*sin(q1 + q2) - g*m2*r2*sin(q1 + q2) - g*l1*m2*sin(q1) - g*l1*m3*sin(q1) - g*m1*r1*sin(q1) - g*m3*r3*sin(q1 + q2 + q3),
      - g*l2*m3*sin(q1 + q2) - g*m2*r2*sin(q1 + q2) - g*m3*r3*sin(q1 + q2 + q3),
      -g*m3*r3*sin(q1 + q2 + q3);

    MatrixXf result(3,5);
    result << mat_m, mat_c, mat_g;
    return result;
  }
  MatrixXf CalculateCOM(){
    X_COM = -1*((lc1*sin(theta1))*m1 + (l1*sin(theta1)+lc2*sin(theta1+theta2))*m2 + (l1*sin(theta1)+l2*sin(theta1+theta2)+lc3*sin(theta1+theta2+theta3))*m3) / (m1+m2+m3); // Center of Mass position in x_direction
    X_dot_COM = -1* (m1*(theta1_dot*lc1*cos(theta1)) + m2*(theta1_dot*l1*cos(theta1) +(theta1_dot+theta2_dot)*lc2*cos(theta1+theta2)) + m3*(theta1_dot*l1*cos(theta1)+(theta1_dot+theta2_dot)*l2*cos(theta1+theta2)+(theta1_dot+theta2_dot+theta3_dot)*lc3*cos(theta1+theta2+theta3)) )/(m1+m2+m3); //velocity of the COM in x_direction
    MatrixXf J_X_COM(1,3);
    J_X_COM << (m3*(l2*cos(theta1 + theta2) + l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3)) + m2*(lc2*cos(theta1 + theta2) + l1*cos(theta1)) + lc1*m1*cos(theta1))/(m1 + m2 + m3),
      (m3*(l2*cos(theta1 + theta2) + lc3*cos(theta1 + theta2 + theta3)) + lc2*m2*cos(theta1 + theta2))/(m1 + m2 + m3),
      (lc3*m3*cos(theta1 + theta2 + theta3))/(m1 + m2 + m3);

    // Jacobian Matrix of X-COM Jx
    J_X_COM = -1* J_X_COM;

    MatrixXf J_X_COM_dot(1,3);
    J_X_COM_dot << (-m1*theta1_dot*lc1*sin(theta1) + m2*(-l1*theta1_dot*sin(theta1)-(theta1_dot+theta2_dot)*lc2*sin(theta1+theta2) )  + m3*(-l2*(theta1_dot+theta2_dot)*sin(theta1+theta2)-l1*theta1_dot*sin(theta1)-(theta1_dot+theta2_dot+theta3_dot)*lc3*sin(theta1+theta2+theta3)) )/(m1+m2+m3),
      ( m3*(-l2*(theta1_dot+theta2_dot)*sin(theta1+theta2) -lc3*(theta1_dot+theta2_dot+theta3_dot)*sin(theta1+theta2+theta3) ) -lc2*(theta1_dot+theta2_dot)*m2*sin(theta1+theta2) )/(m1+m2+m3),
      (-lc3*(theta1_dot+theta2_dot+theta3_dot)*m3*sin(theta1+theta2+theta3) )/(m1+m2+m3); // Time Derivative of Jacobian Matrix
    J_X_COM_dot = -1 * J_X_COM_dot;


    float Z_COM = ((lc1*cos(theta1))*m1 + (l1*cos(theta1)+lc2*cos(theta1+theta2))*m2 + (l1*cos(theta1)+l2*cos(theta1+theta2)+lc3*cos(theta1+theta2+theta3))*m3  ) / (m1+m2+m3); // Center of Mass position in z_direction (desired is 0.6859)
    float Z_dot_COM = (  -m1*(theta1_dot*lc1*sin(theta1)) + m2*(-theta1_dot*l1*sin(theta1) - (theta1_dot+theta2_dot)*lc2*sin(theta1+theta2) ) + m3*(-theta1_dot*l1*sin(theta1) - (theta1_dot+theta2_dot)*l2*sin(theta1+theta2) - (theta1_dot+theta2_dot+theta3_dot)*lc3*sin(theta1+theta2+theta3)) )/(m1+m2+m3); // velocity of the COM in z_direction

    MatrixXf J_Z_COM(1,3);
    J_Z_COM << (m3*(-l2*sin(theta1 + theta2) - l1*sin(theta1) - lc3*sin(theta1 + theta2 + theta3)) + m2*(-lc2*sin(theta1 + theta2) - l1*sin(theta1)) - lc1*m1*sin(theta1))/(m1 + m2 + m3),
      (m3*(-l2*sin(theta1 + theta2) - lc3*sin(theta1 + theta2 + theta3)) - lc2*m2*sin(theta1 + theta2))/(m1 + m2 + m3),
      (-lc3*m3*sin(theta1 + theta2 + theta3))/(m1 + m2 + m3); // Jacobian Matrix of Z-COM Jz

    MatrixXf J_COM(2,3);
    J_COM << J_X_COM, J_Z_COM; // Linear part of Jacobian matrix of COM


    MatrixXf J_W_COM(1,3);
    J_W_COM << 1, 1, 1;
    MatrixXf J_total_COM(2,3);
    J_total_COM << J_X_COM, J_W_COM; // linear and angular part of Jacobian of COM
    MatrixXf J_total_COM_dot(2,3), J_total_COM_pseduo(3,2);
    J_total_COM_dot << J_X_COM_dot, 0, 0, 0;// time derivative of total Jacobian of COM
    J_total_COM_pseduo=J_total_COM.completeOrthogonalDecomposition().pseudoInverse(); // pseduo inverse of the total Jacobian of COM

    MatrixXf result(2,3);
    result << J_X_COM, J_X_COM_dot;
    return result;

  }

  template <typename T>
  int sign (const T &val) { return (val > 0) - (val < 0); }
  MatrixXf SMCController( RowVectorXf cop, MatrixXf J_X_COM, MatrixXf J_X_COM_dot){
    // SMC for Linear motion rate of change of linear momentum

    float error_in_x=X_COM;
    float error_dot_in_x=X_dot_COM;

    MatrixXf theta_dot(3,1);
    theta_dot << theta1_dot, theta2_dot, theta3_dot;

    float L11 = 1.9, k11 = -1, p11 = 1, c11 = 7, Q11 = 0.001, a11 = 2, z11 = 2.5; // after editing the first term/first approach
    float s_linear_motion = error_dot_in_x + L11*error_in_x;

    float f11 = c11 * ( 1 - exp( k11*pow(abs(s_linear_motion),p11)));
    float s_linear_motion_dot = -Q11 * pow(abs(s_linear_motion),f11)  *sign(s_linear_motion) - z11 * pow(abs(s_linear_motion),a11)  * s_linear_motion ;

    // Angular Momentum control part
    MatrixXf A_theta(1,3);
    A_theta << I1 + I2 + I3 - pow(m3*(l2*sin(theta1 + theta2) + l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3)) + m2*(lc2*sin(theta1 + theta2) + l1*sin(theta1)) + lc1*m1*sin(theta1),2)/(m1 + m2 + m3) + pow(l1,2)*m2 + pow(l1,2)*m3 + pow(l2,2)*m3 + pow(lc1,2)*m1 + pow(lc2,2)*m2 + pow(lc3,2)*m3 - pow(m3*(l2*cos(theta1 + theta2) + l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3)) + m2*(lc2*cos(theta1 + theta2) + l1*cos(theta1)) + lc1*m1*cos(theta1),2)/(m1 + m2 + m3) + 2*l1*lc3*m3*cos(theta2 + theta3) + 2*l1*l2*m3*cos(theta2) + 2*l1*lc2*m2*cos(theta2) + 2*l2*lc3*m3*cos(theta3),
      I2 + I3 + pow(l2,2)*m3 + pow(lc2,2)*m2 + pow(lc3,2)*m3 - ((m3*(l2*sin(theta1 + theta2) + l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3)) + m2*(lc2*sin(theta1 + theta2) + l1*sin(theta1)) + lc1*m1*sin(theta1))*(lc3*m3*sin(theta1 + theta2 + theta3) + l2*m3*sin(theta1 + theta2) + lc2*m2*sin(theta1 + theta2)))/(m1 + m2 + m3) - ((lc3*m3*cos(theta1 + theta2 + theta3) + l2*m3*cos(theta1 + theta2) + lc2*m2*cos(theta1 + theta2))*(m3*(l2*cos(theta1 + theta2) + l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3)) + m2*(lc2*cos(theta1 + theta2) + l1*cos(theta1)) + lc1*m1*cos(theta1)))/(m1 + m2 + m3) + l1*lc3*m3*cos(theta2 + theta3) + l1*l2*m3*cos(theta2) + l1*lc2*m2*cos(theta2) + 2*l2*lc3*m3*cos(theta3),
      I3 + pow(lc3,2)*m3 + l1*lc3*m3*cos(theta2 + theta3) + l2*lc3*m3*cos(theta3) - (lc3*m3*cos(theta1 + theta2 + theta3)*(m3*(l2*cos(theta1 + theta2) + l1*cos(theta1) + lc3*cos(theta1 + theta2 + theta3)) + m2*(lc2*cos(theta1 + theta2) + l1*cos(theta1)) + lc1*m1*cos(theta1)))/(m1 + m2 + m3) - (lc3*m3*sin(theta1 + theta2 + theta3)*(m3*(l2*sin(theta1 + theta2) + l1*sin(theta1) + lc3*sin(theta1 + theta2 + theta3)) + m2*(lc2*sin(theta1 + theta2) + l1*sin(theta1)) + lc1*m1*sin(theta1)))/(m1 + m2 + m3);

    MatrixXf A_theta_pseudo(3,1);
    A_theta_pseudo << (I1*m1 + I1*m2 + I2*m1 + I1*m3 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + pow(l1,2)*m1*m2 + pow(l1,2)*m1*m3 + pow(l2,2)*m1*m3 + pow(l2,2)*m2*m3 + pow(lc1,2)*m1*m2 + pow(lc1,2)*m1*m3 + pow(lc2,2)*m1*m2 + pow(lc2,2)*m2*m3 + pow(lc3,2)*m1*m3 + pow(lc3,2)*m2*m3 - 2*l1*lc1*m1*m2 - 2*l1*lc1*m1*m3 - 2*l2*lc2*m2*m3 + 2*l1*lc3*m1*m3*cos(theta2 + theta3) - 2*lc1*lc3*m1*m3*cos(theta2 + theta3) + 2*l1*l2*m1*m3*cos(theta2) + 2*l1*lc2*m1*m2*cos(theta2) - 2*l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - 2*lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3))/((m1 + m2 + m3)*(pow(I1*m1 + I1*m2 + I2*m1 + I1*m3 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + pow(l1,2)*m1*m2 + pow(l1,2)*m1*m3 + pow(l2,2)*m1*m3 + pow(l2,2)*m2*m3 + pow(lc1,2)*m1*m2 + pow(lc1,2)*m1*m3 + pow(lc2,2)*m1*m2 + pow(lc2,2)*m2*m3 + pow(lc3,2)*m1*m3 + pow(lc3,2)*m2*m3 - 2*l1*lc1*m1*m2 - 2*l1*lc1*m1*m3 - 2*l2*lc2*m2*m3 + 2*l1*lc3*m1*m3*cos(theta2 + theta3) - 2*lc1*lc3*m1*m3*cos(theta2 + theta3) + 2*l1*l2*m1*m3*cos(theta2) + 2*l1*lc2*m1*m2*cos(theta2) - 2*l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - 2*lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3),2)/pow((m1 + m2 + m3),2) + pow(I3*m1 + I3*m2 + I3*m3 + pow(lc3,2)*m1*m3 + pow(lc3,2)*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l2*lc3*m1*m3*cos(theta3) + l2*lc3*m2*m3*cos(theta3) - lc2*lc3*m2*m3*cos(theta3),2)/pow((m1 + m2 + m3),2) + pow(I2*m1 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + pow(l2,2)*m1*m3 + pow(l2,2)*m2*m3 + pow(lc2,2)*m1*m2 + pow(lc2,2)*m2*m3 + pow(lc3,2)*m1*m3 + pow(lc3,2)*m2*m3 - 2*l2*lc2*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l1*l2*m1*m3*cos(theta2) + l1*lc2*m1*m2*cos(theta2) - l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3),2)/pow((m1 + m2 + m3),2))),
      (I2*m1 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + pow(l2,2)*m1*m3 + pow(l2,2)*m2*m3 + pow(lc2,2)*m1*m2 + pow(lc2,2)*m2*m3 + pow(lc3,2)*m1*m3 + pow(lc3,2)*m2*m3 - 2*l2*lc2*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l1*l2*m1*m3*cos(theta2) + l1*lc2*m1*m2*cos(theta2) - l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3))/((m1 + m2 + m3)*pow(I1*m1 + I1*m2 + I2*m1 + I1*m3 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + pow(l1,2)*m1*m2 + pow(l1,2)*m1*m3 + pow(l2,2)*m1*m3 + pow(l2,2)*m2*m3 + pow(lc1,2)*m1*m2 + pow(lc1,2)*m1*m3 + pow(lc2,2)*m1*m2 + pow(lc2,2)*m2*m3 + pow(lc3,2)*m1*m3 + pow(lc3,2)*m2*m3 - 2*l1*lc1*m1*m2 - 2*l1*lc1*m1*m3 - 2*l2*lc2*m2*m3 + 2*l1*lc3*m1*m3*cos(theta2 + theta3) - 2*lc1*lc3*m1*m3*cos(theta2 + theta3) + 2*l1*l2*m1*m3*cos(theta2) + 2*l1*lc2*m1*m2*cos(theta2) - 2*l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - 2*lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3),2)/pow((m1 + m2 + m3),2) + pow(I3*m1 + I3*m2 + I3*m3 + pow(lc3,2)*m1*m3 + pow(lc3,2)*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l2*lc3*m1*m3*cos(theta3) + l2*lc3*m2*m3*cos(theta3) - lc2*lc3*m2*m3*cos(theta3),2)/pow((m1 + m2 + m3),2) + pow(I2*m1 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + pow(l2,2)*m1*m3 + pow(l2,2)*m2*m3 + pow(lc2,2)*m1*m2 + pow(lc2,2)*m2*m3 + pow(lc3,2)*m1*m3 + pow(lc3,2)*m2*m3 - 2*l2*lc2*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l1*l2*m1*m3*cos(theta2) + l1*lc2*m1*m2*cos(theta2) - l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3),2)/pow((m1 + m2 + m3),2)),
      (I3*m1 + I3*m2 + I3*m3 + pow(lc3,2)*m1*m3 + pow(lc3,2)*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l2*lc3*m1*m3*cos(theta3) + l2*lc3*m2*m3*cos(theta3) - lc2*lc3*m2*m3*cos(theta3))/((m1 + m2 + m3)*(pow(I1*m1 + I1*m2 + I2*m1 + I1*m3 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + pow(l1,2)*m1*m2 + pow(l1,2)*m1*m3 + pow(l2,2)*m1*m3 + pow(l2,2)*m2*m3 + pow(lc1,2)*m1*m2 + pow(lc1,2)*m1*m3 + pow(lc2,2)*m1*m2 + pow(lc2,2)*m2*m3 + pow(lc3,2)*m1*m3 + pow(lc3,2)*m2*m3 - 2*l1*lc1*m1*m2 - 2*l1*lc1*m1*m3 - 2*l2*lc2*m2*m3 + 2*l1*lc3*m1*m3*cos(theta2 + theta3) - 2*lc1*lc3*m1*m3*cos(theta2 + theta3) + 2*l1*l2*m1*m3*cos(theta2) + 2*l1*lc2*m1*m2*cos(theta2) - 2*l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - 2*lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3),2)/pow((m1 + m2 + m3),2) + pow(I3*m1 + I3*m2 + I3*m3 + pow(lc3,2)*m1*m3 + pow(lc3,2)*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l2*lc3*m1*m3*cos(theta3) + l2*lc3*m2*m3*cos(theta3) - lc2*lc3*m2*m3*cos(theta3),2)/pow((m1 + m2 + m3),2) + pow(I2*m1 + I2*m2 + I3*m1 + I2*m3 + I3*m2 + I3*m3 + pow(l2,2)*m1*m3 + pow(l2,2)*m2*m3 + pow(lc2,2)*m1*m2 + pow(lc2,2)*m2*m3 + pow(lc3,2)*m1*m3 + pow(lc3,2)*m2*m3 - 2*l2*lc2*m2*m3 + l1*lc3*m1*m3*cos(theta2 + theta3) - lc1*lc3*m1*m3*cos(theta2 + theta3) + l1*l2*m1*m3*cos(theta2) + l1*lc2*m1*m2*cos(theta2) - l2*lc1*m1*m3*cos(theta2) + 2*l2*lc3*m1*m3*cos(theta3) + 2*l2*lc3*m2*m3*cos(theta3) - lc1*lc2*m1*m2*cos(theta2) - 2*lc2*lc3*m2*m3*cos(theta3),2)/pow((m1 + m2 + m3),2)));

    MatrixXf A_theta_dot(1,3);
    A_theta_dot << -(2*l1*lc3*m1*m3*theta2_dot*sin(theta2 + theta3) + 2*l1*lc3*m1*m3*theta3_dot*sin(theta2 + theta3) - 2*lc1*lc3*m1*m3*theta2_dot*sin(theta2 + theta3) - 2*lc1*lc3*m1*m3*theta3_dot*sin(theta2 + theta3) + 2*l1*l2*m1*m3*theta2_dot*sin(theta2) + 2*l1*lc2*m1*m2*theta2_dot*sin(theta2) - 2*l2*lc1*m1*m3*theta2_dot*sin(theta2) + 2*l2*lc3*m1*m3*theta3_dot*sin(theta3) + 2*l2*lc3*m2*m3*theta3_dot*sin(theta3) - 2*lc1*lc2*m1*m2*theta2_dot*sin(theta2) - 2*lc2*lc3*m2*m3*theta3_dot*sin(theta3))/(m1 + m2 + m3),
      -(l1*lc3*m1*m3*theta2_dot*sin(theta2 + theta3) + l1*lc3*m1*m3*theta3_dot*sin(theta2 + theta3) - lc1*lc3*m1*m3*theta2_dot*sin(theta2 + theta3) - lc1*lc3*m1*m3*theta3_dot*sin(theta2 + theta3) + l1*l2*m1*m3*theta2_dot*sin(theta2) + l1*lc2*m1*m2*theta2_dot*sin(theta2) - l2*lc1*m1*m3*theta2_dot*sin(theta2) + 2*l2*lc3*m1*m3*theta3_dot*sin(theta3) + 2*l2*lc3*m2*m3*theta3_dot*sin(theta3) - lc1*lc2*m1*m2*theta2_dot*sin(theta2) - 2*lc2*lc3*m2*m3*theta3_dot*sin(theta3))/(m1 + m2 + m3),
      -(lc3*m3*(l1*m1*theta2_dot*sin(theta2 + theta3) + l1*m1*theta3_dot*sin(theta2 + theta3) - lc1*m1*theta2_dot*sin(theta2 + theta3) - lc1*m1*theta3_dot*sin(theta2 + theta3) + l2*m1*theta3_dot*sin(theta3) + l2*m2*theta3_dot*sin(theta3) - lc2*m2*theta3_dot*sin(theta3)))/(m1 + m2 + m3);

    float k_tunning = 2; // For second option
    float Desired_Angular_Momentum = k_tunning * ( (cop(1)) - X_COM ) *  ( 1 ); // Second option

    //Desired accleration to achieve both linear and angular tasks without LPF
    MatrixXf mat(2,3);
    mat << J_X_COM, A_theta;
    MatrixXf mat_pinv(3,2);
    mat_pinv = mat.completeOrthogonalDecomposition().pseudoInverse();


    MatrixXf mat2(2,1);
    MatrixXf mult_temp(1,1);
    mult_temp << J_X_COM_dot*theta_dot ;
    float mult = mult_temp(0);
    MatrixXf mult2_temp(1,1);
    mult2_temp << A_theta_dot * theta_dot ;
    float mult2 = mult2_temp(0);

    mat2 << ( s_linear_motion_dot - L11 * error_dot_in_x - mult), ( Desired_Angular_Momentum - (mult2));
    MatrixXf Desired_Theta_ddot(3,3);
    Desired_Theta_ddot =  mat_pinv *  mat2;
    MatrixXf mat_smc(3,1);
    mat_smc << Desired_Theta_ddot(0,0), Desired_Theta_ddot(1,0), Desired_Theta_ddot(2,0);
    return mat_smc;
  }
  MatrixXf SMCPOstureCorrection(){
    MatrixXf theta(3,1), error_in_q(3,1);
    theta << theta1, theta2, theta3;

    MatrixXf theta_dot(3,1), errordot_in_q(3,1);
    theta_dot << theta1_dot, theta2_dot, theta3_dot;

    error_in_q = theta  ;
    errordot_in_q = theta_dot ;

    // For above 80 N
    MatrixXf lamda_of_q(3,3), k_of_q(3,3);
    lamda_of_q << 2.8, 0, 0,
      0, 2.8, 0,
      0, 0, 2.8;
    k_of_q << 3, 0, 0,
      0, 3, 0,
      0, 0, 3;

    float w1 = 1, w2 = 1, w3 = 1;

    MatrixXf s_of_q(3,1);
    s_of_q = errordot_in_q + lamda_of_q * error_in_q;

    float phi1 = 0.002, phi2 = 0.02, phi3 = 0.02;
    float sat1 = 0, sat2 = 0, sat3 = 0;

    if (s_of_q(0,0) >= phi1) { //Note: norm was here
        sat1 = sign(s_of_q(0,0));
      }
    else {
      sat1 = s_of_q(0,0)/ phi1;
    }


    if (s_of_q(1,0) >= phi2 ){
        sat2 = sign(s_of_q(1,0));
      }
    else {
      sat2 = s_of_q(1,0) / phi2;
    }

    if (s_of_q(2,0) >= phi3){
      sat3 = sign(s_of_q(2,0));
      }
    else {
      sat3 = s_of_q(2,0) / phi3;
    }

    MatrixXf saturation_function(3,1);
    saturation_function <<  sat1 , sat2 , sat3;

    float q_double_dot1 = (-k_of_q(0,0) * ( pow(abs(s_of_q(0,0)),w1)  ) * sat1 )  - (lamda_of_q(0,0) * errordot_in_q(0,0)) ; // constant power rate reaching law
    float q_double_dot2 = (-k_of_q(1,1) * ( pow(abs(s_of_q(1,0)),w2)  ) * sat2 )  - (lamda_of_q(1,0) * errordot_in_q(1,0)) ; // constant power rate reaching law
    float q_double_dot3 = (-k_of_q(2,2) * ( pow(abs(s_of_q(2,0)),w3)  ) * sat3 )  - (lamda_of_q(2,0) * errordot_in_q(2,0)) ; // constant power rate reaching law

    MatrixXf q_double_dot(3,1);
    q_double_dot <<  q_double_dot1 , q_double_dot2 , q_double_dot3 ;

    return q_double_dot;

  }
  MatrixXf GetTorque(float r1_ft_torque[], float l1_ft_torque[], float r1_ft_force[], float l1_ft_force[]){

    float x_cop = CalculateXCOP(r1_ft_torque, l1_ft_torque, r1_ft_force, l1_ft_force);
    RowVectorXf cop(2), filtered_cop(2);
    cop << 0, x_cop;
    filtered_cop<< 0, 0*alpha + (1-alpha)*x_cop;

    MatrixXf mat_m(3,3);
    MatrixXf mat_c(3,1);
    MatrixXf mat_g(3,1);
    MatrixXf mat_n(3,1);

    MatrixXf result(3,5);

    result = ModelCalculation();
    //Assigning matrices values
    mat_m.col(0) = result.col(0);
    mat_m.col(1) = result.col(1);
    mat_m.col(2) = result.col(2);
    mat_c.col(0) = result.col(3);
    mat_g.col(0) = result.col(4);

    mat_n = mat_c + mat_g;

    MatrixXf result_COM(2,3);
    result_COM = CalculateCOM();
    MatrixXf J_X_COM(1,3), J_X_COM_dot(1,3);
    J_X_COM << result_COM.row(0);
    J_X_COM_dot << result_COM.row(1);
    MatrixXf Torque_SMC_Linear_plus_angular_compensation(3,1);
    MatrixXf mat_smc(3,1);
    mat_smc << SMCController(cop, J_X_COM, J_X_COM_dot);
    Torque_SMC_Linear_plus_angular_compensation << mat_m * mat_smc   +  mat_n ;

    MatrixXf q_double_dot(3,1);
    q_double_dot = SMCPOstureCorrection();

    MatrixXf Pseudo_J_X_COM(3,1);
    Pseudo_J_X_COM=J_X_COM.completeOrthogonalDecomposition().pseudoInverse(); // Pseudo Inverse of J_X_COM
    MatrixXf Phi_N_of_q(3,1), eye(3,3), T_posture_of_q(3,1),T(3,1);
    eye<< 1,0,0,
      0,1,0,
      0,0,1;
    Phi_N_of_q = (eye - Pseudo_J_X_COM*J_X_COM)* q_double_dot; // second approach for angular momentum
    T_posture_of_q = mat_m* Phi_N_of_q;
    T = Torque_SMC_Linear_plus_angular_compensation + T_posture_of_q ;
    return T;
  }

};

int main()
{
  cout << "This is meant to use for testing" ;
  PushRecoveryControl Ibrahim;
  VectorXd v(3);
  v << 1, 2, 3;
  std::cout << "v2 =" << std::endl << v(2) << std::endl;
  float right[3] = {1, 2,3};
  float left[3] = {5, 6,7};
  std::cout << "Prinitng =" << std::endl << Ibrahim.GetTorque(right, left, right, left) << std::endl;


}