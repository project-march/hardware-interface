// Copyright 2020 Project March.

#ifndef MARCH_WS_INERTIA_ESTIMATOR_H
#define MARCH_WS_INERTIA_ESTIMATOR_H

#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <urdf/model.h>

void absolute(std::vector<double> a, std::vector<double>& b);

// Compute the absolute value of a double and return it.
double absolute(double a);

int maximum(int a, int b, int c);

double median(double a[], size_t sz);

double mean(std::vector<double> a);

class InertiaEstimator
{
public:
  InertiaEstimator();
  InertiaEstimator(hardware_interface::JointHandle joint);
  ~InertiaEstimator();

  double getPosition();

  void setJoint(hardware_interface::JointHandle joint);
  void setNodeHandle(ros::NodeHandle& nh);
  void configurePublisher(std::string name);
  void publishInertia();

  std::string getJointName();

  // Applies the Butterworth filter over the last two samples and returns the resulting filtered value.
  void apply_butter();

  // Estimate the inertia using the acceleration and torque
  void inertia_estimate();
  // Calculate a discrete derivative of the speed measurements
  void discrete_speed_derivative(double velocity, const ros::Duration&);
  // Calculate the alpha coefficient for the inertia estimate
  double alpha_calculation();
  // Calculate the inertia gain for the inertia estimate
  double gain_calculation();
  // Calculate the correlation coefficient of the acceleration buffer
  void correlation_calculation();
  // Calculate the vibration based on the acceleration
  double vibration_calculation();

  std::vector<double> getFilAcc()
  {
    return filtered_acceleration_array_;
  }

  // Fill the buffers with corresponding values.
  bool fill_buffers(const ros::Duration& period);
  bool fill_buffers(double velocity, double effort, const ros::Duration& period);
  bool fill_buffers(double velocity, double effort);

  urdf::JointConstSharedPtr joint_urdf_;
  std::string joint_name;

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  hardware_interface::JointHandle joint_;

  float min_alpha_ = 0.4;  // You might want to be able to adjust this value from a yaml/launch file
  float max_alpha_ = 0.9;  // You might want to be able to adjust this value from a yaml/launch file

  double sos_[3][6] = {
    { 2.31330497e-05, 4.62660994e-05, 2.31330497e-05, 1.00000000e+00, -1.37177561e+00, 4.75382129e-01 },
    { 1.00000000e+00, 2.00000000e+00, 1.00000000e+00, 1.00000000e+00, -1.47548044e+00, 5.86919508e-01 },
    { 1.00000000e+00, 2.00000000e+00, 1.00000000e+00, 1.00000000e+00, -1.69779140e+00, 8.26021017e-01 }
  };
  // namespace joint_inertia_controller

  // Replace the 8 with he number of joints later
  // Of length 12 for the acceleration buffer
  size_t vel_size_ = 12;
  std::vector<double> velocity_array_;
  // Of length 12 for the alpha calculation
  size_t acc_size_ = 12;
  std::vector<double> acceleration_array_;
  // Of length 2 for the error calculation
  size_t fil_acc_size_ = acc_size_;
  std::vector<double> filtered_acceleration_array_;
  // Of length 3 for the butterworth filter
  size_t torque_size_ = 2;
  std::vector<double> joint_torque_;
  // Of length 2 for the error calculation
  size_t fil_tor_size_ = 2;
  std::vector<double> filtered_joint_torque_;

  // Correlation coefficient used to calculate the inertia gain
  double corr_coeff_;
  double K_a_;
  double K_i_;
  double moa_;
  double aom_;
  double joint_inertia_ = 0.0;
  double lambda_ = 0.96;  // You might want to be able to adjust this value from a yaml/launch file
};

#endif  // MARCH_WS_INERTIA_ESTIMATOR_H