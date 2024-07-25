/**
 * UWB localization ceres utilities.
 *
 * Giorgio Manca <giorgio.manca97@gmail.com>
 *
 * July 24, 2024
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <uwb_ceres/uwb_ceres.hpp>

namespace uwb_ceres {

/**
 * @brief Ceres cost function constructor.
 *
 * @param n1 Number of anchors seen by tag1.
 * @param n2 Number of anchors seen by tag2.
 * @param cost Cost for distance constraint.
 * @param dist_bars Distance between UWBs poles..
 * @param vectors
 */
Function::Function(
  bool two_d_mode,
  bool squared,
  unsigned int count, 
  std::vector<double>& distances, 
  std::vector<double>& anchors_x, 
  std::vector<double>& anchors_y, 
  std::vector<double>& anchors_z)
  :
  two_d_mode_(two_d_mode),
  squared_(squared),
  count_(count),
  distances_(distances),
  anchors_x_(anchors_x),
  anchors_y_(anchors_y),
  anchors_z_(anchors_z)
{
  std::vector<int32_t> *block_sizes = mutable_parameter_block_sizes();
  block_sizes->reserve(1);
  block_sizes->push_back(two_d_mode ? 2 : 3);
  set_num_residuals(count);
}

/**
 * @brief Ceres cost function Evaluate.
 *
 * @param parameters
 * @param residuals
 * @param jacobians
 */
bool Function::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const 
{
  if(squared_) {
    if(two_d_mode_) {
      return EvaluateSquare2d(parameters, residuals, jacobians);
    } else {
      return EvaluateSquare3d(parameters, residuals, jacobians);
    }
  } else {
    if(two_d_mode_) {
      return EvaluateLinear2d(parameters, residuals, jacobians);
    } else {
      return EvaluateLinear3d(parameters, residuals, jacobians);
    }
  }
}

/**
 * @brief Ceres cost function Evaluate without squared distances.
 *
 * @param parameters
 * @param residuals
 * @param jacobians
 */
bool Function::EvaluateLinear2d(double const* const* parameters, double* residuals, double** jacobians) const 
{
  double x = parameters[0][0], y = parameters[0][1], z = parameters[0][2];
  double dx, dy, dz;
  double* nrms = new double[count_];
  int param_size = parameter_block_sizes()[0];

  for (unsigned int i = 0; i < count_; i++) {
    dx = (x - anchors_x_[i]);
    dy = (y - anchors_y_[i]);
    dz = (z - anchors_z_[i]);
    nrms[i] = sqrt(dx*dx + dy*dy + dz*dz);
    residuals[i] = nrms[i] - distances_[i];
  }

  bool res = true;
  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr)
    {
      for (unsigned int i = 0; i < param_size * count_; i++) {
        jacobians[0][i] = 0.0;
      }
      
      for (unsigned int i = 0; i < count_; i++) {
        if(nrms[i] > 0.0) {
          jacobians[0][i * param_size + 0] += (x - anchors_x_[i]) / nrms[i];
          jacobians[0][i * param_size + 1] += (y - anchors_y_[i]) / nrms[i];
        } else {
          res = false;
        }
      }
    }
  }

  delete[] nrms;

  return res;
}

/**
 * @brief Ceres cost function Evaluate without squared distances.
 *
 * @param parameters
 * @param residuals
 * @param jacobians
 */
bool Function::EvaluateLinear3d(double const* const* parameters, double* residuals, double** jacobians) const 
{
  double x = parameters[0][0], y = parameters[0][1], z = parameters[0][2];
  double dx, dy, dz;
  double* nrms = new double[count_];
  int param_size = parameter_block_sizes()[0];

  for (unsigned int i = 0; i < count_; i++) {
    dx = (x - anchors_x_[i]);
    dy = (y - anchors_y_[i]);
    dz = (z - anchors_z_[i]);
    nrms[i] = sqrt(dx*dx + dy*dy + dz*dz);
    residuals[i] = nrms[i] - distances_[i];
  }

  bool res = true;
  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr)
    {
      for (unsigned int i = 0; i < param_size * count_; i++) {
        jacobians[0][i] = 0.0;
      }
      
      for (unsigned int i = 0; i < count_; i++) {
        if(nrms[i] > 0.0) {
          jacobians[0][i * param_size + 0] += (x - anchors_x_[i]) / nrms[i];
          jacobians[0][i * param_size + 1] += (y - anchors_y_[i]) / nrms[i];
          jacobians[0][i * param_size + 2] += (z - anchors_z_[i]) / nrms[i];
        } else {
          res = false;
        }
      }
    }
  }

  delete[] nrms;

  return res;
}

/**
 * @brief Ceres cost function Evaluate with squared distances.
 *
 * @param parameters
 * @param residuals
 * @param jacobians
 */
bool Function::EvaluateSquare2d(double const* const* parameters, double* residuals, double** jacobians) const 
{
  double x = parameters[0][0], y = parameters[0][1], z = parameters[0][2];
  double dx, dy, dz;
  int param_size = parameter_block_sizes()[0];

  for (unsigned int i = 0; i < count_; i++) {
    dx = (x - anchors_x_[i]);
    dy = (y - anchors_y_[i]);
    dz = (z - anchors_z_[i]);
    residuals[i] = dx*dx + dy*dy + dz*dz - distances_[i]*distances_[i];
  }
  
  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr)
    {
      for (unsigned int i = 0; i < param_size * count_; i++) {
        jacobians[0][i] = 0.0;
      }
      
      for (unsigned int i = 0; i < count_; i++) {
        jacobians[0][i * param_size + 0] += 2 * (x - anchors_x_[i]);
        jacobians[0][i * param_size + 1] += 2 * (y - anchors_y_[i]);
      }
    }
  }

  return true;
}

/**
 * @brief Ceres cost function Evaluate with squared distances.
 *
 * @param parameters
 * @param residuals
 * @param jacobians
 */
bool Function::EvaluateSquare3d(double const* const* parameters, double* residuals, double** jacobians) const 
{
  double x = parameters[0][0], y = parameters[0][1], z = parameters[0][2];
  double dx, dy, dz;
  int param_size = parameter_block_sizes()[0];

  for (unsigned int i = 0; i < count_; i++) {
    dx = (x - anchors_x_[i]);
    dy = (y - anchors_y_[i]);
    dz = (z - anchors_z_[i]);
    residuals[i] = dx*dx + dy*dy + dz*dz - distances_[i]*distances_[i];
  }

  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr)
    {
      for (unsigned int i = 0; i < param_size * count_; i++) {
        jacobians[0][i] = 0.0;
      }
      
      for (unsigned int i = 0; i < count_; i++) {
        jacobians[0][i * param_size + 0] += 2 * (x - anchors_x_[i]);
        jacobians[0][i * param_size + 1] += 2 * (y - anchors_y_[i]);
        jacobians[0][i * param_size + 2] += 2 * (z - anchors_z_[i]);
      }
    }
  }

  return true;
}


Result solve(Function *function, std::array<double, 3> &init)
{
  double pos[3];
  pos[0] = init[0];
  pos[1] = init[1];
  pos[2] = init[2];

  std::vector<double*> parameters;
  parameters.reserve(1);
  parameters.push_back(pos);

  ceres::Problem problem;
  problem.AddResidualBlock(function, nullptr, parameters);

  Solver::Options options;
  options.max_num_iterations = 10000;
  options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
  options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
  options.linear_solver_type = ceres::LinearSolverType::DENSE_NORMAL_CHOLESKY;

  Result result;
  Solve(options, &problem, &result.summary);
  result.position[0] = pos[0];
  result.position[1] = pos[1];
  result.position[2] = pos[2];

  return result;
}

}