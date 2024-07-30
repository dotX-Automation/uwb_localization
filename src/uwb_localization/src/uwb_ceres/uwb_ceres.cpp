/**
 * UWB localization ceres utilities.
 *
 * Giorgio Manca <giorgio.manca97@gmail.com>
 *
 * July 24, 2024
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
  std::vector<Measure>& measures)
  :
  two_d_mode(two_d_mode),
  squared(squared),
  measures(measures)
{
  std::vector<int32_t> *block_sizes = mutable_parameter_block_sizes();
  block_sizes->reserve(1);
  block_sizes->push_back(two_d_mode ? 2 : 3);
  set_num_residuals(measures.size());
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
  if (squared) {
    if (two_d_mode) {
      return EvaluateSquare2d(parameters, residuals, jacobians);
    } else {
      return EvaluateSquare3d(parameters, residuals, jacobians);
    }
  } else {
    if (two_d_mode) {
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
  double* nrms = new double[measures.size()];
  int param_size = parameter_block_sizes()[0];

  for (unsigned int i = 0; i < measures.size(); i++) {
    dx = (x - measures[i].anchor.x());
    dy = (y - measures[i].anchor.y());
    dz = (z - measures[i].anchor.z());
    nrms[i] = sqrt(dx*dx + dy*dy + dz*dz);
    residuals[i] = nrms[i] - measures[i].distance;
  }

  bool res = true;
  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr)
    {
      for (unsigned int i = 0; i < param_size * measures.size(); i++) {
        jacobians[0][i] = 0.0;
      }
      
      for (unsigned int i = 0; i < measures.size(); i++) {
        if(nrms[i] > 0.0) {
          jacobians[0][i * param_size + 0] += (x - measures[i].anchor.x()) / nrms[i];
          jacobians[0][i * param_size + 1] += (y - measures[i].anchor.y()) / nrms[i];
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
  double* nrms = new double[measures.size()];
  int param_size = parameter_block_sizes()[0];

  for (unsigned int i = 0; i < measures.size(); i++) {
    dx = (x - measures[i].anchor.x());
    dy = (y - measures[i].anchor.y());
    dz = (z - measures[i].anchor.z());
    nrms[i] = sqrt(dx*dx + dy*dy + dz*dz);
    residuals[i] = nrms[i] - measures[i].distance;
  }

  bool res = true;
  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr)
    {
      for (unsigned int i = 0; i < param_size * measures.size(); i++) {
        jacobians[0][i] = 0.0;
      }
      
      for (unsigned int i = 0; i < measures.size(); i++) {
        if(nrms[i] > 0.0) {
          jacobians[0][i * param_size + 0] += (x - measures[i].anchor.x()) / nrms[i];
          jacobians[0][i * param_size + 1] += (y - measures[i].anchor.y()) / nrms[i];
          jacobians[0][i * param_size + 2] += (z - measures[i].anchor.z()) / nrms[i];
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

  for (unsigned int i = 0; i < measures.size(); i++) {
    dx = (x - measures[i].anchor.x());
    dy = (y - measures[i].anchor.y());
    dz = (z - measures[i].anchor.z());
    residuals[i] = dx*dx + dy*dy + dz*dz - measures[i].distance*measures[i].distance;
  }
  
  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr)
    {
      for (unsigned int i = 0; i < param_size * measures.size(); i++) {
        jacobians[0][i] = 0.0;
      }
      
      for (unsigned int i = 0; i < measures.size(); i++) {
        jacobians[0][i * param_size + 0] += 2 * (x - measures[i].anchor.x());
        jacobians[0][i * param_size + 1] += 2 * (y - measures[i].anchor.y());
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

  for (unsigned int i = 0; i < measures.size(); i++) {
    dx = (x - measures[i].anchor.x());
    dy = (y - measures[i].anchor.y());
    dz = (z - measures[i].anchor.z());
    residuals[i] = dx*dx + dy*dy + dz*dz - measures[i].distance*measures[i].distance;
  }

  if (jacobians != nullptr) {
    if (jacobians[0] != nullptr)
    {
      for (unsigned int i = 0; i < param_size * measures.size(); i++) {
        jacobians[0][i] = 0.0;
      }
      
      for (unsigned int i = 0; i < measures.size(); i++) {
        jacobians[0][i * param_size + 0] += 2 * (x - measures[i].anchor.x());
        jacobians[0][i * param_size + 1] += 2 * (y - measures[i].anchor.y());
        jacobians[0][i * param_size + 2] += 2 * (z - measures[i].anchor.z());
      }
    }
  }

  return true;
}


Result solve(Function &function, Eigen::Vector3d &init)
{
  double pos[3];
  pos[0] = init.x();
  pos[1] = init.y();
  pos[2] = init.z();

  std::vector<double*> parameters;
  parameters.reserve(1);
  parameters.push_back(pos);

  ceres::Problem problem;
  problem.AddResidualBlock(&function, nullptr, parameters);

  Solver::Options options;
  options.max_num_iterations = 10000;
  options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
  options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
  options.linear_solver_type = ceres::LinearSolverType::DENSE_NORMAL_CHOLESKY;

  Result result;
  Solve(options, &problem, &result.summary);
  result.position.x() = pos[0];
  result.position.y() = pos[1];
  result.position.z() = pos[2];

  return result;
}

}