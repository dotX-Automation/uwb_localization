/**
 * UWB localization ceres utilities header.
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

#include <ceres/ceres.h>
#include <Eigen/Core>

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;

namespace uwb_ceres {

struct Point {
  Point() {}

  double x;
  double y;
  double z;
};


struct Measure {
  Measure() {}

  double distance;
  Eigen::Vector3d anchor;
};


class Function : public CostFunction {
public:
  Function(
    bool two_d_mode,
    bool squared,
    std::vector<Measure>& measures_); 

  bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;
  bool EvaluateLinear2d(double const* const* parameters, double* residuals, double** jacobians) const;
  bool EvaluateLinear3d(double const* const* parameters, double* residuals, double** jacobians) const;
  bool EvaluateSquare2d(double const* const* parameters, double* residuals, double** jacobians) const;
  bool EvaluateSquare3d(double const* const* parameters, double* residuals, double** jacobians) const;

  bool two_d_mode;
  bool squared;
  std::vector<Measure>& measures;
};


struct Result {
  Result() {}

  Eigen::Vector3d position;
  ceres::Solver::Summary summary;
};

Result solve(Function &function, Eigen::Vector3d &init);

}