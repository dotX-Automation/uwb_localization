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

#include <ceres/ceres.h> // sudo ln -s /usr/local/include/eigen3/Eigen /usr/include/Eigen

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;


namespace uwb_ceres {

class Function : public CostFunction {
public:
  Function(
    bool two_d_mode,
    bool squared,
    unsigned int count,
    std::vector<double>& distances, 
    std::vector<double>& anchors_x, 
    std::vector<double>& anchors_y, 
    std::vector<double>& anchors_z); 

  bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;
  bool EvaluateLinear2d(double const* const* parameters, double* residuals, double** jacobians) const;
  bool EvaluateLinear3d(double const* const* parameters, double* residuals, double** jacobians) const;
  bool EvaluateSquare2d(double const* const* parameters, double* residuals, double** jacobians) const;
  bool EvaluateSquare3d(double const* const* parameters, double* residuals, double** jacobians) const;

private:
  bool two_d_mode_;
  bool squared_;
  unsigned int count_;
  std::vector<double>& distances_;
  std::vector<double>& anchors_x_;
  std::vector<double>& anchors_y_;
  std::vector<double>& anchors_z_;
};


struct Result {
  Result() {}

  std::array<double, 3> position;
  ceres::Solver::Summary summary;
};

Result solve(Function *function, std::array<double, 3> &init);

}