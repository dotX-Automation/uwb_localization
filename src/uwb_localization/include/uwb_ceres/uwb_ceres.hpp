/**
 * UWB localization ceres utilities header.
 *
 * Giorgio Manca <giorgio.manca97@gmail.com>
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 27, 2023
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

#include <ceres/ceres.h> // sudo ln -s /usr/local/include/eigen3/Eigen /usr/include/Eigen

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;


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

  inline bool is2d() {
      return two_d_mode_;
  }

  inline bool isSquared() {
      return squared_;
  }

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