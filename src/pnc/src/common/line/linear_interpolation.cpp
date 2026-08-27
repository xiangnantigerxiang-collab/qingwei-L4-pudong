/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "common/line/linear_interpolation.h"

#include <cmath>
#include <common/math/math_utils.h>

namespace math {

double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
    if (std::abs(t1 - t0) <= kMathEpsilon) {
      return NormalizeAngle(a0);
    }
    const double a0_n = NormalizeAngle(a0);
    const double a1_n = NormalizeAngle(a1);
    double d = a1_n - a0_n;
    if (d > M_PI) {
      d = d - 2 * M_PI;
    } else if (d < -M_PI) {
      d = d + 2 * M_PI;
    }

    const double r = (t - t0) / (t1 - t0);
    const double a = a0_n + d * r;
    return NormalizeAngle(a);
}

SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0,
                                            const SLPoint &p1, const double w) {
    SLPoint p;
    p.setS(((1 - w) * p0.s() + w * p1.s()));
    p.setL(((1 - w) * p0.l() + w * p1.l()));
    return p;
}

planning::PathPoint InterpolateUsingLinearApproximation(const planning::PathPoint &p0,
                                                        const planning::PathPoint &p1,
                                                        const double s)
{
      double s0 = p0.s();
      double s1 = p1.s();

      planning::PathPoint path_point;
      double weight = (s - s0) / (s1 - s0);
      double x = (1 - weight) * p0.x() + weight * p1.x();
      double y = (1 - weight) * p0.y() + weight * p1.y();
      double theta = slerp(p0.theta(), p0.s(), p1.theta(), p1.s(), s);
      double kappa = (1 - weight) * p0.kappa() + weight * p1.kappa();
      double dkappa = (1 - weight) * p0.dkappa() + weight * p1.dkappa();
      double ddkappa = (1 - weight) * p0.ddkappa() + weight * p1.ddkappa();
      path_point.setX(x);
      path_point.setY(y);
      path_point.setTheta(theta);
      path_point.setKappa(kappa);
      path_point.setDkappa(dkappa);
      path_point.setDdkappa(ddkappa);
      path_point.setS(s);
      return path_point;
}

}  // namespace math
