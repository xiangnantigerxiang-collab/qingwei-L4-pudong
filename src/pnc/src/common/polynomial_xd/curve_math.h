
#ifndef PLANNING_CURVE_MATH_H
#define PLANNING_CURVE_MATH_H
namespace planning {

class CurveMath {
 public:
  CurveMath() = delete;
  /**
   * @brief Compute the curvature (kappa) given curve X = (x(t), y(t))
   *        which t is an arbitrary parameter.
   * @param dx dx / dt
   * @param d2x d(dx) / dt
   * @param dy dy / dt
   * @param d2y d(dy) / dt
   * @return the curvature
   */
  static double ComputeCurvature_(const double dx, const double d2x,
                                 const double dy, const double d2y);

    static double ComputeCurvature_(const double dy, const double d2y);

  /**
   * @brief Compute the curvature change rate w.r.t. curve length (dkappa) given
   * curve X = (x(t), y(t))
   *        which t is an arbitrary parameter.
   * @param dx dx / dt
   * @param d2x d(dx) / dt
   * @param dy dy / dt
   * @param d2y d(dy) / dt
   * @param d3x d(d2x) / dt
   * @param d3y d(d2y) / dt
   * @return the curvature change rate
   */
  static double ComputeCurvatureDerivative_(const double dx, const double d2x,
                                           const double d3x, const double dy,
                                           const double d2y, const double d3y);
};

}  // namespace planning
#endif //PLANNING_CURVE_MATH_H
