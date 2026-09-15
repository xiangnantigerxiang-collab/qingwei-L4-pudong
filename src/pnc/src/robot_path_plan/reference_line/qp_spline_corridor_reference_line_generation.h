
#ifndef PLANNING_QP_SPLINE_REFERENCE_LINE_SMOOTH_H
#define PLANNING_QP_SPLINE_REFERENCE_LINE_SMOOTH_H

#include "reference_line.h"
#include "common/config/reference_line_config/smooth_reference_line_config.h"
#include "common/smooth_line/smooth_spline/spline_2d_solver.h"
#include "osqp/osqp.h"
#include "common/struct_type.h"
#include "common/smooth_line/qp_spline_corridor/qp_spline_corridor.h"

namespace planning {
    struct AnchorPoint {
        had_map::MapPoint pointInfo;
        double abs_s;
        double xds = 0.0;
        double yds = 0.0;
        double xseconds = 0.0;
        double yseconds = 0.0;
        double lateral_bound = 0.0;
        double longitudinal_bound = 0.0;
        bool enforced = false;
    };

    class QpSplineReferenceLineSmooth {
    public:
        QpSplineReferenceLineSmooth();

        void setAnchorPoints(const std::vector<AnchorPoint>& anchor_points);

        std::vector<OriginalInsData> smooth(const ReferenceLine& raw_reference_line,
                                            ReferenceLine* const smoothed_reference_line);

    private:
        bool trajectoryGeneration(const std::vector<double>& sampleX, const std::vector<double>& sampleT, std::vector<c_float>& coefX);

        bool sample(const ReferenceLine& raw_reference_line, std::vector<double>& sampleX, std::vector<double>& sampleY, std::vector<double>& sampleT);

        bool addKernel(const std::vector<double>& sampleX, const std::vector<double>& sampleT,
                       Eigen::MatrixXd& P, Eigen::MatrixXd& Q);

        bool addConstraint(const std::vector<double>& sampleX, const std::vector<double>& sampleT,
                           Eigen::MatrixXd& A, Eigen::VectorXd& lowerBound, Eigen::VectorXd& upperBound);

        bool OSQP_Solve(const std::vector<double>& sampleX, const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q,
                        const Eigen::MatrixXd& A, const Eigen::VectorXd& lowerBound, const Eigen::VectorXd& upperBound,
                        std::vector<c_float>& coefX);

        std::vector<OriginalInsData> dataStitching(const std::vector<double>& coefX,
                                                   const std::vector<double>& coefY, std::vector<double>& sampleT,
                                                   const ReferenceLine& raw_reference_line,
                                                   ReferenceLine* const smoothed_reference_line);

        bool IsApproach(const double num1, const double num2, const double factor);
        double GetLineDirection(double xsecond, double ysecond, double xfirst, double yfirst);

    private:
        std::vector<AnchorPoint> anchorPoints;
        SplineCommon splineCommon;

        double ref_x_ = 0.0;
        double ref_y_ = 0.0;
    };
}
#endif  //PLANNING_QP_SPLINE_REFERENCE_LINE_SMOOTH_H
