#include "common/math/math_utils.h"
#include "reference_line/qp_spline_corridor_reference_line_generation.h"
#include "reference_line/matrix_operation.h"
#include "common/config/reference_line_config/smooth_reference_line_config.h"

namespace planning
{
    QpSplineReferenceLineSmooth::QpSplineReferenceLineSmooth(){}

    void QpSplineReferenceLineSmooth::setAnchorPoints(const std::vector<AnchorPoint> &anchor_points)
    {
        anchorPoints = anchor_points;
    }

    bool QpSplineReferenceLineSmooth::trajectoryGeneration(const std::vector<double>& sampleX,const std::vector<double>& sampleT,std::vector<c_float>& coefX)
    {
        Eigen::MatrixXd P,Q,A;
        Eigen::VectorXd lowerBound,upperBound;

        addKernel(sampleX,sampleT,P, Q );

        addConstraint(sampleX,sampleT,A, lowerBound,upperBound);

        OSQP_Solve(sampleX,P,Q,A,lowerBound,upperBound,coefX);

        return true;
    }

    std::vector<OriginalInsData> QpSplineReferenceLineSmooth::dataStitching(const std::vector<double>& coefX,
                       const std::vector<double>& coefY,std::vector<double>& sampleT,
                       const ReferenceLine &raw_reference_line,
                       ReferenceLine *const smoothed_reference_line)
    {
        std::vector<double> segmentS{0};
        std::vector<double> accumulateS{0};
        std::vector<ReferencePoint> ref_points;
        std::vector<double> ref_s;
        for(int i=0;i< static_cast<int>(coefX.size()/6);i++)
        {
            double para_x[6];
            double para_y[6];
            for(int j=i*6;j<(i+1)*6;j++)
            {
                para_x[j-6*i] = coefX[j];
                para_y[j-6*i] = coefY[j];
            }
            segmentS.emplace_back(splineCommon.ComputeS(sampleT[i],sampleT[i+1],para_x,para_y));
            accumulateS.emplace_back(accumulateS.back()+segmentS.back());
            int num_t = static_cast<int>( segmentS.back()/spacingDis);
            std::vector<double> ti;
            splineCommon.linspace(sampleT[i],sampleT[i+1],num_t,ti);

            for(int j=0;j<ti.size()-1;j++)
            {
                const double xs = splineCommon.PositionX(para_x,ti[j]);
                const double ys = splineCommon.PositionX(para_y,ti[j]);
                const double xds = splineCommon.DerivativeX(para_x,ti[j]);
                const double yds = splineCommon.DerivativeX(para_y,ti[j]);
                const double xseconds = splineCommon.SecondDerivativeX(para_x,ti[j]);
                const double yseconds = splineCommon.SecondDerivativeX(para_y,ti[j]);
                const double xthirds = splineCommon.ThirdDerivativeX(para_x,ti[j]);
                const double ythirds = splineCommon.ThirdDerivativeX(para_y,ti[j]);
                const double kappa  = splineCommon.ComputeCurvature(xds,xseconds,yds,yseconds);
                const double dkappa = splineCommon.ComputeCurvatureDerivative(xds,xseconds,xthirds,yds,yseconds,ythirds);
                double si = accumulateS[i] + splineCommon.ComputeS(sampleT[i],ti[j],para_x,para_y);
                ref_s.emplace_back(si);
                LinkLaneSegment linkLaneSegment = raw_reference_line.getLinkLaneByS(si + anchorPoints.front().abs_s);
                double heading =math::getAngle(xds,yds);
                had_map::MapPoint point_info;
                point_info.set_x(xs);
                point_info.set_y(ys);
                point_info.setHeading(heading);
                double left_width = 0;
                double right_width = 0;
                if(!raw_reference_line.getLeftAndRightWidth(point_info,left_width,right_width))
                {
                }
                double minSpeed=0;
                double maxSpeed=0;
                if(!raw_reference_line.getSpeed(point_info,maxSpeed,minSpeed))
                {
                }
                ref_points.emplace_back(ReferencePoint(point_info,kappa,dkappa,xds,yds,xseconds,yseconds,left_width,right_width,maxSpeed,minSpeed,linkLaneSegment));
            }
        }
	
        *smoothed_reference_line = ReferenceLine(ref_points,ref_s);
        smoothed_reference_line->setSpacingDis(spacingDis);
        smoothed_reference_line->reCalculateSegments();
        std::vector<double> xf,yf,yawf,kappaf,dkappaf;

        OriginalInsData insert;
        std::vector<OriginalInsData> smooth_point;

        for(int i=0;i<ref_points.size();i++)
        {
            insert.x = ref_points[i].pointInfo().x();
            insert.y = ref_points[i].pointInfo().y();
            insert.heading = ref_points[i].pointInfo().heading();
            insert.kappa = ref_points[i].kappa();
            insert.dkappa = ref_points[i].dkappa();
            smooth_point.emplace_back(insert);
        }

        return smooth_point;
    }

    std::vector<OriginalInsData> QpSplineReferenceLineSmooth::smooth(const ReferenceLine &raw_reference_line,
                                             ReferenceLine *const smoothed_reference_line)
    {
        //sample from anchorPoints
        std::vector<double> sampleX,sampleY;
        std::vector<double> sampleT{0};
        if (!sample(raw_reference_line,sampleX,sampleY,sampleT))
        {
            std::cout << "Fail to sample reference line smoother points!";
        }

        // coefficient calculation by optimization
        std::vector<c_float> coefX;
        std::vector<c_float> coefY;
        clock_t start,finish;
        start = clock();
        if (! trajectoryGeneration(sampleX,sampleT,coefX))
        {
            std::cout << "Fail to calculate t-x coefficient!";
        }
        if (! trajectoryGeneration(sampleY,sampleT,coefY))
        {
            std::cout << "Fail to calculate t-y coefficient!";
        }
        finish = clock();

        auto point_final = dataStitching(coefX,coefY,sampleT,raw_reference_line,smoothed_reference_line);

        return point_final;
    }

    bool QpSplineReferenceLineSmooth::sample(const ReferenceLine &raw_reference_line,std::vector<double>& sampleX,std::vector<double>& sampleY,
            std::vector<double>& sampleT)
    {
        int cout = raw_reference_line.accumulate_s_.size();
        std::vector<double> anchor_point_index;
        anchor_point_index.emplace_back(0);
        int count_last = 0;
        for(int i = 0;i<cout;++i)
        {
            if((raw_reference_line.accumulate_s_[i] - raw_reference_line.accumulate_s_[count_last])>=2.0) 
            {
                count_last = i;
                anchor_point_index.emplace_back(i);
            }
            if(i==cout-1)  anchor_point_index.emplace_back(i);
        }
        
        auto referencePoints = raw_reference_line.referencePoints();
        for(int j = 0;j<anchor_point_index.size()-1;++j)
        {    
            sampleX.emplace_back(referencePoints[anchor_point_index[j]].pointInfo().x());
            sampleY.emplace_back(referencePoints[anchor_point_index[j]].pointInfo().y());
        }
        anchor_point_index.clear();
        double interval = totalTime/(sampleX.size()-1);
        for(int i=1;i<sampleX.size();i++)
        {
            sampleT.emplace_back(sampleT.back()+interval);
        }
        return true;
    }

    bool QpSplineReferenceLineSmooth::addKernel(const std::vector<double>& sampleX,const std::vector<double>& sampleT,
                   Eigen::MatrixXd& P, Eigen::MatrixXd& Q )
    {
        int segmentNum = sampleX.size() - 1;
        int coefNum = splineOrder + 1;
        P = Eigen::MatrixXd::Zero(segmentNum*coefNum,segmentNum*coefNum);
        Eigen::MatrixXd P2 = Eigen::MatrixXd::Zero(segmentNum*coefNum,segmentNum*coefNum);
        Eigen::MatrixXd P3 = Eigen::MatrixXd::Zero(segmentNum*coefNum,segmentNum*coefNum);

        for(int i=0;i<segmentNum;i++)
        {
            Eigen::MatrixXd p = Eigen::MatrixXd::Zero(coefNum,coefNum);
            splineCommon.computeP(splineOrder,2,sampleT[i],sampleT[i+1],p);
            P2.block(i*coefNum,i*coefNum, coefNum, coefNum) = p;
            p = Eigen::MatrixXd::Zero(coefNum,coefNum);
            splineCommon.computeP(splineOrder,3,sampleT[i],sampleT[i+1],p);
            P3.block(i*coefNum,i*coefNum, coefNum, coefNum) = p;
        }

        Q = Eigen::MatrixXd::Zero(segmentNum*coefNum,1);
        Eigen::MatrixXd PGuide = Eigen::MatrixXd::Zero(segmentNum*coefNum,segmentNum*coefNum);
        Eigen::MatrixXd QGuide = Eigen::MatrixXd::Zero(segmentNum*coefNum,1);

        for(int i=0;i<segmentNum;i++)
        {
            double t1 = sampleT[i];
            double t2 = sampleT[i+1];
            double p1 = sampleX[i];
            double p2 = sampleX[i+1];
            double a1 = (p2-p1)/(t2-t1);
            double a0 = p1 - a1*t1;
            Eigen::MatrixXd ci = Eigen::MatrixXd::Zero(coefNum,1);
            ci(0,0) = a0;
            ci(1,0) = a1;
            Eigen::MatrixXd bi = Eigen::MatrixXd::Zero(coefNum,coefNum);
            splineCommon.computeP(splineOrder,0,t1,t2,bi);
            Eigen::MatrixXd qi = -bi*ci;
            PGuide.block(i*coefNum,i*coefNum,coefNum,coefNum) = bi;
            QGuide.block(i*coefNum,0,coefNum,1) = qi;
        }

        P = 2*P2+0.5*P3+lambda*PGuide;
        Q += lambda*QGuide;

        return true;
    }

    bool QpSplineReferenceLineSmooth::addConstraint(const std::vector<double>& sampleX,const std::vector<double>& sampleT,
                       Eigen::MatrixXd& A, Eigen::VectorXd& lowerBound,Eigen::VectorXd& upperBound)
    {
        int segmentNum = sampleX.size() - 1;
        int coefNum = splineOrder + 1;
        Eigen::MatrixXd Aequal = Eigen::MatrixXd::Zero(3*segmentNum-1,segmentNum*coefNum);
        Eigen::MatrixXd Bequal = Eigen::MatrixXd::Zero(3*segmentNum-1,1);
        Eigen::MatrixXd Ai = Eigen::MatrixXd::Zero(1,coefNum);
        splineCommon.computeTvec(sampleT[0],splineOrder,0,Ai);
        Aequal.row(0).segment(0,coefNum) = Ai;
        splineCommon.computeTvec(sampleT.back(),splineOrder,0,Ai);
        Aequal.row(1).segment(coefNum*(segmentNum-1),coefNum) =Ai;
        Bequal(0,0) = sampleX.front();
        Bequal(1,0) = sampleX.back();
        int neq = 1;
        Eigen::MatrixXd pos = Eigen::MatrixXd::Zero(1,coefNum);
        Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(1,coefNum);
        Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(1,coefNum);
        for(int i=0;i<segmentNum-1;i++)
        {
            splineCommon.computeTvec(sampleT[i+1],splineOrder,0,pos);
            splineCommon.computeTvec(sampleT[i+1],splineOrder,1,vel);
            splineCommon.computeTvec(sampleT[i+1],splineOrder,2,acc);
            neq += 1;
            Aequal.row(neq).segment(coefNum*i,coefNum) = pos;
            Aequal.row(neq).segment(coefNum*(i+1),coefNum) = -pos;
            neq += 1;
            Aequal.row(neq).segment(coefNum*i,coefNum) = vel;
            Aequal.row(neq).segment(coefNum*(i+1),coefNum) = -vel;
            neq += 1;
            Aequal.row(neq).segment(coefNum*i,coefNum) = acc;
            Aequal.row(neq).segment(coefNum*(i+1),coefNum) = -acc;
        }

        Eigen::MatrixXd Aieq = Eigen::MatrixXd::Zero(segmentNum-1,coefNum*segmentNum);
        Eigen::MatrixXd BieqL = Eigen::MatrixXd::Zero(segmentNum-1,1);
        Eigen::MatrixXd BieqH = Eigen::MatrixXd::Zero(segmentNum-1,1);
        Eigen::MatrixXd ieqPos = Eigen::MatrixXd::Zero(1,coefNum);
        for(int i=1;i<segmentNum;i++)
        {
            splineCommon.computeTvec(sampleT[i],splineOrder,0,ieqPos);
            Aieq.row(i-1).segment(i*coefNum,coefNum) = ieqPos;
            BieqL(i-1,0) = sampleX[i] - corridor;
            BieqH(i-1,0) = sampleX[i] + corridor;
        }

        Bequal.resize(Bequal.rows(),1);
        BieqL.resize(BieqL.rows(),1);
        lowerBound.resize(Bequal.rows()+BieqL.rows(),1);
        lowerBound<<Bequal,BieqL;
        BieqH.resize(BieqH.rows(),1);
        upperBound.resize(Bequal.rows()+BieqH.rows(),1);
        upperBound<<Bequal,BieqH;
        Aequal.resize(Aequal.rows(),Aequal.cols());
        Aieq.resize(Aieq.rows(),Aieq.cols());
        A.resize(Aequal.rows()+Aieq.rows(),Aequal.cols());
        A << Aequal, Aieq;
        return true;
    }

    bool QpSplineReferenceLineSmooth::OSQP_Solve(const std::vector<double>& sampleX,const Eigen::MatrixXd& P,const Eigen::MatrixXd& Q,
                    const Eigen::MatrixXd& A, const Eigen::VectorXd& lowerBound,const Eigen::VectorXd& upperBound,
                    std::vector<c_float>& res_x)
    {
        int segmentNum = sampleX.size() - 1;
        int coefNum = splineOrder + 1;
        std::vector<c_float> P_data;
        std::vector<c_int > P_indices;
        std::vector<c_int > P_indptr;
        math::DenseToCSCMatrix( P,P_data,P_indices,P_indptr);
        std::vector<c_float> A_data;
        std::vector<c_int > A_indices;
        std::vector<c_int > A_indptr;
        math::DenseToCSCMatrix( A,A_data,A_indices,A_indptr);
        c_int num_var = coefNum*segmentNum;
        c_int num_constraints = A.rows();
        std::vector<c_float> mini_q(Q.data(),Q.data()+Q.rows()*Q.cols());
        std::vector<c_float> mini_l(lowerBound.data(),lowerBound.data()+lowerBound.rows()*lowerBound.cols());
        std::vector<c_float> mini_u(upperBound.data(),upperBound.data()+upperBound.rows()*upperBound.cols());
        std::vector<c_float> primal_warm_start;

        OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
        OSQPSettings* settings =
                reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));

        // Define Solver settings
        osqp_set_default_settings(settings);
        settings->verbose = FLAGS_default_soqp_solver_verbose;
        settings->eps_abs = FLAGS_default_soqp_solver_eps_abs;
        settings->eps_rel = FLAGS_default_soqp_solver_eps_rel;
        settings->max_iter = FLAGS_default_osqp_iteration_num;
        OSQPWorkspace* work = nullptr;
        splineCommon.optimizeWithOsqp(num_var, num_constraints,
                         &P_data, &P_indices,&P_indptr,
                         &A_data, &A_indices,&A_indptr,
                         &mini_l,&mini_u,
                         &mini_q,&primal_warm_start,
                         data, &work, settings);
        auto res = work->solution->x;
        for(int i=0;i<coefNum*segmentNum;i++){
            res_x.emplace_back(res[i]);
        }

	return 1;
    }

    bool QpSplineReferenceLineSmooth::IsApproach(const double num1, const double num2, const double factor)
   {
    return ((num2 - num1) <= factor && (num2 - num1) >= -factor);
   }

   double QpSplineReferenceLineSmooth::GetLineDirection(double xsecond, double ysecond, double xfirst, double yfirst)
   {
    double alpha = 0.0;

    if ((IsApproach(xsecond, xfirst, MIN_)) && (ysecond > yfirst))
    {
        alpha = 90;
    }
    else if ((IsApproach(xsecond, xfirst, MIN_)) && (ysecond < yfirst))
    {
        alpha = -90;
    }
    else if ((IsApproach(xsecond, xfirst, MIN_)) && (IsApproach(ysecond, yfirst, MIN_)))
    {
        alpha = 0;
    }
    else
    {
      alpha = 180/M_PI*atan2((ysecond - yfirst),(xsecond - xfirst));
    }

    return alpha;
   }

}
