#ifndef _PATHMATCHER_H_
#define _PATHMATCHER_H_

#include <vector>
#include "common/pnc_point/path_point.h"

using namespace planning;

class PathMatcher
{
public:
    PathMatcher();
    ~PathMatcher();
    
    PathPoint MatchToPath(const std::vector<PathPoint>& reference_line,
                          const double x, const double y);

    PathPoint MatchToPath(const std::vector<PathPoint>& reference_line,
                          const double s);
    
    PathPoint FindProjectionPoint(const PathPoint& p0, const PathPoint& p1, 
		                  const double x, const double y); 
   
    PathPoint InterpolateUsingLinearApproximation(
                  const PathPoint &p0, const PathPoint &p1, const double s);
};

#endif //PATHMATCHER_H
