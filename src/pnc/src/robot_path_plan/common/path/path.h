

#ifndef PLANNING_PATH_H
#define PLANNING_PATH_H

#include "common/pnc_point/path_point.h"

#include <string>
#include <vector>

namespace planning {
    class Path {
    public:
        std::string name() const {
            return m_name;
        }

        const std::vector<PathPoint> &pathPoints() const {
            return m_pathPoints;
        }

        void setName(const std::string name) {
            m_name = name;
        }

        void setPathPoints(const std::vector<PathPoint> pathPoints) {
            m_pathPoints = (std::move(pathPoints));
        }

    private:
        std::string m_name;

        std::vector<PathPoint> m_pathPoints;
    };
}

#endif  //PLANNING_PATH_H
