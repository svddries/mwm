#ifndef MWM_RENDERING_H_
#define MWM_RENDERING_H_

#include <geolib/datatypes.h>
#include <opencv2/core/core.hpp>

namespace mwm
{

class WorldModel;
class ProjectionMatrix;

namespace render
{

void renderDepth(const WorldModel& wm, const ProjectionMatrix& P, const geo::Pose3D& sensor_pose, cv::Mat& depth);

}

} // end namespace mwm

#endif
