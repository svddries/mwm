#ifndef MWM_RENDERING_H_
#define MWM_RENDERING_H_

#include <geolib/datatypes.h>
#include <opencv2/core/core.hpp>

namespace mwm
{

class WorldModel;
class ProjectionMatrix;
class Triangle;

namespace render
{

struct Result
{

    Result(cv::Mat& z_buffer_) : z_buffer(z_buffer_) {}

    virtual void triangleHook(const Triangle& t, const geo::Vec3& v1, const geo::Vec3& v2, const geo::Vec3& v3) { }

    virtual void renderPixel(int x, int y, float depth)
    {
        float& d = z_buffer.at<float>(y, x);
        if (d == 0 || depth < d)
            d = depth;
    }

    cv::Mat& z_buffer;
    bool first_triangle_pixel;
    unsigned int triangle_num;
};

void renderDepth(const WorldModel& wm, const ProjectionMatrix& P, const geo::Pose3D& sensor_pose, Result& res);

}

} // end namespace mwm

#endif
