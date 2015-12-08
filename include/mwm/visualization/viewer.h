#ifndef MWM_VIEWER_H_
#define MWM_VIEWER_H_

#include <string>

#include <opencv2/core/core.hpp>

#include "mwm/projection_matrix.h"

namespace mwm
{

class WorldModel;

struct CamControlData
{
    geo::Vector3 cam_lookat;
    double cam_dist, cam_yaw, cam_pitch;
    geo::Pose3D cam_pose;
    cv::Point last_mouse_pos;
};

class Viewer
{

public:

    Viewer();

    Viewer(const std::string& name, int width, int height);

    ~Viewer();

    void initialize(const std::string& name, int width, int height);

    void tick(const WorldModel& wm);

private:

    std::string name_;

    cv::Mat canvas_;

    ProjectionMatrix P_;


    // Camera control

    bool cam_initialized_;

    CamControlData cam_control_;


    //

    void render(const WorldModel& wm);

};

} // end namespace mwm

#endif
