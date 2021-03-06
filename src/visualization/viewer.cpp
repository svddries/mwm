#include "mwm/visualization/viewer.h"

#include "mwm/world_model.h"
#include "mwm/rendering.h"
#include "mwm/triangle.h"

#include <opencv2/highgui/highgui.hpp>

#include <iostream>

namespace mwm
{

class LightingRenderer : public render::Result
{

// ----------------------------------------------------------------------------------------------------

public:

    LightingRenderer(cv::Mat& z_buffer, cv::Mat& canvas, const geo::Pose3D& sensor_pose)
        : render::Result(z_buffer), canvas_(canvas), sensor_pose_(sensor_pose) {}

    void triangleHook(const Triangle& t, const geo::Vec3& p1, const geo::Vec3& p2, const geo::Vec3& p3)
    {
        geo::Vec3 n = ((p3 - p1).cross(p2 - p1)).normalized();
        n = sensor_pose_.R * n;
        double v = (1 + n.dot(geo::Vec3(0, 0.3, -1).normalized())) / 2;
//        color_ = v * t.color;
        color_ = t.color;
    }

    void renderPixel(int x, int y, float depth)
    {
        float& d = z_buffer.at<float>(y, x);
        if (d == 0 || depth < d)
        {
            d = depth;
            canvas_.at<cv::Vec3b>(y, x) = color_;
        }
    }

private:

    cv::Vec3b color_;
    cv::Mat& canvas_;
    geo::Pose3D sensor_pose_;
};

// ----------------------------------------------------------------------------------------------------

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    CamControlData* data = static_cast<CamControlData*>(userdata);

    if (event == cv::EVENT_LBUTTONDBLCLK)
    {
        std::cout << x << ", " << y << std::endl;
    }
    else if (event == cv::EVENT_LBUTTONDOWN)
    {
        data->last_mouse_pos = cv::Point(x, y);
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        data->last_mouse_pos = cv::Point(x, y);

    }
    else if (event == cv::EVENT_MBUTTONDOWN)
    {
        data->last_mouse_pos = cv::Point(x, y);

    }
    else if (event == cv::EVENT_MOUSEMOVE)
    {
        double dx = x - data->last_mouse_pos.x;
        double dy = y - data->last_mouse_pos.y;

        if (flags & cv::EVENT_FLAG_LBUTTON)
        {
            data->cam_yaw -= dx * 0.003;
            data->cam_pitch += dy * 0.003;

            if (data->cam_pitch > 1.57)
                data->cam_pitch = 1.57;
            else if (data->cam_pitch < -1.57)
                data->cam_pitch = -1.57;
        }
        else if (flags & cv::EVENT_FLAG_MBUTTON)
        {
            data->cam_dist += data->cam_dist * dy * 0.002;
        }
        else if (flags & cv::EVENT_FLAG_RBUTTON)
        {
            data->cam_lookat += data->cam_pose.R * (geo::Vector3(-dx, dy, 0) * 0.001 * data->cam_dist);
        }

        data->last_mouse_pos = cv::Point(x, y);
    }
}

// ----------------------------------------------------------------------------------------------------

Viewer::Viewer() : cam_initialized_(false) {}

// ----------------------------------------------------------------------------------------------------

Viewer::Viewer(const std::string& name, int width, int height)
{
    initialize(name, width, height);
}

// ----------------------------------------------------------------------------------------------------

Viewer::~Viewer()
{
}

// ----------------------------------------------------------------------------------------------------

void Viewer::initialize(const std::string& name, int width, int height)
{
    name_ = name;

    //Create a window
    cv::namedWindow(name_, 1);

    //set the callback function for any mouse event
    cv::setMouseCallback(name_, CallBackFunc, &cam_control_);

    canvas_ = cv::Mat(height, width, CV_8UC3, cv::Scalar(20, 20, 20));

    // Set camera specs
    P_.setFocalLengths(0.87 * width, 0.87 * width);
    P_.setOpticalCenter(width / 2 + 0.5, height / 2 + 0.5);
    P_.setOpticalTranslation(0, 0);
}

// ----------------------------------------------------------------------------------------------------

void Viewer::render(const WorldModel& wm)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if (!cam_initialized_)
    {
        if (!wm.points().empty() || !wm.vertices().empty())
        {
            geo::Vec3 p_total(0, 0, 0);
            for(unsigned int i = 0; i < wm.points().size(); ++i)
                p_total += wm.points()[i];

            for(unsigned int i = 0; i < wm.vertices().size(); ++i)
                p_total += wm.vertices()[i];

            cam_control_.cam_lookat = p_total / (wm.points().size() + wm.vertices().size());
            cam_initialized_ = true;
        }
        else
        {
            cam_control_.cam_lookat = geo::Vec3(0, 0, 0);
        }

        cam_control_.cam_dist = 5;
        cam_control_.cam_pitch = 0.7;
        cam_control_.cam_yaw = 3.1415;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Calculate camera pose

    cam_control_.cam_pose.t = geo::Vector3(cos(cam_control_.cam_yaw), sin(cam_control_.cam_yaw), 0)
                                * cos(cam_control_.cam_pitch) * cam_control_.cam_dist;
    cam_control_.cam_pose.t.z = sin(cam_control_.cam_pitch) * cam_control_.cam_dist;
    cam_control_.cam_pose.t += cam_control_.cam_lookat;

    geo::Vec3 rz = -(cam_control_.cam_lookat - cam_control_.cam_pose.t).normalized();
    geo::Vec3 rx = geo::Vector3(0, 0, 1).cross(rz).normalized();
    geo::Vec3 ry = rz.cross(rx).normalized();

    cam_control_.cam_pose.R = geo::Mat3(rx.x, ry.x, rz.x,
                                        rx.y, ry.y, rz.y,
                                        rx.z, ry.z, rz.z);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    canvas_.setTo(cv::Vec3b(20, 20, 20));
    cv::Mat depth_image(canvas_.rows, canvas_.cols, CV_32FC1, 0.0);

    LightingRenderer res(depth_image, canvas_, cam_control_.cam_pose);
    mwm::render::renderDepth(wm, P_, cam_control_.cam_pose, res);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    geo::Pose3D sensor_pose_inv = cam_control_.cam_pose.inverse();

    for(unsigned int i = 0; i < wm.points().size(); ++i)
    {
        const geo::Vec3& p = wm.points()[i];
        const cv::Vec3b& color = wm.point_colors()[i];

        geo::Vec3 p_sensor = sensor_pose_inv * p;
        geo::Vec2i p_2d = P_.project3Dto2D(p_sensor);
        double z = -p_sensor.z;

//        if (z < 0)
//            continue;

        if (p_2d.x < 0 || p_2d.y < 0 || p_2d.x >= canvas_.cols || p_2d.y >= canvas_.rows)
            continue;

        float& d = depth_image.at<float>(p_2d.y, p_2d.x);
        if (d == 0 || z < d)
        {
            d = z;
            canvas_.at<cv::Vec3b>(p_2d.y, p_2d.x) = color;
//            cv::circle(canvas_, cv::Point(p_2d.x, p_2d.y), 3, cv::Scalar(color[0], color[1], color[2]), CV_FILLED);
        }
    }

//    unsigned int size = canvas_.rows * canvas_.cols;
//    for(unsigned int i = 0; i < size; ++i)
//    {
//        float d = depth_image.at<float>(i);
//        if (d == 0)
//            canvas_.at<cv::Vec3b>(i) = cv::Vec3b(20, 20, 20);
//        else
//            canvas_.at<cv::Vec3b>(i) = (d / 10) * cv::Vec3b(255, 255, 255);
//    }
}

// ----------------------------------------------------------------------------------------------------

void Viewer::tick(const WorldModel& wm)
{
    render(wm);

    cv::imshow(name_, canvas_);
    cv::waitKey(3);
}

// ----------------------------------------------------------------------------------------------------

} // end namespace mwm

