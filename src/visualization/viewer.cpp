#include "mwm/visualization/viewer.h"

#include "mwm/rendering.h"

#include <opencv2/highgui/highgui.hpp>

#include <iostream>

namespace mwm
{

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

Viewer::Viewer() {}

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

    // Camera control
    cam_control_.cam_lookat = geo::Vec3(0, 0, 0);
    cam_control_.cam_dist = 5;
    cam_control_.cam_pitch = 0.7;
    cam_control_.cam_yaw = 3.1415;

}

// ----------------------------------------------------------------------------------------------------

void Viewer::render(const WorldModel& wm)
{
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

    cv::Mat depth_image(canvas_.rows, canvas_.cols, CV_32FC1, 0.0);
    mwm::render::renderDepth(wm, P_, cam_control_.cam_pose, depth_image);

    unsigned int size = canvas_.rows * canvas_.cols;
    for(unsigned int i = 0; i < size; ++i)
    {
        float d = depth_image.at<float>(i);
        if (d == 0)
            canvas_.at<cv::Vec3b>(i) = cv::Vec3b(20, 20, 20);
        else
            canvas_.at<cv::Vec3b>(i) = (d / 10) * cv::Vec3b(255, 255, 255);
    }
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

