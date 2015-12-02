#include <mwm/ros/image_buffer.h>

#include <opencv2/highgui/highgui.hpp>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <ros/init.h>

#include <mwm/world_model.h>
#include <mwm/rendering.h>
#include <mwm/projection_matrix.h>
#include <mwm/visualization/viewer.h>

#include "timer.h"

// ----------------------------------------------------------------------------------------------------

void update(mwm::WorldModel& wm, const cv::Mat& depth_image, const geo::Pose3D& sensor_pose, const mwm::ProjectionMatrix& P)
{
    int width = depth_image.cols;
    int height = depth_image.rows;

    int step = 10;

    cv::Mat vertex_map(height / step, width / step, CV_32SC1, -1);

    for(int y = 0; y < height; y += step)
    {
        for(int x = 0; x < width; x += step)
        {
            float d = depth_image.at<float>(y, x);
            if (d != d || d == 0)
                continue;

            geo::Vec3 p = P.project2Dto3D(x, y) * d;

            vertex_map.at<int>(y / step, x / step) = wm.addVertex(sensor_pose * p);
        }
    }

    for(int y = 0; y < vertex_map.rows - 1; ++y)
    {
        for(int x = 0; x < vertex_map.cols - 1; ++x)
        {
            int i1 = vertex_map.at<int>(y, x);
            int i2 = vertex_map.at<int>(y, x + 1);
            int i3 = vertex_map.at<int>(y + 1, x);
            int i4 = vertex_map.at<int>(y + 1, x + 1);

            if (i1 >= 0 && i2 >= 0 && i3 >= 0)
                wm.addTriangle(i2, i1, i3);

            if (i2 >= 0 && i3 >= 0 && i4 >= 0)
                wm.addTriangle(i2, i3, i4);
        }
    }
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mwm_test_amigo");

    mwm::ImageBuffer image_buffer;
    image_buffer.initialize("/amigo/top_kinect/rgbd");

    mwm::Viewer viewer("world model", 640, 480);

    while(ros::ok())
    {
        geo::Pose3D sensor_pose;
        rgbd::ImageConstPtr image;

        if (!image_buffer.waitForRecentImage("/amigo/base_link", image, sensor_pose, 1.0))
            continue;

        const cv::Mat& depth_image = image->getDepthImage();

        rgbd::View view(*image, depth_image.cols);
        const geo::DepthCamera& cam_model = view.getRasterizer();

        mwm::ProjectionMatrix P;
        P.setFocalLengths(cam_model.getFocalLengthX(), cam_model.getFocalLengthY());
        P.setOpticalCenter(cam_model.getOpticalCenterX(), cam_model.getOpticalCenterY());
        P.setOpticalTranslation(cam_model.getOpticalTranslationX(), cam_model.getOpticalTranslationY());

        std::cout << "-----------------" << std::endl;

        mwm::WorldModel wm;

        Timer timer;
        update(wm, depth_image, sensor_pose, P);
        std::cout << "Update took " << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;

        std::cout << "Num vertices: " << wm.vertices().size() << std::endl;
        std::cout << "Num triangles: " << wm.triangles().size() << std::endl;

        cv::Mat rendered_depth_image(depth_image.rows, depth_image.cols, CV_32FC1, 0.0);

        timer.reset();
        mwm::render::Result res(rendered_depth_image);
        mwm::render::renderDepth(wm, P, sensor_pose, res);
        std::cout << "Rendering took " << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;

        viewer.tick(wm);


        cv::imshow("depth", depth_image / 10);
        cv::imshow("rendered_depth_image", rendered_depth_image / 10);
        cv::waitKey(3);
    }

    return 0;
}



