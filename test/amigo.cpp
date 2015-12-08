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

struct Image
{
    cv::Mat depth;
    cv::Mat rgb;
    mwm::ProjectionMatrix P;
    geo::Pose3D pose;
};

// ----------------------------------------------------------------------------------------------------

void update(mwm::WorldModel& wm, const Image& image)
{
    int width = image.depth.cols;
    int height = image.depth.rows;

//    cv::Mat depth = image.depth.clone();

    // - - - - - - - - - - - - - - - - - - - - - - - - - -

    geo::Pose3D sensor_pose_inv = image.pose.inverse();

    geo::Vec3 Rx = sensor_pose_inv.R.getRow(0);
    geo::Vec3 Ry = sensor_pose_inv.R.getRow(1);
    geo::Vec3 Rz = sensor_pose_inv.R.getRow(2);

    int step = 10;
    double depth_res = 0.1;
    double z_min = 0.5;
    double z_max = 5;

    cv::Mat vertex_map_z_min(image.depth.rows / step, image.depth.cols / step, CV_32FC1, 0.0);

    const std::vector<geo::Vec3>& points = wm.points();
    for(unsigned int i = 0; i < points.size(); ++i)
    {
        const geo::Vec3& p_world = points[i];

        geo::Vec3 p_sensor;
        p_sensor.z = Rz.dot(p_world) + sensor_pose_inv.t.z;

        float vz = -p_sensor.z;
        if (vz < z_min || vz > z_max) // Filter vertices that are behind the camera
            continue;

        p_sensor.x = Rx.dot(p_world) + sensor_pose_inv.t.x;
        p_sensor.y = Ry.dot(p_world) + sensor_pose_inv.t.y;

        int p_2d_x = image.P.project3Dto2DX(p_sensor);
        if (p_2d_x < 0 || p_2d_x >= width)
            continue;

        int p_2d_y = image.P.project3Dto2DY(p_sensor);
        if (p_2d_y < 0 || p_2d_y >= height)
            continue;

        float z = image.depth.at<float>(p_2d_y, p_2d_x);
        if (z == z && z > 0)
        {
            if (vz < z - 0.1)
            {
                // Remove vertex
            }
            else
            {
                if (vz < z + 0.1)
                {
                    // Update vertex
//                    float vz_new = vz;//(0.1 * z + 0.9 * vz);
//                    geo::Vec3 p_new = image.P.project2Dto3D(p_2d_x, p_2d_y) * vz_new;
//                    wm.setPoint(i, image.pose * p_new);
                }
                // else vertex is occluded

                int vmx = p_2d_x / step;
                int vmy = p_2d_y / step;

                float& vz_old = vertex_map_z_min.at<float>(vmy, vmx);
                if (vz_old == 0 || vz < vz_old)
                    vz_old = vz;
            }
        }
    }

//    cv::imshow("vertex_map_z_min", vertex_map_z_min / 10);
//    cv::waitKey(3);

    // - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Add missing vertices

    float f = (float)image.rgb.cols / image.depth.cols;

    for(int y = step / 2; y < image.depth.rows; y += step)
    {
        for(int x = step / 2; x < image.depth.cols; x += step)
        {
            float z = image.depth.at<float>(y, x);
            if (z < z_min || z > z_max || z != z)
                continue;

            int vmx = x / step;
            int vmy = y / step;
            float vmz = vertex_map_z_min.at<float>(vmy, vmx);

//            std::cout << vmx << ", " << vmy << ": " << vmz << std::endl;

            if (vmz > 0 && z > vmz - 0.1)
                continue;

            geo::Vec3 p_sensor = image.P.project2Dto3D(x, y) * z;
            geo::Vec3 p = image.pose * p_sensor;

            int x_rgb = f * x;
            int y_rgb = f * y;
            cv::Vec3b color = image.rgb.at<cv::Vec3b>(y_rgb, x_rgb);

            wm.addPoint(p, color, 1);
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

    mwm::WorldModel wm;

    while(ros::ok())
    {
        geo::Pose3D sensor_pose;
        rgbd::ImageConstPtr rgbd_image;

        if (!image_buffer.waitForRecentImage("/amigo/odom", rgbd_image, sensor_pose, 1.0))
            continue;

        Image image;
        image.pose = sensor_pose;
        image.depth = rgbd_image->getDepthImage();
        image.rgb = rgbd_image->getRGBImage();

        rgbd::View view(*rgbd_image, image.depth.cols);
        const geo::DepthCamera& cam_model = view.getRasterizer();
        image.P.setFocalLengths(cam_model.getFocalLengthX(), cam_model.getFocalLengthY());
        image.P.setOpticalCenter(cam_model.getOpticalCenterX(), cam_model.getOpticalCenterY());
        image.P.setOpticalTranslation(cam_model.getOpticalTranslationX(), cam_model.getOpticalTranslationY());

        std::cout << "-----------------" << std::endl;

        Timer timer;
        update(wm, image);
        std::cout << "Update took " << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;
        std::cout << wm.points().size() << " points" << std::endl;

//        std::cout << "Num vertices: " << wm.vertices().size() << std::endl;
//        std::cout << "Num triangles: " << wm.triangles().size() << std::endl;

//        cv::Mat rendered_depth_image(image.depth.rows, image.depth.cols, CV_32FC1, 0.0);

//        timer.reset();
//        mwm::render::Result res(rendered_depth_image);
//        mwm::render::renderDepth(wm, image.P, image.pose, res);
//        std::cout << "Rendering took " << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;

        viewer.tick(wm);


        cv::imshow("depth", image.depth / 10);
//        cv::imshow("rendered_depth_image", rendered_depth_image / 10);
        cv::waitKey(3);
    }

    return 0;
}



