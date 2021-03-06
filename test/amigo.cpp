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

void updateTriangles(mwm::WorldModel& wm, const Image& image)
{
    int width = image.depth.cols;
    int height = image.depth.rows;

    int step = 10;

    // - - - - - - - - - - - - - - - - - - - - - - - - - -

    cv::Mat vertex_map(height / step, width / step, CV_32SC1, cv::Scalar(-1));
    cv::Mat vertex_map_z(height / step, width / step, CV_32FC1, 0.0);

    geo::Pose3D sensor_pose_inv = image.pose.inverse();

    const std::vector<geo::Vec3>& vertices = wm.vertices();
    for(unsigned int i = 0; i < vertices.size(); ++i)
    {
        const geo::Vec3& p = vertices[i];

        geo::Vec3 p_sensor = sensor_pose_inv * p;

        float vz = -p_sensor.z;
        if (vz < 0) // Filter vertices that are behind the camera
            continue;

        geo::Vec2i p_2d = image.P.project3Dto2D(p_sensor);
        if (p_2d.x < 0 || p_2d.y < 0 || p_2d.x >= width || p_2d.y >= height)
            continue;

        int vx = p_2d.x / step;
        int vy = p_2d.y / step;

        int vi = vertex_map.at<int>(vy, vx);
        if (vi <= 0 || vertex_map_z.at<float>(vy, vx) < vz)
        {
            vertex_map.at<int>(p_2d.y / step, p_2d.x / step) = i;
            vertex_map_z.at<float>(p_2d.y / step, p_2d.x / step) = -p_sensor.z;
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Add missing vertices

    cv::Mat vertex_map_new(height / step, width / step, CV_8UC1, cv::Scalar(0));

    for(int y = 0; y < vertex_map.rows - 1; ++y)
    {
        for(int x = 0; x < vertex_map.cols - 1; ++x)
        {
            int ix = step * (x + 0.5);
            int iy = step * (y + 0.5);

            float d = image.depth.at<float>(iy, ix);

            if (d != d || d <= 0.5 || d > 5)
                continue;

            int i = vertex_map.at<int>(y, x);
            if (i >= 0 && std::abs(vertex_map_z.at<float>(y, x) - d) < 0.2)
                continue;

            geo::Vec3 p_sensor = image.P.project2Dto3D(ix, iy) * d;
            geo::Vec3 p = image.pose * p_sensor;
            vertex_map.at<int>(y, x) = wm.addVertex(p);
            vertex_map_z.at<float>(y, x) = d;
            vertex_map_new.at<unsigned char>(y, x) = 1;
        }
    }

//    cv::imshow("vertices", vertex_map_z / 10);
//    cv::imshow("vertices_new", vertex_map_new * 255);
//    cv::waitKey(3);


    // - - - - - - - - - - - - - - - - - - - - - - - - - -

    float d_thr = 0.2;

    for(int y = 0; y < vertex_map.rows - 1; ++y)
    {
        for(int x = 0; x < vertex_map.cols - 1; ++x)
        {
//            int ix = (0.5 + step) * x;
//            int iy = (0.5 + step) * y;

//            float d = depth_image.at<float>(iy, ix);
//            if (d != d || d == 0)
//                continue;


            int i1 = vertex_map.at<int>(y, x);
            int i2 = vertex_map.at<int>(y, x + 1);
            int i3 = vertex_map.at<int>(y + 1, x);
            int i4 = vertex_map.at<int>(y + 1, x + 1);

            float z1 = vertex_map_z.at<float>(y, x);
            float z2 = vertex_map_z.at<float>(y, x + 1);
            float z3 = vertex_map_z.at<float>(y + 1, x);
            float z4 = vertex_map_z.at<float>(y + 1, x + 1);

            float z1z2 = std::abs(z1 - z2);
            float z1z3 = std::abs(z1 - z3);
            float z2z3 = std::abs(z2 - z3);
            float z2z4 = std::abs(z2 - z4);
            float z3z4 = std::abs(z3 - z4);

            bool is_new1 = (vertex_map_new.at<unsigned char>(y, x) == 1);
            bool is_new2 = (vertex_map_new.at<unsigned char>(y, x) == 1);
            bool is_new3 = (vertex_map_new.at<unsigned char>(y, x) == 1);
            bool is_new4 = (vertex_map_new.at<unsigned char>(y, x) == 1);

            if (i1 >= 0 && i2 >= 0 && i3 >= 0 && (is_new1 || is_new2 || is_new3)
                    && z1z2 < d_thr && z1z3 < d_thr && z2z3 < d_thr)
            {
                float f = (float)image.rgb.cols / vertex_map.cols;
                int x_rgb = f * x;
                int y_rgb = f * y;
                cv::Vec3b clr = image.rgb.at<cv::Vec3b>(y_rgb, x_rgb);
                wm.addTriangle(i2, i1, i3, clr);
            }

            if (i2 >= 0 && i3 >= 0 && i4 >= 0 && (is_new2 || is_new3 || is_new4)
                    && z2z4 < d_thr && z3z4 < d_thr && z2z3 < d_thr)
            {
                float f = (float)image.rgb.cols / vertex_map.cols;
                int x_rgb = f * x;
                int y_rgb = f * y;
                cv::Vec3b clr = image.rgb.at<cv::Vec3b>(y_rgb, x_rgb);
                wm.addTriangle(i2, i3, i4, clr);
            }
        }
    }
}

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
        updateTriangles(wm, image);
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



