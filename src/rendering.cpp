#include "mwm/rendering.h"

#include "mwm/world_model.h"
#include "mwm/projection_matrix.h"

namespace mwm
{

namespace render
{


// -------------------------------------------------------------------------------

void sort(const geo::Vec3f& p1, const geo::Vec3f& p2, const geo::Vec3f& p3, int i,
          const geo::Vec3f*& p_min, const geo::Vec3f*& p_mid, const geo::Vec3f*& p_max)
{
    if (p1.m[i] < p2.m[i])
    {
        if (p2.m[i] < p3.m[i]) {
            p_min = &p1; p_mid = &p2; p_max = &p3;
        } else if (p3.m[i] < p1.m[i]) {
            p_min = &p3; p_mid = &p1; p_max = &p2;
        } else {
            p_min = &p1; p_mid = &p3; p_max = &p2;
        }
    }
    else
    {
        if (p1.m[i] < p3.m[i]) {
            p_min = &p2; p_mid = &p1; p_max = &p3;
        } else if (p3.m[i] < p2.m[i]) {
            p_min = &p3; p_mid = &p2; p_max = &p1;
        } else {
            p_min = &p2; p_mid = &p3; p_max = &p1;
        }
    }
}

// -------------------------------------------------------------------------------

void drawTrianglePart(int y_start, int y_end,
                      float x_start, float x_start_delta, float x_end, float x_end_delta,
                      float d_start, float d_start_delta, float d_end, float d_end_delta,
                      Result& res)
{

    cv::Mat& depth_image = res.z_buffer;

    if (y_start < 0)
    {
        d_start += d_start_delta * -y_start;
        d_end += d_end_delta * -y_start;
        x_start += x_start_delta * -y_start;
        x_end += x_end_delta * -y_start;
        y_start = 0;
    }

    y_end = std::min(depth_image.rows - 1, y_end);

    for(int y = y_start; y <= y_end; ++y) {
        float d = d_start;
        float d_delta = (d_end - d_start) / (x_end - x_start);

        int x_start2;
        if (x_start < 0) {
            d += d_delta * -x_start;
            x_start2 = 0;
        } else {
            x_start2 = x_start;
        }

        int x_end2 = std::min(depth_image.cols - 1, (int)x_end);

        for(int x = x_start2; x <= x_end2; ++x)
        {
            float depth = 1.0f / d;
            res.renderPixel(x, y, depth);

//            float& di = depth_image.at<float>(y, x);
//            if (di == 0 || depth < di)
//                di = depth;
            d += d_delta;
        }

        d_start+= d_start_delta;
        d_end += d_end_delta;
        x_start += x_start_delta;
        x_end += x_end_delta;
    }
}

// -------------------------------------------------------------------------------

void drawTriangle2D(const geo::Vec3f& p1, const geo::Vec3f& p2, const geo::Vec3f& p3, Result& res)
{
    bool back_face_culling = true;
    const cv::Mat& depth = res.z_buffer;

    if (back_face_culling && (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y) >= 0)
        return;

    int min_y = std::min<int>(p1.y, std::min<int>(p2.y, p3.y));
    int max_y = std::max<int>(p1.y, std::max<int>(p2.y, p3.y));
    int min_x = std::min<int>(p1.x, std::min<int>(p2.x, p3.x));
    int max_x = std::max<int>(p1.x, std::max<int>(p2.x, p3.x));

    if (min_x >= depth.cols || max_x < 0 || min_y >= depth.rows || max_y < 0)
        return;

    if (min_y == max_y)
    {
        const geo::Vec3f *p_min, *p_mid, *p_max;
        sort(p1, p2, p3, 0, p_min, p_mid, p_max);

        drawTrianglePart(p_min->y, p_mid->y,
                         p_min->x, 0, p_max->x, 0,
                         p_min->z, 0, p_max->z, 0,
                         res);
    }
    else
    {
        const geo::Vec3f *p_min, *p_mid, *p_max;
        sort(p1, p2, p3, 1, p_min, p_mid, p_max);

        int y_min_mid = (int)p_mid->y - (int)p_min->y;
        int y_mid_max = (int)p_max->y - (int)p_mid->y;
        int y_min_max = (int)p_max->y - (int)p_min->y;

        geo::Vec3f p_prime = (y_mid_max * *p_min + y_min_mid * *p_max) / y_min_max;

        geo::Vec3f p_a, p_b;
        if (p_prime.x < p_mid->x)
        {
            p_a = p_prime; p_b = *p_mid;
        }
        else
        {
            p_a = *p_mid; p_b = p_prime;
        }

        drawTrianglePart(p_min->y, p_mid->y,
                         p_min->x, (p_a.x - p_min->x) / y_min_mid, p_min->x, (p_b.x - p_min->x) / y_min_mid,
                         p_min->z, (p_a.z - p_min->z) / y_min_mid, p_min->z, (p_b.z - p_min->z) / y_min_mid,
                         res);

        drawTrianglePart(p_mid->y, p_max->y,
                         p_a.x, (p_max->x - p_a.x) / y_mid_max, p_b.x, (p_max->x - p_b.x) / y_mid_max,
                         p_a.z, (p_max->z - p_a.z) / y_mid_max, p_b.z, (p_max->z - p_b.z) / y_mid_max,
                         res);

    }
}

// -------------------------------------------------------------------------------

void drawTriangle(const geo::Vec3& p1_3d, const geo::Vec3& p2_3d, const geo::Vec3& p3_3d, const ProjectionMatrix& P, Result& res)
{
    geo::Vec2i p1_2d = P.project3Dto2D(p1_3d);
    geo::Vec2i p2_2d = P.project3Dto2D(p2_3d);
    geo::Vec2i p3_2d = P.project3Dto2D(p3_3d);

    drawTriangle2D(geo::Vec3f(p1_2d.x, p1_2d.y, 1.0f / -p1_3d.z),
                   geo::Vec3f(p2_2d.x, p2_2d.y, 1.0f / -p2_3d.z),
                   geo::Vec3f(p3_2d.x, p3_2d.y, 1.0f / -p3_3d.z),
                   res);
}

// ----------------------------------------------------------------------------------------------------

void renderDepth(const WorldModel& wm, const ProjectionMatrix& P, const geo::Pose3D& sensor_pose, Result& res)
{
    // Parameters
    double near_clip_z = -0.1;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    const std::vector<Triangle>& triangles = wm.triangles();
    const std::vector<geo::Vec3>& vertices = wm.vertices();

    // transform points
    std::vector<geo::Vec3> vertices_t(vertices.size());
    std::vector<geo::Vec2i> vertices_proj(vertices.size());

    geo::Pose3D sensor_pose_inv = sensor_pose.inverse();

    for(unsigned int i = 0; i < vertices.size(); ++i)
    {
        vertices_t[i] = sensor_pose_inv * vertices[i];
        vertices_proj[i] = P.project3Dto2D(vertices_t[i]);
    }

    for(const auto& t : triangles)
    {
        const geo::Vec3& p1_3d = vertices_t[t.i1];
        const geo::Vec3& p2_3d = vertices_t[t.i2];
        const geo::Vec3& p3_3d = vertices_t[t.i3];

        res.triangleHook(t, p1_3d, p2_3d, p3_3d);

        int n_verts_in = 0;
        bool v1_in = false;
        bool v2_in = false;
        bool v3_in = false;
        const geo::Vec3* vIn[3];

        if (p1_3d.z < near_clip_z) {
            ++n_verts_in;
            v1_in = true;
        }

        if (p2_3d.z < near_clip_z) {
            ++n_verts_in;
            v2_in = true;
        }

        if (p3_3d.z < near_clip_z) {
            ++n_verts_in;
            v3_in = true;
        }

        if (n_verts_in == 1)
        {
            if (v1_in) { vIn[0] = &(p1_3d); vIn[1] = &(p2_3d); vIn[2] = &(p3_3d); }
            if (v2_in) { vIn[0] = &(p2_3d); vIn[1] = &(p3_3d); vIn[2] = &(p1_3d); }
            if (v3_in) { vIn[0] = &(p3_3d); vIn[1] = &(p1_3d); vIn[2] = &(p2_3d); }

            //Parametric line stuff
            // p = v0 + v01*t
            geo::Vec3 v01 = *vIn[1] - *vIn[0];

            float t1 = ((near_clip_z - (*vIn[0]).z) / v01.z );

            geo::Vec3 new2(vIn[0]->x + v01.x * t1, vIn[0]->y + v01.y * t1, near_clip_z);

            // Second vert point
            geo::Vec3 v02 = *vIn[2] - *vIn[0];

            float t2 = ((near_clip_z - (*vIn[0]).z) / v02.z);

            geo::Vec3 new3(vIn[0]->x + v02.x * t2, vIn[0]->y + v02.y * t2, near_clip_z);

            drawTriangle(*vIn[0], new2, new3, P, res);
        }
        else if (n_verts_in == 2)
        {
            if (!v1_in) { vIn[0]=&(p2_3d); vIn[1]=&(p3_3d); vIn[2]=&(p1_3d); }
            if (!v2_in) { vIn[0]=&(p3_3d); vIn[1]=&(p1_3d); vIn[2]=&(p2_3d); }
            if (!v3_in) { vIn[0]=&(p1_3d); vIn[1]=&(p2_3d); vIn[2]=&(p3_3d); }

            //Parametric line stuff
            // p = v0 + v01*t
            geo::Vec3 v01 = *vIn[2] - *vIn[0];

            float t1 = ((near_clip_z - (*vIn[0]).z)/v01.z );

            geo::Vec3 new2((*vIn[0]).x + v01.x * t1,(*vIn[0]).y + v01.y * t1, near_clip_z);

            // Second point
            geo::Vec3 v02 = *vIn[2] - *vIn[1];

            float t2 = ((near_clip_z - (*vIn[1]).z)/v02.z);

            geo::Vec3 new3((*vIn[1]).x + v02.x * t2, (*vIn[1]).y + v02.y * t2, near_clip_z);

            drawTriangle(*vIn[0], *vIn[1], new2, P, res);

            drawTriangle(new2, *vIn[1], new3, P, res);

        }
        else if (n_verts_in == 3)
        {
            const geo::Vec2i& p1_2d = vertices_proj[t.i1];
            const geo::Vec2i& p2_2d = vertices_proj[t.i2];
            const geo::Vec2i& p3_2d = vertices_proj[t.i3];

            drawTriangle2D(geo::Vec3f(p1_2d.x, p1_2d.y, 1.0f / -p1_3d.z),
                           geo::Vec3f(p2_2d.x, p2_2d.y, 1.0f / -p2_3d.z),
                           geo::Vec3f(p3_2d.x, p3_2d.y, 1.0f / -p3_3d.z),
                           res);
        }
    }
}

}

} // end namespace mwm

