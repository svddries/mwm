#ifndef MWM_WORLD_MODEL_H_
#define MWM_WORLD_MODEL_H_

#include "mwm/triangle.h"

#include <vector>
#include <geolib/datatypes.h>

namespace mwm
{

class WorldModel
{

public:

    WorldModel();

    ~WorldModel();

    void addTriangle(const geo::Vec3& v1, const geo::Vec3& v2, const geo::Vec3& v3)
    {
        triangles_.push_back(Triangle(addVertex(v1), addVertex(v2), addVertex(v3)));
    }

    void addTriangle(unsigned int i1, unsigned int i2, unsigned int i3, const cv::Vec3b& clr)
    {
        triangles_.push_back(Triangle(i1, i2, i3, clr));
    }

    unsigned int addVertex(const geo::Vec3& v)
    {
        vertices_.push_back(v);
        return vertices_.size() - 1;
    }

    const std::vector<geo::Vec3>& vertices() const { return vertices_; }

    const std::vector<Triangle>& triangles() const { return triangles_; }

    // - - - - -

    const std::vector<geo::Vec3>& points() const { return points_; }

    const std::vector<cv::Vec3b>& point_colors() const { return point_colors_; }

    const std::vector<double>& point_probabilities() const { return point_probs_; }

    unsigned int addPoint(const geo::Vec3& p, const cv::Vec3b& color, double prob)
    {
        points_.push_back(p);
        point_colors_.push_back(color);
        point_probs_.push_back(prob);
    }

    void setPoint(unsigned int idx, const geo::Vec3& p)
    {
        points_[idx] = p;
    }

    void setPoint(unsigned int idx, const geo::Vec3& p, const cv::Vec3b& color, double prob)
    {
        points_[idx] = p;
        point_colors_[idx] = color;
        point_probs_[idx] = prob;
    }

private:

    std::vector<geo::Vec3> vertices_;

    std::vector<Triangle> triangles_;

    // - - - - -

    std::vector<geo::Vec3> points_;

    std::vector<cv::Vec3b> point_colors_;

    std::vector<double> point_probs_;

};

} // end namespace mwm

#endif
