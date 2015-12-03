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

private:

    std::vector<geo::Vec3> vertices_;

    std::vector<Triangle> triangles_;

};

} // end namespace mwm

#endif
