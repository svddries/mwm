#ifndef MWM_PROJECTION_MATRIX_H_
#define MWM_PROJECTION_MATRIX_H_

#include <geolib/datatypes.h>

namespace mwm
{

class ProjectionMatrix
{

public:

    ProjectionMatrix() {}

    ~ProjectionMatrix() {}

    inline int project3Dto2DX(const geo::Vec3& p) const
    {
        return (fx_ * p.x + tx_) / -p.z + cx_;
    }

    inline int project3Dto2DY(const geo::Vec3& p) const
    {
        return (fy_ * -p.y + ty_) / -p.z + cy_;
    }

    inline geo::Vec2i project3Dto2D(const geo::Vec3& p) const
    {
        return geo::Vec2i(project3Dto2DX(p), project3Dto2DY(p));
    }

    inline double project2Dto3DX(int x) const
    {
        return (x - cx_plus_tx_) / fx_;
    }

    inline double project2Dto3DY(int y) const
    {
        return -(y - cy_plus_ty_) / fy_;
    }

    inline geo::Vec3 project2Dto3D(int x, int y) const
    {
        return geo::Vec3(project2Dto3DX(x), project2Dto3DY(y), -1.0);
    }

    inline void setFocalLengths(double fx, double fy)
    {
        fx_ = fx;
        fy_ = fy;
    }

    inline void setOpticalCenter(double cx, double cy)
    {
        cx_ = cx;
        cy_ = cy;
        cx_plus_tx_ = cx_ + tx_;
        cy_plus_ty_ = cy_ + ty_;
    }

    inline void setOpticalTranslation(double tx, double ty)
    {
        tx_ = tx;
        ty_ = ty;
        cx_plus_tx_ = cx_ + tx_;
        cy_plus_ty_ = cy_ + ty_;
    }

    inline double getFocalLengthX() const { return fx_; }

    inline double getFocalLengthY() const { return fy_; }

    inline double getOpticalCenterX() const { return cx_; }

    inline double getOpticalCenterY() const { return cy_; }

    inline double getOpticalTranslationX() const { return tx_; }

    inline double getOpticalTranslationY() const { return ty_; }

private:

    double fx_, fy_;
    double cx_, cy_;
    double tx_, ty_;
    double cx_plus_tx_;
    double cy_plus_ty_;
};

} // end namespace mwm

#endif
