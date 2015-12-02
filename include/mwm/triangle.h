#ifndef _TRIANGLE_H_
#define _TRIANGLE_H_

namespace mwm
{

struct Triangle
{

    Triangle(unsigned int i1_, unsigned int i2_, unsigned int i3_) : i1(i1_), i2(i2_), i3(i3_) {}
    unsigned int i1, i2, i3;

};

} // end namespace mwm

#endif
