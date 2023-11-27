#pragma once
#include "constant.h"


enum rotationOrder {
    XYZ,
    XZY,
    YXZ,
    YZX,
    ZXY,
    ZYX
};

const char order2String[][4] = { "XYZ","XZY","YXZ","YZX","ZXY","ZYX" };

typedef double angle;
typedef double intrinsic;
typedef double coordinate;