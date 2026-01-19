#pragma once
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h> // for using macros PCL_ADD_

struct EIGEN_ALIGN16 VilensPointT
{
    PCL_ADD_POINT4D;
    PCL_ADD_NORMAL4D;
    float curvature;
    PCL_MAKE_ALIGNED_OPERATOR_NEW

    VilensPointT()
    {
        x = NAN;
        y = NAN;
        z = NAN;
        normal_x = NAN;
        normal_y = NAN;
        normal_z = NAN;
        curvature = NAN;
    };
};

// need this for pcl/conversion to work
POINT_CLOUD_REGISTER_POINT_STRUCT(VilensPointT,
                                (float, x, x)
                                (float, y, y)
                                (float, z, z)
                                (float, normal_x, normal_x)
                                (float, normal_y, normal_y)
                                (float, normal_z, normal_z)
                                (float, curvature, curvature)
);


// ================= NOTES =================
// intensity, timestamp, ring are specified as datatype = 7, 8, 4 in sensor_msgs
// document, which corresponds to FLOAT32, FLOAT64, UINT16.

// for pcl conversion to work, we need to register the variable using datatype
// with matching bits for correct memory allocation

// using 'double' instead of 'uint64_t' because pcl doesn't support 'uint64_t'
// ========================================