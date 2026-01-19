#pragma once
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h> // for using macros PCL_ADD_

struct EIGEN_ALIGN16 BagPointT
{
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;
    std::uint16_t ring;
    PCL_MAKE_ALIGNED_OPERATOR_NEW

    BagPointT()
    {
        x = NAN;
        y = NAN;
        z = NAN;
        intensity = NAN;
        timestamp = NAN;
        ring = 0;
    };
};

// need this for pcl/conversion to work
POINT_CLOUD_REGISTER_POINT_STRUCT(BagPointT,
                                (float, x, x)
                                (float, y, y)
                                (float, z, z)
                                (float, intensity, intensity)
                                (double, timestamp, timestamp)
                                (std::uint16_t, ring, ring)
);


// ================= NOTES =================
// intensity, timestamp, ring are specified as datatype = 7, 8, 4 in sensor_msgs
// document, which corresponds to FLOAT32, FLOAT64, UINT16.

// for pcl conversion to work, we need to register the variable using datatype
// with matching bits for correct memory allocation

// using 'double' instead of 'uint64_t' because pcl doesn't support 'uint64_t'
// ========================================