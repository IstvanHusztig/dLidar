#include "unitree_lidar_utilities.h"

namespace unitree_lidar_sdk
{

/**
 * @brief Point Type
 */
typedef struct
{
    float x;
    float y;
    float z;
    float intensity;
    double time;    // relative time of this point from cloud stamp
    uint32_t ring; // the ring number indicates which channel of the sensor that this point belongs to
} DLidarPoint;

/**
 * @brief DLidar Point Cloud Type
 */
typedef struct
{
    double stamp;     // cloud start timestamp, the point timestamp is relative to this
    uint32_t id;      // sequence id
    uint32_t ringNum; // number of rings
    std::vector<DLidarPoint> points;
} DLidarPointCloud;

/**
 * @brief Parse from a point packet to a 3D point cloud
 * @param[out] cloud
 * @param[in] packet lidar point data packet
 * @param[in] use_system_timestamp use system timestamp, otherwise use lidar hardware timestamp
 * @param[in] range_min allowed minimum point range in meters
 * @param[in] range_max allowed maximum point range in meters
 */
inline void ParseFrom_UniTreePointCloud_To_DLidarPointCloud(
    DLidarPointCloud &cloud,
    PointCloudUnitree &cloudIn)
{
    // scan info
    const int num_of_points = cloudIn.points.size();
       
    cloud.stamp = cloudIn.stamp;
  
    cloud.id = 1;
    cloud.ringNum = 1;
    cloud.points.clear();
    cloud.points.reserve(300);

    DLidarPoint point3d;
    point3d.ring = 1;

    for (int j = 0; j < num_of_points; j += 1)
    {   
        point3d.x =cloudIn.points[j].x;
        point3d.y = cloudIn.points[j].y;
        point3d.z = cloudIn.points[j].z;
        point3d.intensity = cloudIn.points[j].intensity;
        point3d.time = cloudIn.points[j].time;//* 1.0e9;
        
        cloud.points.emplace_back(point3d);
    }
}
}