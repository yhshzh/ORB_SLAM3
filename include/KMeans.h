//
// Created by zheng on 22-11-30.
//

#ifndef ORB_SLAM3_INCLUDE_KMEANS_H_
#define ORB_SLAM3_INCLUDE_KMEANS_H_
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//笛卡尔坐标系中三维点坐标
typedef struct st_pointxyz
{
    float x;
    float y;
    float z;
}st_pointxyz;
typedef struct st_point
{
    st_pointxyz pnt;
    int groupID;
    st_point()
    {

    }
    st_point(st_pointxyz &p, int id)
    {
        pnt = p;
        groupID = id;
    }
}st_point;

class KMeans
{
 public:
    int m_k;

    typedef std::vector<st_point> VecPoint_t;

    VecPoint_t mv_pntcloud;    //要聚类的点云
    std::vector<VecPoint_t> m_grp_pntcloud;    //K类，每一类存储若干点
    std::vector<st_pointxyz> mv_center;    //每个类的中心

    KMeans()
    {
        m_k = 0;
    }

    inline void SetK(int k_)
    {
        m_k = k_;
        m_grp_pntcloud.resize(m_k);
    }
    //设置输入点云
    bool SetInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud);

    //初始化最初的K个类的中心
    bool InitKCenter(st_pointxyz pc_arr[]);

    //聚类
    bool Cluster();

    //更新K类的中心
    bool UpdateGroupCenter(std::vector<VecPoint_t> &grp_pntcloud, std::vector<st_pointxyz> &center);

    //计算两个点间的欧氏距离
    double DistBetweenPoints(st_pointxyz &p1, st_pointxyz &p2);

    //是否存在中心点移动
    bool ExistCenterShift(std::vector<st_pointxyz> &prev_center, std::vector<st_pointxyz> &cur_center);

    //将聚类的点分别存到各自的pcd文件中
    bool SaveFile();
    //将聚类的点分别存到各自的pcd文件中
    bool SaveFile(const char *dir_name, const char *prex_name);
};
#endif //ORB_SLAM3_INCLUDE_KMEANS_H_
