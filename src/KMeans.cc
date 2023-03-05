//
// Created by zheng on 22-11-30.
//
// #include "KMeans.h"

const float DIST_NEAR_ZERO = 0.001;

//extern char szFileName[256];

bool KMeans::InitKCenter(st_pointxyz pnt_arr[])
{
    if (m_k == 0)
    {
        PCL_ERROR("在此之前必须要调用setK()函数\n");
        return false;
    }

    mv_center.resize(m_k);
    for (size_t i = 0; i < m_k; ++i)
    {
        mv_center[i] = pnt_arr[i];
    }
    return true;
}

bool KMeans::SetInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud)
{
    size_t pntCount = (size_t)pPntCloud->points.size();
    //mv_pntcloud.resize(pntCount);
    for (size_t i = 0; i < pntCount; ++i)
    {
        st_point point;
        point.pnt.x = pPntCloud->points[i].x;
        point.pnt.y = pPntCloud->points[i].y;
        point.pnt.z = pPntCloud->points[i].z;
        point.groupID = 0;

        mv_pntcloud.push_back(point);
    }

    return true;
}

bool KMeans::Cluster()
{
    std::vector<st_pointxyz> v_center(mv_center.size());

    do
    {
        for (size_t i = 0, pntCount = mv_pntcloud.size(); i < pntCount; ++i)
        {
            double min_dist = DBL_MAX;
            int pnt_grp = 0;
            for (size_t j = 0; j < m_k; ++j)
            {
                double dist = DistBetweenPoints(mv_pntcloud[i].pnt, mv_center[j]);
                if (min_dist - dist > 0.000001)
                {
                    min_dist = dist;
                    pnt_grp = j;
                }
            }
            m_grp_pntcloud[pnt_grp].push_back(st_point(mv_pntcloud[i].pnt, pnt_grp));
        }

        //保存上一次迭代的中心点
        for (size_t i = 0; i < mv_center.size(); ++i)
        {
            v_center[i] = mv_center[i];
        }

        if (!UpdateGroupCenter(m_grp_pntcloud, mv_center))
        {
            return false;
        }
        if ( !ExistCenterShift(v_center, mv_center))
        {
            break;
        }
        for (size_t i = 0; i < m_k; ++i){
            m_grp_pntcloud[i].clear();
        }

    }while(true);

    return true;
}

double KMeans::DistBetweenPoints(st_pointxyz &p1, st_pointxyz &p2)
{
    double dist = 0;
    double x_diff = 0, y_diff = 0, z_diff = 0;

    x_diff = p1.x - p2.x;
    y_diff = p1.y - p2.y;
    z_diff = p1.z - p2.z;
    dist = sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);

    return dist;
}

bool KMeans::UpdateGroupCenter(std::vector<VecPoint_t> &grp_pntcloud, std::vector<st_pointxyz> &center)
{
    if (center.size() != m_k)
    {
        PCL_ERROR("类别的个数不为K\n");
        return false;
    }

    for (size_t i = 0; i < m_k; ++i)
    {
        float x = 0, y = 0, z = 0;
        size_t pnt_num_in_grp = grp_pntcloud[i].size();

        for (size_t j = 0; j < pnt_num_in_grp; ++j)
        {
            x += grp_pntcloud[i][j].pnt.x;
            y += grp_pntcloud[i][j].pnt.y;
            z += grp_pntcloud[i][j].pnt.z;
        }
        x /= pnt_num_in_grp;
        y /= pnt_num_in_grp;
        z /= pnt_num_in_grp;
        center[i].x = x;
        center[i].y = y;
        center[i].z = z;
    }
    return true;
}

//是否存在中心点移动
bool KMeans::ExistCenterShift(std::vector<st_pointxyz> &prev_center, std::vector<st_pointxyz> &cur_center)
{
    for (size_t i = 0; i < m_k; ++i)
    {
        double dist = DistBetweenPoints(prev_center[i], cur_center[i]);
        if (dist > DIST_NEAR_ZERO)
        {
            return true;
        }
    }

    return false;
}

//将聚类的点分别存到各自的pcd文件中
bool KMeans::SaveFile()
{
    for (size_t i = 0; i < m_k; ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr p_pnt_cloud(new pcl::PointCloud<pcl::PointXYZ> ());

        for (size_t j = 0, grp_pnt_count = m_grp_pntcloud[i].size(); j < grp_pnt_count; ++j)
        {
            pcl::PointXYZ pt;
            pt.x = m_grp_pntcloud[i][j].pnt.x;
            pt.y = m_grp_pntcloud[i][j].pnt.y;
            pt.z = m_grp_pntcloud[i][j].pnt.z;

            p_pnt_cloud->points.push_back(pt);
        }

        p_pnt_cloud->width = (int)m_grp_pntcloud[i].size();
        p_pnt_cloud->height = 1;

        char newFileName[256] = "test.pcd";
//        char indexStr[16] = {0};

//        strcat(newFileName, szFileName);
//        strcat(newFileName, "-");
//        strcat(newFileName, prex_name);
//        strcat(newFileName, "-");
//        sprintf(indexStr, "%d", i + 1);
//        strcat(newFileName, indexStr);
//        strcat(newFileName, ".pcd");
        pcl::io::savePCDFileASCII(newFileName, *p_pnt_cloud);
    }

    return true;
}

bool KMeans::SaveFile(const char *dir_name, const char *prex_name)
{
    for (size_t i = 0; i < m_k; ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr p_pnt_cloud(new pcl::PointCloud<pcl::PointXYZ> ());

        for (size_t j = 0, grp_pnt_count = m_grp_pntcloud[i].size(); j < grp_pnt_count; ++j)
        {
            pcl::PointXYZ pt;
            pt.x = m_grp_pntcloud[i][j].pnt.x;
            pt.y = m_grp_pntcloud[i][j].pnt.y;
            pt.z = m_grp_pntcloud[i][j].pnt.z;

            p_pnt_cloud->points.push_back(pt);
        }

        p_pnt_cloud->width = (int)m_grp_pntcloud[i].size();
        p_pnt_cloud->height = 1;

        char newFileName[256] = {0};
        char indexStr[16] = {0};

        strcat(newFileName, dir_name);
        strcat(newFileName, "/");
        strcat(newFileName, prex_name);
        strcat(newFileName, "-");
        sprintf(indexStr, "%d", i + 1);
        strcat(newFileName, indexStr);
        strcat(newFileName, ".pcd");
        if (!(*p_pnt_cloud).empty())
            pcl::io::savePCDFileASCII(newFileName, *p_pnt_cloud);
    }

    return true;
}