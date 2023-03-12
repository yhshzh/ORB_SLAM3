/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include <iostream>

namespace ORB_SLAM3 {

MapDrawer::MapDrawer(Atlas *pAtlas, const string &strSettingPath, Settings *settings) : mpAtlas(pAtlas) {
    if (settings) {
        newParameterLoader(settings);
    } else {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        bool is_correct = ParseViewerParamFile(fSettings);

        if (!is_correct) {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try {
                throw -1;
            }
            catch (exception &e) {

            }
        }
    }
}

void MapDrawer::newParameterLoader(Settings *settings) {
    mKeyFrameSize = settings->keyFrameSize();
    mKeyFrameLineWidth = settings->keyFrameLineWidth();
    mGraphLineWidth = settings->graphLineWidth();
    mPointSize = settings->pointSize();
    mCameraSize = settings->cameraSize();
    mCameraLineWidth = settings->cameraLineWidth();
}

bool MapDrawer::ParseViewerParamFile(cv::FileStorage &fSettings) {
    bool b_miss_params = false;

    cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
    if (!node.empty()) {
        mKeyFrameSize = node.real();
    } else {
        std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.KeyFrameLineWidth"];
    if (!node.empty()) {
        mKeyFrameLineWidth = node.real();
    } else {
        std::cerr << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.GraphLineWidth"];
    if (!node.empty()) {
        mGraphLineWidth = node.real();
    } else {
        std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.PointSize"];
    if (!node.empty()) {
        mPointSize = node.real();
    } else {
        std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraSize"];
    if (!node.empty()) {
        mCameraSize = node.real();
    } else {
        std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraLineWidth"];
    if (!node.empty()) {
        mCameraLineWidth = node.real();
    } else {
        std::cerr << "*Viewer.CameraLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

void MapDrawer::DrawMapPoints() {
    Map *pActiveMap = mpAtlas->GetCurrentMap();
    if (!pActiveMap)
        return;

    const vector<MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if (vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
        if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        Eigen::Matrix<float, 3, 1> pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos(0), pos(1), pos(2));
//        cout << "MapPoints:  x=" + to_string(pos(0)) + " y=" + to_string(pos(1)) + "  z=" + to_string(pos(2))
//             << endl;
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++) {
        if ((*sit)->isBad())
            continue;
        Eigen::Matrix<float, 3, 1> pos = (*sit)->GetWorldPos();
        glVertex3f(pos(0), pos(1), pos(2));
        cout << "ReferenceMapPoints:  x=" + to_string(pos(0)) + " y=" + to_string(pos(1)) + "  z=" + to_string(pos(2))
             << endl;
    }

    glEnd();
}

// TODO DrawMPAndKF
void MapDrawer::DrawMPAndKF(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph,
                            const bool bDrawOptLba) {
    const float &w = mKeyFrameSize;
    const float h = w * 0.75;
    const float z = w * 0.6;

    Map *pActiveMap = mpAtlas->GetCurrentMap();
    if (!pActiveMap)
        return;

    const vector<MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();
    std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
    std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;
    const vector<KeyFrame *> vpKFs = pActiveMap->GetAllKeyFrames();

    set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if (vpMPs.empty())
        return;

    pangolin::glDrawAxis(5.0);
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);


    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
        if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        Eigen::Matrix<float, 3, 1> pos = vpMPs[i]->GetWorldPos();
//        if ((pos(0) > 2 && pos(0) < 12) && (pos(1) > 2 && pos(1) < 8) && (pos(2) > 0.2 && pos(2) < 3))
        {
            glVertex3f(pos(0), pos(1), pos(2));
            // cloud_temp->push_back(pcl::PointXYZ(pos(0), pos(1), 0));
        }

//        cout << "MapPoints:  x=" + to_string(pos(0)) + " y=" + to_string(pos(1)) + "  z=" + to_string(pos(2))
//             << endl;
    }
//    cout<<vpMPs.size()<<endl;
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);


    for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++) {
        if ((*sit)->isBad())
            continue;
        Eigen::Matrix<float, 3, 1> pos = (*sit)->GetWorldPos();
//        if ((pos(0) > 2 && pos(0) < 12) && (pos(1) > 2 && pos(1) < 8) && (pos(2) > 0.2 && pos(2) < 3))
        {
            glVertex3f(pos(0), pos(1), pos(2));
            // cloud_temp->push_back(pcl::PointXYZ(pos(0), pos(1), 0));

//            cout <<"ReferenceMapPoints:  x=" + to_string(pos(0)) + " y=" + to_string(pos(1)) + "  z=" + to_string(pos(2)) << endl;
        }
    }

//    glEnd();
//
//    //滤波算法
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
//    ror.setInputCloud(cloud_temp);
//    ror.setRadiusSearch(0.1);
//    ror.setMinNeighborsInRadius(5);
//    ror.filter(*cloud_filtered);
//
//
//    //KMeans预设聚类中心
//    st_pointxyz standardvec[8] = {
//        {2.825, 2.825, 0},
//        {6.000, 2.825, 0},
//        {9.175, 2.825, 0},
////        {2.825,9.175},
////        {6.000,9.175},
////        {9.175,9.175},
//        {6.000, 6.000, 0},
//        {7.275, 4.725, 0},
//        {4.725, 4.725, 0},
////        {7.275, 7.275, 0},
////        {4.725, 7.275, 0},
//    };
//
//    //KMeans聚类，详见include/KMeans.h & src/KMeans.h
//    KMeans KM;
//    KM.SetInputCloud(cloud_filtered);
//    KM.SetK(6);
//    KM.InitKCenter(standardvec);
//    KM.Cluster();
//    //输出聚类中心并将聚类点云输出到pcd文件
//    int i = 0;
//    bool flag = false;
//    ofstream fout;
//    fout.open("Centers.txt",ios::out);
//    for (std::vector<st_pointxyz>::const_iterator it = KM.mv_center.begin(); it != KM.mv_center.end(); it++)
//        if (!isnan(it->x) && !isnan(it->y) && !isnan(it->z)) {
//            fout << "centorid " << i++ << endl << it->x << " " << it->y << " " << it->z << endl;
//            flag = true;
//        }
//    if (flag) {
//        fout << "complete filing" << endl;
//        fout.close();
//        KM.SaveFile(".","k");
//    }




//欧式聚类，效果不合适，暂废
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr  tree(new pcl::search::KdTree<pcl::PointXYZ>);
//    tree->setInputCloud(cloud_filtered);
//
//    //Set parameters
//    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//    ec.setClusterTolerance(0.05); // 临近半径
//    ec.setMinClusterSize(15);
//    ec.setMaxClusterSize(50);
//    ec.setSearchMethod(tree);
//    ec.setInputCloud(cloud_filtered);
//    ec.extract(cluster_indices); //执行
//
//    //Save pcd
//    int j = 0;
//    std::vector<Eigen::Vector4f>centorArray;
//    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it!=cluster_indices.end();++it)
//    {
//        // 划分出不同聚类层次的点云，计算中心
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
//
//        for(std::vector<int>::const_iterator pit = it->indices.begin();pit!=it->indices.end();++pit)
//            cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
//        cloud_cluster->width = cloud_cluster->points.size();
//        cloud_cluster->height = 1;
//        cloud_cluster->is_dense = true;
//        //计算重心
//        Eigen::Vector4f centroid;
//        pcl::compute3DCentroid(*cloud_cluster, centroid);//qi次坐标(x,y,z,1)
//        centorArray.push_back(centroid);
//        j++;
//    }
//    //遍历输出聚类重心
//    int i =0;
//    for(std::vector<Eigen::Vector4f>::const_iterator it = centorArray.begin();it!=centorArray.end();it++)
//        std::cout<<"centorid "<<i++<<std::endl<<*it<<std::endl;
//    std::cout<<"complete filing"<<std::endl;


    if (bDrawKF) {
        pflag++;
        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKF = vpKFs[i];
            Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
//            Eigen::Vector3f CPose = pKF->GetCameraCenter();
//            Eigen::Vector3f IMUPos = pKF->GetImuPosition();
            Eigen::Matrix4f CPos = mCameraPose.matrix();
//            for (int ii = 0;ii<4;ii++){
//                for (int jj = 0;jj<4;jj++){
//                    cout<<to_string(CPos(ii,jj))<<" ";
//                }
//                cout<<endl;
//            }
//            cout<<"end"<<endl;
            if(pflag>=30) {
                pflag = 0;
                cout << (0.99974491*CPos(0, 3) + 0.00686359*CPos(1, 3) + 0.02151778*CPos(2, 3))*0.987097551 << " ";
                cout << (-0.00669559*CPos(0, 3) + 0.99994662*CPos(1, 3) + -0.00786956*CPos(2, 3))*0.987097551 << " ";
                cout << (-0.02157064*CPos(0, 3) + 0.00772348*CPos(1, 3) + 0.99973749*CPos(2, 3))*0.987097551 << " ";
//                for (int ii = 0; ii < 3; ii++)
//                    cout << to_string(CPos(ii, 3)) << " ";
                cout << endl;
            }

            unsigned int index_color = pKF->mnOriginMapId;

            glPushMatrix();

            glMultMatrixf((GLfloat *) Twc.data());

            if (!pKF->GetParent()) // It is the first KF in the map
            {
                glLineWidth(mKeyFrameLineWidth * 5);
                glColor3f(1.0f, 0.0f, 0.0f);
                glBegin(GL_LINES);
            } else {
                //cout << "Child KF: " << vpKFs[i]->mnId << endl;
                glLineWidth(mKeyFrameLineWidth);
                if (bDrawOptLba) {
                    if (sOptKFs.find(pKF->mnId) != sOptKFs.end()) {
                        glColor3f(0.0f, 1.0f, 0.0f); // Green -> Opt KFs
                    } else if (sFixedKFs.find(pKF->mnId) != sFixedKFs.end()) {
                        glColor3f(1.0f, 0.0f, 0.0f); // Red -> Fixed KFs
                    } else {
                        glColor3f(0.0f, 0.0f, 1.0f); // Basic color
                    }
                } else {
                    glColor3f(0.0f, 0.0f, 1.0f); // Basic color
                }
                glBegin(GL_LINES);
            }

            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }

    if (bDrawGraph) {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
        glBegin(GL_LINES);

        // cout << "-----------------Draw graph-----------------" << endl;
        for (size_t i = 0; i < vpKFs.size(); i++) {
            // Covisibility Graph
            const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            Eigen::Vector3f Ow = vpKFs[i]->GetCameraCenter();
            if (!vCovKFs.empty()) {
                for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end();
                     vit != vend; vit++) {
                    if ((*vit)->mnId < vpKFs[i]->mnId)
                        continue;
                    Eigen::Vector3f Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow(0), Ow(1), Ow(2));
                    glVertex3f(Ow2(0), Ow2(1), Ow2(2));
                }
            }

            // Spanning tree
            KeyFrame *pParent = vpKFs[i]->GetParent();
            if (pParent) {
                Eigen::Vector3f Owp = pParent->GetCameraCenter();
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owp(0), Owp(1), Owp(2));
            }

            // Loops
            set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
                if ((*sit)->mnId < vpKFs[i]->mnId)
                    continue;
                Eigen::Vector3f Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owl(0), Owl(1), Owl(2));
            }
        }

        glEnd();
    }

    if (bDrawInertialGraph && pActiveMap->isImuInitialized()) {
        glLineWidth(mGraphLineWidth);
        glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
        glBegin(GL_LINES);

        //Draw inertial links
        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKFi = vpKFs[i];
            Eigen::Vector3f Ow = pKFi->GetCameraCenter();
            KeyFrame *pNext = pKFi->mNextKF;
            if (pNext) {
                Eigen::Vector3f Owp = pNext->GetCameraCenter();
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owp(0), Owp(1), Owp(2));
            }
        }

        glEnd();
    }

    vector<Map *> vpMaps = mpAtlas->GetAllMaps();

    if (bDrawKF) {
        for (Map *pMap : vpMaps) {
            if (pMap == pActiveMap)
                continue;

            vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

            for (size_t i = 0; i < vpKFs.size(); i++) {
                KeyFrame *pKF = vpKFs[i];
                Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                unsigned int index_color = pKF->mnOriginMapId;

                glPushMatrix();

                glMultMatrixf((GLfloat *) Twc.data());

                if (!vpKFs[i]->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth * 5);
                    glColor3f(1.0f, 0.0f, 0.0f);
                    glBegin(GL_LINES);
                } else {
                    glLineWidth(mKeyFrameLineWidth);
                    glColor3f(mfFrameColors[index_color][0], mfFrameColors[index_color][1],
                              mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

                glVertex3f(0, 0, 0);
                glVertex3f(w, h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, h, z);

                glVertex3f(w, h, z);
                glVertex3f(w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(-w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);

                glVertex3f(-w, -h, z);
                glVertex3f(w, -h, z);
                glEnd();

                glPopMatrix();
            }
        }
    }
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph,
                              const bool bDrawOptLba) {
    const float &w = mKeyFrameSize;
    const float h = w * 0.75;
    const float z = w * 0.6;

    Map *pActiveMap = mpAtlas->GetCurrentMap();
    // DEBUG LBA
    std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
    std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

    if (!pActiveMap)
        return;

    const vector<KeyFrame *> vpKFs = pActiveMap->GetAllKeyFrames();

    if (bDrawKF) {
        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKF = vpKFs[i];
            Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
//             Eigen::Vector3f CPose = pKF->GetCameraCenter();
            Eigen::Vector3f IMUPos = pKF->GetImuPosition();
//             cout << "CameraCenter: x=" + to_string(CPose(0)) + "    y=" + to_string(CPose(1)) + "   z=" + to_string(CPose(2)) << endl;
            cout << "CameraCenter: x=" + to_string(IMUPos(0)) + "    y=" + to_string(IMUPos(1)) + "   z="
                + to_string(IMUPos(2)) << endl;
            unsigned int index_color = pKF->mnOriginMapId;

            glPushMatrix();

            glMultMatrixf((GLfloat *) Twc.data());

            if (!pKF->GetParent()) // It is the first KF in the map
            {
                glLineWidth(mKeyFrameLineWidth * 5);
                glColor3f(1.0f, 0.0f, 0.0f);
                glBegin(GL_LINES);
            } else {
                //cout << "Child KF: " << vpKFs[i]->mnId << endl;
                glLineWidth(mKeyFrameLineWidth);
                if (bDrawOptLba) {
                    if (sOptKFs.find(pKF->mnId) != sOptKFs.end()) {
                        glColor3f(0.0f, 1.0f, 0.0f); // Green -> Opt KFs
                    } else if (sFixedKFs.find(pKF->mnId) != sFixedKFs.end()) {
                        glColor3f(1.0f, 0.0f, 0.0f); // Red -> Fixed KFs
                    } else {
                        glColor3f(0.0f, 0.0f, 1.0f); // Basic color
                    }
                } else {
                    glColor3f(0.0f, 0.0f, 1.0f); // Basic color
                }
                glBegin(GL_LINES);
            }

            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }

    if (bDrawGraph) {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
        glBegin(GL_LINES);

        // cout << "-----------------Draw graph-----------------" << endl;
        for (size_t i = 0; i < vpKFs.size(); i++) {
            // Covisibility Graph
            const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            Eigen::Vector3f Ow = vpKFs[i]->GetCameraCenter();
            if (!vCovKFs.empty()) {
                for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end();
                     vit != vend; vit++) {
                    if ((*vit)->mnId < vpKFs[i]->mnId)
                        continue;
                    Eigen::Vector3f Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow(0), Ow(1), Ow(2));
                    glVertex3f(Ow2(0), Ow2(1), Ow2(2));
                }
            }

            // Spanning tree
            KeyFrame *pParent = vpKFs[i]->GetParent();
            if (pParent) {
                Eigen::Vector3f Owp = pParent->GetCameraCenter();
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owp(0), Owp(1), Owp(2));
            }

            // Loops
            set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
                if ((*sit)->mnId < vpKFs[i]->mnId)
                    continue;
                Eigen::Vector3f Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owl(0), Owl(1), Owl(2));
            }
        }

        glEnd();
    }

    if (bDrawInertialGraph && pActiveMap->isImuInitialized()) {
        glLineWidth(mGraphLineWidth);
        glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
        glBegin(GL_LINES);

        //Draw inertial links
        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKFi = vpKFs[i];
            Eigen::Vector3f Ow = pKFi->GetCameraCenter();
            KeyFrame *pNext = pKFi->mNextKF;
            if (pNext) {
                Eigen::Vector3f Owp = pNext->GetCameraCenter();
                glVertex3f(Ow(0), Ow(1), Ow(2));
                glVertex3f(Owp(0), Owp(1), Owp(2));
            }
        }

        glEnd();
    }

    vector<Map *> vpMaps = mpAtlas->GetAllMaps();

    if (bDrawKF) {
        for (Map *pMap : vpMaps) {
            if (pMap == pActiveMap)
                continue;

            vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

            for (size_t i = 0; i < vpKFs.size(); i++) {
                KeyFrame *pKF = vpKFs[i];
                Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                unsigned int index_color = pKF->mnOriginMapId;

                glPushMatrix();

                glMultMatrixf((GLfloat *) Twc.data());

                if (!vpKFs[i]->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth * 5);
                    glColor3f(1.0f, 0.0f, 0.0f);
                    glBegin(GL_LINES);
                } else {
                    glLineWidth(mKeyFrameLineWidth);
                    glColor3f(mfFrameColors[index_color][0], mfFrameColors[index_color][1],
                              mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

                glVertex3f(0, 0, 0);
                glVertex3f(w, h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, h, z);

                glVertex3f(w, h, z);
                glVertex3f(w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(-w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);

                glVertex3f(-w, -h, z);
                glVertex3f(w, -h, z);
                glEnd();

                glPopMatrix();
            }
        }
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
    const float &w = mCameraSize;
    const float h = w * 0.75;
    const float z = w * 0.6;

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);

    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);

    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);
    glEnd();

    glPopMatrix();
}

void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw) {
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.inverse();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw) {
    Eigen::Matrix4f Twc;
    {
        unique_lock<mutex> lock(mMutexCamera);
        Twc = mCameraPose.matrix();
    }

    for (int i = 0; i < 4; i++) {
        M.m[4 * i] = Twc(0, i);
        M.m[4 * i + 1] = Twc(1, i);
        M.m[4 * i + 2] = Twc(2, i);
        M.m[4 * i + 3] = Twc(3, i);
    }

    MOw.SetIdentity();
    MOw.m[12] = Twc(0, 3);
    MOw.m[13] = Twc(1, 3);
    MOw.m[14] = Twc(2, 3);
}

Eigen::Matrix4f MapDrawer::GetCurrectCameraPose() {
    Eigen::Matrix4f Twc;
    {
        unique_lock<mutex> lock(mMutexCamera);
        Twc = mCameraPose.matrix();
    }
    return Twc;
}

} //namespace ORB_SLAM
