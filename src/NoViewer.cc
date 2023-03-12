#include "NoViewer.h"

#include <mutex>
#include <termios.h>

namespace ORB_SLAM3
{

NoViewer::NoViewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, Settings* settings):
    both(false), mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    if(settings){
        newParameterLoader(settings);
    }
    else{

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        bool is_correct = ParseViewerParamFile(fSettings);

        if(!is_correct)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }

    mbStopTrack = false;
}

void NoViewer::newParameterLoader(Settings *settings) {
    mImageViewerScale = 1.f;

    float fps = settings->fps();
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::Size imSize = settings->newImSize();
    mImageHeight = imSize.height;
    mImageWidth = imSize.width;

    mImageViewerScale = settings->imageViewerScale();
    mViewpointX = settings->viewPointX();
    mViewpointY = settings->viewPointY();
    mViewpointZ = settings->viewPointZ();
    mViewpointF = settings->viewPointF();
}

bool NoViewer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    mImageViewerScale = 1.f;

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::FileNode node = fSettings["Camera.width"];
    if(!node.empty())
    {
        mImageWidth = node.real();
    }
    else
    {
        std::cerr << "*Camera.width parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Camera.height"];
    if(!node.empty())
    {
        mImageHeight = node.real();
    }
    else
    {
        std::cerr << "*Camera.height parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.imageViewScale"];
    if(!node.empty())
    {
        mImageViewerScale = node.real();
    }

    node = fSettings["Viewer.ViewpointX"];
    if(!node.empty())
    {
        mViewpointX = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointX parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointY"];
    if(!node.empty())
    {
        mViewpointY = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointY parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointZ"];
    if(!node.empty())
    {
        mViewpointZ = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointZ parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointF"];
    if(!node.empty())
    {
        mViewpointF = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointF parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

void NoViewer::Run()
{
    mbFinished = false;
    mbStopped = false;
    bool Reset = false;
    bool Stop = false;
    float Angle_D = 0;
    clock_t start = clock();
    while(1){
        sleep(2);
        Map *pActiveMap = mpMapDrawer->mpAtlas->GetCurrentMap();
        if (!pActiveMap){
            cout<<"!!!!!!!!"<<endl;
            return;
        }

        const vector<MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();
        const vector<MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();
        std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
        std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;
        const vector<KeyFrame *> vpKFs = pActiveMap->GetAllKeyFrames();

        set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        if (vpMPs.empty()) {
            cout<<"!!!!!!!!"<<endl;
            continue;
        }


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

        }


        for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++) {
            if ((*sit)->isBad())
                continue;
            Eigen::Matrix<float, 3, 1> pos = (*sit)->GetWorldPos();
//        if ((pos(0) > 2 && pos(0) < 12) && (pos(1) > 2 && pos(1) < 8) && (pos(2) > 0.2 && pos(2) < 3))
            {
                glVertex3f(pos(0), pos(1), pos(2));
                // cloud_temp->push_back(pcl::PointXYZ(pos(0), pos(1), 0));
            }
        }


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

//    // 欧式聚类，效果不合适，暂废
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

        if (true) {
            for (size_t i = 0; i < vpKFs.size(); i++) {
                KeyFrame *pKF = vpKFs[i];
                Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
            }
            Eigen::Matrix4f CPos = mpMapDrawer->GetCurrectCameraPose();
            for (int ii = 0;ii<4;ii++){
                for (int jj = 0;jj<4;jj++){
                    cout<<to_string(CPos(ii,jj))<<" ";
                }
                cout<<endl;
            }
            cout<<"end"<<endl;
//                if (true) {
//                    for (int ii = 0; ii < 3; ii++)
//                        cout << to_string(CPos(ii, 3)) << " ";
//                    cout << endl;
//                }
//
//            int fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
//            if (fd < 0) {
//                perror("open serial port error");
//            }
//
//            // 设置串口参数
//            struct termios options;
//            tcgetattr(fd, &options);
//            options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
//            options.c_iflag = IGNPAR;
//            options.c_oflag = 0;
//            options.c_lflag = 0;
//            tcflush(fd, TCIFLUSH);
//            tcsetattr(fd, TCSANOW, &options);
//
//            // 发送字符串
//            float Angle = atan2(CPos(1, 0), CPos(0, 0))/3.141592653*180;
//            if((clock() - start)*1000<3.0){
//                cout<<"1 90"<<endl;
////                write(fd, "1 90", strlen("1 90"));
//                cout<<"0 0"<<endl;
////                write(fd, "0 0", strlen("0 0"));
//                cout<<"-1 90"<<endl;
////                write(fd, "-1 90", strlen("-1 90"));
//                cout<<"0 0"<<endl;
////                write(fd, "0 0", strlen("0 0"));
//            }else{
//                if(Angle_D > 90 || Angle_D < -90){
//                    cout<<"0 0"<<endl;
////                    write(fd, "0 0", strlen("0 0"));
//                }
//                if(Angle<0){
//                    cout<<"1"<<" "<<0-Angle<<endl;
////                    write(fd, "1 90", strlen("1 90"));
//                    Angle_D += Angle;
//                }else{
//                    cout<<"-1"<<" "<<Angle<<endl;
////                    write(fd, "1 180", strlen("1 180"));
//                    Angle_D -= Angle;
//                }
//            }
//
//            // 关闭串口
//            close(fd);

        }

        if(Reset)
        {
            mpSystem->ResetActiveMap();
        }

        if(Stop)
        {
            // Stop all threads
            mpSystem->Shutdown();
            // Save camera trajectory
            mpSystem->SaveTrajectoryEuRoC("CameraTrajectory.txt");
            mpSystem->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
        }

        if(NoViewer::Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
            printf("0");
        }

        if(CheckFinish()) {
            printf("1");
            break;
        }
    }

    SetFinish();
}

void NoViewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool NoViewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void NoViewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool NoViewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void NoViewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool NoViewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool NoViewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void NoViewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

/*void Viewer::SetTrackingPause()
{
    mbStopTrack = true;
}*/

}
