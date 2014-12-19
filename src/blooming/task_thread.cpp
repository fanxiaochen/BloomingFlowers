
#include <iostream>

#include "main_window.h"
#include "parameters.h"
#include "point_cloud.h"
#include "flower.h"
#include "petal.h"
#include "mesh_model.h"
#include "points_file_system.h"
#include "mesh_file_system.h"
#include "scene_widget.h"
#include "tracking_system.h"
#include "trajectory_model.h"
#include "task_thread.h"


PointsTrackThread::PointsTrackThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

PointsTrackThread::~PointsTrackThread()
{}

void PointsTrackThread::run()
{
    std::cout << "Point Cloud Tracking Starts..." << std::endl;

    PointsFileSystem* points_file_system = tracking_system_->getPointsFileSystem();

    int start_frame = points_file_system->getStartFrame();
    int end_frame = points_file_system->getEndFrame();

    PointCloud* tracking_template = points_file_system->getPointCloud(start_frame);
    MainWindow::getInstance()->getSceneWidget()->addSceneChild(tracking_template);

    for (int i = start_frame + 1; i <= end_frame; i ++)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;

        PointCloud* tracked_frame = points_file_system->getPointCloud(i);	
        points_file_system->showPointCloud(i - 1);
        tracking_template->expire();

        tracking_system_->cpd_registration(*tracked_frame, *tracking_template);

        points_file_system->hidePointCloud(i - 1);
    }

    std::cout << "Point Cloud Tracking Finished!" << std::endl;
}


MeshTrackThread::MeshTrackThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

MeshTrackThread::~MeshTrackThread()
{}

void MeshTrackThread::run()
{
    std::cout << "Mesh Tracking Starts..." << std::endl;

    PointsFileSystem* points_file_system = tracking_system_->getPointsFileSystem();
    MeshFileSystem* mesh_file_system = tracking_system_->getMeshFileSystem();

    int start_frame = points_file_system->getStartFrame();
    int end_frame = points_file_system->getEndFrame();

    QSet<QPersistentModelIndex> mesh_indexes = mesh_file_system->getCheckedIndexes();
    MeshModel* tracking_template = mesh_file_system->getMeshModel(mesh_indexes.values().at(0));

    std::vector<int> deform_idx;
    for (size_t i = 0, i_end = tracking_template->getVertices()->size(); i < i_end; i ++)
        deform_idx.push_back(i);

    for (int i = start_frame + 1; i <= end_frame; i ++)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;

        PointCloud* tracked_frame = points_file_system->getPointCloud(i);	
        points_file_system->showPointCloud(i - 1);

        tracking_system_->cpd_registration(*tracked_frame, *tracking_template);

        //VectorFArray m_vfa;
        //VectorIArray m_via;
        //MESHMODEL_TO_VECTORARRAY(*tracking_template, m_vfa, m_via);

        //VectorFArray p_vfa;
        //VectorIArray p_via;
        //osg::ref_ptr<PointCloud> cloud = new PointCloud;
        //tracking_template->searchNearestIdx(tracked_frame, p_via);
        //tracked_frame->reordering(cloud, p_via);
        //POINTCLOUD_TO_VECTORARRAY(*cloud, p_vfa, p_via);

        //// stable now!
        //tracking_template->deform(m_vfa, m_via, p_vfa, p_via);

        tracking_template->expire();
        points_file_system->hidePointCloud(i - 1);
    }

    std::cout << "Mesh Tracking Finished!" << std::endl;

//    MeshFileSystem* mesh_file_system = tracking_system_->getMeshFileSystem();
//    QSet<QPersistentModelIndex> mesh_indexes = mesh_file_system->getCheckedIndexes();
//    MeshModel* tracking_template = mesh_file_system->getMeshModel(mesh_indexes.values().at(0));
//    osg::Vec3Array* indicators = new osg::Vec3Array;
//    std::vector<int> deform_idx;
//
//    /*indicators->push_back(osg::Vec3(0.05, 0.05, 0.1));
//    deform_idx.push_back(2);
//    tracking_template->deform(*indicators, deform_idx);
//    tracking_template->expire();*/
//
//    //////for (size_t i = 0, i_end = 1; i < i_end; i ++)
//    deform_idx.push_back(0);
//    deform_idx.push_back(440);
//
//    indicators->push_back(osg::Vec3(0.0, 0.0, 0.0));
//    indicators->push_back(osg::Vec3(1.0, 0.8, 0.5));
//
//    //////for (size_t i = 0, i_end = 1; i < i_end; i ++)
//    //deform_idx.push_back(84);
//    //deform_idx.push_back(336);
//
////    indicators->push_back(osg::Vec3(0.05, 0.05, 0.1));
////    indicators->push_back(osg::Vec3(0.1, 0.0, 0.1));
//
//    //for (size_t i = 0, i_end = 1; i < i_end; i ++)
////    deform_idx.push_back(2);
////    deform_idx.push_back(2);
//
//    tracking_template->deform(*indicators, deform_idx);
//    tracking_template->expire();
//
//
//    //for (int k = 0; k < 10; k ++)
//    //{
//    //    indicators->clear();
//    //    deform_idx.clear();
//
//    //    indicators->push_back(osg::Vec3(0.2+0.05*k, 0.4, 0.3));
//    //    indicators->push_back(osg::Vec3(0.8-0.05*k, 0.4, -0.3));
//
//    //    //for (size_t i = 0, i_end = 1; i < i_end; i ++)
//    //    deform_idx.push_back(84);
//    //    deform_idx.push_back(336);
//
//    //    tracking_template->deform(*indicators, deform_idx);
//    //    tracking_template->expire();
//    //}
//     
//
//    std::string file = MainWindow::getInstance()->getWorkspace() + "/deformed_mesh.obj";
//    tracking_template->save(file);
}

FlowerTrackThread::FlowerTrackThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

FlowerTrackThread::~FlowerTrackThread()
{}

void FlowerTrackThread::run()
{
    std::cout << "Flower Tracking Starts..." << std::endl;

    int key_frame = MainWindow::getInstance()->getParameters()->getKeyFrame();
    Flowers& flowers = tracking_system_->getFlowers();

    PointsFileSystem* points_file_system = tracking_system_->getPointsFileSystem();
    MeshFileSystem* mesh_file_system = tracking_system_->getMeshFileSystem();

    int start_frame = points_file_system->getStartFrame();
    int end_frame = points_file_system->getEndFrame();

    QSet<QPersistentModelIndex> mesh_indexes = mesh_file_system->getCheckedIndexes();
    
    Flower flower;
    for (size_t i = 0, i_end = mesh_indexes.size(); i < i_end; ++ i)
    {
        MeshModel* petal_template = mesh_file_system->getMeshModel(mesh_indexes.values().at(i));
        osg::ref_ptr<Petal> petal(new Petal(i));
        petal->setMeshModel(petal_template);
        flower.getPetals().push_back(petal);
    }
    
    flower.show();
   // flowers.push_back(flower);

    Flower simplified_flower = flower.simplifyMesh(25);
    std::vector<std::vector<int> > hard_knn_idx; 
    flower.searchNearestIdx(simplified_flower, hard_knn_idx);


  //  simplified_flower.show();

    for (size_t i = key_frame, i_end = end_frame;
        i < i_end; ++ i)
    {
        std::cout << "tracking [frame " << i+1 << "]" << std::endl;
        points_file_system->showPointCloud(i);

        PointCloud* forward_cloud = points_file_system->getPointCloud(i + 1);
        QString large_file = QString(forward_cloud->getFilename().c_str());
        QString old_str("small"), new_str("large");
        large_file.replace(large_file.indexOf(old_str), old_str.size(), new_str);
        PointCloud* large_forward_cloud = points_file_system->getPointCloud(large_file.toStdString());

        tracking_system_->cpd_registration(*forward_cloud, simplified_flower);

        std::vector<osg::ref_ptr<osg::Vec3Array> > hard_ctrs;
        for (size_t j = 0, j_end = simplified_flower.getPetals().size(); j < j_end; ++ j)
        {
            Petals& petals = simplified_flower.getPetals();
            hard_ctrs.push_back(petals[j]->getVertices());
        }

        flower.deform(hard_ctrs, hard_knn_idx);

        /*std::vector<osg::ref_ptr<osg::Vec3Array> > soft_ctrs;
        std::vector<std::vector<int> > soft_knn_idx;

        flower.searchNearestIdx(*large_forward_cloud, soft_ctrs);

        for (size_t j = 0, j_end = flower.getPetals().size(); j < j_end; ++ j)
        {
        Petals& petals = flower.getPetals();
        std::vector<int> soft_idx;
        for (size_t k = 0, k_end = petals[j]->getVertices()->size(); k < k_end; ++ k)
        soft_idx.push_back(k);
        soft_knn_idx.push_back(soft_idx);
        }

        for (size_t j = 0, j_end = hard_knn_idx.size(); j < j_end; ++ j)
        {
        osg::Vec3Array* hard_ctr = hard_ctrs.at(j);
        osg::Vec3Array* soft_ctr = soft_ctrs.at(j);
        std::vector<int>& hard_idx = hard_knn_idx.at(j);
        for (size_t k = 0, k_end = hard_idx.size(); k < k_end; ++ k)
        {
        soft_ctr->at(hard_idx[k]) = hard_ctr->at(k);
        }
        }

        flower.deform(hard_ctrs, hard_knn_idx, soft_ctrs, soft_knn_idx);*/

        flower.update();

        points_file_system->hidePointCloud(i);
    }

    /*std::vector<int> deform_idx;
    for (size_t i = 0, i_end = tracking_template->getVertices()->size(); i < i_end; i ++)
    deform_idx.push_back(i);*/

    //for (int i = start_frame + 1; i <= end_frame; i ++)
    //{
    //    std::cout << "tracking [frame " << i << "]" << std::endl;

    //    PointCloud* tracked_frame = points_file_system->getPointCloud(i);	
    //    points_file_system->showPointCloud(i - 1);
    //    tracking_template->expire();

    //    tracking_system_->cpd_registration(*tracked_frame, *tracking_template);

    //    // unstable
    //    // tracking_template->deform(*tracking_template->getVertices(), deform_idx);
    //    tracking_template->deform(*tracking_template->getVertices());

    //    points_file_system->hidePointCloud(i - 1);
    //}

    std::cout << "Flower Tracking Finished!" << std::endl;
}


TrajectoryTrackThread::TrajectoryTrackThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

TrajectoryTrackThread::~TrajectoryTrackThread()
{}

void TrajectoryTrackThread::run()
{
    std::cout << "Trajectory Tracking Starts..." << std::endl;

    PointsFileSystem* points_file_system = tracking_system_->getPointsFileSystem();
    Trajectories* trajectories = tracking_system_->getTrajectories();

    int start_frame = points_file_system->getStartFrame();
    int end_frame = points_file_system->getEndFrame();

    PointCloud* source = points_file_system->getPointCloud(start_frame);
    points_file_system->showPointCloud(start_frame);
    trajectories->showTrajectories();

    std::vector<int> src_idx;
    for (size_t i = 0, i_end = source->size(); i < i_end; i ++)
    {
        src_idx.push_back(i);

        Trajectories::TrajectoryPath traj_path;
        traj_path._trajectory.push_back(i);
        traj_path._id = i;
        traj_path._label = -1;
        trajectories->getPaths().push_back(traj_path);
    }

    for (int i = start_frame + 1; i <= end_frame; i ++)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;

        PointCloud* target = points_file_system->getPointCloud(i);
        points_file_system->showPointCloud(i);

        std::vector<int> tar_idx;

        tracking_system_->cpd_registration(*source, *target, src_idx, tar_idx);

        trajectories->expire();

        //points_file_system->hidePointCloud(i - 1);

        source = target;
        src_idx = tar_idx;
    }

    //for (auto& it = trajectories->getPaths().begin(); it != trajectories->getPaths().end(); ++ it)
    //{
    //    std::cout << "id " << it->_id << std::endl;
    //    for (auto& tt = it->_trajectory.begin(); tt != it->_trajectory.end(); ++ tt)
    //        std::cout << *tt << " ";
    //    std::cout << std::endl;
    //}

    std::string workspace = MainWindow::getInstance()->getWorkspace();
    std::string traj_file = workspace + "/trajectories.json";
    if (!trajectories->save(traj_file))
        std::cerr << "trajectories save error!" << std::endl;

    std::cout << "Trajectory Tracking Finished!" << std::endl;
}


TrajClusteringThread::TrajClusteringThread(Trajectories* trajectories)
    :QThread()
{
    trajectories_ = trajectories;
}

TrajClusteringThread::~TrajClusteringThread()
{}

void TrajClusteringThread::run()
{
    std::cout << "Start Trajectory Clustering..." << std::endl;

    trajectories_->clustering();

    trajectories_->expire();

    std::cout << "Trajectory Clustering Finished..." << std::endl;
    return;
}

SegmentThread::SegmentThread(PointsFileSystem* points_file_system, int frame)
    :QThread()
{
    points_file_system_ = points_file_system;
    frame_ = frame;
}

SegmentThread::~SegmentThread()
{}

void SegmentThread::run()
{
    std::cout << "Start Segmentation..." << std::endl;

    points_file_system_->segmentPointCloud(frame_);

    std::cout << "Segmentation Finished..." << std::endl;
    return;
}

PropagateSegmentsThread::PropagateSegmentsThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

PropagateSegmentsThread::~PropagateSegmentsThread()
{}

void PropagateSegmentsThread::run()
{
    std::cout << "Start Propagating Segments..." << std::endl;

    int key_frame = tracking_system_->getKeyFrame();
    PointsFileSystem* points_file_system = tracking_system_->getPointsFileSystem();
    PointCloud* key_cloud = points_file_system->getPointCloud(key_frame);

    std::cout << "forward propagation starts" << std::endl;
    for (size_t i = key_frame, i_end = points_file_system->getEndFrame();
        i < i_end; ++ i)
    {
        PointCloud* forward_cloud = points_file_system->getPointCloud(i + 1);

        std::vector<int> src_idx, tar_idx;
        for (size_t i = 0, i_end = forward_cloud->size(); i < i_end; ++ i)
            src_idx.push_back(i);

        tracking_system_->cpd_registration(*forward_cloud, *key_cloud, src_idx, tar_idx);
        
        for (size_t j = 0, j_end = src_idx.size(); j < j_end; ++ j)
        {
            std::vector<PointCloud::FlowerPoint>& cluster_points = 
                forward_cloud->getFlowerPoints();
            
            PointCloud::FlowerPoint key_cp = 
                key_cloud->getFlowerPoints()[tar_idx[j]];
            PointCloud::FlowerPoint fw_cp;
            fw_cp._pt = key_cp._pt;
            fw_cp._label = key_cp._label;
            cluster_points.push_back(fw_cp);
        }

        forward_cloud->expire();
        key_cloud = forward_cloud;
    }

    std::cout << "backward propagation starts" << std::endl;
    for (size_t i = key_frame, i_end = points_file_system->getStartFrame();
        i > i_end; -- i)
    {
        PointCloud* backward_cloud = points_file_system->getPointCloud(i - 1);

        std::vector<int> src_idx, tar_idx;
        for (size_t i = 0, i_end = backward_cloud->size(); i < i_end; ++ i)
            src_idx.push_back(i);

        tracking_system_->cpd_registration(*backward_cloud, *key_cloud, src_idx, tar_idx);

        for (size_t j = 0, j_end = src_idx.size(); j < j_end; ++ j)
        {
            std::vector<PointCloud::FlowerPoint>& cluster_points =
                backward_cloud->getFlowerPoints();

            PointCloud::FlowerPoint key_cp =
                key_cloud->getFlowerPoints()[tar_idx[j]];

            PointCloud::FlowerPoint bw_cp;
            bw_cp._pt = key_cp._pt;
            bw_cp._label = key_cp._label;
            cluster_points.push_back(bw_cp);
        }

        backward_cloud->expire();
        key_cloud = backward_cloud;
    }



    std::cout << "Propagating Segments Finished..." << std::endl;
    return;
}

