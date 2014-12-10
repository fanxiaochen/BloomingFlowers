#include <boost/math/special_functions/round.hpp>

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>

#include "point_cloud.h"
#include "flower.h"
#include "points_file_system.h"
#include "color_map.h"
#include "main_window.h"
#include "trajectory_model.h"

Trajectories::Trajectories(PointsFileSystem* points_file_system)
{
    points_file_system_ = points_file_system;
}

Trajectories::~Trajectories()
{

}

bool Trajectories::load(const std::string& file)
{
    QFile load_file(QString(file.c_str()));

    if (!load_file.open(QIODevice::ReadOnly)) {
        qWarning("Couldn't open save file.");
        return false;
    }

    QByteArray data = load_file.readAll();
    QJsonDocument load_doc(QJsonDocument::fromJson(data));
    QJsonArray trajs = load_doc.array();

   // read
    traj_paths_.clear();

    for (size_t i = 0, i_end = trajs.size(); i < i_end; i ++)
    {
        TrajectoryPath traj_path;
        QJsonObject traj = trajs[i].toObject();
        traj_path._id = boost::math::iround(traj["id"].toDouble());
        traj_path._label = boost::math::iround(traj["label"].toDouble());

        QJsonArray traj_array = traj["trajectory"].toArray();
        for (size_t j = 0, j_end = traj_array.size(); j < j_end; j ++)
        {
            QJsonValue traj_id = traj_array[j];
            traj_path._trajectory.push_back(boost::math::iround(traj_id.toDouble()));
        }

        traj_paths_.push_back(traj_path);
    }

    return true;
}

bool Trajectories::save(const std::string& file)
{
    QFile save_file(QString(file.c_str()));

    if (!save_file.open(QIODevice::WriteOnly)) 
    {
        qWarning("Couldn't open save file.");
        return false;
    }

    //write
    QJsonArray trajs;
    for (size_t i = 0, i_end = traj_paths_.size(); i < i_end; i ++)
    {
        TrajectoryPath traj_path = traj_paths_[i];

        QJsonObject traj;
        traj["id"] = traj_path._id;
        traj["label"] = traj_path._label;
        
        QJsonArray traj_array;
        for (size_t j = 0, j_end = traj_path._trajectory.size(); j < j_end; j ++)
        {
            QJsonValue traj_id(traj_path._trajectory[j]);
            traj_array.append(traj_id);
        }
        traj["trajectory"] = traj_array;

        trajs.append(traj);
    }


    QJsonDocument save_doc(trajs);
    save_file.write(save_doc.toJson());

    return true;
}

void Trajectories::clustering()
{
    k_means();
    return;
}

void Trajectories::setCeterTrajectories()
{
    PointCloud* start_cloud = points_file_system_->getPointCloud(points_file_system_->getStartFrame());
    const std::vector<int>& picked_points = dynamic_cast<Flower*>(
        start_cloud)->getPickedIndices();

    for (size_t i = 0, i_end = picked_points.size(); i < i_end; ++ i)
    {
        TrajectoryPoint traj_point;
        getPointsFromPath(i, traj_point);
        center_trajs_.push_back(traj_point);
    }

    cluster_num_ = picked_points.size();
}

void Trajectories::updateCenterTrajectories()
{
    PointCloud* start_cloud = points_file_system_->getPointCloud(points_file_system_->getStartFrame());
    std::vector<osg::Vec3>& picked_points = dynamic_cast<Flower*>(
        start_cloud)->getPickedPoints();

    for (size_t i = 0, i_end = picked_points.size(); i < i_end; ++ i)
    {
        TrajectoryPoint center_traj = center_trajs_[i];
        const Point& point = center_traj[0];

        picked_points[i].x() = point.x;
        picked_points[i].y() = point.y;
        picked_points[i].z() = point.z;
    }

    start_cloud->expire();
}

float Trajectories::distance(const TrajectoryPath& path_1, const TrajectoryPath& path_2)
{
    TrajectoryPoint point_1, point_2;
    getPointsFromPath(path_1._id, point_1);
    getPointsFromPath(path_2._id, point_2);

    return distance(point_1, point_2);
}

float Trajectories::distance(const TrajectoryPoint& point_1, const TrajectoryPoint& point_2)
{
    int point1_size = point_1.size();
    int point2_size = point_2.size();
    assert(point1_size == point2_size);

    float dist = 0;
    int n = point1_size;

    for (size_t i = 0, i_end = n; i < i_end; i ++)
    {
        const Point& pt1 = point_1.at(i);
        const Point& pt2 = point_2.at(i);

        dist += sqrt(pow(pt1.x-pt2.x,2)+pow(pt1.y-pt2.y,2)+pow(pt1.z-pt2.z,2));
    }

    return dist;
}

void Trajectories::k_means()
{
    setCeterTrajectories();

    TrajectoryPoints next_centers;
    
    do 
    {
        if (!next_centers.empty())
        {
            center_trajs_ = next_centers;
            next_centers.clear();
        }

        size_t path_num = traj_paths_.size();
        for (size_t i = 0; i < path_num; i ++)
        {
            int cluster_id = determineCluster(traj_paths_[i]);
            traj_paths_[i]._label = cluster_id;
        }

        for (size_t i = 0; i < cluster_num_; i ++)
        {
            std::vector<int> ids;
            for (size_t j = 0; j < path_num; j ++)
            {
                if (traj_paths_[j]._label == i)
                    ids.push_back(j);
            }

            TrajectoryPoint next_center;
            mean_path(ids, next_center);
            next_centers.push_back(next_center);
        }
        
        updateCenterTrajectories();
        expire();

    } while (!terminal(center_trajs_, next_centers));

}

int Trajectories::determineCluster(const TrajectoryPath& path)
{
    TrajectoryPoint traj_point;
    getPointsFromPath(path._id, traj_point);

    float min = std::numeric_limits<float>::max();
    int cluster_id = -1;

    for (size_t i = 0, i_end = cluster_num_; i < i_end; i ++)
    {
        float temp_dist = distance(traj_point, center_trajs_[i]);
        if (min > temp_dist)
        {
            min = temp_dist;
            cluster_id = i;
        }
    }

    return cluster_id;
}

void Trajectories::getPointsFromPath(int id, TrajectoryPoint& traj_point)
{
    TrajectoryPath traj_path = traj_paths_.at(id);
    int start_frame = points_file_system_->getStartFrame();
    for (size_t j = 0, j_end = traj_path._trajectory.size(); j < j_end; j ++)
    {
        PointCloud* point_cloud = points_file_system_->getPointCloud(start_frame + j);
        const Point& point = point_cloud->at(traj_path._trajectory[j]);
        Point center_point;
        center_point.x = point.x;
        center_point.y = point.y;
        center_point.z = point.z;

        traj_point.push_back(center_point);
    }
}

void Trajectories::mean_path(const std::vector<int>& ids, TrajectoryPoint& traj_point)
{
    TrajectoryPoints traj_points;
    for (size_t i = 0, i_end = ids.size(); i < i_end; i ++)
    {
        TrajectoryPoint temp_point;
        getPointsFromPath(ids[i], temp_point);
        traj_points.push_back(temp_point);
    }

    size_t traj_length = traj_points[0].size();
    for (size_t i = 0, i_end = traj_length; i < i_end; i ++)
    {
        Point mean_point;
        float x = 0, y = 0, z = 0;

        size_t traj_num = traj_points.size();
        for (size_t j = 0, j_end = traj_num; j < j_end; j ++)
        {
            const Point& point = traj_points[j][i];

            x += point.x;
            y += point.y;
            z += point.z;
        }

        mean_point.x = x / traj_num;
        mean_point.y = y / traj_num;
        mean_point.z = z / traj_num;

        traj_point.push_back(mean_point);
    }
}

bool Trajectories::terminal(const TrajectoryPoints& current_centers, const TrajectoryPoints& next_centers)
{
    const float eps = 1e-3;

    for (size_t i = 0; i < cluster_num_; i ++)
    {
        float delta = distance(current_centers[i], next_centers[i]);
        if (delta > eps)
            return false;
    }

    return true;
}

void Trajectories::updateImpl()
{
    for (size_t i = 0, i_end = traj_paths_.size(); i < i_end; i ++)
    {
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;

        TrajectoryPath trajectory = traj_paths_.at(i);
        int cluster_id = trajectory._label;

        TrajectoryPoint traj_point;
        getPointsFromPath(trajectory._id, traj_point);

        for (size_t j = 0, j_end = traj_point.size(); j < j_end; j ++)
        {
            vertices->push_back(osg::Vec3(traj_point[j].x, traj_point[j].y, traj_point[j].z));
        }

        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        colors->push_back(ColorMap::getInstance().getDiscreteColor(cluster_id + 1));

        osg::ref_ptr<osg::Geode> geode(new osg::Geode);
        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
        geometry->setUseDisplayList(true);
        geometry->setVertexArray(vertices);
        geometry->setColorArray(colors);
        colors->setBinding(osg::Array::BIND_PER_PRIMITIVE_SET);
        geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));

        geode->addDrawable(geometry);
        content_root_->addChild(geode);

    }
    return;
}

void Trajectories::showTrajectories()
{
    MainWindow::getInstance()->getSceneWidget()->addSceneChild(this);
}

void Trajectories::hideTrajectories()
{
    MainWindow::getInstance()->getSceneWidget()->removeSceneChild(this);
}