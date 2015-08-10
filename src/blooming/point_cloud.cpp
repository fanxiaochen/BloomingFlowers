
#include <QRegExp>
#include <QFileInfo>
#include <QDockWidget>
#include <QFileDialog>

#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/Point>

#include <pcl/kdtree/kdtree_flann.h>

#include "main_window.h"
#include "points_file_system.h"
#include "osg_viewer_widget.h"
#include "tracking_system.h"
#include "color_map.h"
#include "point_cloud.h"
#include "flower.h"

PointCloud::PointCloud(void)
    :segmented_(false),
    show_boundary_(false),
    show_tips_(false),
    show_probs_(false)
{
  
}

PointCloud::~PointCloud(void)
{
}

bool PointCloud::open(const std::string& filename)
{
  clearData();

  if (pcl::io::loadPCDFile(filename, *this) != 0)
    return false;

  filename_ = filename;

  expire();

  return true;
}

void PointCloud::reload(void)
{
  clearData();
  open(filename_);

  return;
}

void PointCloud::clearData()
{
    Renderable::clear();
    return;
}


void PointCloud::updateImpl()
{
    visualizePoints();
    return;
}

void PointCloud::visualizePoints()
{
    // for points
    osg::ref_ptr<osg::Vec3Array>  vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array>  colors = new osg::Vec4Array;

    for (size_t i = 0, i_end = size(); i < i_end; i ++)
    {
        if (show_probs_)
        {
            const Point& point = at(i);
            vertices->push_back(osg::Vec3(point.x, point.y, point.z));
            colors->push_back(ColorMap::getInstance().getContinusColor(color_flags_[i]));
        }
        else {
            if (!segmented_)
            {
                const Point& point = at(i);
                vertices->push_back(osg::Vec3(point.x, point.y, point.z));
                colors->push_back(osg::Vec4(point.r / 255.0, point.g / 255.0, point.b / 255.0, 0));
            }
            else
            {
                if (segment_flags_[i] == -1)
                {
                    const Point& point = at(i);
                    vertices->push_back(osg::Vec3(point.x, point.y, point.z));
                    colors->push_back(osg::Vec4(point.r / 255.0, point.g / 255.0, point.b / 255.0, 0));
                }
                else 
                {
                    const Point& point = at(i);
                    vertices->push_back(osg::Vec3(point.x, point.y, point.z));
                    colors->push_back(ColorMap::getInstance().getDiscreteColor(segment_flags_[i]));
                }
            }
        }
    }

    osg::Geometry* geometry = new osg::Geometry;
    geometry->setVertexArray(vertices);
    geometry->setColorArray(colors);
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, size()));
    geometry->getOrCreateStateSet()->setAttribute(new osg::Point(5.0f));

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(geometry);
   

    // for tips
    if (show_tips_)
    {
        osg::ref_ptr<osg::Vec3Array>  tvertices = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec4Array>  tcolors = new osg::Vec4Array;
        for (size_t i = 0, i_end = tip_indices_.size(); i < i_end; ++ i)
        {
            const Point& point = at(tip_indices_[i]);
            tvertices->push_back(osg::Vec3(point.x, point.y, point.z));
            tcolors->push_back(ColorMap::getInstance().getDiscreteColor(18));

        }

        osg::Geometry* tgeometry = new osg::Geometry;
        tgeometry->setVertexArray(tvertices);
        tgeometry->setColorArray(tcolors);
        tgeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
        tgeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, tvertices->size()));
        tgeometry->getOrCreateStateSet()->setAttribute(new osg::Point(15.0f));
        geode->addDrawable(tgeometry);
    }

    // for boundary
    if (show_boundary_)
    {
        osg::ref_ptr<osg::Vec3Array>  bvertices = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec4Array>  bcolors = new osg::Vec4Array;
        for (size_t i = 0, i_end = boundary_indices_.size(); i < i_end; ++ i)
        {
            const Point& point = at(boundary_indices_[i]);
            bvertices->push_back(osg::Vec3(point.x, point.y, point.z));
            bcolors->push_back(ColorMap::getInstance().getDiscreteColor(8));

        }
        osg::Geometry* bgeometry = new osg::Geometry;
        bgeometry->setVertexArray(bvertices);
        bgeometry->setColorArray(bcolors);
        bgeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
        bgeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, bvertices->size()));
        bgeometry->getOrCreateStateSet()->setAttribute(new osg::Point(10.0f));
        geode->addDrawable(bgeometry);
    }
    
    content_root_->addChild(geode);

    return;
}

PointCloud* PointCloud::getPrevFrame(void)
{
    PointsFileSystem* model = dynamic_cast<PointsFileSystem*>(MainWindow::getInstance()->getPointsSystem());
    return model->getPointCloud(getFrame()-1);
}

PointCloud* PointCloud::getNextFrame(void)
{
    PointsFileSystem* model = dynamic_cast<PointsFileSystem*>(MainWindow::getInstance()->getPointsSystem());
    return model->getPointCloud(getFrame()+1);
}


int PointCloud::getFrame(void) const      
{
  QRegExp frame("[\\/]frame_([0-9]{5,5})[\\/]");
  frame.indexIn(filename_.c_str());
  QString index = frame.cap(1);

  return index.toInt();
}


bool PointCloud::isShown(void) const
{
  PointsFileSystem* model = dynamic_cast<PointsFileSystem*>(MainWindow::getInstance()->getPointsSystem());

  return model->isShown(filename_);
}

void PointCloud::searchNearestIdx(MeshModel* mesh_model, std::vector<int>& knn_idx, std::vector<float>& knn_dists)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0, i_end = mesh_model->getVertices()->size(); i < i_end; ++ i)
    {
        // we only use visible vertices to segment point cloud
        if (mesh_model->getVisibility().size() != 0 && mesh_model->getVisibility()[i] == 0)
        {
             pcl::PointXYZ pcl_point(0.0f, 0.0f, 0.0f);  // origin point is far from our dataset
             cloud->push_back(pcl_point);
             continue;
        }

        const osg::Vec3& point = mesh_model->getVertices()->at(i);
        pcl::PointXYZ pcl_point(point.x(), point.y(), point.z());
        cloud->push_back(pcl_point);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (cloud);

    int K = 1;

    // K nearest neighbor search

    for (size_t i = 0, i_end = this->size(); i < i_end; ++ i)
    {
        pcl::PointXYZ searchPoint;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        Point& point = this->at(i);

        searchPoint.x = point.x;
        searchPoint.y = point.y;
        searchPoint.z = point.z;

        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            knn_idx.push_back(pointIdxNKNSearch[0]);
            knn_dists.push_back(pointNKNSquaredDistance[0]);
        }
    }
}

osg::ref_ptr<PointCloud> PointCloud::getPetalCloud(int id)
{
    osg::ref_ptr<PointCloud> petal_cloud = new PointCloud;

    for (size_t i = 0, i_end = segment_flags_.size(); i < i_end; ++ i)
    {
        if (segment_flags_[i] == id)
            petal_cloud->push_back(this->at(i));
    }

    if (petal_cloud->size() == 0)
        return NULL;

    return petal_cloud;
}

//void PointCloud::flower_segmentation(Flower* flower)
//{
//    segment_flags_.clear();
//
//    std::vector<std::vector<int> > knns_idx;
//    std::vector<std::vector<float> > knns_dists;
//
//    for (size_t i = 0, i_end = flower->getPetals().size(); i < i_end; ++ i)
//    {
//        std::vector<int> knn_idx;
//        std::vector<float> knn_dists;
//
//        Petal& petal = flower->getPetals().at(i);
//
//        searchNearestIdx(&petal, knn_idx, knn_dists);
//
//        knns_idx.push_back(knn_idx);
//        knns_dists.push_back(knn_dists);
//    }
//
//
//    for (size_t i = 0, i_end = this->size(); i < i_end; ++ i)
//    {
//        float min_dist = std::numeric_limits<float>::max();
//        int min_j = std::numeric_limits<int>::max();
//
//        for (size_t j = 0, j_end = knns_idx.size(); j < j_end; ++ j)
//        {
//            if (min_dist > knns_dists[j][i])
//            {
//                min_j = j;
//                min_dist = knns_dists[j][i];
//            }
//        }
//
//        segment_flags_.push_back(min_j);
//    }
//
//    segmented_ = true;
//}

void PointCloud::flower_segmentation(Flower* flower)
{
    region_matching(flower);

    segment_flags_ = std::vector<int>(this->size(), -1);

    for (size_t i = 0; i < match_regions_.size(); i ++)
    {
        std::vector<int>& indices = match_regions_[i].first;
        for (size_t j = 0; j < indices.size(); j ++)
        {
            segment_flags_[indices[j]] = i;
        }
    }

    segment_number_ = match_regions_.size();
    segmented_ = true;

    boundary_segmentation();
}

void PointCloud::boundary_segmentation()
{
    boundary_segments_.resize(segment_number_);

    for (size_t i = 0, i_end = boundary_indices_.size(); i < i_end; ++ i)
    {
        int petal_id = segment_flags_[boundary_indices_[i]];
        if (petal_id != -1)
        {
            boundary_segments_[petal_id].push_back(boundary_indices_[i]);
        }
    }
}

osg::ref_ptr<PointCloud> PointCloud::getBoundary(int id)
{
    osg::ref_ptr<PointCloud> petal_cloud = new PointCloud;

    for (size_t i = 0, i_end = boundary_segments_[id].size(); i < i_end; ++ i)
    {
        petal_cloud->push_back(this->at(boundary_segments_[id][i]));
    }

    if (petal_cloud->size() == 0)
        return NULL;

    return petal_cloud;
}


void PointCloud::region_matching(Flower* flower)
{

    Petals& petals = flower->getPetals();

    if (petals.size() == 1) 
    {
        match_regions_.resize(petals.size());
        for (size_t i = 0; i < match_regions_.size(); i ++)
        {
            MatchRegion mr;
            mr.first = [](int n){
                std::vector<int> indice;
                for (size_t i = 0; i < n; ++i){
                    indice.push_back(i);
                }
                return indice;
            }(petals[0].getVertices()->size());
            mr.second = this;
            match_regions_[i] = mr;
        }
        return;
    }

    Eigen::MatrixXd P(this->size(), petals.size());

    // compute p(point belongs to petal)
    for (size_t i = 0, i_end = size(); i < i_end; ++ i)
    {
        const Point& point = at(i);
        std::vector<double> m(petals.size(), 0);
        for (size_t j = 0, j_end = petals.size(); j < j_end; ++ j)
        {
            Petal& petal = petals.at(j);
            osg::ref_ptr<osg::Vec3Array> vertices = petal.getVertices();
            for (size_t t = 0, t_end = vertices->size(); t < t_end; ++ t)
            {
                m[j] += gaussian(t, i, &petal);
            }
        }

        double m_sum = 0;
        for (size_t j = 0, j_end = petals.size(); j < j_end; ++ j)
            m_sum += m[j];

        for (size_t j = 0, j_end = petals.size(); j < j_end; ++ j)
            P(i, j) = m[j]/m_sum;
    }

    // each point's belongs
    // better way to determine belong lists??
    std::vector<std::vector<int>> belong_list(this->size());
    double delta = 0.1;
    for (size_t i = 0; i < P.rows(); ++ i)
    {
        std::vector<int> belongs;
        Eigen::VectorXd pvec = P.row(i);
        double k = pvec.maxCoeff();
        for (size_t j = 0; j < pvec.rows(); ++ j)
        {
            if (pvec(j) / k > delta)
            {
                belongs.push_back(j);
            }
        }
        belong_list[i] = belongs;
    }

    // build initial match regions
    match_regions_.resize(petals.size());
    for (size_t i = 0; i < match_regions_.size(); i ++)
    {
        MatchRegion mr;
        /*mr.first = [](int n){
        std::vector<int> indice;
        for (size_t i = 0; i < n; ++i){
        indice.push_back(i);
        }
        return indice;
        }(petals[0].getVertices()->size());*/
        mr.second = new PointCloud;
        match_regions_[i] = mr;
    }

    //// build self kdtree for petals
    //for (size_t i = 0; i < petals.size(); ++ i)
    //{
    //    petals[i].buildSelfKdTree();
    //}

    // fill the match regions
   // std::vector<std::unordered_set<int>> remove_set(petals.size());
    for (int i = 0; i < belong_list.size(); i ++)
    {
        std::vector<int> belongs = belong_list[i];
        if (belongs.size() >= 2)
        {
            //Point& point = this->at(i);
            //osg::Vec3 vp(point.x, point.y, point.z);
            //for (int j = 0; j < belongs.size(); j ++)
            //{
            //    Petal& petal = petals[belongs[j]];
            //    int idx = petal.searchNearestIdx(vp); // use knn to find mesh vertices which should be removed
            //    remove_set[belongs[j]].insert(idx);
            //}
        }
        else {
            int petal_id = belongs[0];
            match_regions_[petal_id].first.push_back(i);
            match_regions_[petal_id].second->push_back(this->at(i));
        }
    }

    /*for (size_t i = 0; i < match_regions_.size(); i ++)
    {
    MatchRegion& mr = match_regions_[i];
    mr.first = [&](int n){
    std::vector<int> ver_idx;
    for (size_t j = 0; j < n; ++ j)
    {
    if (remove_set[i].find(j) == remove_set[i].end())
    ver_idx.push_back(j);
    }
    return ver_idx;
    }(petals[i].getVertices()->size());
    }*/

    /*for (size_t i = 0; i < match_regions_.size(); i ++)
    {
        MainWindow::getInstance()->getSceneWidget()->addSceneChild(match_regions_[i].second);
    }
*/

    std::cout << "finish region matching!" << std::endl;
}

double PointCloud::gaussian(int m_id, int c_id, Petal* petal)
{
    double p;

    Eigen::Matrix3Xd& cov_mat = petal->getGaussianSphere();
    osg::Vec3& vertice = petal->getVertices()->at(m_id);
    Point& point = this->at(c_id);

    Eigen::Vector3d xu = Eigen::Vector3d(vertice.x(), vertice.y(), vertice.z()) 
        - Eigen::Vector3d(point.x, point.y, point.z);
     p = pow(2*M_PI, -3/2.0) * pow((cov_mat.col(m_id).asDiagonal()).toDenseMatrix().determinant(), -1/2.0) * 
    exp((-1/2.0)*xu.transpose()*cov_mat.col(m_id).asDiagonal().inverse()*xu);
    // direct distance
   // p = xu.norm();

    return p;
}

osg::ref_ptr<PointCloud> PointCloud::getFittingCloud(int id)
{
    return match_regions_[id].second;
}

std::vector<int> PointCloud::getFittingMesh(int id)
{
    return match_regions_[id].first;
}


void PointCloud::region_segmentation(Flower* flower)
{
    Petals& petals = flower->getPetals();

    P_.resize(this->size(), petals.size());

    // compute p(point belongs to petal)
    for (size_t i = 0, i_end = size(); i < i_end; ++ i)
    {
        const Point& point = at(i);
        std::vector<double> m(petals.size(), 0);
        for (size_t j = 0, j_end = petals.size(); j < j_end; ++ j)
        {
            Petal& petal = petals.at(j);
            osg::ref_ptr<osg::Vec3Array> vertices = petal.getVertices();
            for (size_t t = 0, t_end = vertices->size(); t < t_end; ++ t)
            {
                m[j] += gaussian(t, i, &petal);
            }
        }

        double m_sum = 0;
        for (size_t j = 0, j_end = petals.size(); j < j_end; ++ j)
            m_sum += m[j];

        std::vector<double> test;
        for (size_t j = 0, j_end = petals.size(); j < j_end; ++ j)
        {
            P_(i, j) = m[j]/m_sum;
            test.push_back(P_(i,j));
        }

        test.clear();
    }

    color_flags_.resize(this->size());
    for (size_t i = 0, i_end = size(); i < i_end; ++ i)
    {
        color_flags_[i] = int(P_(i, 0) * 64);
    }
    //std::cout << P_ << std::endl;
    show_probs_ = true;
    expire();
}

void PointCloud::region_growing(std::vector<int>& segment_index, int petal_id)
{
    std::unordered_set<int> segment_set(segment_index.begin(), segment_index.end());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0, i_end = this->size(); i < i_end; ++ i)
    {
        const Point& point = this->at(i);

        pcl::PointXYZ pcl_point(point.x, point.y, point.z);
        cloud->push_back(pcl_point);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (cloud);

    float growing_radius = 2;

    // radius neighbor search 

    for (int k = 0; k < segment_index.size(); ++ k)
    {
        pcl::PointXYZ searchPoint;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        searchPoint.x = this->at(segment_index[k]).x;
        searchPoint.y = this->at(segment_index[k]).y;
        searchPoint.z = this->at(segment_index[k]).z;

        if ( kdtree.radiusSearch (searchPoint, growing_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            bool growing = true;
            for (int i = 0, i_end = pointIdxRadiusSearch.size(); i < i_end; ++ i)
            {
                if (segment_flags_[pointIdxRadiusSearch[i]] != -1 && segment_flags_[pointIdxRadiusSearch[i]] != petal_id)
                {
                    growing = false;
                    break;
                }
            }

            if (growing)
            {
                for (int i = 0, i_end = pointIdxRadiusSearch.size(); i < i_end; ++ i)
                {
                    if (segment_set.find(pointIdxRadiusSearch[i]) == segment_set.end())
                    {
                        segment_flags_[pointIdxRadiusSearch[i]] = petal_id;
                        segment_index.push_back(pointIdxRadiusSearch[i]);
                        segment_set.insert(pointIdxRadiusSearch[i]);
                    }
                }
            }
        }
    }
}