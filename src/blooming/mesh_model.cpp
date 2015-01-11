#include <random>

#include <boost/filesystem.hpp>

#include <osgDB/WriteFile>
#include <osg/Point>

#include <pcl/kdtree/kdtree_flann.h>


#include "tiny_obj_loader.h"
#include "obj_writer.h"

#include "point_cloud.h"
#include "mesh_model.h"


MeshModel::MeshModel()
    :vertices_(new osg::Vec3Array),
    texcoords_(new osg::Vec2Array),
    vertex_normals_(new osg::Vec3Array),
    colors_(new osg::Vec4Array),
    /*face_normals_(new osg::Vec3Array),*/
    smoothing_visitor_(new osgUtil::SmoothingVisitor)
{
}

MeshModel::MeshModel(const MeshModel& mesh_model) // deep copy
    :vertices_(new osg::Vec3Array),
    texcoords_(new osg::Vec2Array),
    vertex_normals_(new osg::Vec3Array),
    colors_(new osg::Vec4Array),
    /*face_normals_(new osg::Vec3Array),*/
    smoothing_visitor_(new osgUtil::SmoothingVisitor)
{
    *(this->getVertices()) = *(mesh_model.getVertices());
    *(this->getTexcoords()) = *(mesh_model.getTexcoords());
    *(this->getVertexNormals()) = *(mesh_model.getVertexNormals());
    *(this->getVertices()) = *(mesh_model.getVertices());
    this->getFaces() = mesh_model.getFaces();
    this->getAdjList() = mesh_model.getAdjList();
    this->getEdgeIndex() = mesh_model.getEdgeIndex();
    this->getHardCtrsIndex() = mesh_model.getHardCtrsIndex();
}

MeshModel::~MeshModel(void)
{
}

void MeshModel::visualizeMesh(void)
{
    osg::ref_ptr<osg::Geode> geode(new osg::Geode);
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
    geometry->setUseDisplayList(true);
    geometry->setVertexArray(vertices_);
    colors_->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 0.0f));
    geometry->setColorArray(colors_);
    colors_->setBinding(osg::Array::BIND_OVERALL);

    if (faces_.empty())
    {
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices_->size()));
    }
    else
    {
        /*geometry->setNormalArray(face_normals_);
        face_normals_->setBinding(osg::Array::BIND_PER_VERTEX);*/
        geode->accept(*smoothing_visitor_);

    for (size_t i = 0, i_end = faces_.size(); i < i_end; ++ i)
    {
        size_t vertex_num = faces_[i].size();
        osg::ref_ptr<osg::DrawElementsUInt> face = new osg::DrawElementsUInt(GL_TRIANGLES, vertex_num);
        for (size_t j = 0; j < vertex_num; ++ j)
        face->at(j) = faces_[i][j];

        geometry->addPrimitiveSet(face.get());
    }
    }

    geode->addDrawable(geometry);
    content_root_->addChild(geode);
    


    if (!visibility_.empty())
    {
        osg::ref_ptr<osg::Geode> vis_geo(new osg::Geode);
        osg::ref_ptr<osg::Geometry> vis_geometry = new osg::Geometry;
        osg::ref_ptr<osg::Vec3Array> vis_vetices = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec4Array> vis_colors = new osg::Vec4Array;
        vis_colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 0.0f));

        for (size_t i = 0, i_end = vertices_->size(); i < i_end; ++ i)
        {
            if (visibility_[i] == 1)
                vis_vetices->push_back(vertices_->at(i));
        }

        vis_geometry->setUseDisplayList(true);
        vis_geometry->setVertexArray(vis_vetices);
        vis_geometry->setColorArray(vis_colors);
        vis_colors->setBinding(osg::Array::BIND_OVERALL);
        vis_geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vis_vetices->size()));
        vis_geometry->getOrCreateStateSet()->setAttribute(new osg::Point(5.0f));

        vis_geo->addDrawable(vis_geometry);
        content_root_->addChild(vis_geo);
    }

    if (!hard_index_.empty())
    {
        osg::ref_ptr<osg::Geode> hc_geo(new osg::Geode);
        osg::ref_ptr<osg::Geometry> hc_geometry = new osg::Geometry;
        osg::ref_ptr<osg::Vec3Array> hc_vetices = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec4Array> hc_colors = new osg::Vec4Array;
        hc_colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 0.0f));

        for (size_t i = 0, i_end = hard_index_.size(); i < i_end; ++ i)
        {
            hc_vetices->push_back(vertices_->at(hard_index_[i]));
        }

        hc_geometry->setUseDisplayList(true);
        hc_geometry->setVertexArray(hc_vetices);
        hc_geometry->setColorArray(hc_colors);
        hc_colors->setBinding(osg::Array::BIND_OVERALL);
        hc_geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, hc_vetices->size()));
        hc_geometry->getOrCreateStateSet()->setAttribute(new osg::Point(5.0f));

        hc_geo->addDrawable(hc_geometry);
        content_root_->addChild(hc_geo);
    }

    return;
}

void MeshModel::updateImpl()
{
    visualizeMesh();
    return;
}

bool MeshModel::save(const std::string& filename)
{
    ObjWriter obj_writer(this);
    return obj_writer.save(filename);
}

bool MeshModel::load(const std::string& filename)
{
    if (!boost::filesystem::exists(filename))
        return false;

    std::string extension = boost::filesystem::path(filename).extension().string();
    if (extension != ".obj")
        return false;

    return readObjFile(filename);
}

bool MeshModel::readObjFile(const std::string& filename)
{
    QString obj_file(filename.c_str());
    obj_file_ = obj_file.toStdString();

    QString mtl_file = obj_file.replace(obj_file.indexOf("obj"), 3, "mtl");
    mtl_file_ = mtl_file.toStdString();

    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string err = tinyobj::LoadObj(shapes, materials, obj_file_.c_str());

    if (!err.empty()) {
      std::cerr << err << std::endl;
      return false;
    }

    // only one shape needed
    if (shapes.size() == 0 || shapes.size() > 1)
        return false;

    tinyobj::shape_t mesh_shape = shapes.at(0);
    tinyobj::mesh_t mesh_model = mesh_shape.mesh;

    size_t pos_size = mesh_model.positions.size();
    size_t tex_size = mesh_model.texcoords.size();
    size_t nl_size = mesh_model.normals.size();
    size_t face_size = mesh_model.indices.size();
    assert(pos_size % 3 == 0);
    assert(tex_size % 2 == 0);
    assert(nl_size % 3 == 0);
    assert(face_size % 3 == 0);

    pos_size = pos_size / 3;
    tex_size = tex_size / 2;
    nl_size = nl_size / 3;
    face_size = face_size / 3;

    for (size_t i = 0; i < pos_size; i ++)
    {
        vertices_->push_back(osg::Vec3(mesh_model.positions[i*3], 
            mesh_model.positions[i*3+1], mesh_model.positions[i*3+2]));
    }

    for (size_t i = 0; i < tex_size; i ++)
    {
        texcoords_->push_back(osg::Vec2(mesh_model.texcoords[i*2], 
            mesh_model.texcoords[i*2+1]));
    }

    for (size_t i = 0; i < nl_size; i ++)
    {
        vertex_normals_->push_back(osg::Vec3(mesh_model.normals[i*3], 
            mesh_model.normals[i*3+1], mesh_model.normals[i*3+2]));
    }

    for (size_t i = 0; i < face_size; i ++)
    {
        std::vector<int> face;
        face.push_back(mesh_model.indices[i*3]);
        face.push_back(mesh_model.indices[i*3+1]);
        face.push_back(mesh_model.indices[i*3+2]);
        faces_.push_back(face);
    }

    recoverAdjList();
    extractEdgeVertices();

    return true;
}

void MeshModel::recoverAdjList()
{
    const size_t pre_ver_num = 20000;
    const size_t ver_num = vertices_->size();

    if (pre_ver_num < ver_num)
    {
        std::cerr << "vertice number exceeds pre-setting!" << std::endl;
        return;
    }

    std::array<std::set<int>, pre_ver_num > index_list;

    for (size_t i = 0, i_end = faces_.size(); i < i_end; i ++)
    {
        std::vector<int> face = faces_.at(i);
        int x = face.at(0);
        int y = face.at(1);
        int z = face.at(2);
        
        index_list[x].insert(y);
        index_list[x].insert(z);

        index_list[y].insert(x);
        index_list[y].insert(z);

        index_list[z].insert(x);
        index_list[z].insert(y);
    }

    for (size_t i = 0, i_end = ver_num; i < i_end; i ++)
    {
        std::set<int> index_i = index_list.at(i);
        std::vector<int> adj_list(index_i.begin(), index_i.end());
        std::sort(adj_list.begin(), adj_list.end());
        adj_list_.push_back(adj_list);
    }
}

void MeshModel::extractEdgeVertices()
{
    std::set<int> edge_idx;

    for (size_t i = 0, i_end = adj_list_.size(); i < i_end; ++ i)
    {
        std::vector<int> list_i = adj_list_.at(i);
        for (size_t j = 0, j_end = list_i.size(); j < j_end; ++ j)
        {
            std::vector<int> shared_vertices;
            findSharedVertices(i, list_i[j], shared_vertices);
            if (shared_vertices.size() == 1)
            {
                edge_idx.insert(i);
                edge_idx.insert(list_i[j]);
            }
        }
    }

    for (std::set<int>::iterator itr = edge_idx.begin(); itr != edge_idx.end(); ++ itr)
        edge_index_.push_back(*itr);
}

void MeshModel::findSharedVertices(int pi, int pj, std::vector<int>& shared_vertices)
{
    std::vector<int> vertices;
    set_intersection(adj_list_[pi].begin(), adj_list_[pi].end(), adj_list_[pj].begin(), adj_list_[pj].end(), back_inserter(vertices));
    for (size_t i = 0, i_end = vertices.size(); i < i_end; ++ i) 
    {
        if (vertices[i] != pi && vertices[i] != pj)
            shared_vertices.push_back(vertices[i]);
    }

    if (shared_vertices.size() > 2) {
        std::cout << "share vertices number warning: " << shared_vertices.size() << std::endl;
    }
}

void MeshModel::searchNearestIdx(PointCloud* point_cloud, std::vector<int>& knn_idx)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0, i_end = point_cloud->size(); i < i_end; ++ i)
    {
        const Point& point = point_cloud->at(i);

        pcl::PointXYZ pcl_point(point.x, point.y, point.z);
        cloud->push_back(pcl_point);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (cloud);

    int K = 1;

    // K nearest neighbor search

    for (size_t i = 0, i_end = this->getVertices()->size(); i < i_end; ++ i)
    {
        pcl::PointXYZ searchPoint;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        osg::Vec3& point = this->getVertices()->at(i);

        searchPoint.x = point.x();
        searchPoint.y = point.y();
        searchPoint.z = point.z();

        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            knn_idx.push_back(pointIdxNKNSearch[0]);
    }
}

void MeshModel::searchNearestIdx(const MeshModel& source_mesh, std::vector<int>& idx)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0, i_end = this->getVertices()->size(); i < i_end; ++ i)
    {
        const osg::Vec3& point = this->getVertices()->at(i);

        pcl::PointXYZ pcl_point(point.x(), point.y(), point.z());
        cloud->push_back(pcl_point);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (cloud);

    int K = 1;

    // K nearest neighbor search

    for (size_t i = 0, i_end = source_mesh.getVertices()->size(); i < i_end; ++ i)
    {
        pcl::PointXYZ searchPoint;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        osg::Vec3& point = source_mesh.getVertices()->at(i);
        searchPoint.x = point.x();
        searchPoint.y = point.y();
        searchPoint.z = point.z();

        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            idx.push_back(pointIdxNKNSearch[0]);
    }
}

int MeshModel::searchNearestIdx(osg::Vec3 point)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0, i_end = this->getVertices()->size(); i < i_end; ++ i)
    {
        const osg::Vec3& point = this->getVertices()->at(i);

        pcl::PointXYZ pcl_point(point.x(), point.y(), point.z());
        cloud->push_back(pcl_point);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (cloud);

    int K = 1;

    // K nearest neighbor search

    int index = -1;
    pcl::PointXYZ searchPoint;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    searchPoint.x = point.x();
    searchPoint.y = point.y();
    searchPoint.z = point.z();

    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        index = pointIdxNKNSearch[0];
    return index;
}



void MeshModel::initializeVisibility()
{
    visibility_.resize(this->getVertices()->size());
    for (size_t i = 0, i_end = vertices_->size(); i < i_end; ++ i)
        visibility_[i] = 0;

    return;
}

void MeshModel::rotate(const osg::Matrix& rot_matrix)
{
    for (size_t i = 0, i_end = vertices_->size(); i < i_end; ++ i)
    {
        osg::Vec3& point = vertices_->at(i);
        point = rot_matrix.preMult(point);
    }

    return;
}

void MeshModel::pickEvent(int pick_mode, osg::Vec3 position)
{
    switch (pick_mode)
    {
    case (osgGA::GUIEventAdapter::MODKEY_CTRL):
        {
            hard_index_.clear();

            const float radius = 10;

            for (size_t i = 0, i_end = vertices_->size(); i < i_end; ++ i)
            {
                osg::Vec3 vertex = vertices_->at(i);
                if ((position-vertex).length() < radius)
                {
                    hard_index_.push_back(i);
                }
            }
            expire();
        }
    default:
        break;
    }
}