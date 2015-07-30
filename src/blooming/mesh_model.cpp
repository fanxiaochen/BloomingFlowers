#include <random>
#include <fstream>

#include <boost/filesystem.hpp>

#include <osgDB/WriteFile>
#include <osg/Point>
#include <osg/Material>
#include <osg/Image>
#include <osgDB/ReadFile>
#include <osg/Texture2D>

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
    color_id_(0),
    skeleton_(new Skeleton),
    has_texture_(false),
    show_texture_(false),
    show_skeleton_(false)
    /*face_normals_(new osg::Vec3Array),*/
{
}

// a class is a natural friend class itself. In the class domain,
// the private member could be accessed directly.
MeshModel::MeshModel(const MeshModel& mesh_model) // deep copy
    :vertices_(new osg::Vec3Array),
    texcoords_(new osg::Vec2Array),
    vertex_normals_(new osg::Vec3Array),
    colors_(new osg::Vec4Array),
    color_id_(0),
    skeleton_(new Skeleton),
    has_texture_(false),
    show_texture_(false),
    show_skeleton_(false)
    /*face_normals_(new osg::Vec3Array),*/
{
    obj_name_ = mesh_model.obj_name_;
    obj_file_ = mesh_model.obj_file_;
    mtl_file_ = mesh_model.mtl_file_;
    *vertices_ = *mesh_model.vertices_;
    *texcoords_ = *mesh_model.texcoords_;
    *vertex_normals_ = *mesh_model.vertex_normals_;
    faces_ = mesh_model.faces_;
    ambient_ = mesh_model.ambient_;
    diffuse_ = mesh_model.diffuse_;
    specular_ = mesh_model.specular_;
    emission_ = mesh_model.emission_;
    map_Ka_ = mesh_model.map_Ka_;
    map_Kd_ = mesh_model.map_Kd_;
    adj_list_ = mesh_model.adj_list_;
    weights_ = mesh_model.weights_;
    edge_index_ = mesh_model.edge_index_;
    hard_index_ = mesh_model.hard_index_;
    color_id_ = mesh_model.color_id_;
    *skeleton_ = *mesh_model.skeleton_;
    biharmonic_weights_ = mesh_model.biharmonic_weights_;
    has_texture_ = mesh_model.has_texture_;
    show_texture_ = show_texture_;
    show_skeleton_ = show_skeleton_;
}

MeshModel::~MeshModel(void)
{
}

MeshModel& MeshModel::operator =(const MeshModel& mesh_model) // deep copy
{
    obj_name_ = mesh_model.obj_name_;
    obj_file_ = mesh_model.obj_file_;
    mtl_file_ = mesh_model.mtl_file_;
    *vertices_ = *mesh_model.vertices_;
    *texcoords_ = *mesh_model.texcoords_;
    *vertex_normals_ = *mesh_model.vertex_normals_;
    faces_ = mesh_model.faces_;
    ambient_ = mesh_model.ambient_;
    diffuse_ = mesh_model.diffuse_;
    specular_ = mesh_model.specular_;
    emission_ = mesh_model.emission_;
    map_Ka_ = mesh_model.map_Ka_;
    map_Kd_ = mesh_model.map_Kd_;
    adj_list_ = mesh_model.adj_list_;
    weights_ = mesh_model.weights_;
    edge_index_ = mesh_model.edge_index_;
    hard_index_ = mesh_model.hard_index_;
    color_id_ = mesh_model.color_id_;
    *skeleton_ = *mesh_model.skeleton_;
    biharmonic_weights_ = mesh_model.biharmonic_weights_;
    has_texture_ = mesh_model.has_texture_;
    show_texture_ = show_texture_;
    show_skeleton_ = show_skeleton_;

    return *this;
}

void MeshModel::visualizeMesh(void)
{
    osg::ref_ptr<osg::Geode> geode(new osg::Geode);
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
    geometry->setUseDisplayList(true);
    geometry->setVertexArray(vertices_);
    

    if (has_texture_ && show_texture_)
    {
        geometry->setTexCoordArray(0, texcoords_);
        osg::StateSet* stateset = this->getOrCreateStateSet();

        osg::Material* material = new osg::Material;
        material->setColorMode(osg::Material::OFF); 
        material->setAmbient(osg::Material::FRONT_AND_BACK, ambient_);
        material->setDiffuse(osg::Material::FRONT_AND_BACK, diffuse_);
        material->setSpecular(osg::Material::FRONT_AND_BACK, specular_);
        material->setEmission(osg::Material::FRONT_AND_BACK, emission_);
        stateset->setAttributeAndModes(material,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);

        osg::Image *image = osgDB::readImageFile(map_Kd_);
        if (!image)
        {
            std::cout << "couldn't find texture." << std::endl;
        }

        osg::Texture2D *texture = new osg::Texture2D;
        texture->setDataVariance(Object::DYNAMIC);
        texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
        texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP);
        texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP);
        texture->setImage(image);
        stateset->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
    }
    else
    {
        colors_->push_back(ColorMap::getInstance().getDiscreteColor(color_id_));
        geometry->setColorArray(colors_);
        colors_->setBinding(osg::Array::BIND_OVERALL);
    }

    /*if (!texcoords_->empty() && !map_Ka_.empty() && !map_Kd_.empty())
    {
    geometry->setTexCoordArray(0, texcoords_);
    osg::StateSet* stateset = this->getOrCreateStateSet();

    osg::Material* material = new osg::Material;
    material->setColorMode(osg::Material::OFF); 
    material->setAmbient(osg::Material::FRONT_AND_BACK, ambient_);
    material->setDiffuse(osg::Material::FRONT_AND_BACK, diffuse_);
    material->setSpecular(osg::Material::FRONT_AND_BACK, specular_);
    material->setEmission(osg::Material::FRONT_AND_BACK, emission_);
    stateset->setAttributeAndModes(material,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);

    osg::Image *image = osgDB::readImageFile(map_Kd_);
    if (!image)
    {
    std::cout << "couldn't find texture." << std::endl;
    }

    osg::Texture2D *texture = new osg::Texture2D;
    texture->setDataVariance(Object::DYNAMIC);
    texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP);
    texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP);
    texture->setImage(image);
    stateset->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
    }*/

    if (faces_.empty())
    {
        geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices_->size()));
    }
    else
    {
        if (!vertex_normals_->empty())
        {
            geometry->setNormalArray(vertex_normals_);
            vertex_normals_->setBinding(osg::Array::BIND_PER_VERTEX);
        }

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
    


    /*if (!visibility_.empty())
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
    }*/

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

bool MeshModel::save(const std::string& filename, bool tex_flag)
{
    ObjWriter obj_writer(this);

    bool flag = obj_writer.save(filename, has_texture_);

    if (flag)
    {
        std::string original_skel_file = this->getObjFile() + ".skel";
        if (QFile(original_skel_file.c_str()).exists())
        {
            std::string new_skel_file = filename + ".skel";
            skeleton_->save(new_skel_file);
        }
    }

    return flag;
}

bool MeshModel::load(const std::string& filename)
{
    if (!boost::filesystem::exists(filename))
        return false;

    std::string extension = boost::filesystem::path(filename).extension().string();
    if (extension != ".obj")
        return false;

    bool flag = readObjFile(filename);

    if (flag)
    {
        std::string skel_file = filename + ".skel";
        if (QFile(skel_file.c_str()).exists())
            skeleton_->load(skel_file);

        std::string bw_file = filename + ".bw";
        if (QFile(bw_file.c_str()).exists())
            loadBiharmonicWeights(bw_file);
    }

    return flag;
}

bool MeshModel::loadBiharmonicWeights(const std::string& filename)
{
    if (skeleton_->isEmpty()) 
    {
        std::cout << "no biharmonic weights file founded!!!!" << std::endl;
        return false;
    }

    int cols = skeleton_->getJointNumber();
    int rows = vertices_->size();

    biharmonic_weights_.resize(rows, cols);

    std::ifstream infile;
    infile.open( filename.c_str());

    std::string line;
    int row_cnt = 0;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        Eigen::VectorXd vec(cols);
        for (int i = 0; i < cols; ++ i)
        {
            if (!(iss >> vec[i]))
                break;
        }

        biharmonic_weights_.row(row_cnt++) = vec;
    }

    assert(biharmonic_weights_.rows() == rows && biharmonic_weights_.cols() == cols);

    return true;
}

bool MeshModel::readObjFile(const std::string& filename)
{
    QString obj_file(filename.c_str());
    obj_file_ = obj_file.toStdString();

    QString mtl_file = obj_file;
    mtl_file = mtl_file.replace(mtl_file.indexOf("obj"), 3, "mtl");
    mtl_file_ = mtl_file.toStdString();
    mtl_file.resize(mtl_file.lastIndexOf('/')+1);
    QString mtl_base(mtl_file);

    obj_name_ = obj_file.replace(mtl_base, "").toStdString();

    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string err = tinyobj::LoadObj(shapes, materials, obj_file_.c_str(), mtl_base.toStdString().c_str());

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

    // only one material needed
    if (materials.size() == 1)
    {
        has_texture_ = true;

        tinyobj::material_t mesh_material = materials.at(0);
        ambient_ = osg::Vec4(mesh_material.ambient[0], mesh_material.ambient[1], mesh_material.ambient[2], 1.0f);
        diffuse_ = osg::Vec4(mesh_material.diffuse[0], mesh_material.diffuse[1], mesh_material.diffuse[2], 1.0f);
        specular_ = osg::Vec4(mesh_material.specular[0], mesh_material.specular[1], mesh_material.specular[2], 1.0f);
        emission_ = osg::Vec4(mesh_material.emission[0], mesh_material.emission[1], mesh_material.emission[2], 1.0f);
        map_Ka_ = mtl_base.toStdString() + mesh_material.ambient_texname;
        map_Kd_ = mtl_base.toStdString() + mesh_material.diffuse_texname;
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

void MeshModel::updateNormals()
{
    if (vertex_normals_->empty())
        return;

    for (size_t i = 0, i_end = faces_.size(); i < i_end; ++ i)
    {
        const std::vector<int>& face = faces_.at(i);
        const osg::Vec3& v1 = vertices_->at(face[0]);
        const osg::Vec3& v2 = vertices_->at(face[1]);
        const osg::Vec3& v3 = vertices_->at(face[2]);

        osg::Vec3 face_normal = ((v2-v1)^(v3-v2));
        face_normal.normalize();
        vertex_normals_->at(face[0]) = face_normal;
        vertex_normals_->at(face[1]) = face_normal;
        vertex_normals_->at(face[2]) = face_normal;
    }
}

void MeshModel::determineWeights(PointCloud* aligned_cloud, int petal_id)
{
    weights_.clear();

    PointCloud segmented_cloud;
    for (size_t k = 0, k_end = aligned_cloud->size(); k < k_end; ++ k)
    {
        if (aligned_cloud->getSegmentFlags()[k] != petal_id)
            continue;;
        segmented_cloud.push_back(Point(aligned_cloud->at(k)));
    }

    int cloud_nums = segmented_cloud.size();

    for (size_t i = 0, i_end = vertices_->size(); i < i_end; ++ i)
    {
        const osg::Vec3& v = vertices_->at(i);
        double s_x = 0, s_y = 0, s_z = 0;
        int adj_size = adj_list_[i].size();

        for (size_t j = 0, j_end = adj_size; j < j_end; ++ j)
        {
            const osg::Vec3& c = vertices_->at(adj_list_[i][j]);
            s_x += abs(v[0] - c[0]);
            s_y += abs(v[1] - c[1]);
            s_z += abs(v[2] - c[2]);
        }

        s_x = pow(s_x / adj_size, 2);
        s_y = pow(s_y / adj_size, 2);
        s_z = pow(s_z / adj_size, 2);

        int point_nums = 0;
        for (size_t k = 0, k_end = segmented_cloud.size(); k < k_end; ++ k)
        {
            const Point& p = segmented_cloud.at(k);
            if ((v[0]-p.x)*(v[0]-p.x)/s_x + (v[1]-p.y)*(v[1]-p.y)/s_y + (v[2]-p.z)*(v[2]-p.z)/s_z <= 1)
                point_nums ++;
        }

        weights_.push_back(double(point_nums)/cloud_nums);
    }
}