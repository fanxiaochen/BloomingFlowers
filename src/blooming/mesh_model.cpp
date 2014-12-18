#include <random>
#include <boost/filesystem.hpp>
#include <osgDB/WriteFile>

#include <pcl/kdtree/kdtree_flann.h>

#include <Deform.h>

#include "tiny_obj_loader.h"
#include "obj_writer.h"

#include "point_cloud.h"
#include "mesh_model.h"


MeshModel::MeshModel()
    :vertices_(new osg::Vec3Array),
    colors_(new osg::Vec4Array),
    face_normals_(new osg::Vec3Array),
    smoothing_visitor_(new osgUtil::SmoothingVisitor)
{
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
      osg::ref_ptr<osg::DrawElementsUInt> face = new osg::DrawElementsUInt(GL_LINE_LOOP, vertex_num);
      for (size_t j = 0; j < vertex_num; ++ j)
        face->at(j) = faces_[i][j];

      geometry->addPrimitiveSet(face.get());
    }
  }

  geode->addDrawable(geometry);
  content_root_->addChild(geode);

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
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string err = tinyobj::LoadObj(shapes, materials, filename.c_str());

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
    size_t face_size = mesh_model.indices.size();
    assert(pos_size % 3 == 0);
    assert(face_size % 3 == 0);

    pos_size = pos_size / 3;
    face_size = face_size / 3;

    for (size_t i = 0; i < pos_size; i ++)
    {
        vertices_->push_back(osg::Vec3(mesh_model.positions[i*3], 
            mesh_model.positions[i*3+1], mesh_model.positions[i*3+2]));
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

    buildDeformModel();

    return true;
}

void MeshModel::recoverAdjList()
{
    const size_t pre_ver_num = 10000;
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
        adj_list_.push_back(adj_list);
    }

}

void MeshModel::buildDeformModel()
{
    Build_triangle<Polyhedron::HalfedgeDS> triangle(this);
//    deform_model_.delegate(triangle);
//    CGAL_assertion( deform_model_.is_triangle(deform_model_.halfedges_begin()));
    return;
}

void MeshModel::deform(const osg::Vec3Array& indicators, const std::vector<int>& index)
{
    float* p;
    int p_num = vertices_->size();
    p = (float*)malloc(sizeof(float)*p_num*3);

    for (size_t i = 0; i < p_num; i ++)
    {
        const osg::Vec3& point = vertices_->at(i);
        p[3*i+0] = point.x();
        p[3*i+1] = point.y();
        p[3*i+2] = point.z();
    }

    ////Deform::FaceList face_list;
    //for (size_t i = 0, i_end = faces_.size(); i < i_end; i ++)
    //{
    //    std::vector<int> face = faces_.at(i);
    //    std::sort(face.begin(), face.end());

    //    /*Eigen::Vector3i face_i;
    //    face_i(0) = face.at(0);
    //    face_i(1) = face.at(1);
    //    face_i(2) = face.at(2);

    //    face_list.push_back(face_i);*/
    //}

    Deform::VectorF points_indicator;
    Deform::VectorI points_index;

    for (size_t i = 0, i_end = indicators.size(); i < i_end; i ++)
    {
        osg::Vec3 indicator = indicators.at(i);
        points_indicator.push_back(indicator.x());
        points_indicator.push_back(indicator.y());
        points_indicator.push_back(indicator.z()); 
        /*points_indicator.push_back(12);
        points_indicator.push_back(6);
        points_indicator.push_back(900); */
    }

    for (size_t i = 0, i_end = index.size(); i < i_end; i ++)
    {
        points_index.push_back(index.at(i));
    }

    float delta;
    Deform deform_model(p, p_num, adj_list_, faces_);
    deform_model.set_hard_ctrs(points_indicator, points_index);
    deform_model.do_Deform(6);
//    deform_model.do_Deform_Iter(delta);
    /*expire();
    deform_model.do_Deform_Iter(delta);
    expire();
    deform_model.do_Deform_Iter(delta);
    expire();
    deform_model.do_Deform_Iter(delta);*/

    float* new_p = deform_model.get_P_Prime();

    for (size_t i = 0; i < p_num; i ++)
    {
        osg::Vec3& point = vertices_->at(i);
        point.x() = new_p[3*i+0];
        point.y() = new_p[3*i+1];
        point.z() = new_p[3*i+2];
    }

    free(p);
}

void MeshModel::deform(const std::vector<float>& hard_ctrs, const std::vector<int>& hard_idx)
{
    float* p;
    int p_num = vertices_->size();
    p = (float*)malloc(sizeof(float)*p_num*3);

    for (size_t i = 0; i < p_num; i ++)
    {
        const osg::Vec3& point = vertices_->at(i);
        p[3*i+0] = point.x();
        p[3*i+1] = point.y();
        p[3*i+2] = point.z();
    }

    float delta;
    Deform deform_model(p, p_num, adj_list_, faces_);
    deform_model.set_hard_ctrs(hard_ctrs, hard_idx);
    deform_model.do_Deform(6);

    float* new_p = deform_model.get_P_Prime();

    for (size_t i = 0; i < p_num; i ++)
    {
        osg::Vec3& point = vertices_->at(i);
        point.x() = new_p[3*i+0];
        point.y() = new_p[3*i+1];
        point.z() = new_p[3*i+2];
    }

    free(p);
}

void MeshModel::deform(const std::vector<float>& hard_ctrs, const std::vector<int>& hard_idx,
            const std::vector<float>& soft_ctrs, const std::vector<int>& soft_idx)
{
    float* p;
    int p_num = vertices_->size();
    p = (float*)malloc(sizeof(float)*p_num*3);

    for (size_t i = 0; i < p_num; i ++)
    {
        const osg::Vec3& point = vertices_->at(i);
        p[3*i+0] = point.x();
        p[3*i+1] = point.y();
        p[3*i+2] = point.z();
    }

    float delta;
    Deform deform_model(p, p_num, adj_list_, faces_);
    deform_model.set_arap_type(Deform::HARD_SOFT);
    deform_model.set_hard_ctrs(hard_ctrs, hard_idx);
    deform_model.set_soft_ctrs(soft_ctrs, soft_idx);
    deform_model.set_lambda(1);
    deform_model.do_Deform(6);

    float* new_p = deform_model.get_P_Prime();

    for (size_t i = 0; i < p_num; i ++)
    {
        osg::Vec3& point = vertices_->at(i);
        point.x() = new_p[3*i+0];
        point.y() = new_p[3*i+1];
        point.z() = new_p[3*i+2];
    }

    free(p);
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

//void MeshModel::deform(const osg::Vec3Array& indicators, const std::vector<int>& index)
//{
//
//    // Init the indices of the halfedges and the vertices.
//    set_halfedgeds_items_id(deform_model_);
//    // Create a deformation object
//    Surface_mesh_deformation deform_mesh(deform_model_);
//    // Definition of the region of interest (use the whole mesh)
//    vertex_iterator vb,ve;
//    boost::tie(vb, ve) = vertices(deform_model_);
//    deform_mesh.insert_roi_vertices(vb, ve);
//    // Select control vertices ...and insert them
//    std::vector<vertex_descriptor> control_vertices;
//    for (size_t i = 0, i_end = index.size(); i < i_end; i ++)
//    {
//        control_vertices.push_back(*CGAL::cpp11::next(vb, index[i]));
//        deform_mesh.insert_control_vertex(control_vertices.at(i));
//    }
//    // The definition of the ROI and the control vertices is done, call preprocess
//    bool is_matrix_factorization_OK = deform_mesh.preprocess();
//    if(!is_matrix_factorization_OK){
//        std::cerr << "Error in preprocessing, check documentation of preprocess()" << std::endl;
//        return;
//    }
//    // Use set_target_position() to set the constained position
//    for (size_t i = 0, i_end = indicators.size(); i < i_end; i ++)
//    {
//        const osg::Vec3& indicator = indicators.at(i);
//        Surface_mesh_deformation::Point constrained_pos(indicator.x(), indicator.y(), indicator.z());
//        deform_mesh.set_target_position(control_vertices[i], constrained_pos);
//    }
//    
//    // Deform the mesh, the positions of vertices of 'mesh' are updated
//    deform_mesh.deform();
//    
//    for (Polyhedron::Vertex_iterator vb = deform_model_.vertices_begin(), ve = deform_model_.vertices_end();
//    vb != ve; vb ++)
//    {
//        osg::Vec3& point = vertices_->at(vb->id());
//        point.x() = vb->point().x();
//        point.y() = vb->point().y();
//        point.z() = vb->point().z();
//    }
//}


