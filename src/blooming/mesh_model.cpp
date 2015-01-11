#include <random>
#include <boost/filesystem.hpp>
#include <osgDB/WriteFile>

#include <pcl/kdtree/kdtree_flann.h>

#include <Deform.h>

#include "tiny_obj_loader.h"
#include "obj_writer.h"
#include "mesh_simplify.h"

#include "point_cloud.h"
#include "mesh_model.h"


MeshModel::MeshModel()
    :vertices_(new osg::Vec3Array),
    texcoords_(new osg::Vec2Array),
    vertex_normals_(new osg::Vec3Array),
    colors_(new osg::Vec4Array),
    /*face_normals_(new osg::Vec3Array),*/
    smoothing_visitor_(new osgUtil::SmoothingVisitor)
    //hard_ctrs_(new osg::Vec3Array)
{
}

MeshModel::MeshModel(const MeshModel& mesh_model) // deep copy
    :vertices_(new osg::Vec3Array),
    texcoords_(new osg::Vec2Array),
    vertex_normals_(new osg::Vec3Array),
    colors_(new osg::Vec4Array),
    /*face_normals_(new osg::Vec3Array),*/
    smoothing_visitor_(new osgUtil::SmoothingVisitor)
    //hard_ctrs_(new osg::Vec3Array)
{
    *(this->getVertices()) = *(mesh_model.getVertices());
    *(this->getTexcoords()) = *(mesh_model.getTexcoords());
    *(this->getVertexNormals()) = *(mesh_model.getVertexNormals());
    *(this->getVertices()) = *(mesh_model.getVertices());
    this->getFaces() = mesh_model.getFaces();
    this->getAdjList() = mesh_model.getAdjList();
    //this->getDeformModel() = mesh_model.getDeformModel();
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

//    buildDeformModel();

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

//void MeshModel::buildDeformModel()
//{
//    Build_triangle<Polyhedron::HalfedgeDS> triangle(this);
//    deform_model_.delegate(triangle);
////    CGAL_assertion( deform_model_.is_triangle(deform_model_.halfedges_begin()));
//    return;
//}

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

//MeshModel MeshModel::simplify(int scale)
//{
//    MeshModel sim_mesh(*this);
//
//    Simplify::vertices.clear();
//    Simplify::triangles.clear();
//
//    for (size_t i = 0, i_end = sim_mesh.getVertices()->size(); i < i_end; ++ i)
//    {
//        Simplify::Vertex v;
//        const osg::Vec3& vertice = sim_mesh.getVertices()->at(i);
//        v.p.x = vertice.x();
//        v.p.y = vertice.y();
//        v.p.z = vertice.z();
//        Simplify::vertices.push_back(v);
//    }
//
//    for (size_t i = 0, i_end = sim_mesh.getFaces().size(); i < i_end; ++ i)
//    {
//        Simplify::Triangle t;
//        const std::vector<int>& face = sim_mesh.getFaces().at(i);
//        t.v[0] = face[0];
//        t.v[1] = face[1];
//        t.v[2] = face[2];
//        Simplify::triangles.push_back(t);
//    }
//
//    Simplify::simplify_mesh(sim_mesh.getFaces().size() / scale);
//
//    sim_mesh.getVertices()->clear();
//    sim_mesh.getFaces().clear();
//
//    for (size_t i = 0, i_end = Simplify::vertices.size(); i < i_end; ++ i)
//    {
//        osg::Vec3 vertice;
//        const Simplify::Vertex& v = Simplify::vertices.at(i);
//        vertice.x() = v.p.x;
//        vertice.y() = v.p.y;
//        vertice.z() = v.p.z;
//        sim_mesh.getVertices()->push_back(vertice);
//    }
//
//    for (size_t i = 0, i_end = Simplify::triangles.size(); i < i_end; ++ i)
//    {
//        std::vector<int> face;
//        const Simplify::Triangle& t = Simplify::triangles.at(i);
//        face.push_back(t.v[0]);
//        face.push_back(t.v[1]);
//        face.push_back(t.v[2]);
//
//        sim_mesh.getFaces().push_back(face);
//    }
//
//    return sim_mesh;
//}

//// remove duplicated elements
//void MeshModel::buildHardCtrsIdx()
//{
//    std::set<int> hard_idx_set;
//    for (size_t i = 0, i_end = hard_index_.size(); i < i_end; ++ i)
//    {
//        hard_idx_set.insert(hard_index_[i]);
//    }
//
//    hard_index_.clear();
//    std::copy(hard_idx_set.begin(), hard_idx_set.end(), back_inserter(hard_index_));
//
//}
//
//void MeshModel::buildHardCtrsIdx(int scale)
//{
//    hard_index_.clear();
//
//    std::vector<int> hard_idx_vec;
//    std::set<int> hard_idx_set; // for unique index 
//
//    MeshModel sim_mesh = simplify(scale);
//    searchNearestIdx(sim_mesh, hard_idx_vec);
//
//    for (size_t i = 0, i_end = hard_idx_vec.size(); i < i_end; ++ i)
//    {
//        if (visibility_.size() == 0)
//            hard_idx_set.insert(hard_idx_vec[i]);
//        else
//        {
//            if (visibility_[hard_idx_vec[i]] == 1)
//                hard_idx_set.insert(hard_idx_vec[i]);
//        }
//    }
//
//    for (size_t i = 0, i_end = edge_index_.size(); i < i_end; ++ i)
//    {
//        if (visibility_.size() == 0)
//            hard_idx_set.insert(edge_index_[i]);
//        else
//        {
//            if (visibility_[hard_idx_vec[i]] == 1)
//                hard_idx_set.insert(edge_index_[i]);
//        }
//    }
//
//    std::copy(hard_idx_set.begin(), hard_idx_set.end(), back_inserter(hard_index_));
//}

//void MeshModel::deform(const osg::Vec3Array& hard_ctrs, const std::vector<int>& hard_idx)
//{
//    float* p;
//    int p_num = vertices_->size();
//    p = (float*)malloc(sizeof(float)*p_num*3);
//
//    for (size_t i = 0; i < p_num; i ++)
//    {
//        const osg::Vec3& point = vertices_->at(i);
//        p[3*i+0] = point.x();
//        p[3*i+1] = point.y();
//        p[3*i+2] = point.z();
//    }
//
//    Deform::VectorF points_indicator;
//    Deform::VectorI points_index;
//
//    for (size_t i = 0, i_end = hard_ctrs.size(); i < i_end; i ++)
//    {
//        osg::Vec3 indicator = hard_ctrs.at(i);
//        points_indicator.push_back(indicator.x());
//        points_indicator.push_back(indicator.y());
//        points_indicator.push_back(indicator.z()); 
//    }
//
//    for (size_t i = 0, i_end = hard_idx.size(); i < i_end; i ++)
//    {
//        points_index.push_back(hard_idx.at(i));
//    }
//
//    Deform deform_model(p, p_num, adj_list_, faces_);
//    deform_model.set_hard_ctrs(points_indicator, points_index);
//    deform_model.do_Deform(1);
//
//
//    float* new_p = deform_model.get_P_Prime();
//
//    for (size_t i = 0; i < p_num; i ++)
//    {
//        osg::Vec3& point = vertices_->at(i);
//        point.x() = new_p[3*i+0];
//        point.y() = new_p[3*i+1];
//        point.z() = new_p[3*i+2];
//    }
//
//    free(p);
//}

//void MeshModel::deform(const osg::Vec3Array& hard_ctrs, const std::vector<int>& hard_idx,
//            const osg::Vec3Array& soft_ctrs, const std::vector<int>& soft_idx)
//{
//    float* p;
//    int p_num = vertices_->size();
//    p = (float*)malloc(sizeof(float)*p_num*3);
//
//    for (size_t i = 0; i < p_num; i ++)
//    {
//        const osg::Vec3& point = vertices_->at(i);
//        p[3*i+0] = point.x();
//        p[3*i+1] = point.y();
//        p[3*i+2] = point.z();
//    }
//
//    Deform::VectorF vf_hard_ctrs;
//    Deform::VectorI vi_hard_idx;
//
//    Deform::VectorF vf_soft_ctrs;
//    Deform::VectorI vi_soft_idx;
//
//
//    for (size_t i = 0, i_end = hard_ctrs.size(); i < i_end; i ++)
//    {
//        osg::Vec3 hard_ctr = hard_ctrs.at(i);
//        vf_hard_ctrs.push_back(hard_ctr.x());
//        vf_hard_ctrs.push_back(hard_ctr.y());
//        vf_hard_ctrs.push_back(hard_ctr.z()); 
//    }
//
//    for (size_t i = 0, i_end = hard_idx.size(); i < i_end; i ++)
//    {
//        vi_hard_idx.push_back(hard_idx.at(i));
//    }
//
//    for (size_t i = 0, i_end = soft_ctrs.size(); i < i_end; i ++)
//    {
//        osg::Vec3 soft_ctr = soft_ctrs.at(i);
//        vf_soft_ctrs.push_back(soft_ctr.x());
//        vf_soft_ctrs.push_back(soft_ctr.y());
//        vf_soft_ctrs.push_back(soft_ctr.z()); 
//    }
//
//    for (size_t i = 0, i_end = soft_idx.size(); i < i_end; i ++)
//    {
//        vi_soft_idx.push_back(soft_idx.at(i));
//    }
//
//    Deform deform_model(p, p_num, adj_list_, faces_);
//    deform_model.set_hard_ctrs(vf_hard_ctrs, vi_hard_idx);
//    deform_model.set_soft_ctrs(vf_soft_ctrs, vi_soft_idx);
//    deform_model.set_arap_type(Deform::HARD_SOFT);
//    deform_model.set_lambda(2);
//    deform_model.do_Deform(1);
//
//
//    float* new_p = deform_model.get_P_Prime();
//
//    for (size_t i = 0; i < p_num; i ++)
//    {
//        osg::Vec3& point = vertices_->at(i);
//        point.x() = new_p[3*i+0];
//        point.y() = new_p[3*i+1];
//        point.z() = new_p[3*i+2];
//    }
//
//    free(p);
//}
//
//
//void MeshModel::deform(const std::vector<float>& hard_ctrs, const std::vector<int>& hard_idx)
//{
//    float* p;
//    int p_num = vertices_->size();
//    p = (float*)malloc(sizeof(float)*p_num*3);
//
//    for (size_t i = 0; i < p_num; i ++)
//    {
//        const osg::Vec3& point = vertices_->at(i);
//        p[3*i+0] = point.x();
//        p[3*i+1] = point.y();
//        p[3*i+2] = point.z();
//    }
//
//    Deform deform_model(p, p_num, adj_list_, faces_);
//    deform_model.set_hard_ctrs(hard_ctrs, hard_idx);
//    deform_model.do_Deform(1);
//
//    float* new_p = deform_model.get_P_Prime();
//
//    for (size_t i = 0; i < p_num; i ++)
//    {
//        osg::Vec3& point = vertices_->at(i);
//        point.x() = new_p[3*i+0];
//        point.y() = new_p[3*i+1];
//        point.z() = new_p[3*i+2];
//    }
//
//    free(p);
//}
//
//void MeshModel::deform(const std::vector<float>& hard_ctrs, const std::vector<int>& hard_idx,
//            const std::vector<float>& soft_ctrs, const std::vector<int>& soft_idx)
//{
//    float* p;
//    int p_num = vertices_->size();
//    p = (float*)malloc(sizeof(float)*p_num*3);
//
//    for (size_t i = 0; i < p_num; i ++)
//    {
//        const osg::Vec3& point = vertices_->at(i);
//        p[3*i+0] = point.x();
//        p[3*i+1] = point.y();
//        p[3*i+2] = point.z();
//    }
//
//    Deform deform_model(p, p_num, adj_list_, faces_);
//    deform_model.set_arap_type(Deform::HARD_SOFT);
//    deform_model.set_hard_ctrs(hard_ctrs, hard_idx);
//    deform_model.set_soft_ctrs(soft_ctrs, soft_idx);
//    deform_model.set_lambda(0);
//    deform_model.do_Deform(1);
//
//    float* new_p = deform_model.get_P_Prime();
//
//    for (size_t i = 0; i < p_num; i ++)
//    {
//        osg::Vec3& point = vertices_->at(i);
//        point.x() = new_p[3*i+0];
//        point.y() = new_p[3*i+1];
//        point.z() = new_p[3*i+2];
//    }
//
//    free(p);
//}
//
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
//        vb != ve; vb ++)
//    {
//        osg::Vec3& point = vertices_->at(vb->id());
//        point.x() = vb->point().x();
//        point.y() = vb->point().y();
//        point.z() = vb->point().z();
//    }
//}

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
//            hard_ctrs_->clear();
            hard_index_.clear();

            const float radius = 10;

            for (size_t i = 0, i_end = vertices_->size(); i < i_end; ++ i)
            {
                osg::Vec3 vertex = vertices_->at(i);
                if ((position-vertex).length() < radius)
                {
 //                   hard_ctrs_->push_back(vertex);
                    hard_index_.push_back(i);
                }
            }
            expire();
        }
    default:
        break;
    }
}