
#ifndef MESH_MODEL_H
#define MESH_MODEL_H

#define CGAL_EIGEN3_ENABLED

#include <osgUtil/SmoothingVisitor>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
// HalfedgeGraph adapters for Polyhedron_3
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/boost/graph/properties_Polyhedron_3.h>
#include <CGAL/Surface_mesh_deformation.h>

#include "renderable.h"

typedef CGAL::Simple_cartesian<double>                                   Kernel;
typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3> Polyhedron;
typedef boost::graph_traits<Polyhedron>::vertex_descriptor    vertex_descriptor;
typedef boost::graph_traits<Polyhedron>::vertex_iterator        vertex_iterator;
typedef CGAL::Surface_mesh_deformation<Polyhedron> Surface_mesh_deformation;

class PointCloud;

class MeshModel : public Renderable
{
public:
	MeshModel();
    MeshModel(const MeshModel& mesh_model);  // deep copy
	virtual ~MeshModel(void);

	bool load(const std::string& filename);
	bool save(const std::string& filename);
  
	bool empty(void) const {return vertices_->empty();}

    // for deep copy
    inline const osg::ref_ptr<osg::Vec3Array> getVertices() const { return vertices_; }
    inline const std::vector<std::vector<int> >& getFaces() const { return faces_; }
    inline const std::vector<std::vector<int> >& getAdjList() const { return adj_list_; }
    inline const Polyhedron& getDeformModel() const { return deform_model_; }

    inline osg::ref_ptr<osg::Vec3Array> getVertices() { return vertices_; }
    inline std::vector<std::vector<int> >& getFaces() { return faces_; }
    inline std::vector<std::vector<int> >& getAdjList() { return adj_list_; }
    inline Polyhedron& getDeformModel() { return deform_model_; }

    void deform(const osg::Vec3Array& hard_ctrs, const std::vector<int>& hard_idx);
    void deform(const osg::Vec3Array& hard_ctrs, const std::vector<int>& hard_idx,
        const osg::Vec3Array& soft_ctrs, const std::vector<int>& soft_idx);

    void deform(const std::vector<float>& hard_ctrs, const std::vector<int>& hard_idx);
    void deform(const std::vector<float>& hard_ctrs, const std::vector<int>& hard_idx,
        const std::vector<float>& soft_ctrs, const std::vector<int>& soft_idx);

    // source is the source_mesh, target is this mesh, knn_idx is based on source_mesh
    void searchNearestIdx(const MeshModel& source_mesh, std::vector<int>& knn_idx);

    // source is this mesh, target is the point_cloud, knn_idx is based on point_cloud
    void searchNearestIdx(PointCloud* point_cloud, std::vector<int>& knn_idx);

protected:
	virtual void updateImpl(void);
    virtual void visualizeMesh(void);

private:
	bool readObjFile(const std::string& filename);
    void recoverAdjList();
    void buildDeformModel();

protected:
	osg::ref_ptr<osg::Vec3Array>        vertices_;
	osg::ref_ptr<osg::Vec4Array>        colors_;
	std::vector<std::vector<int> >      faces_;
	osg::ref_ptr<osg::Vec3Array>        face_normals_;
    std::vector<std::vector<int> >      adj_list_;

    osg::ref_ptr<osgUtil::SmoothingVisitor>     smoothing_visitor_;

    Polyhedron                          deform_model_;
};

// A modifier creating a triangle with the incremental builder.
template <class HDS>
class Build_triangle : public CGAL::Modifier_base<HDS> {
public:
    Build_triangle() {}
    Build_triangle(MeshModel* mesh_model) { mesh_model_ = mesh_model; }
    void operator()( HDS& hds) {
        // Postcondition: hds is a valid polyhedral surface.
        CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
        B.begin_surface(mesh_model_->getVertices()->size(), mesh_model_->getFaces().size());
        typedef typename HDS::Vertex   Vertex;
        typedef typename Vertex::Point Point;

        for (size_t i = 0, i_end = mesh_model_->getVertices()->size(); i < i_end; i ++)
        {
            const osg::Vec3& point = mesh_model_->getVertices()->at(i);
            B.add_vertex(Point(point.x(), point.y(), point.z()));
        }


        for (size_t i = 0, i_end = mesh_model_->getFaces().size(); i < i_end; i ++)
        {
            const std::vector<int> face = mesh_model_->getFaces().at(i);
            B.begin_facet();
            B.add_vertex_to_facet(face[0]);
            B.add_vertex_to_facet(face[1]);
            B.add_vertex_to_facet(face[2]);
            B.end_facet();
        }

        B.end_surface();
    }

private:
    MeshModel* mesh_model_;
};

#endif // MESH_MODEL_H