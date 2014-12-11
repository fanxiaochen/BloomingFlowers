
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


class MeshModel : virtual public Renderable
{
public:
	MeshModel();
	virtual ~MeshModel(void);

	bool load(const std::string& filename);
	bool save(const std::string& filename);
  
	bool empty(void) const {return vertices_->empty();}

    inline osg::ref_ptr<osg::Vec3Array> getVertices(){ return vertices_; }
    inline std::vector<std::vector<int> >& getFaces(){ return faces_; }
    inline std::vector<std::vector<int> >& getAdjList(){ return adj_list_; }
    inline Polyhedron& getDeformModel(){ return deform_model_; }

    void deform(const osg::Vec3Array& indicators, const std::vector<int>& index);
 //   void deform(const osg::Vec3Array& indicators);

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