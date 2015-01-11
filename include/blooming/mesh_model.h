
#ifndef MESH_MODEL_H
#define MESH_MODEL_H

#include <osgUtil/SmoothingVisitor>

#include "renderable.h"

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

    const std::string& getObjName() { return obj_name_; }
    const std::string& getObjFile() { return obj_file_; }
    const std::string& getMtlFile() { return mtl_file_; }

    // for deep copy
    inline const osg::ref_ptr<osg::Vec3Array> getVertices() const { return vertices_; }
    inline const osg::ref_ptr<osg::Vec2Array> getTexcoords() const { return texcoords_; }
    inline const osg::ref_ptr<osg::Vec3Array> getVertexNormals() const { return vertex_normals_; }
    inline const std::vector<std::vector<int> >& getFaces() const { return faces_; }
    inline const std::vector<std::vector<int> >& getAdjList() const { return adj_list_; }
    inline const std::vector<int>& getEdgeIndex() const { return edge_index_; }
    inline const std::vector<int>& getHardCtrsIndex() const { return hard_index_; }
    inline const std::vector<int>& getVisibility() const { return visibility_; }

    inline osg::ref_ptr<osg::Vec3Array> getVertices() { return vertices_; }
    inline osg::ref_ptr<osg::Vec2Array> getTexcoords() { return texcoords_; }
    inline osg::ref_ptr<osg::Vec3Array> getVertexNormals() { return vertex_normals_; }
    inline std::vector<std::vector<int> >& getFaces() { return faces_; }
    inline std::vector<std::vector<int> >& getAdjList() { return adj_list_; }
    inline std::vector<int>& getEdgeIndex() { return edge_index_; }
    inline std::vector<int>& getHardCtrsIndex(){ return hard_index_; }
    inline std::vector<int>& getVisibility() { return visibility_; }

    // source is this mesh, target is the point_cloud, knn_idx is based on point_cloud
    void searchNearestIdx(PointCloud* point_cloud, std::vector<int>& knn_idx);

    // source is point, target is this mesh, return index based on mesh
    int searchNearestIdx(osg::Vec3 point);

    void initializeVisibility();

    // change mesh position truly, not by MatrixTransform in the graph scene
    void rotate(const osg::Matrix& rot_matrix);

    void pickEvent(int pick_mode, osg::Vec3 position);

protected:
    virtual void updateImpl(void);
    virtual void visualizeMesh(void);

private:
    bool readObjFile(const std::string& filename);
    void recoverAdjList();

    void findSharedVertices(int pi, int pj, std::vector<int>& shared_vertices);
    void extractEdgeVertices();

    // source is the source_mesh, target is this mesh, knn_idx is based on this mesh
    void searchNearestIdx(const MeshModel& source_mesh, std::vector<int>& knn_idx);


protected:
    std::string                         obj_name_;
    std::string                         obj_file_;
    std::string                         mtl_file_;

    osg::ref_ptr<osg::Vec3Array>        vertices_;
    osg::ref_ptr<osg::Vec2Array>        texcoords_;
    osg::ref_ptr<osg::Vec3Array>        vertex_normals_;
    osg::ref_ptr<osg::Vec4Array>        colors_;
    std::vector<std::vector<int> >      faces_;
//	osg::ref_ptr<osg::Vec3Array>        face_normals_;
    std::vector<std::vector<int> >      adj_list_;

    osg::ref_ptr<osgUtil::SmoothingVisitor>     smoothing_visitor_;


    std::vector<int>                    edge_index_;
    std::vector<int>                    hard_index_; // hard constraints 
    std::vector<int>                    visibility_; // the same size of vertices
};

#endif // MESH_MODEL_H