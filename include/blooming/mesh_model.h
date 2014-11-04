
#ifndef MESH_MODEL_H
#define MESH_MODEL_H

#include "renderable.h"

class MeshModel : public Renderable
{
public:
	MeshModel();
	virtual ~MeshModel(void);

	bool load(const std::string& filename);
	bool save(const std::string& filename);
  
	bool empty(void) const {return vertices_->empty();}

	inline osg::ref_ptr<osg::Vec3Array> getVertices() { return vertices_; }

protected:
	virtual void updateImpl(void);

private:
	bool readObjFile(const std::string& filename);

protected:
	osg::ref_ptr<osg::Vec3Array>        vertices_;
	osg::ref_ptr<osg::Vec4Array>        colors_;
	std::vector<std::vector<int> >      faces_;
	osg::ref_ptr<osg::Vec3Array>        face_normals_;
};

#endif // MESH_MODEL_H