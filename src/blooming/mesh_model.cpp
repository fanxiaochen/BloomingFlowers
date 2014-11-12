#include <random>
#include <boost/filesystem.hpp>
#include <osgDB/WriteFile>

#include <Deform.h>

#include "tiny_obj_loader.h"

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

void MeshModel::updateImpl(void)
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

	return;
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

	return true;
}

void MeshModel::recoverAdjList()
{

}

void MeshModel::deform()
{
    Eigen::Matrix3Xf p;
    int p_num = vertices_->size();
    p.resize(3, p_num);

    for (size_t i = 0; i < p_num; i ++)
    {
        const osg::Vec3& point = vertices_->at(i);
        p(0, i) = point.x();
        p(1, i) = point.y();
        p(2, i) = point.z();
    }

    Deform::FaceList face_list;
    for (size_t i = 0, i_end = faces_.size(); i < i_end; i ++)
    {
        std::vector<int> face = faces_.at(i);
        Eigen::Vector3i face_i;
        face_i(0) = face.at(0);
        face_i(1) = face.at(1);
        face_i(2) = face.at(2);

        face_list.push_back(face_i);
    }

    Deform deform_model(p, p_num, adj_list_, face_list);
    
}


