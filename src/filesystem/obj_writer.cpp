
#include <fstream>

#include "mesh_model.h"
#include "obj_writer.h"

ObjWriter::ObjWriter(MeshModel* mesh_model)
    :mesh_model_(mesh_model)
{

}

ObjWriter::~ObjWriter()
{

}

bool ObjWriter::save(const std::string& obj_file)
{
    std::fstream fs(obj_file, std::ios_base::out);
    
    osg::ref_ptr<osg::Vec3Array> vertices = mesh_model_->getVertices();
    osg::ref_ptr<osg::Vec2Array> texcoords = mesh_model_->getTexcoords();
    osg::ref_ptr<osg::Vec3Array> vertex_normals = mesh_model_->getVertexNormals();
    std::vector<std::vector<int> > faces = mesh_model_->getFaces();

    fs << "# Obj File Created by Xiaochen Fan" << "\n" << "\n";

//    fs << QString("g %1").arg(mesh_model_->getObjName().c_str()).toStdString() << "\n";
//    fs << "usemtl default" << "\n";
//    fs << "s 1" << "\n";

    fs << "\n\n";

//    fs << "mtllib petal.mtl" << "\n";

    fs << "\n\n";

    for (size_t i = 0, i_end = vertices->size(); i < i_end; ++ i)
    {
        osg::Vec3& vertice = vertices->at(i);
        fs << "v " << vertice.x() << " " << vertice.y() << " " << vertice.z() << "\n";
    }

    fs << "\n\n";

    for (size_t i = 0, i_end = texcoords->size(); i < i_end; ++ i)
    {
        osg::Vec2& texcoord = texcoords->at(i);
        fs << "vt " << texcoord.x() << " " << texcoord.y() << " " << "0.0" << "\n";
    }

    fs << "\n\n";

    for (size_t i = 0, i_end = vertex_normals->size(); i < i_end; ++ i)
    {
        osg::Vec3& vertex_normal = vertex_normals->at(i);
        fs << "vn " << vertex_normal.x() << " " << vertex_normal.y() << " " << vertex_normal.z() << "\n";
    }

    fs << "\n\n";

    // default: 3 vertices a face, since in the tiny_obj_loader, the index of
    // face starts from zero, I have to add one to it
    for (size_t i = 0, i_end = faces.size(); i < i_end; ++ i)
    {
        std::vector<int>& face = faces.at(i);
        if (face.size() != 3)
            return false;

        fs << "f " << face[0]+1 << "/" << face[0]+1 << "/" << face[0]+1;
        fs << " " << face[1]+1 << "/" << face[1]+1 << "/" << face[1]+1;
        fs << " " << face[2]+1 << "/" << face[2]+1 << "/" << face[2]+1 << "\n";

    }

    fs.close();

    return true;

}