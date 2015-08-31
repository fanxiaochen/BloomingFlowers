
#include <fstream>

#include "flower.h"
#include "mesh_model.h"
#include "obj_writer.h"

ObjWriter::ObjWriter(Flower* flower)
    :flower_(flower)
{

}

ObjWriter::ObjWriter(MeshModel* mesh_model)
    :mesh_model_(mesh_model)
{

}

ObjWriter::~ObjWriter()
{

}

bool ObjWriter::save(const std::string& new_obj_file, bool tex_flag)
{

    // obj file
    std::fstream fs(new_obj_file, std::ios_base::out);
    
    osg::ref_ptr<osg::Vec3Array> vertices = mesh_model_->getVertices();
    osg::ref_ptr<osg::Vec2Array> texcoords = mesh_model_->getTexcoords();
    osg::ref_ptr<osg::Vec3Array> vertex_normals = mesh_model_->getVertexNormals();
    std::vector<std::vector<int> > faces = mesh_model_->getFaces();

    fs << "# Obj File Created by Xiaochen Fan" << "\n" << "\n";

//    fs << QString("g %1").arg(mesh_model_->getObjName().c_str()).toStdString() << "\n";
//    fs << "usemtl default" << "\n";
//    fs << "s 1" << "\n";

//    fs << "\n\n";
    if (tex_flag)
    {
        std::string obj_name = mesh_model_->getObjName();
        std::string mtl_name = obj_name + ".mtl";
        /*std::string mtl_name = QString(obj_name.c_str()).replace(
            QString(obj_name.c_str()).indexOf("obj"), 3, "mtl").toStdString();*/

        fs << QString("mtllib %1").arg(QString(mtl_name.c_str())).toStdString() << "\n";

        fs << "\n\n";
    }
    

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

        std::string f0 = std::to_string(face[0] + 1);
        std::string f1 = std::to_string(face[1] + 1);
        std::string f2 = std::to_string(face[2] + 1);

        int v_num = vertices->size();
        int vt_num = texcoords->size();
        int vn_num = vertex_normals->size();


        if (vn_num != 0)
        {
            fs << "f " << (v_num != 0 ? f0 : "") << "/" << (vt_num != 0 ? f0 : "") << "/" << (vn_num != 0 ? f0 : "");
            fs << " " << (v_num != 0 ? f1 : "") << "/" << (vt_num != 0 ? f1 : "") << "/" << (vn_num != 0 ? f1 : "");
            fs << " " << (v_num != 0 ? f2 : "") << "/" << (vt_num != 0 ? f2 : "") << "/" << (vn_num != 0 ? f2 : "") << "\n";
        }
        else
        {
            fs << "f " << (v_num != 0 ? f0 : "") << "/" << (vt_num != 0 ? f0 : "");
            fs << " " << (v_num != 0 ? f1 : "") << "/" << (vt_num != 0 ? f1 : "");
            fs << " " << (v_num != 0 ? f2 : "") << "/" << (vt_num != 0 ? f2 : "") << "\n";
        }

    }

    fs.close();

    if (tex_flag)
    {
        // mtl file
        std::string new_mtl_file = new_obj_file + ".mtl";
        /*new_mtl_file = QString(new_mtl_file.c_str()).replace(
        QString(new_mtl_file.c_str()).indexOf("obj"), 3, "mtl").toStdString();*/
        QFile::copy(QString(mesh_model_->getMtlFile().c_str()), QString(new_mtl_file.c_str()));

        // tga file
        std::string new_tga_file = new_obj_file + ".tga";
        /*new_tga_file = QString(new_tga_file.c_str()).replace(
        QString(new_tga_file.c_str()).indexOf("obj"), 3, "tga").toStdString();*/
        QFile::copy(QString(mesh_model_->getMapKd().c_str()), QString(new_tga_file.c_str()));
    }
    

    return true;

}

// mtl file is the same for all obj files, I call it 'flower.mtl'
bool ObjWriter::saveAll(const std::string& new_obj_file, bool tex_flag /* = false */)
{

    // obj file
    std::fstream fs(new_obj_file, std::ios_base::out);

    fs << "# Obj File Created by Xiaochen Fan" << "\n" << "\n";

    if (tex_flag)
    {

        fs << "mtllib flower.mtl" << "\n";

        fs << "\n\n";
    }
    
    Petals& petals = flower_->getPetals();

    int id_count = 0;
    for (size_t i = 0, i_end = petals.size(); i < i_end; ++ i)
    {
        Petal& petal = petals[i];

        osg::ref_ptr<osg::Vec3Array> vertices = petal.getVertices();
        osg::ref_ptr<osg::Vec2Array> texcoords = petal.getTexcoords();
        osg::ref_ptr<osg::Vec3Array> vertex_normals = petal.getVertexNormals();
        std::vector<std::vector<int> > faces = petal.getFaces();


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

        fs << "g " << petal.getObjName() << std::endl;
        fs << "usemtl " << (tex_flag?(petal.getObjName()+".mtl"):"default") << std::endl;
        fs << "s 1" << std::endl;

        fs << "\n\n";

        // default: 3 vertices a face, since in the tiny_obj_loader, the index of
        // face starts from zero, I have to add one to it
        for (size_t i = 0, i_end = faces.size(); i < i_end; ++ i)
        {
            std::vector<int>& face = faces.at(i);
            if (face.size() != 3)
                return false;

            assert(face.size() == 3);

            std::string f0 = std::to_string(face[0] + 1 + id_count) ;
            std::string f1 = std::to_string(face[1] + 1 + id_count) ;
            std::string f2 = std::to_string(face[2] + 1 + id_count) ;

            int v_num = vertices->size();
            int vt_num = texcoords->size();
            int vn_num = vertex_normals->size();


            if (vn_num != 0)
            {
                fs << "f " << (v_num != 0 ? f0 : "") << "/" << (vt_num != 0 ? f0 : "") << "/" << (vn_num != 0 ? f0 : "");
                fs << " " << (v_num != 0 ? f1 : "") << "/" << (vt_num != 0 ? f1 : "") << "/" << (vn_num != 0 ? f1 : "");
                fs << " " << (v_num != 0 ? f2 : "") << "/" << (vt_num != 0 ? f2 : "") << "/" << (vn_num != 0 ? f2 : "") << "\n";
            }
            else
            {
                fs << "f " << (v_num != 0 ? f0 : "") << "/" << (vt_num != 0 ? f0 : "");
                fs << " " << (v_num != 0 ? f1 : "") << "/" << (vt_num != 0 ? f1 : "");
                fs << " " << (v_num != 0 ? f2 : "") << "/" << (vt_num != 0 ? f2 : "") << "\n";
            }

        }

        id_count += vertices->size();

        fs << "\n\n";

    }
    
    fs.close();

    return true;
}

bool ObjWriter::saveMtl(const std::string& new_obj_file, bool tex_flag /* = false */)
{
    if (!tex_flag)
        return false;

    std::string base_folder = new_obj_file;
    base_folder.resize(QString(base_folder.c_str()).lastIndexOf('/')+1);

    std::string new_mtl_file = base_folder + "/flower.mtl";
    // mtl file
    std::fstream fs(new_mtl_file, std::ios_base::out);

    fs << "# Mtl File Created by Xiaochen Fan" << "\n" << "\n";

    Petals& petals = flower_->getPetals();

    for (size_t i = 0, i_end = petals.size(); i < i_end; ++ i)
    {
        Petal& petal = petals[i];

        osg::Vec4& ambient = petal.getAmbient();
        osg::Vec4& diffuse = petal.getDiffuse();
        osg::Vec4& specular = petal.getSpecular();
        osg::Vec4& emission = petal.getEmission();
        std::string& ka = petal.getMapKa();
        std::string& kd = petal.getMapKd();

        fs << "newmtl " << petal.getObjName()+".mtl" << std::endl;
        fs << "\t" << "Ns 10.0000" << "\n";
        fs << "\t" << "Ni 1.5000" << "\n";
        fs << "\t" << "d 1.0000" << "\n";
        fs << "\t" << "Tr 0.0000" << "\n";
        fs << "\t" << "Tf 1.0000 1.0000 1.0000" << "\n";
        fs << "\t" << "illum 2" << "\n";
        fs << "\t" << "Ka " << ambient[0] << " " << ambient[1] << " " << ambient[2]<< "\n";
        fs << "\t" << "Kd " << diffuse[0] << " " << diffuse[1] << " " << diffuse[2]<< "\n";
        fs << "\t" << "Ks " << specular[0] << " " << specular[1] << " " << specular[2]<< "\n";
        fs << "\t" << "Ke " << emission[0] << " " << emission[1] << " " << emission[2]<< "\n";
        fs << "\t" << "map_Ka " << ka.substr((QString(ka.c_str()).lastIndexOf('/')+1)) << "\n";
        fs << "\t" << "map_Kd " << kd.substr((QString(kd.c_str()).lastIndexOf('/')+1)) << "\n";

        fs << "\n\n";

        // tga file
        
        std::string new_tga_file = base_folder + "/" + petal.getObjName() + ".tga";
        QFile::copy(QString(petal.getMapKd().c_str()), QString(new_tga_file.c_str()));

    }

    fs.close();

    

    return true;

}