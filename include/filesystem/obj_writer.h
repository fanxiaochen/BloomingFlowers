#ifndef OBJ_WRITER_H
#define OBJ_WRITER_H

#include <string>

class MeshModel;

class ObjWriter
{
public:
    ObjWriter(MeshModel* mesh_model);
    ~ObjWriter();

    bool save(const std::string& obj_file);

private:
    MeshModel* mesh_model_;
};
#endif