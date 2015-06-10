#ifndef OBJ_WRITER_H
#define OBJ_WRITER_H

#include <string>

class MeshModel;

class ObjWriter
{
public:
    ObjWriter(MeshModel* mesh_model);
    ~ObjWriter();

    bool save(const std::string& new_obj_file, bool tex_flag = false);

private:
    MeshModel* mesh_model_;
};
#endif