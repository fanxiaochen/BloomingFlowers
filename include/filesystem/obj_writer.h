#ifndef OBJ_WRITER_H
#define OBJ_WRITER_H

#include <string>

class MeshModel;
class Flower;
class FlowersViewer;

class ObjWriter
{
public:
    ObjWriter();
    ObjWriter(MeshModel* mesh_model);
    ObjWriter(Flower* flower);
    ~ObjWriter();

    bool save(const std::string& new_obj_file, bool tex_flag = false);

    bool saveAll(const std::string& new_obj_file, bool tex_flag = false);
    bool saveMtl(const std::string& new_obj_file, bool tex_flag = false);

    void merge(FlowersViewer* flower_viewer);

private:
    MeshModel* mesh_model_;
    Flower* flower_;
};
#endif