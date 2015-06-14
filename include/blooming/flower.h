#ifndef FLOWER_H
#define FLOWER_H

#include "renderable.h"
#include "mesh_model.h"

typedef MeshModel   Petal;
typedef std::vector<Petal>  Petals;

// Flower could not be template of osg::ref_ptr unless inherited from osg::Referenced
class Flower: public osg::Referenced
{
public:
    Flower();
    Flower(const Flower& flower); // deep copy
    virtual ~Flower(); 

    inline const Petals& getPetals() const { return petals_; }
    inline Petals& getPetals() { return petals_; }

    void save(const std::string& flower_path);
    void save(const std::string& flower_folder, int frame);
    void load(const std::string& flower_path);

    void clear();

    void show();
    void update();
    void hide();
    void remove();

    void rotate(const osg::Matrix& rot_matrix);

    void initVisibility();
    void determineVisibility();
    void determineWeights(PointCloud* aligned_cloud);

private:
    int contains(Petal* petal);

    void determineIntersection();
    
private:
    Petals            petals_;

};

typedef std::vector<Flower> Flowers;

class FlowersViewer
{
public:
    FlowersViewer(const std::string& flowers_folder);
    virtual ~FlowersViewer();

    void setFrame(int frame);

    void getFlower();
    void getFlower(int frame);
    void next();
    void previous();

    void show();
    void update();

    void computeFrameRange();

private:
    void extractStartEndFrame(const QStringList& entries, int& start_frame, int& end_frame);

private:
    std::string flowers_folder_;

    Flower current_flower_;
    int current_frame_;

    int start_frame_;
    int end_frame_;
};

#endif 