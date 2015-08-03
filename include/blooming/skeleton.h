#ifndef SKELETON_H
#define SKELETON_H

#include "point_cloud.h"
#include "renderable.h"

class Skeleton: public Renderable
{
public:
    typedef std::vector<Point> Joints;
    typedef std::vector<int> Branch; // indices of joints
    typedef std::vector<Branch> Branches;

public:
    Skeleton();
    Skeleton(const Skeleton& skeleton);
    Skeleton operator = (const Skeleton& skeleton);

    void load(std::string file);
    void save(std::string file);
    bool isEmpty();

    void show();
    void hide();

    /*inline const Branches& getBranches() const { return branches_; }
    inline const Branch& getBranch(int i) const { return branches_[i]; }*/

//    osg::ref_ptr<Skeleton> sample(int sample_ratio);
    inline Joints& getJoints() { return joints_; }
    inline Branches& getBranches() { return branches_; }
    inline Branch& getBranch(int i) { return branches_[i]; }
    inline int getJointNumber(){ return joint_number_; }

protected:
    virtual void updateImpl(void);
    virtual void visualizeSkeleton(void);
    

private:
    Joints joints_;
    Branches branches_;

    bool show_skeleton_;

    int joint_number_;

};
#endif