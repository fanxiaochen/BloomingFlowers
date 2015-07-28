#ifndef SKELETON_H
#define SKELETON_H

#include "point_cloud.h"
#include "renderable.h"

class Skeleton: public Renderable
{
public:
    typedef std::vector<Point> Branch;
    typedef std::vector<Branch> Branches;

public:
    Skeleton(const Skeleton& skeleton);
  //  Skeleton operator = (const Skeleton& skeleton);

    void load(std::string file);
    bool isEmpty();

    void show();
    void hide();

    osg::ref_ptr<Skeleton> sample(int sample_ratio);
    inline Branches& getBranches(){ return branches_; }
    inline Branch& getBranch(int i){ return branches_[i]; }
    inline int getJointNumber(){ return joint_number_; }

protected:
    virtual void updateImpl(void);
    virtual void visualizeSkeleton(void);
    

private:
    Branches branches_;

    int joint_number_;

};
#endif