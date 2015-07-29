#include <fstream>
#include <sstream>

#include <osg/ShapeDrawable>

#include "main_window.h"
#include "skeleton.h"

Skeleton::Skeleton()
    :joint_number_(0),
    show_skeleton_(true)
{

}

Skeleton::Skeleton(const Skeleton& skeleton)
    :joint_number_(0),
    show_skeleton_(true)
{
    this->branches_ = skeleton.branches_;
    this->joint_number_ = skeleton.joint_number_;
}

Skeleton Skeleton::operator = (const Skeleton& skeleton)
{
    this->branches_ = skeleton.branches_;
    this->joint_number_ = skeleton.joint_number_;

    return *this;
}

void Skeleton::load(std::string file)
{
    if (!QFile(file.c_str()).exists())
    {
        std::cout << "no skeleton file founded!!!" << std::endl;
        return;
    }
    //load the skeleton file 
    std::ifstream infile;
    infile.open( file.c_str());

    std::stringstream sem; 
    sem << infile.rdbuf(); 

    std::string str;
    int num;
    int num2;

    while( sem >> str ){

        if (str == "CN")
        {
            // get number of branches
            sem >> num;

            branches_.resize(num); 

            for (int i = 0; i < num; i++)
            {
                sem >> str;
                sem >> num2;
                for(int j = 0; j < num2; j++)
                {
                    Point p;
                    sem >> p.x >> p.y >> p.z;
                    branches_[i].push_back(p) ;
                    joint_number_ ++; // count total joint number
                }
            }
        }
    }
    infile.close() ;
}

bool Skeleton::isEmpty()
{
    if (branches_.empty())
        return true;
    else return false;
}

// the input skeleton has already been uniformly sampled in order
osg::ref_ptr<Skeleton> Skeleton::sample(int sample_ratio)
{
    if (branches_.empty() || sample_ratio == 0)
        return NULL;

    osg::ref_ptr<Skeleton> sampled_skel = new Skeleton;
    
    for (size_t i = 0, branch_num = branches_.size(); i < branch_num; ++ i)
    {
        if (branches_[i].size() < 3)
            continue;

        int start = 0;
        int end = branches_[i].size()-1;
        Point start_point = branches_[i][start]*0.5 + branches_[i][start+1]*0.5;
        Point end_point = branches_[i][end]*0.5 + branches_[i][end-1]*0.5;

        Branch sampled_branch;
        sampled_branch.push_back(start_point);

        int index = start+sample_ratio;
        while (index < end)
        {
            sampled_branch.push_back(branches_[i][index]);
            index += sample_ratio;
        }
        
        sampled_branch.push_back(end_point);

        sampled_skel->getBranches().push_back(sampled_branch);
    }

    return sampled_skel;
}

void Skeleton::updateImpl()
{
    visualizeSkeleton();
    return;
}

void Skeleton::visualizeSkeleton(void)
{
    if (!show_skeleton_) return;

    osg::ref_ptr<osg::Geode> geode(new osg::Geode);

    for (int i = 0, branch_num = branches_.size(); i < branch_num; ++ i)
    {
        for (int j = 0, points_num = branches_[i].size(); j < points_num-1; ++ j)
        {
            osg::Vec3 start(branches_[i][j].x, branches_[i][j].y, branches_[i][j].z);
            osg::Vec3 end(branches_[i][j+1].x, branches_[i][j+1].y, branches_[i][j+1].z);
            double height = (start-end).length();
            osg::Vec3 center = (start+end) / 2;
            double radius = 1;

            osg::Vec3 z_axis(0.0, 0.0, 1.0);
            osg::Quat rotation;
            rotation.makeRotate(z_axis, end-start);

            //	Create a cylinder between the two points with the given radius
            osg::Cylinder* cylinder = new osg::Cylinder(center,radius,height);
            cylinder->setRotation(rotation);
            osg::ShapeDrawable* drawable = new osg::ShapeDrawable(cylinder);
            drawable->setColor(ColorMap::getInstance().getDiscreteColor(8));
            geode->addDrawable(drawable);
        }
    }

    content_root_->addChild(geode);
    
    return;
}

void Skeleton::show()
{
    MainWindow::getInstance()->getSceneWidget()->addSceneChild(this);
}

void Skeleton::hide()
{
    MainWindow::getInstance()->getSceneWidget()->removeSceneChild(this);
}
