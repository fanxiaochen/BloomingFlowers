#include <fstream>
#include <sstream>

#include <osg/ShapeDrawable>

#include "main_window.h"
#include "skeleton.h"

Skeleton::Skeleton()
    :joint_number_(0),
    show_skeleton_(false)
{

}

Skeleton::Skeleton(const Skeleton& skeleton)
    :joint_number_(0),
    show_skeleton_(false)
{
    this->joints_ = skeleton.joints_;
    this->branches_ = skeleton.branches_;
    this->joint_number_ = skeleton.joint_number_;
}

// do not copy show_skeleton_, it's used in Flower Viewer
// to record skeleton state of last frame...bad design
Skeleton Skeleton::operator = (const Skeleton& skeleton)
{
    this->joints_ = skeleton.joints_;
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
    int cn;
    int cv;
    int cnn;

    if (sem >> str && str == "CN")
    {
        sem >> cn;
        branches_.resize(cn);
    }
    else return;

    if (sem >> str && str == "CV")
    {
        sem >> cv;
        joints_.resize(cv);
        joint_number_ = cv;
        for (int i = 0; i < cv; ++ i)
        {
            Point p;
            sem >> p.x >> p.y >> p.z;
            joints_[i] = p;
        }
    }
    else return;

    int branch_idx = 0;
    while( sem >> str ){

        if (str == "CNN")
        {
            sem >> cnn;

            branches_[branch_idx].resize(cnn); 

            for (int i = 0; i < cnn; i++)
            {
                int index;
                sem >> index;
                branches_[branch_idx][i] = index - 1; // original index starts from 1
            }
        }

        branch_idx ++;
    }
    infile.close();
}

void Skeleton::save(std::string file)
{
    if (isEmpty()) 
    {
    std::cout << "empty skeleton!!" << std::endl;
    return;
    }

    std::ofstream outfile;
    outfile.open( file.c_str());

    outfile << "CN " << branches_.size() << "\n";
    outfile << "CV " << joint_number_ << "\n";
    for (auto& joint : joints_)
    {
        outfile << joint.x << " " << joint.y << " " << joint.z << "\n";
    }
    for (size_t i = 0, i_end = branches_.size(); i < i_end; ++ i)
    {
        outfile << "CNN " << branches_[i].size() << "\n";
        for (size_t j = 0, j_end = branches_[i].size(); j < j_end; ++ j)
            outfile << branches_[i][j] + 1 << "\n"; // add extra one
    }

    return;
}

bool Skeleton::isEmpty()
{
    if (branches_.empty())
        return true;
    else return false;
}

//// the input skeleton has already been uniformly sampled in order
//osg::ref_ptr<Skeleton> Skeleton::sample(int sample_ratio)
//{
//    if (branches_.empty() || sample_ratio == 0)
//        return NULL;
//
//    osg::ref_ptr<Skeleton> sampled_skel = new Skeleton;
//    
//    for (size_t i = 0, branch_num = branches_.size(); i < branch_num; ++ i)
//    {
//        if (branches_[i].size() < 3)
//            continue;
//
//        int start = 0;
//        int end = branches_[i].size()-1;
//        Point start_point = branches_[i][start]*0.5 + branches_[i][start+1]*0.5;
//        Point end_point = branches_[i][end]*0.5 + branches_[i][end-1]*0.5;
//
//        Branch sampled_branch;
//        sampled_branch.push_back(start_point);
//
//        int index = start+sample_ratio;
//        while (index < end)
//        {
//            sampled_branch.push_back(branches_[i][index]);
//            index += sample_ratio;
//        }
//        
//        sampled_branch.push_back(end_point);
//
//        sampled_skel->getBranches().push_back(sampled_branch);
//    }
//
//    return sampled_skel;
//}

void Skeleton::updateImpl()
{
    visualizeSkeleton();
    return;
}

void Skeleton::visualizeSkeleton(void)
{
    osg::ref_ptr<osg::Geode> geode(new osg::Geode);

    for (int i = 0, branch_num = branches_.size(); i < branch_num; ++ i)
    {
        for (int j = 0, points_num = branches_[i].size(); j < points_num-1; ++ j)
        {
            osg::Vec3 start(joints_[branches_[i][j]].x, joints_[branches_[i][j]].y, joints_[branches_[i][j]].z);
            osg::Vec3 end(joints_[branches_[i][j+1]].x, joints_[branches_[i][j+1]].y, joints_[branches_[i][j+1]].z);
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
            drawable->setColor(ColorMap::getInstance().getDiscreteColor(26));
            geode->addDrawable(drawable);

        }
    }

    for (auto& joint : joints_)
    {
            // create a sphere for each joint point
            osg::Vec3 point(joint.x, joint.y, joint.z);
            osg::Sphere* sphere = new osg::Sphere(point, 2);
            osg::ShapeDrawable* drawable = new osg::ShapeDrawable(sphere);
            drawable->setColor(ColorMap::getInstance().getDiscreteColor(27));
            geode->addDrawable(drawable);
    }

    content_root_->addChild(geode);
    
    return;
}

void Skeleton::show()
{
    if (show_skeleton_ == false)
    {
        MainWindow::getInstance()->getSceneWidget()->addSceneChild(this);
        show_skeleton_ = !show_skeleton_;
    }
}

void Skeleton::hide()
{
    if (show_skeleton_ == true)
    {
        MainWindow::getInstance()->getSceneWidget()->removeSceneChild(this);
        show_skeleton_ = !show_skeleton_;
    }
}
