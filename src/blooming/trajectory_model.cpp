#include <osg/ShapeDrawable>

#include "main_window.h"
#include "flower.h"
#include "trajectory_model.h"

TrajectoryModel::TrajectoryModel()
    :show_traj_(false)
{

}

// first reference flower
void TrajectoryModel::init(Flower* flower)
{
    Petals& petals = flower->getPetals();
    trajs_set_.resize(petals.size());

    for (int i = 0; i < petals.size(); i ++)
    {
        osg::ref_ptr<Skeleton> skeleton = petals[i].getSkeleton();
        int joint_num = skeleton->getJointNumber();
        Trajectories& trajs = trajs_set_[i];
        trajs.resize(joint_num);
    }
}

void TrajectoryModel::updateImpl()
{
    visualizeTrajectory();
    return;
}

void TrajectoryModel::visualizeTrajectory()
{
    osg::ref_ptr<osg::Geode> geode(new osg::Geode);

    for (int i = 0, i_end = trajs_set_.size(); i < i_end; ++ i)
    {
        Trajectories& trajs = trajs_set_[i];
        for (int j = 0, j_end = trajs.size(); j < j_end; ++ j)
        {
            Trajectory& traj = trajs[j];
            for (int k = 0, k_end = traj.size()-1; k < k_end; ++ k)
            {
                osg::Vec3 start(traj[k].x, traj[k].y, traj[k].z);
                osg::Vec3 end(traj[k+1].x, traj[k+1].y, traj[k+1].z);
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
                drawable->setColor(ColorMap::getInstance().getDiscreteColor(20));
                geode->addDrawable(drawable);
            }
        }
    }

    content_root_->addChild(geode);

    return;
}

void TrajectoryModel::show()
{
    if (show_traj_ == false)
    {
        MainWindow::getInstance()->getSceneWidget()->addSceneChild(this);
        show_traj_ = !show_traj_;
    }
}

void TrajectoryModel::hide()
{
    if (show_traj_ == true)
    {
        MainWindow::getInstance()->getSceneWidget()->removeSceneChild(this);
        show_traj_ = !show_traj_;
    }
}

void TrajectoryModel::update()
{
    if (show_traj_)
        this->expire();
}

void TrajectoryModel::addFlowerPosition(Flower* flower)
{
    Petals& petals = flower->getPetals();
    for (int i = 0, i_end = petals.size(); i < i_end; ++ i)
    {
        Trajectories& trajs = trajs_set_[i];
        osg::ref_ptr<Skeleton> skeleton = petals[i].getSkeleton();
        Skeleton::Joints& joints = skeleton->getJoints();
        for (int j = 0, j_end = joints.size(); j < j_end; ++ j)
        {
            Trajectory& traj = trajs[j];
            auto& joint = joints[j];
            traj.push_back(joint);
        }
    }
}

void TrajectoryModel::reverseAll()
{
    for (int i = 0, i_end = trajs_set_.size(); i < i_end; ++ i)
    {
        Trajectories& trajs = trajs_set_[i];
        for (int j = 0, j_end = trajs.size(); j < j_end; ++ j)
        {
            trajs[j].reverse();
        }
    }
}