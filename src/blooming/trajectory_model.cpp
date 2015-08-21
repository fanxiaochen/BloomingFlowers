#include <osg/ShapeDrawable>
#include <osg/Point>

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

            // for trajectory
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

            ON_NurbsCurve& fitting_curve = traj.getFittingCurve();
            Eigen::VectorXd& t = traj.getParas();

            // for fitting curve
            osg::ref_ptr<osg::Vec3Array>  vertices = new osg::Vec3Array;
            osg::ref_ptr<osg::Vec4Array>  colors = new osg::Vec4Array;
            for (int i = 0, i_end = t.size(); i < i_end; ++ i)
            {
                ON_3dPoint p = fitting_curve.PointAt(t(i));
                vertices->push_back(osg::Vec3(p.x, p.y, p.z));
                colors->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0));
            }

            /* for (int i = 0, i_end = 100; i <= i_end; ++ i)
            {
            ON_3dPoint p = fitting_curve.PointAt(i*0.01);
            vertices->push_back(osg::Vec3(p.x, p.y, p.z));
            colors->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0));
            }*/

            //// test
            //for (int i = 0; i < 3; ++ i)
            //{
            //    double t = 1 + i * 0.05;
            //    ON_3dPoint p = fitting_curve.PointAt(t);
            //    vertices->push_back(osg::Vec3(p.x, p.y, p.z));
            //    colors->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0));
            //}

            osg::Geometry* geometry = new osg::Geometry;
            geometry->setVertexArray(vertices);
            geometry->setColorArray(colors);
            geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
            geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));
            geometry->getOrCreateStateSet()->setAttribute(new osg::Point(10.0f));
            geode->addDrawable(geometry);
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

void TrajectoryModel::fittingAll()
{
    for (int i = 0, i_end = trajs_set_.size(); i < i_end; ++ i)
    {
        Trajectories& trajs = trajs_set_[i];
        for (int j = 0, j_end = trajs.size(); j < j_end; ++ j)
        {
            trajs[j].curve_fitting();
        }
    }
}

void TrajectoryModel::recoverFromFlowerViewer(FlowersViewer* flower_viewer)
{
    int start_frame = flower_viewer->getStartFrame();
    int current_frame = flower_viewer->getCurrentFrame();

    this->init(flower_viewer->flower(start_frame));
    for (int i = start_frame; i <= current_frame; ++ i)
    {
        this->addFlowerPosition(flower_viewer->flower(i));
    }
}