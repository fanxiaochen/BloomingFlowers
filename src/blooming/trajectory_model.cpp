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

    int sample_ratio = 1;

    for (int i = 0; i < petals.size(); i ++)
    {
        petals[i].sampleTrajsVertices(sample_ratio);
        int trajs_num = petals[i].getTrajsVertices().size();
        Trajectories& trajs = trajs_set_[i];
        trajs.resize(trajs_num);
        trajs_indices_.push_back(petals[i].getTrajsVertices());
    }

    petal_order_ = flower->getPetalOrder();
}

void TrajectoryModel::updateImpl()
{
    visualizeTrajectory();
    return;
}

void TrajectoryModel::visualizeTrajectory()
{
    osg::ref_ptr<osg::Geode> geode(new osg::Geode);

    for (int i = 0, i_end = trajs_set_.size(); i < i_end ; ++ i)
    {
		if( i<2 )
			continue;

        Trajectories& trajs = trajs_set_[i];
        for (int j = 0, j_end = 1/*trajs.size()*/; j < j_end; ++ j)
        {
            Trajectory& traj = trajs[j];

            // for trajectory
            for (int k = 0, k_end = traj.size()-1; k < k_end; ++ k)
            {
                osg::Vec3 start(traj[k].x, traj[k].y, traj[k].z);
                osg::Vec3 end(traj[k+1].x, traj[k+1].y, traj[k+1].z);
                double height = (start-end).length();
                osg::Vec3 center = (start+end) / 2;
                double radius = 0.4;

                osg::Vec3 z_axis(0.0, 0.0, 1.0);
                osg::Quat rotation;
                rotation.makeRotate(z_axis, end-start);

                //	Create a cylinder between the two points with the given radius
                osg::Cylinder* cylinder = new osg::Cylinder(center,radius,height);
                cylinder->setRotation(rotation);
                osg::ShapeDrawable* drawable = new osg::ShapeDrawable(cylinder);
                drawable->setColor(ColorMap::getInstance().getDiscreteColor(i));
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

void TrajectoryModel::setShowTrajState(bool traj_state)
{
    if (traj_state == show_traj_)
        return;

    if (!traj_state)
    {
        this->hide();
    }
    else 
    {
        this->show();
    }
}

void TrajectoryModel::show()
{
    if (show_traj_ == false)
    {
       /* this->setNodeMask(1);*/
        MainWindow::getInstance()->getSceneWidget()->addSceneChild(this);
        show_traj_ = !show_traj_;
    }
}

void TrajectoryModel::hide()
{
    if (show_traj_ == true)
    {
        //this->setNodeMask(0X0);
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
        std::vector<int>& trajs_indice = trajs_indices_[i];
        osg::ref_ptr<osg::Vec3Array> vertices = petals[i].getVertices();
        for (int j = 0, j_end = trajs_indice.size(); j < j_end; ++ j)
        {
            Trajectory& traj = trajs[j];
            osg::Vec3& vertice = vertices->at(trajs_indice[j]);
            Point traj_point;
            traj_point.x = vertice.x();
            traj_point.y = vertice.y();
            traj_point.z = vertice.z();

            traj.push_back(traj_point);
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

void TrajectoryModel::showState( bool show_traj )
{
	if( show_traj )
		this->setNodeMask(1);
	else
		this->setNodeMask(0x0);
}


void TrajectoryModel::recover(FlowersViewer* flower_viewer)
{
    int start_frame = flower_viewer->getStartFrame();
    int end_frame = flower_viewer->getEndFrame();

    this->init(flower_viewer->flower(start_frame));
    for (int i = start_frame; i <= end_frame; ++ i)
    {
        this->addFlowerPosition(flower_viewer->flower(i));
    }
}

void TrajectoryModel::interpolate(FlowersViewer* flower_viewer, const std::string& flower_folder)
{
    // recover trajs from flowers viewer
    recover(flower_viewer);

    // fitting nurbs
    fittingAll();

    // interpolation
    int start_frame = flower_viewer->getStartFrame();
    int interpolate_frame = start_frame + 8;

    osg::ref_ptr<Flower> flower = flower_viewer->flower(start_frame);

    std::vector<double> pos_index;
//    Eigen::VectorXd paras = trajs_set_[0][0].getParas(); // get original pos value in nurbs

    pos_index = uniform_sampling(trajs_set_[0][0], interpolate_frame-start_frame);

    /*for (size_t i = 0, i_end = paras.size(); i < i_end-1; ++ i)
    {
    pos_index.push_back(paras[i]);
    pos_index.push_back((paras[i] + paras[i+1])/2);
    }*/

    /*for (size_t i = 0, i_end = pos_index.size(); i < i_end; ++ i)
    {
    std::cout << pos_index[i] << " ";
    }*/

    // Don't interpolate before start frame
    for (size_t i = start_frame; i < interpolate_frame; ++ i)
    {
        osg::ref_ptr<Flower> f = flower_viewer->flower(i);
        f->save(flower_folder, i);
    }

    // interpolate after start frame
    for (size_t t = 0, t_end = pos_index.size(); t < t_end; ++ t)
    {
        Petals& petals = flower->getPetals();
        for (int i = 0, i_end = trajs_set_.size(); i < i_end; ++ i)
        {
            Petal& petal = petals[i];

            Trajectories& trajs = trajs_set_[i];
            for (int j = 0, j_end = trajs.size(); j < j_end; ++ j)
            {
                ON_NurbsCurve& fitting_curve = trajs[j].getFittingCurve();
                ON_3dPoint p = fitting_curve.PointAt(pos_index[t]);
                osg::Vec3 point;
                point.x() = p.x;
                point.y() = p.y;
                point.z() = p.z;

                petal.getVertices()->at(j) = point;
            }

            petal.updateNormals();
        }

        flower->save(flower_folder, interpolate_frame + t);
    }

}


std::vector<double> TrajectoryModel::uniform_sampling(Trajectory& traj, int start_frame)
{
    // default parameter domain is [0, 1]

    int sample_points = 10000;
    int paras_num = 150;

    std::vector<ON_3dPoint> samples;
    ON_NurbsCurve& fitting_curve = traj.getFittingCurve();
    Eigen::VectorXd& ts = traj.getParas();

    double start_t = ts[start_frame];

    for (int i = 0; i < sample_points; ++ i)
    {
        double t = double(i) / (sample_points-1);
        ON_3dPoint point = fitting_curve.PointAt(t);
        samples.push_back(point);
    }

    double curve_length = 0;

    for (int i = 1; i < sample_points; ++ i)
    {
        curve_length += (samples[i] - samples[i-1]).Length();
    }
    
    double segment_length = curve_length / paras_num;

    /*std::cout << "curve length " << curve_length << std::endl;
    std::cout << "segment length " << segment_length << std::endl;*/

    std::vector<double> paras;

    double length = 0;
    int segment_count = 1;
    int i = 1;
    while (i < sample_points)
    {
        length += (samples[i] - samples[i-1]).Length();
        if (length >= segment_length*segment_count)
        {
            double current_t = double(i-1)/(sample_points-1);
            if (start_t <= current_t)
                paras.push_back(current_t);
            segment_count ++;
        }
        i ++;
    }

    paras.push_back(1);

    return paras;
}