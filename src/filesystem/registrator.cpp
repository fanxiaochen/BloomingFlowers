#include <cstdio>

#include "color_map.h"
#include "main_window.h"
#include "point_cloud.h"
#include "osg_viewer_widget.h"
#include "file_system_model.h"

#include "registrator.h"

Registrator::Registrator(void)
    :pivot_point_(new osg::MatrixTransform),
    normal_point_(new osg::MatrixTransform)
{
    pivot_point_->setMatrix(osg::Matrix::translate(osg::Vec3(0, 0, 0)));
    normal_point_->setMatrix(osg::Matrix::translate(osg::Vec3(0, -1, 0)));
}

Registrator::~Registrator(void)
{
}

osg::Vec3 Registrator::getPivotPoint() const
{
    return pivot_point_->getMatrix().getTrans();
}

osg::Vec3 Registrator::getAxisNormal() const
{
    osg::Vec3 pivot_point = pivot_point_->getMatrix().getTrans();
    osg::Vec3 normal_point = normal_point_->getMatrix().getTrans();
    osg::Vec3 axis_normal = normal_point-pivot_point;
    axis_normal.normalize();

    return axis_normal;
}

void Registrator::setPivotPoint(const osg::Vec3& pivot_point)
{
    osg::Vec3 old_pivot_point = pivot_point_->getMatrix().getTrans();
    osg::Vec3 old_normal_point = normal_point_->getMatrix().getTrans();
    osg::Vec3 axis_normal = old_normal_point-old_pivot_point;

    osg::Vec3 normal_point = pivot_point + axis_normal;

    pivot_point_->setMatrix(osg::Matrix::translate(pivot_point));
    normal_point_->setMatrix(osg::Matrix::translate(normal_point));

    expire();
}

void Registrator::setAxisNormal(const osg::Vec3& axis_normal)
{
    osg::Vec3 pivot_point = pivot_point_->getMatrix().getTrans();
    osg::Vec3 old_normal_point = normal_point_->getMatrix().getTrans();
    osg::Vec3 old_normal = old_normal_point-pivot_point;
    double length = old_normal.length();
    if (old_normal*axis_normal < 0)
        length = -length;

    osg::Vec3 normal = axis_normal;
    normal.normalize();

    osg::Vec3 normal_point = pivot_point + normal*length;
    normal_point_->setMatrix(osg::Matrix::translate(normal_point));

    expire();
}

void Registrator::reset(void)
{
    initilized_ = false;
    clear();

    return;
}


void Registrator::init(void)
{
    if (initilized_)
        return;

    osg::BoundingSphere boundingSphere = MainWindow::getInstance()->getOSGViewerWidget()->getBound();
    double radius = boundingSphere.radius();

    if (load())
    {
        normal_point_->setMatrix(osg::Matrix::translate(getPivotPoint()+getAxisNormal()*radius));
    }
    else
    {
        pivot_point_->setMatrix(osg::Matrix::translate(boundingSphere.center()));
        normal_point_->setMatrix(osg::Matrix::translate(boundingSphere.center()+osg::Vec3(0, -radius, 0)));
    }

    initilized_ = true;

    return;
}

bool Registrator::load(const QString& filename)
{
    FILE *file = fopen(filename.toStdString().c_str(),"r");
    if (file == NULL)
        return false;

    double x, y, z;
    double nx, ny, nz;
    fscanf(file, "%lf %lf %lf", &x, &y, &z);
    fscanf(file, "%lf %lf %lf", &nx, &ny, &nz);
    fclose(file);

    setPivotPoint(osg::Vec3(x, y, z));
    setAxisNormal(osg::Vec3(nx, ny, nz));

    return true;
}

bool Registrator::load(void)
{
    const QString& workspace = MainWindow::getInstance()->getWorkspace();
    return load(workspace+"/axis.txt");
}

void Registrator::save(const QString& filename)
{
    FILE *file = fopen(filename.toStdString().c_str(),"w");
    if (file == NULL)
        return;

    osg::Vec3 pivot_point = getPivotPoint();
    osg::Vec3 axis_normal = getAxisNormal();

    fprintf(file, "%f %f %f\n", pivot_point.x(), pivot_point.y(), pivot_point.z());
    fprintf(file, "%f %f %f\n", axis_normal.x(), axis_normal.y(), axis_normal.z());
    fclose(file);

    return;
}

void Registrator::save(void)
{
    MainWindow* main_window = MainWindow::getInstance();
    QString filename = QFileDialog::getSaveFileName(main_window,
        "Save Registrator", main_window->getWorkspace(), "Registrator (*.txt)");
    if (filename.isEmpty())
        return;

    save(filename);

    return;
}


osg::Matrix Registrator::getRotationMatrix(double angle) const
{
    osg::Vec3 pivot_point = getPivotPoint();
    osg::Vec3 axis_normal = getAxisNormal();

    osg::Matrix matrix = osg::Matrix::identity();
    matrix = matrix*osg::Matrix::translate(-pivot_point);
    matrix = matrix*osg::Matrix::rotate(angle, axis_normal);
    matrix = matrix*osg::Matrix::translate(pivot_point);

    return matrix;
}



