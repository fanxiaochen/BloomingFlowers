#ifndef REGISTRATOR_H
#define REGISTRATOR_H

#include <QObject>

#include <osg/MatrixTransform>

class Registrator : public QObject
{
    Q_OBJECT
public:
    Registrator(void);
    virtual ~Registrator(void);

    virtual const char* className() const {return "Registrator";}

    osg::Vec3 getPivotPoint() const;
    osg::Vec3 getAxisNormal() const;
    osg::Matrix getRotationMatrix(double angle) const;

    void setPivotPoint(const osg::Vec3& pivot_point);
    void setAxisNormal(const osg::Vec3& axis_normal);

    bool load(const QString& filename);

protected:
    osg::ref_ptr<osg::MatrixTransform>                  pivot_point_;
    osg::ref_ptr<osg::MatrixTransform>                  normal_point_;

};

#endif // REGISTRATOR_H