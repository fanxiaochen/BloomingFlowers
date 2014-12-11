
#ifndef RENDERABLE_H
#define RENDERABLE_H

#include <QMutex>
#include <QReadWriteLock>
#include <osg/BoundingBox>
#include <osg/MatrixTransform>

#define META_Renderable(name) \
  virtual bool isSameKindAs(const osg::Object* obj) const { return dynamic_cast<const name *>(obj)!=NULL; } \
  virtual const char* className() const { return #name; } \
  virtual void accept(osg::NodeVisitor& nv) { if (nv.validNodeMask(*this)) { nv.pushOntoNodePath(this); nv.apply(*this); nv.popFromNodePath(); } } \

class UpdateCallback;

class Renderable : public osg::MatrixTransform
{
public:
    Renderable(void);
    virtual ~Renderable(void);

    META_Renderable(Renderable);

    void expire(void);

    inline bool isHidden(void) const {return hidden_;}
    virtual void toggleHidden(void);

    QReadWriteLock& getReadWriteLock(void) {return read_write_lock_;}

    osg::BoundingBox getBoundingBox(void);

    virtual void pickEvent(int pick_mode, osg::Vec3 position) {}

protected:
    friend class UpdateCallback;
    void update(void);
    virtual void updateImpl(void) = 0;

protected:
    mutable QReadWriteLock              read_write_lock_;
    osg::ref_ptr<osg::MatrixTransform>  content_root_;
    bool                                expired_;
    bool                                hidden_;

    static QMutex						mutex_graphics_context_;
};

#endif // RENDERABLE_H