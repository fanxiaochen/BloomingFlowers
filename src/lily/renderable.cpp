#include "lily.h"

#include <iostream>
#include <random>

#include <QMenu>
#include <QImage>
#include <QColor>
#include <QCursor>

#include <osg/Point>
#include <osg/Geode>
#include <osg/StateSet>
#include <osg/Material>
#include <osgViewer/Viewer>
#include <osgUtil/UpdateVisitor>
#include <osg/ComputeBoundsVisitor>


QMutex Renderable::mutex_graphics_context_;

Renderable::Renderable(void)
  :read_write_lock_(QReadWriteLock::NonRecursive),
  content_root_(new osg::MatrixTransform),
  expired_(true),
  hidden_(false)
{
  addChild(content_root_);

  setUpdateCallback(new UpdateCallback);
  setDataVariance(Object::DYNAMIC);

  return;
}

Renderable::~Renderable(void)
{
}

void Renderable::expire(void)
{
  QWriteLocker locker(&read_write_lock_);
  expired_ = true;

  return;
}

void Renderable::update(void)
{
  if (!read_write_lock_.tryLockForRead()){
    std::cout << "bug here?" << std::endl;
	  return;
  }

  if (!expired_)
  {
	  std::cout <<"expired_ = false " << std::endl;
    read_write_lock_.unlock();
    return;
  }
  std::cout << "expired_ = true " << std::endl;
  expired_ = false;
  content_root_->removeChildren(0, content_root_->getNumChildren());

  if (!hidden_){
	std::cout << "update" << std::endl;
    updateImpl();
  }

  read_write_lock_.unlock();
  return;
}

void Renderable::toggleHidden(void)
{
  QWriteLocker locker(&read_write_lock_);
  expired_ = true;
  hidden_ = !hidden_;

  return;
}

osg::BoundingBox Renderable::getBoundingBox(void)
{
  osgUtil::UpdateVisitor update_visitor;
  this->accept(update_visitor);

  osg::ComputeBoundsVisitor visitor;
  content_root_->accept(visitor);

  return visitor.getBoundingBox();
}

static void saveColorImage(osg::Image* color_image, osg::Image* depth_image, const std::string& filename)
{
  int width = color_image->s();
  int height = color_image->t();
	float* z_buffer = (float*)(depth_image->data());

  QImage q_image(width, height, QImage::Format_ARGB32);
  for (int x = 0; x < width; ++ x)
  {
    for (int y = 0; y < height; ++ y)
    {
			float z = z_buffer[y*width+x];
      osg::Vec4 color = color_image->getColor(x, y);
      color = color*255;
      QRgb rgba = QColor(color.r(), color.g(), color.b(), (z==1.0)?(0):(255)).rgba();
      q_image.setPixel(x, height-1-y, rgba);
    }
  }

  q_image.save(filename.c_str());

  return;
}

static void saveDepthImage(osg::Image* depth_image, const std::string& filename)
{
	int width = depth_image->s();
	int height = depth_image->t();

	float z_min, z_max;
	z_min = std::numeric_limits<float>::max();
	z_max = std::numeric_limits<float>::lowest();
	float* z_buffer = (float*)(depth_image->data());
	for (size_t x = 0; x < width; ++ x)
	{
		for (size_t y = 0; y < height; ++ y)
		{
			float z = z_buffer[y*width+x];
			if (z == 1.0)
				continue;

			z_min = std::min(z_min, z);
			z_max = std::max(z_max, z);
		}
	}

	QImage q_image(width, height, QImage::Format_ARGB32);
	for (size_t x = 0; x < width; ++ x)
	{
		for (size_t y = 0; y < height; ++ y)
		{
			float z = z_buffer[y*width+x];
			float value = (z==1.0)?(1.0):(z-z_min)*0.8/(z_max-z_min);
			value *= 255;
			QRgb rgba = QColor(value, value, value, (z==1.0)?(0):(255)).rgba();
			q_image.setPixel(x, height-1-y, rgba);
		}
	}

	q_image.save(filename.c_str());

	return;
}
