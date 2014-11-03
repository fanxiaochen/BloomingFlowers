#include "lily.h"

#include <cassert>


UpdateCallback::UpdateCallback(void)
{
}


UpdateCallback::~UpdateCallback(void)
{
}


void UpdateCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
  Renderable* renderable = dynamic_cast<Renderable*>(node);
  assert(renderable != NULL);
  renderable->update();

  traverse(node, nv);

  return;
}