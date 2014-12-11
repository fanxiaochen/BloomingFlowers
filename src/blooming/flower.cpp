

#include "flower.h"
#include "main_window.h"

Flower::Flower()
    :show_points_(false), show_mesh_(false)
{
}

Flower::~Flower()
{}

void Flower::show()
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        osg::ref_ptr<Petal> petal = petals_[i];
        MainWindow::getInstance()->getSceneWidget()->addSceneChild(petal);
    }
}

void Flower::update()
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        osg::ref_ptr<Petal> petal = petals_[i];
        petal->expire();
    }
}



