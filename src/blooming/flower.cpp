
#include "flower.h"
#include "main_window.h"

Flower::Flower()
{
}

Flower::~Flower()
{}

void Flower::save(const std::string& flower_path)
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        std::string& petal_file = flower_path + QString("/petal-%1.obj").arg(i).toStdString();
        Petal& petal = petals_[i];
        petal.save(petal_file);
    }
}

void Flower::show()
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        Petal& petal = petals_[i];
        MainWindow::getInstance()->getSceneWidget()->addSceneChild(&petal);
    }
}

void Flower::update()
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        Petal& petal = petals_[i];
        petal.expire();
    }
}

void Flower::hide()
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        Petal& petal = petals_[i];
        MainWindow::getInstance()->getSceneWidget()->removeSceneChild(&petal);
    }
}

Flower Flower::simplifyMesh(int scale)
{
    Flower simplified_flower;

    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        Petal& petal = petals_[i];
        Petal simplified_petal = petal.simplify(scale);
        simplified_flower.getPetals().push_back(simplified_petal);
    }

    return simplified_flower;
}

void Flower::buildHardCtrs(int scale)
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        Petal& petal = petals_[i];
        petal.buildHardCtrsIdx(scale);
    }
}

void Flower::deform(const std::vector<osg::ref_ptr<osg::Vec3Array> >& pos, 
            const std::vector<std::vector<int> > idx)
{
    if (pos.size() != idx.size())
    {
        std::cerr << "the number of position groups is not the same as indices' group number" << std::endl;
        return;
    }

    if (pos.size() != petals_.size())
    {
        std::cerr << "the number of position groups is not the same as petals' number" << std::endl;
        return;
    }

    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        Petal& petal = petals_.at(i);
        /*std::vector<float> ctrs;
        osg::ref_ptr<osg::Vec3Array> ctrs_pos = petal->getVertices();
        for (size_t j = 0, j_end = ctrs_pos->size(); j < j_end; ++ j)
        {
        ctrs.push_back(ctrs_pos->at(j).x());
        ctrs.push_back(ctrs_pos->at(j).x());
        ctrs.push_back(ctrs_pos->at(j).x());
        }*/

        petal.deform(*pos[i], idx[i]);
    }
}

void Flower::deform(const std::vector<osg::ref_ptr<osg::Vec3Array> >& hard_ctrs, const std::vector<std::vector<int> > hard_idx, 
            const std::vector<osg::ref_ptr<osg::Vec3Array> >& soft_ctrs, const std::vector<std::vector<int> > soft_idx)
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        Petal petal = petals_.at(i);
        /*std::vector<float> ctrs;
        osg::ref_ptr<osg::Vec3Array> ctrs_pos = petal->getVertices();
        for (size_t j = 0, j_end = ctrs_pos->size(); j < j_end; ++ j)
        {
        ctrs.push_back(ctrs_pos->at(j).x());
        ctrs.push_back(ctrs_pos->at(j).x());
        ctrs.push_back(ctrs_pos->at(j).x());
        }*/

        petal.deform(*hard_ctrs[i], hard_idx[i], *soft_ctrs[i], soft_idx[i]);
    }
}


//void Flower::searchNearestIdx(Flower& simplified_flower, 
//                      std::vector<std::vector<int> >& knn_idx)
//{
//    Petals& petals = this->getPetals();
//    Petals& s_petals = simplified_flower.getPetals();
//
//    for (size_t i = 0, i_end = petals.size(); i < i_end; ++ i)
//    {
//        std::vector<int> idx;
//        petals[i].searchNearestIdx(s_petals[i], idx);
//        knn_idx.push_back(idx);
//    }
//}

void Flower::searchNearestIdx(PointCloud& point_cloud, 
                              std::vector<osg::ref_ptr<osg::Vec3Array> >& knn_pos)
{
    Petals& petals = this->getPetals();

    for (size_t i = 0, i_end = petals.size(); i < i_end; ++ i)
    {
        osg::ref_ptr<osg::Vec3Array> pos = new osg::Vec3Array;
 //       petals[i]->searchNearestIdx(point_cloud, *pos);
        knn_pos.push_back(pos);
    }
}