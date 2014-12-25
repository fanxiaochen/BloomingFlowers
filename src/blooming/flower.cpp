
#include "flower.h"
#include "main_window.h"
#include "registrator.h"
#include "scene_widget.h"

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

int Flower::contains(Petal* petal)
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        if (&petals_[i] == petal)
            return i;
    }
    return -1;
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

//void Flower::searchNearestIdx(PointCloud& point_cloud, 
//                              std::vector<osg::ref_ptr<osg::Vec3Array> >& knn_pos)
//{
//    Petals& petals = this->getPetals();
//
//    for (size_t i = 0, i_end = petals.size(); i < i_end; ++ i)
//    {
//        osg::ref_ptr<osg::Vec3Array> pos = new osg::Vec3Array;
// //       petals[i]->searchNearestIdx(point_cloud, *pos);
//        knn_pos.push_back(pos);
//    }
//}

void Flower::determineVisibility()
{
    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
        petals_[i].initializeVisibility();

    Registrator* registrator = MainWindow::getInstance()->getRegistrator();

    osg::BoundingSphere bounding_sphere = MainWindow::getInstance()->getSceneWidget()->getBoundingSphere();
    osg::ref_ptr<osg::Group> scene_root = MainWindow::getInstance()->getSceneWidget()->getSceneRoot();

    osg::Vec3d start = bounding_sphere.center() + osg::Vec3d(0.0, 0.0, bounding_sphere.radius());
    osg::Vec3d end = bounding_sphere.center() - osg::Vec3d(0.0, 0.0, bounding_sphere.radius());

    osg::Vec3d deltaRow(bounding_sphere.radius()*0.01, 0.0, 0.0);
    osg::Vec3d deltaColumn(0.0, bounding_sphere.radius()*0.01, 0.0);

    unsigned int numRows = 200;
    unsigned int numColumns = 200;

    osg::ref_ptr<osgUtil::IntersectorGroup> intersectorGroup = new osgUtil::IntersectorGroup();

    for(unsigned int r = 0; r < numRows; ++r)
    {
        for(unsigned int c = 0; c < numColumns; ++c)
        {
            osg::Vec3d s = start + deltaColumn * double(c) + deltaRow * double(r);
            osg::Vec3d e = end + deltaColumn * double(c) + deltaRow * double(r);
            osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector(s, e);
            intersectorGroup->addIntersector( intersector.get() );
        }
     }

    osgUtil::IntersectionVisitor intersectVisitor( intersectorGroup.get());
    scene_root->accept(intersectVisitor);

    if (intersectorGroup->containsIntersections())
    {
         std::cout<<"Found intersections "<<std::endl;
         int count = 0;
         osgUtil::IntersectorGroup::Intersectors& intersectors = intersectorGroup->getIntersectors();
         for(osgUtil::IntersectorGroup::Intersectors::iterator intersector_itr = intersectors.begin();
             intersector_itr != intersectors.end(); ++intersector_itr)
         {
             osgUtil::LineSegmentIntersector* lsi = dynamic_cast<osgUtil::LineSegmentIntersector*>(intersector_itr->get());
             if (lsi)
             {
                 osgUtil::LineSegmentIntersector::Intersection& intersection = lsi->getFirstIntersection();
                 osg::NodePath& node_path = intersection.nodePath;
                 Petal* intersection_object = NULL;

                 while (!node_path.empty()) {
                     intersection_object = dynamic_cast<Petal*>(node_path.back());
                     if (intersection_object != NULL) {
                         break;
                     }
                     node_path.pop_back();
                 }

                 if (intersection_object != NULL)
                 {
                     count ++;
                      std::cout << intersection.getWorldIntersectPoint().x() << std::endl;
                      std::cout << intersection.getWorldIntersectPoint().y() << std::endl;
                      std::cout << intersection.getWorldIntersectPoint().z() << std::endl;

                     // Petal* petal = dynamic_cast<Petal*>(intersection_object);

                      int index = intersection_object->searchNearestIdx(intersection.getWorldIntersectPoint());
                      intersection_object->getVisibility().at(index) = 1;
                  //    std::cout << "  indices " << intersection.indexList.at(0) << std::endl;
                 }
                
              //       std::cout << "  ratio " << intersection.ratio << std::endl;
              //       std::cout << "  point " << intersection.localIntersectionPoint << std::endl;
              //       std::cout << "  normal " << intersection.localIntersectionNormal << std::endl;
              //       std::cout << "  indices " << intersection.indexList.size() << std::endl;
              //       std::cout << "  primitiveIndex " << intersection.primitiveIndex << std::endl;
              //       std::cout << std::endl;
                
                 
             }
         }
         std::cout << count << std::endl;
    }
    else
    {
        std::cout << "No intersections" << std::endl;
    }
}