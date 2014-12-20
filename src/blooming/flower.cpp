
#include "flower.h"
#include "main_window.h"
#include "mesh_simplify.h"

Flower::Flower()
{
}

Flower::~Flower()
{}

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

Flower Flower::simplifyMesh(int delta)
{
    Flower simplified_flower;

    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        Simplify::vertices.clear();
        Simplify::triangles.clear();

        Petal& petal = petals_[i];
        Petal simplified_petal(petal);  // deep copy

        for (size_t i = 0, i_end = simplified_petal.getVertices()->size(); i < i_end; ++ i)
        {
            Simplify::Vertex v;
            const osg::Vec3& vertice = simplified_petal.getVertices()->at(i);
            v.p.x = vertice.x();
            v.p.y = vertice.y();
            v.p.z = vertice.z();
            Simplify::vertices.push_back(v);
        }

        for (size_t i = 0, i_end = simplified_petal.getFaces().size(); i < i_end; ++ i)
        {
            Simplify::Triangle t;
            const std::vector<int>& face = simplified_petal.getFaces().at(i);
            t.v[0] = face[0];
            t.v[1] = face[1];
            t.v[2] = face[2];
            Simplify::triangles.push_back(t);
        }

        Simplify::simplify_mesh(simplified_petal.getFaces().size() / delta);

        simplified_petal.getVertices()->clear();
        simplified_petal.getFaces().clear();

        for (size_t i = 0, i_end = Simplify::vertices.size(); i < i_end; ++ i)
        {
            osg::Vec3 vertice;
            const Simplify::Vertex& v = Simplify::vertices.at(i);
            vertice.x() = v.p.x;
            vertice.y() = v.p.y;
            vertice.z() = v.p.z;
            simplified_petal.getVertices()->push_back(vertice);
        }

        for (size_t i = 0, i_end = Simplify::triangles.size(); i < i_end; ++ i)
        {
            std::vector<int> face;
            const Simplify::Triangle& t = Simplify::triangles.at(i);
            face.push_back(t.v[0]);
            face.push_back(t.v[1]);
            face.push_back(t.v[2]);

            simplified_petal.getFaces().push_back(face);
        }

        simplified_flower.getPetals().push_back(simplified_petal);
    }

    return simplified_flower;
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


void Flower::searchNearestIdx(Flower& simplified_flower, 
                      std::vector<std::vector<int> >& knn_idx)
{
    Petals& petals = this->getPetals();
    Petals& s_petals = simplified_flower.getPetals();

    for (size_t i = 0, i_end = petals.size(); i < i_end; ++ i)
    {
        std::vector<int> idx;
        petals[i].searchNearestIdx(s_petals[i], idx);
        knn_idx.push_back(idx);
    }
}

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