

#include "flower.h"
#include "main_window.h"
#include "mesh_simplify.h"

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

Flower Flower::simplifyMesh(int delta)
{
    Flower simplified_flower;

    for (size_t i = 0, i_end = petals_.size(); i < i_end; ++ i)
    {
        Simplify::vertices.clear();
        Simplify::triangles.clear();

        osg::ref_ptr<Petal> petal = petals_[i];
        osg::ref_ptr<Petal> simplified_petal= new Petal(*petal.get());

        for (size_t i = 0, i_end = simplified_petal->getVertices()->size(); i < i_end; ++ i)
        {
            Simplify::Vertex v;
            osg::Vec3& vertice = simplified_petal->getVertices()->at(i);
            v.p.x = vertice.x();
            v.p.y = vertice.y();
            v.p.z = vertice.z();
            Simplify::vertices.push_back(v);
        }

        for (size_t i = 0, i_end = simplified_petal->getFaces().size(); i < i_end; ++ i)
        {
            Simplify::Triangle t;
            std::vector<int>& face = simplified_petal->getFaces().at(i);
            t.v[0] = face[0];
            t.v[1] = face[1];
            t.v[2] = face[2];
            Simplify::triangles.push_back(t);
        }

        Simplify::simplify_mesh(simplified_petal->getVertices()->size() / delta);

        simplified_petal->getVertices()->clear();
        simplified_petal->getFaces().clear();

        for (size_t i = 0, i_end = Simplify::vertices.size(); i < i_end; ++ i)
        {
            osg::Vec3 vertice;
            Simplify::Vertex v = Simplify::vertices.at(i);
            vertice.x() = v.p.x;
            vertice.y() = v.p.y;
            vertice.z() = v.p.z;
            simplified_petal->getVertices()->push_back(vertice);
        }

        for (size_t i = 0, i_end = Simplify::triangles.size(); i < i_end; ++ i)
        {
            std::vector<int> face;
            Simplify::Triangle t = Simplify::triangles.at(i);
            face.push_back(t.v[0]);
            face.push_back(t.v[1]);
            face.push_back(t.v[2]);

            simplified_petal->getFaces().push_back(face);
        }

        simplified_flower.getPetals().push_back(simplified_petal);
    }

    return simplified_flower;
}


