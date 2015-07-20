#include "flower.h"
#include "point_cloud.h"
#include "solver.h"

void Solver::setFlower(Flower* flower)
{
    flower_ = flower;
}

void Solver::setPointCloud(PointCloud* point_cloud)
{
    point_cloud_ = point_cloud;
}

void Solver::initSolver()
{
    initParas();
    initTerms();
}

void Solver::initParas()
{
    Petals& petals = flower_->getPetals();
    petal_num_ = petals.size();

    deform_petals_.resize(petal_num_);

    // init origin petal
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals.at(i);
        int petal_size = petal.getVertices()->size();
        PetalMatrix pm(3, petal_size);

        for (size_t j = 0, j_end = petal_size; j < j_end; ++ j)
        {
            pm.col(j) << petal.getVertices()->at(j).x(), petal.getVertices()->at(j).y(), petal.getVertices()->at(j).z();
        }

        deform_petals_[i]._origin_petal = pm;
    }

    // init cloud matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        osg::ref_ptr<PointCloud> petal_cloud = point_cloud_->getPetalCloud(i);
        CloudMatrix cm(3, petal_cloud->size());
        if (petal_cloud != NULL)
        {
            for (size_t j = 0, j_end = petal_cloud->size(); j < j_end; ++ j)
            {
                cm.col(j) << petal_cloud->at(j).x, petal_cloud->at(j).y, petal_cloud->at(j).z;
            }
        }

        deform_petals_[i]._cloud_matrix = cm;
    }


    // init petal matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals.at(i);
        osg::ref_ptr<PointCloud> petal_cloud = point_cloud_->getPetalCloud(i);
        std::vector<int> knn_idx;
        petal.searchNearestIdx(petal_cloud, knn_idx);

        int petal_size = petal.getVertices()->size();
        PetalMatrix pm(3, petal_size);

        for (size_t j = 0, j_end = petal_size; j < j_end; ++ j)
        {
            pm.col(j) << petal_cloud->at(knn_idx[j]).x, petal_cloud->at(knn_idx[j]).y, petal_cloud->at(knn_idx[j]).z;
        }

        deform_petals_[i]._petal_matrix = pm;
    }

    // init correspondence matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        CloudMatrix& cloud_mat = deform_petals_[i]._cloud_matrix;
        PetalMatrix& petal_mat = deform_petals_[i]._petal_matrix;
        CorresMatrix corres_mat = CorresMatrix::Zero(petal_mat.cols(), cloud_mat.cols());
        deform_petals_[i]._corres_matrix = corres_mat;
    }

    // init weight list
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals.at(i);
        deform_petals_[i]._weight_list = petal.getWeights();
    }

    // init adjacent list
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals.at(i);
        deform_petals_[i]._adj_list = petal.getAdjList();
    }

    // init face list
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals.at(i);
        std::vector<std::vector<int> >& triangle_list = petal.getFaces(); 
        std::vector<Eigen::Vector3i>& face_list = deform_petals_[i]._face_list;
        for (size_t i = 0, i_end = triangle_list.size(); i < i_end; ++ i)
        {
            std::vector<int> face = triangle_list[i];
            std::sort(face.begin(), face.end());
            assert(face.size() == 3);
            face_list.push_back(Eigen::Vector3i(face[0], face[1], face[2]));
        }
    }

    // init weight matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        std::vector<Eigen::Triplet<double> > weight_list;
        DeformPetal& deform_petal = deform_petals_[i];
        WeightMatrix& weight_matrix = deform_petal._weight_matrix;
        int ver_num = deform_petal._petal_matrix.cols();

        for (size_t i = 0; i != ver_num; ++i) 
        {
            for (size_t j = 0, j_end = deform_petal._adj_list[i].size(); j != j_end; ++j) 
            {
                int id_j = deform_petal._adj_list[i][j];

                std::vector<int> share_vertex;
                deform_petal.findSharedVertex(i, id_j, share_vertex);

                double wij = 0;
                if (share_vertex.size()==2) wij = deform_petal.wij(i, id_j, share_vertex[0], share_vertex[1]);
                else wij = deform_petal.wij(i, id_j, share_vertex[0]);

                weight_list.push_back(Eigen::Triplet<double>(i, id_j, wij));
            }
        }

        weight_matrix.resize(ver_num, ver_num);
        weight_matrix.setFromTriplets(weight_list.begin(), weight_list.end());
    }

    // init covariance matrix 
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        AdjList& adj_list = deform_petals_[i]._adj_list;
        PetalMatrix& origin_petal = deform_petals_[i]._origin_petal;
        CovMatrix& cov_matrix = deform_petals_[i]._cov_matrix;
        cov_matrix.resize(3, origin_petal.cols());

        for (size_t k = 0, k_end = adj_list.size(); k < k_end; ++ k)
        {
            Eigen::Vector3d c = origin_petal.col(k);
            int adj_size = adj_list[k].size();
            double s_x = 0, s_y = 0, s_z = 0;

            for (size_t j = 0, j_end = adj_size; j < j_end; ++ j)
            {
                int id_j = adj_list[k][j];
                Eigen::Vector3d v = origin_petal.col(id_j);

                s_x += abs(c[0] - v[0]);
                s_y += abs(c[1] - v[1]);
                s_z += abs(c[2] - v[2]);
            }

            cov_matrix.col(k) << pow(s_x/adj_size, 2.0), pow(s_y/adj_size, 2.0), pow(s_z/adj_size, 2.0); 
        }
    }

    // init hard constraints
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals[i];
        HardCtrsIdx& hc_idx = deform_petals_[i]._hc_idx;
        hc_idx = petal.getHardCtrsIndex();
        std::sort(hc_idx.begin(), hc_idx.end());
    }
}

void Solver::initTerms()
{
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        data_term_.push_back(DataFittingTerm(i));
    }

    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        arap_term_.push_back(ARAPTerm(i));
    }
}

void Solver::solve()
{

}

Flower* Solver::getFlower()
{
    return flower_;
}




