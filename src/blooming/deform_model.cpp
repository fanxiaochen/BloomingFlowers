
#include "point_cloud.h"
#include "flower.h"

#include "deform_model.h"

DeformModel::DeformModel()
    :petal_num_(0),
    iter_num_(50), 
    eps_(1e-3),
    noise_p_(0.05)
{

}

DeformModel::DeformModel(PointCloud* point_cloud, Flower* flower)
    :petal_num_(0),
    iter_num_(50), 
    eps_(1e-3),
    noise_p_(0.05),
    point_cloud_(point_cloud),
    flower_(flower)
{

}

DeformModel::~DeformModel()
{

}

void DeformModel::setIterNum(int iter_num)
{
    iter_num_ = iter_num;
}

void DeformModel::setEps(float eps)
{
    eps_ = eps;
}

void DeformModel::setPointCloud(PointCloud* point_cloud)
{
    point_cloud_ = point_cloud;
}

void DeformModel::setFlower(Flower* flower)
{
    flower_ = flower;
}

void DeformModel::deform()
{
    // initial step

    int iter_num = 0;

    do 
    {
        e_step();

        m_step();

    } while (iter_num < iter_num_);
}


void DeformModel::e_step()
{
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        e_step(i);
    }
}

void DeformModel::m_step()
{

}

void DeformModel::e_step(int petal_id)
{
    CorresMatrix& corres_mat = deform_petals_[petal_id]._corres_matrix;
    VisList& vis_list = deform_petals_[petal_id]._vis_list;

    for (size_t i = 0, i_end = corres_mat.rows(); i < i_end; ++ i)
    {
        for (size_t j = 0, j_end = corres_mat.cols(); j < j_end; ++ j)
        {
            corres_mat(i, j) = gaussian(petal_id, i, j) * vis_list[i];
        }
    }

    for (size_t i = 0, i_end = corres_mat.rows(); i < i_end; ++ i)
    {
        float sum_gaussian = corres_mat.row(i).sum() + noise_p_;
        for (size_t j = 0, j_end = corres_mat.cols(); j < j_end; ++ j)
        {
            corres_mat(i, j) = corres_mat(i, j) / sum_gaussian;
        }
    }
}

void DeformModel::visibility()
{
    // now just using knn to determine visibility
}


void DeformModel::initialize()
{
    // init petal matrix
    Petals& petals = flower_->getPetals();
    petal_num_ = petals.size();

    deform_petals_.resize(petal_num_);

    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals.at(i);

        std::vector<int> knn_idx;
        petal.searchNearestIdx(point_cloud_, knn_idx);
        
        int petal_size = petal.getVertices()->size();
        PetalMatrix pm(3, petal_size);

        for (size_t j = 0, j_end = petal_size; j < j_end; ++ j)
        {
            pm.col(j) << point_cloud_->at(knn_idx[j]).x, point_cloud_->at(knn_idx[j]).y, point_cloud_->at(knn_idx[j]).z;
        }

        deform_petals_[i]._petal_matrix = pm;
    }

    // init cloud matrix
    point_cloud_->flower_segmentation(flower_); // using knn to segment the point cloud to petals, and along with visibility determination

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

    // init covariance matrix 
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        CloudMatrix& cloud_mat = deform_petals_[i]._cloud_matrix;
        CovMatrix cov_mat;
        covariance(cloud_mat, cov_mat);
        deform_petals_[i]._cov_matrix = cov_mat;
    }

    // init correspondence matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        CloudMatrix& cloud_mat = deform_petals_[i]._cloud_matrix;
        PetalMatrix& petal_mat = deform_petals_[i]._petal_matrix;
        CorresMatrix corres_mat(petal_mat.cols(), cloud_mat.cols());
        deform_petals_[i]._corres_matrix = corres_mat;
    }

    // init visibility list
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals.at(i);
        deform_petals_[i]._vis_list = petal.getVisibility();
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
        deform_petals_[i]._face_list = petal.getFaces();
    }
}

void DeformModel::covariance(const CloudMatrix& cloud_mat, CovMatrix& cov_mat)
{
    Eigen::VectorXf es(3);
    CloudMatrix cm(cloud_mat);
    for (size_t i = 0; i < 3; ++ i)
    {
        es[i] = cm.row(i).mean();
        cm.row(i).array() -= es[i];
        cov_mat[i] += cm.row(i).squaredNorm() / cm.rows();
    }

}

float DeformModel::gaussian(int petal_id, int m_id, int c_id)
{
    float p;

    CovMatrix cov_mat = deform_petals_[petal_id]._cov_matrix;
    CloudMatrix cloud_mat = deform_petals_[petal_id]._cloud_matrix;
    PetalMatrix petal_mat = deform_petals_[petal_id]._petal_matrix;

    Eigen::Vector3f xu = cloud_mat.col(c_id) - petal_mat.col(m_id);
    p = pow(2*M_PI, -3/2.0) * pow(cov_mat.determinant(), -1/2.0) * exp((-1/2.0)*xu.transpose()*cov_mat.asDiagonal()*xu);

    return p;
}