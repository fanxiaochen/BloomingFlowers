
#include "WunderSVD3x3.h"

#include "point_cloud.h"
#include "flower.h"

#include "deform_model.h"

DeformModel::DeformModel()
    :petal_num_(0),
    iter_num_(10), 
    eps_(1e-2),
    lambda_(2.0),
    noise_p_(0.05)
{

}

DeformModel::DeformModel(PointCloud* point_cloud, Flower* flower)
    :petal_num_(0),
    iter_num_(10), 
    eps_(1e-2),
    lambda_(2.0),
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
    initialize();

    int iter_num = 0;
    float eps = 1;

    float e = 0;

    std::cout << "Start EM Iteration..." << std::endl;

    do 
    {
        e_step();

        float e_n = m_step();

        eps = std::fabs((e_n - e) / e_n);
        e = e_n;

        std::cout << "In EM Iteration\t" << "iter: " << ++ iter_num << "\tdelta: " << eps << std::endl;

    } while (iter_num < 5 && eps > eps_ );

    // flower deformed
    deforming();
}


void DeformModel::e_step()
{
    std::cout << "E-Step: No explicit output" << std::endl;

    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        e_step(i);
    }
}

float DeformModel::m_step()
{
    std::cout << "M-Step:" << std::endl;

    int iter = 0;
    float eps = 0;
    
    float e = 0;

    updateLeftSys();
//    std::cout << L_[0] << std::endl;
    updateRightSys();
//    std::cout << d_.row(0) << std::endl;

    do {
        float e_n = solve();
        eps = std::fabs((e_n - e) / e_n);
        e = e_n;

        updateRotation();
        updateRightSys();
        std::cout << "In M-Step Iteration\t" << "iter: " << ++ iter << "\tdelta: " << eps << std::endl;

    }while(eps > eps_ && iter < iter_num_);

    return e;
}

void DeformModel::e_step(int petal_id)
{
    CorresMatrix& corres_mat = deform_petals_[petal_id]._corres_matrix;
    VisList& vis_list = deform_petals_[petal_id]._vis_list;

    for (size_t i = 0, i_end = corres_mat.rows(); i < i_end; ++ i)
    {
        for (size_t j = 0, j_end = corres_mat.cols(); j < j_end; ++ j)
        {
            corres_mat(i, j) = gaussian(petal_id, i, j) /** vis_list[i]*/;
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

    // init petal matrix
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
        CorresMatrix corres_mat = CorresMatrix::Zero(petal_mat.cols(), cloud_mat.cols());
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
        buildWeightMatrix(i);
    }

    // init rotation matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        DeformPetal& deform_petal = deform_petals_[i];
        RotList& R_list = deform_petal._R_list;
        PetalMatrix& origin_petal = deform_petal._origin_petal;
        for (size_t j = 0, j_end = origin_petal.cols(); j < j_end; ++ j)
        {
            R_list.push_back(Eigen::Matrix3f::Identity());
        }
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
        cov_mat[i] = cm.row(i).squaredNorm() / cm.cols();
    }

}

float DeformModel::gaussian(int petal_id, int m_id, int c_id)
{
    float p;

    CovMatrix cov_mat = deform_petals_[petal_id]._cov_matrix;
    CloudMatrix cloud_mat = deform_petals_[petal_id]._cloud_matrix;
    PetalMatrix petal_mat = deform_petals_[petal_id]._petal_matrix;

    Eigen::Vector3f xu = cloud_mat.col(c_id) - petal_mat.col(m_id);
    p = pow(2*M_PI, -3/2.0) * pow((cov_mat.asDiagonal()).toDenseMatrix().determinant(), -1/2.0) * 
        exp((-1/2.0)*xu.transpose()*cov_mat.asDiagonal().inverse()*xu);

    return p;
}

void DeformModel::buildWeightMatrix(int petal_id)
{
    std::vector<Eigen::Triplet<float> > weight_list;
    DeformPetal& deform_petal = deform_petals_[petal_id];
    WeightMatrix& weight_matrix = deform_petal._weight_matrix;
    int ver_num = deform_petal._petal_matrix.cols();

    for (size_t i = 0; i != ver_num; ++i) 
    {
        for (size_t j = 0, j_end = deform_petal._adj_list[i].size(); j != j_end; ++j) 
        {
            int id_j = deform_petal._adj_list[i][j];

            std::vector<int> share_vertex;
            deform_petal.findSharedVertex(i, id_j, share_vertex);

            float wij = 0;
            if (share_vertex.size()==2) wij = deform_petal.wij(i, id_j, share_vertex[0], share_vertex[1]);
            else wij = deform_petal.wij(i, id_j, share_vertex[0]);

            weight_list.push_back(Eigen::Triplet<float>(i, id_j, wij));
        }
    }

    weight_matrix.resize(ver_num, ver_num);
    weight_matrix.setFromTriplets(weight_list.begin(), weight_list.end());
}

void DeformModel::updateLeftSys()
{
    L_.clear();
    L_.resize(3); // x, y, z

    for (size_t i = 0; i < petal_num_; ++ i)
    {
        updateLeftSys(i);
    }

    L_[0].makeCompressed();
    L_[1].makeCompressed();
    L_[2].makeCompressed();
}

void DeformModel::updateLeftSys(int petal_id)
{
    DeformPetal& deform_petal = deform_petals_[petal_id];
    CovMatrix& cov_matrix = deform_petal._cov_matrix;
    CorresMatrix& corres_matrix = deform_petal._corres_matrix;
    WeightMatrix& weight_matrix = deform_petal._weight_matrix;
    int ver_num = deform_petal._petal_matrix.cols();

    std::vector<std::vector<Eigen::Triplet<float> > > weight_sums;
    weight_sums.resize(3); // for x,y,z coordinates

    for (int i = 0; i < ver_num; ++i) 
    {
        float wi_x = 0, wi_y = 0, wi_z = 0;
        for (size_t j = 0, j_end = deform_petal._adj_list[i].size(); j < j_end; ++j)
        {
            int id_j = deform_petal._adj_list[i][j];
            wi_x += weight_matrix.coeffRef(i, id_j);
            wi_y += weight_matrix.coeffRef(i, id_j);
            wi_z += weight_matrix.coeffRef(i, id_j);
        }

        wi_x += lambda_*(2/cov_matrix[0])*corres_matrix.row(i).sum();
        wi_y += lambda_*(2/cov_matrix[1])*corres_matrix.row(i).sum();
        wi_z += lambda_*(2/cov_matrix[2])*corres_matrix.row(i).sum();

        weight_sums[0].push_back(Eigen::Triplet<float>(i, i, wi_x));
        weight_sums[1].push_back(Eigen::Triplet<float>(i, i, wi_y));
        weight_sums[2].push_back(Eigen::Triplet<float>(i, i, wi_z));
    }

    Eigen::SparseMatrix<float> diag_coeff_x(ver_num, ver_num);
    Eigen::SparseMatrix<float> diag_coeff_y(ver_num, ver_num);
    Eigen::SparseMatrix<float> diag_coeff_z(ver_num, ver_num);
    diag_coeff_x.setFromTriplets(weight_sums[0].begin(), weight_sums[0].end());
    diag_coeff_y.setFromTriplets(weight_sums[1].begin(), weight_sums[1].end());
    diag_coeff_z.setFromTriplets(weight_sums[2].begin(), weight_sums[2].end());

    //L_[0] = diag_coeff_x - weight_matrix;
    //L_[1] = diag_coeff_y - weight_matrix;
    //L_[2] = diag_coeff_z - weight_matrix;


    // expand L to fill in new petal vertices
    int row_idx = L_[0].rows(), col_idx = L_[0].cols();  // rows and cols should be the same for x, y, z

    L_[0].resize(row_idx+ver_num, col_idx+ver_num);
    L_[1].resize(row_idx+ver_num, col_idx+ver_num);
    L_[2].resize(row_idx+ver_num, col_idx+ver_num);

    // block assignment is not supported by SparseMatrix...I have to update one by one
    Eigen::SparseMatrix<float> L_p_x = diag_coeff_x - weight_matrix;
    Eigen::SparseMatrix<float> L_p_y = diag_coeff_y - weight_matrix;
    Eigen::SparseMatrix<float> L_p_z = diag_coeff_z - weight_matrix;


    for (size_t i = 0; i < ver_num; ++ i)
    {
        for (size_t j = 0; j < ver_num; ++ j)
        {
            L_[0].coeffRef(row_idx+i, col_idx+j) = L_p_x.coeffRef(i, j);
            L_[1].coeffRef(row_idx+i, col_idx+j) = L_p_y.coeffRef(i, j);
            L_[2].coeffRef(row_idx+i, col_idx+j) = L_p_z.coeffRef(i, j);
        }
    }
}

void DeformModel::updateRightSys(int petal_id)
{
    DeformPetal& deform_petal = deform_petals_[petal_id];
    PetalMatrix& origin_petal = deform_petal._origin_petal;
    CloudMatrix& cloud_matrix = deform_petal._cloud_matrix;
    CorresMatrix& corres_matrix = deform_petal._corres_matrix;
    CovMatrix& cov_matrix = deform_petal._cov_matrix;
    AdjList& adj_list = deform_petal._adj_list;
    WeightMatrix& weight_matrix = deform_petal._weight_matrix;
    RotList& R_list = deform_petal._R_list;
    int ver_num = origin_petal.cols();

    d_.resize(3, d_.cols()+ver_num);

    for (size_t i = 0; i < ver_num; ++i) 
    {
        d_.bottomRightCorner(3, ver_num).col(i) = Eigen::Vector3f::Zero();
        for (size_t j = 0, j_end = adj_list[i].size(); j < j_end; ++j)
        {
            d_.bottomRightCorner(3, ver_num).col(i) += ((weight_matrix.coeffRef(i, adj_list[i][j])/2)*
                (R_list[i]+R_list[adj_list[i][j]])*(origin_petal.col(i) - origin_petal.col(adj_list[i][j]))).transpose();
        }

        Eigen::Vector3f weight_cloud;
        weight_cloud.setZero();
        for (size_t n = 0, n_end = corres_matrix.cols(); n < n_end; ++ n)
            weight_cloud += corres_matrix(i, n)*cloud_matrix.col(i);

        d_.bottomRightCorner(3, ver_num).col(i) += lambda_*2*cov_matrix.asDiagonal().inverse()*weight_cloud;
    }
}

void DeformModel::updateRightSys()
{
    d_.resize(3, 0);

    for (size_t i = 0; i < petal_num_; ++ i)
    {
        updateRightSys(i);
    }
}

float DeformModel::solve()
{
    // solve x, y ,z independently
    for (size_t i = 0; i < 3; ++ i)
    {
        lu_solver_.analyzePattern(L_[i]);
        lu_solver_.factorize(L_[i]);

        Eigen::VectorXf next_pos = lu_solver_.solve(d_.row(i).transpose());
     //   std::cout << next_pos << std::endl;
        int start_idx = 0;

        for (size_t j = 0, j_end = petal_num_; j < j_end; ++ j)
        {
            PetalMatrix& petal_matrix = deform_petals_[j]._petal_matrix;
            int petal_size = petal_matrix.cols();
            petal_matrix.row(i) = next_pos.segment(start_idx, petal_size);

            start_idx += petal_size;
        }
    }

    return energy();
}

void DeformModel::updateRotation(int petal_id)
{
    DeformPetal& deform_petal = deform_petals_[petal_id];
    PetalMatrix& origin_petal = deform_petal._origin_petal;
    PetalMatrix& petal_matrix = deform_petal._petal_matrix;
    RotList& rot_list = deform_petal._R_list;
    AdjList& adj_list = deform_petal._adj_list;
    WeightMatrix& weight_matrix = deform_petal._weight_matrix;

    Eigen::Matrix3f Si;
    Eigen::MatrixXf Di;

    Eigen::Matrix3Xf Pi_Prime;
    Eigen::Matrix3Xf Pi;

    for (size_t i = 0, i_end = rot_list.size(); i < i_end; ++i) 
    {
        Di = Eigen::MatrixXf::Zero(adj_list[i].size(), adj_list[i].size());
        Pi_Prime.resize(3, adj_list[i].size());
        Pi.resize(3, adj_list[i].size());

        for (size_t j = 0, j_end = adj_list[i].size(); j < j_end; ++j) 
        {
            Di(j, j) = weight_matrix.coeffRef(i, adj_list[i][j]);
            Pi.col(j) = origin_petal.col(i) - origin_petal.col(adj_list[i][j]);
            Pi_Prime.col(j) = petal_matrix.col(i) - petal_matrix.col(adj_list[i][j]);
        }
        Si = Pi * Di * Pi_Prime.transpose();
        Eigen::Matrix3f Ui;
        Eigen::Vector3f Wi;
        Eigen::Matrix3f Vi;
        wunderSVD3x3(Si, Ui, Wi, Vi);
        rot_list[i] = Vi * Ui.transpose();

        if (rot_list[i].determinant() < 0)
            std::cout << "determinant is negative!" << std::endl;
    }
}

void DeformModel::updateRotation()
{
    for (size_t i = 0; i < petal_num_; ++ i)
    {
        updateRotation(i);
    }
}

float DeformModel::energy()
{
    float e = 0;

    for (size_t i = 0; i < petal_num_; ++ i)
    {
        DeformPetal& deform_petal = deform_petals_[i];
        CorresMatrix& corres_matrix = deform_petal._corres_matrix;
        CloudMatrix& cloud_matrix = deform_petal._cloud_matrix;
        PetalMatrix& petal_matrix = deform_petal._petal_matrix;
        CovMatrix& cov_matrix = deform_petal._cov_matrix;
        PetalMatrix& origin_petal = deform_petal._origin_petal;
        RotList& rot_list = deform_petal._R_list;
        WeightMatrix& weight_matrix = deform_petal._weight_matrix;
        AdjList& adj_list = deform_petal._adj_list;

        float e1 = 0, e2 = 0;

        for (size_t k = 0, k_end = corres_matrix.rows(); k < k_end; ++ k)
        {

            for (size_t n = 0, n_end = corres_matrix.cols(); n < n_end; ++ n)
            {
                Eigen::Vector3f cm = cloud_matrix.col(n) - petal_matrix.col(k);
                e1 += corres_matrix(k, n) * cm.transpose() * cov_matrix.asDiagonal().inverse() * cm;
            }

            for (size_t j = 0, j_end = adj_list[k].size(); j < j_end; ++ j)
            {
                Eigen::Vector3f mkj = petal_matrix.col(k)-petal_matrix.col(adj_list[k][j]);
                Eigen::Vector3f pre_mkj = origin_petal.col(k)-origin_petal.col(adj_list[k][j]);
                e2 += weight_matrix.coeffRef(k, adj_list[k][j]) * (mkj-rot_list[k]*pre_mkj).squaredNorm();
            }
        }

        e += (e1 + e2);
    }

    return e;
}

void DeformModel::deforming()
{
    Petals& petals = flower_->getPetals();

    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals.at(i);
        PetalMatrix& pm = deform_petals_[i]._petal_matrix;

        for (size_t j = 0, j_end = petal.getVertices()->size(); j < j_end; ++ j)
        {
            petal.getVertices()->at(j).x() = pm(0, j);
            petal.getVertices()->at(j).y() = pm(1, j);
            petal.getVertices()->at(j).z() = pm(2, j);
        }
    }
}