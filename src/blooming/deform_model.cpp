
#include "WunderSVD3x3.h"

#include "point_cloud.h"
#include "flower.h"

#include "deform_model.h"

DeformModel::DeformModel()
    :petal_num_(0),
    iter_num_(10), 
    eps_(1e-2),
    lambda_(1.0),
    noise_p_(0.0)
{

}

DeformModel::DeformModel(PointCloud* point_cloud, Flower* flower)
    :petal_num_(0),
    iter_num_(10), 
    eps_(1e-2),
    lambda_(0.1),
    noise_p_(0.0),
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

void DeformModel::setEps(double eps)
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
    double eps = 1;

    double e = 0;

    std::cout << "Start EM Iteration..." << std::endl;

    do 
    {
        e_step();

        double e_n = m_step();

        eps = std::fabs((e_n - e) / e_n);
        e = e_n;

        std::cout << "In EM Iteration \t" << "iter: " << ++ iter_num << "\tdelta: " << eps << std::endl;

    } while (iter_num < iter_num_ && eps > eps_ );

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

double DeformModel::m_step()
{
    std::cout << "M-Step:" << std::endl;

    int iter = 0;
    double eps = 0;
    
    double e = 0;

    initRotation();
    updateLeftSys();
    updateRightSys();

    do {
        double e_n = solve();
        eps = std::fabs((e_n - e) / e_n);
        e = e_n;

        updateRotation();
        updateRightSys();

    }while(eps > eps_ && iter < iter_num_);

    return e;
}

void DeformModel::e_step(int petal_id)
{
    CorresMatrix& corres_mat = deform_petals_[petal_id]._corres_matrix;
    VisList& vis_list = deform_petals_[petal_id]._vis_list;

    for (size_t i = 0, i_end = corres_mat.cols(); i < i_end; ++ i)
    {
        for (size_t j = 0, j_end = corres_mat.rows(); j < j_end; ++ j)
        {
            corres_mat(j, i) = gaussian(petal_id, j, i) /** vis_list[j]*/;
        }
    }

    for (size_t i = 0, i_end = corres_mat.cols(); i < i_end; ++ i)
    {
        double sum_gaussian = corres_mat.col(i).sum() + noise_p_;

        for (size_t j = 0, j_end = corres_mat.rows(); j < j_end; ++ j)
        {
            corres_mat(j, i) = corres_mat(j, i) / zero_correction(sum_gaussian);
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

                s_x += pow((c[0] - v[0]), 2.0);
                s_y += pow((c[1] - v[1]), 2.0);
                s_z += pow((c[2] - v[2]), 2.0);
            }

            cov_matrix.col(k) << s_x / adj_size, s_y / adj_size, s_z / adj_size; 
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


double DeformModel::gaussian(int petal_id, int m_id, int c_id)
{
    double p;

    CovMatrix& cov_mat = deform_petals_[petal_id]._cov_matrix;
    CloudMatrix& cloud_mat = deform_petals_[petal_id]._cloud_matrix;
    PetalMatrix& petal_mat = deform_petals_[petal_id]._petal_matrix;

    Eigen::Vector3d xu = cloud_mat.col(c_id) - petal_mat.col(m_id);
    p = pow(2*M_PI, -3/2.0) * pow((cov_mat.col(m_id).asDiagonal()).toDenseMatrix().determinant(), -1/2.0) * 
        exp((-1/2.0)*xu.transpose()*cov_mat.col(m_id).asDiagonal().inverse()*xu);

    return p;
}

void DeformModel::buildWeightMatrix(int petal_id)
{
    std::vector<Eigen::Triplet<double> > weight_list;
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

            double wij = 0;
            if (share_vertex.size()==2) wij = deform_petal.wij(i, id_j, share_vertex[0], share_vertex[1]);
            else wij = deform_petal.wij(i, id_j, share_vertex[0]);

            weight_list.push_back(Eigen::Triplet<double>(i, id_j, wij));
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
    AdjList& adj_list = deform_petal._adj_list;
    HardCtrsIdx& hc_idx = deform_petal._hc_idx;
    int ver_num = deform_petal._petal_matrix.cols();

    std::vector<std::vector<Eigen::Triplet<double> > > weight_sums;
    weight_sums.resize(3); // for x,y,z coordinates

    for (int i = 0; i < ver_num; ++i) 
    {
        double wi_x = 0, wi_y = 0, wi_z = 0;

        for (size_t j = 0, j_end = deform_petal._adj_list[i].size(); j < j_end; ++j)
        {
            int id_j = deform_petal._adj_list[i][j];
            wi_x += weight_matrix.coeffRef(i, id_j);
            wi_y += weight_matrix.coeffRef(i, id_j);
            wi_z += weight_matrix.coeffRef(i, id_j);
        }

        wi_x += zero_correction(lambda_*(2/cov_matrix.col(i)[0])*corres_matrix.row(i).sum());
        wi_y += zero_correction(lambda_*(2/cov_matrix.col(i)[1])*corres_matrix.row(i).sum());
        wi_z += zero_correction(lambda_*(2/cov_matrix.col(i)[2])*corres_matrix.row(i).sum());
        
        weight_sums[0].push_back(Eigen::Triplet<double>(i, i, wi_x));
        weight_sums[1].push_back(Eigen::Triplet<double>(i, i, wi_y));
        weight_sums[2].push_back(Eigen::Triplet<double>(i, i, wi_z));
    }

    Eigen::SparseMatrix<double> diag_coeff_x(ver_num, ver_num);
    Eigen::SparseMatrix<double> diag_coeff_y(ver_num, ver_num);
    Eigen::SparseMatrix<double> diag_coeff_z(ver_num, ver_num);
    diag_coeff_x.setFromTriplets(weight_sums[0].begin(), weight_sums[0].end());
    diag_coeff_y.setFromTriplets(weight_sums[1].begin(), weight_sums[1].end());
    diag_coeff_z.setFromTriplets(weight_sums[2].begin(), weight_sums[2].end());

    // expand L to fill in new petal vertices
    int row_idx = L_[0].rows(), col_idx = L_[0].cols();  // rows and cols should be the same for x, y, z

    L_[0].conservativeResize(row_idx+ver_num, col_idx+ver_num);
    L_[1].conservativeResize(row_idx+ver_num, col_idx+ver_num);
    L_[2].conservativeResize(row_idx+ver_num, col_idx+ver_num);

    // block assignment is not supported by SparseMatrix...I have to update one by one
    Eigen::SparseMatrix<double> L_p_x = diag_coeff_x - weight_matrix;
    Eigen::SparseMatrix<double> L_p_y = diag_coeff_y - weight_matrix;
    Eigen::SparseMatrix<double> L_p_z = diag_coeff_z - weight_matrix;


    for (size_t i = 0; i < ver_num; ++ i)
    {
        int hc_id = deform_petal.isHardCtrs(i);

        if ( hc_id != -1)
        {
            L_[0].coeffRef(row_idx+i, col_idx+i) = 1;
            L_[1].coeffRef(row_idx+i, col_idx+i) = 1;
            L_[2].coeffRef(row_idx+i, col_idx+i) = 1;
        }
        else
        {
            L_[0].coeffRef(row_idx+i, col_idx+i) = L_p_x.coeffRef(i, i);
            L_[1].coeffRef(row_idx+i, col_idx+i) = L_p_y.coeffRef(i, i);
            L_[2].coeffRef(row_idx+i, col_idx+i) = L_p_z.coeffRef(i, i);

            for (size_t j = 0, j_end = adj_list[i].size(); j < j_end; ++ j)
            {
                int id_j = adj_list[i][j];
                L_[0].coeffRef(row_idx+i, col_idx+id_j) = L_p_x.coeffRef(i, id_j);
                L_[1].coeffRef(row_idx+i, col_idx+id_j) = L_p_y.coeffRef(i, id_j);
                L_[2].coeffRef(row_idx+i, col_idx+id_j) = L_p_z.coeffRef(i, id_j);
            }
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

    d_.conservativeResize(3, d_.cols()+ver_num);

    for (size_t i = 0; i < ver_num; ++i) 
    {
        int hc_id = deform_petal.isHardCtrs(i);

        if ( hc_id != -1)
            d_.bottomRightCorner(3, ver_num).col(i) << origin_petal.col(i);
        else
        {
            d_.bottomRightCorner(3, ver_num).col(i) = Eigen::Vector3d::Zero();
            for (size_t j = 0, j_end = adj_list[i].size(); j < j_end; ++j)
            {
                d_.bottomRightCorner(3, ver_num).col(i) += ((weight_matrix.coeffRef(i, adj_list[i][j])/2)*
                    (R_list[i]+R_list[adj_list[i][j]])*(origin_petal.col(i) - origin_petal.col(adj_list[i][j]))).transpose();
            }

            Eigen::Vector3d weight_cloud;
            weight_cloud.setZero();
            for (size_t n = 0, n_end = corres_matrix.cols(); n < n_end; ++ n)
            {
                weight_cloud += corres_matrix(i, n)*cloud_matrix.col(n);
            }

            d_.bottomRightCorner(3, ver_num).col(i) += lambda_*2*cov_matrix.col(i).asDiagonal().inverse()*weight_cloud;
        }
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

double DeformModel::solve()
{
    // solve x, y ,z independently
    for (size_t i = 0; i < 3; ++ i)
    {
        lu_solver_.analyzePattern(L_[i]);
        lu_solver_.factorize(L_[i]);

        Eigen::VectorXd next_pos = lu_solver_.solve(d_.row(i).transpose());

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
    Eigen::MatrixXd Di;

    Eigen::Matrix3Xd Pi_Prime;
    Eigen::Matrix3Xd Pi;

    for (size_t i = 0, i_end = rot_list.size(); i < i_end; ++i) 
    {
        Di = Eigen::MatrixXd::Zero(adj_list[i].size(), adj_list[i].size());
        Pi_Prime.resize(3, adj_list[i].size());
        Pi.resize(3, adj_list[i].size());

        for (size_t j = 0, j_end = adj_list[i].size(); j < j_end; ++j) 
        {
            Di(j, j) = weight_matrix.coeffRef(i, adj_list[i][j]);
            Pi.col(j) = origin_petal.col(i) - origin_petal.col(adj_list[i][j]);
            Pi_Prime.col(j) = petal_matrix.col(i) - petal_matrix.col(adj_list[i][j]);
        }
        Si = Pi.cast<float>() * Di.cast<float>() * Pi_Prime.transpose().cast<float>();
        Eigen::Matrix3f Ui;
        Eigen::Vector3f Wi;
        Eigen::Matrix3f Vi;
        wunderSVD3x3(Si, Ui, Wi, Vi);
        rot_list[i] = Vi.cast<double>() * Ui.transpose().cast<double>();

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

double DeformModel::energy()
{
    double e = 0;

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

        double e1 = 0, e2 = 0;

        for (size_t k = 0, k_end = corres_matrix.rows(); k < k_end; ++ k)
        {

            for (size_t n = 0, n_end = corres_matrix.cols(); n < n_end; ++ n)
            {
                Eigen::Vector3d cm = cloud_matrix.col(n) - petal_matrix.col(k);
                e1 += corres_matrix(k, n) * cm.transpose() * cov_matrix.col(k).asDiagonal().inverse() * cm;
            }

            for (size_t j = 0, j_end = adj_list[k].size(); j < j_end; ++ j)
            {
                Eigen::Vector3d mkj = petal_matrix.col(k)-petal_matrix.col(adj_list[k][j]);
                Eigen::Vector3d pre_mkj = origin_petal.col(k)-origin_petal.col(adj_list[k][j]);
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

void DeformModel::initRotation()
{
    // init rotation matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        DeformPetal& deform_petal = deform_petals_[i];
        PetalMatrix& origin_petal = deform_petal._origin_petal;
        RotList R_list;

        for (size_t j = 0, j_end = origin_petal.cols(); j < j_end; ++ j)
        {
            R_list.push_back(Eigen::Matrix3d::Identity());
        }

        deform_petal._R_list = R_list;
    }
}

double DeformModel::zero_correction(double value)
{
    double min_double = std::numeric_limits<double>::min();
    if (min_double > value)
        return min_double;
    return value;
}
