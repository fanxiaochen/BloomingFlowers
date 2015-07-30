#include "flower.h"
#include "point_cloud.h"
#include "solver.h"

int Solver::iter_num_ = 30;
double Solver::eps_ = 1e-3;
double Solver::lambda_ = 0.05;
double Solver::noise_p_ = 0.0;
std::vector<Solver::DeformPetal> Solver::deform_petals_;

Solver::Solver(PointCloud* point_cloud, Flower* flower)
{
    point_cloud_ = point_cloud;
    flower_ = flower;
   
    init();
}

void Solver::setFlower(Flower* flower)
{
    flower_ = flower;
}

void Solver::setPointCloud(PointCloud* point_cloud)
{
    point_cloud_ = point_cloud;
}

void Solver::init()
{
    initParas();
    initTerms();
}

void Solver::initParas()
{
    Petals& petals = flower_->getPetals();
    petal_num_ = petals.size();

    L_ = std::vector<std::vector<Eigen::SparseMatrix<double>>>(petal_num_, std::vector<Eigen::SparseMatrix<double>>(3));
    b_.resize(petal_num_);
    M_.resize(petal_num_);

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

    // init biharmonic weights
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals.at(i);
        deform_petals_[i]._biweight_matrix = petal.getBiharmonicWeights();
    }

    // init M matrix and T matrix(affine transformation)
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        BiWeightMatrix& biweight_matrix = deform_petals_[i]._biweight_matrix;
        PetalMatrix& original_matrix = deform_petals_[i]._origin_petal;

        int ver_num = original_matrix.cols();
        int hdl_num = biweight_matrix.cols();
        Eigen::MatrixXd M(ver_num, 4*hdl_num);

        // convert original matrix to homogeneous coordinates
        Eigen::MatrixXd petal_matrix(ver_num, 4);
        petal_matrix.setOnes();
        petal_matrix.block(0, 0, ver_num, 3) = original_matrix.transpose();
        
        for (size_t j = 0, j_end = hdl_num; j < j_end; ++ j)
        {
            M.block(0, 4*j, ver_num, 4) = biweight_matrix.col(j).asDiagonal() * petal_matrix;
        }
        M_[i] = M;

        AffineMatrix& affine_matrix = deform_petals_[i]._affine_matrix;
        affine_matrix.resize(4*hdl_num, 3);
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

double Solver::solve(int petal_id)
{
    // solve T
    for (size_t i = 0; i < 3; ++ i)
    {
        Eigen::MatrixXd A = M_[petal_id].transpose() * (L_[petal_id][i] * M_[petal_id]);
        Eigen::VectorXd b = M_[petal_id].transpose() * b_[petal_id].row(i).transpose();
        
        Eigen::VectorXd next_values = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        // new T
        AffineMatrix& affine_matrix = deform_petals_[petal_id]._affine_matrix;
        affine_matrix.col(i) = next_values;
    }

    // update vertices
    lbs(petal_id);

    //// solve x, y ,z independently
    //for (size_t i = 0; i < 3; ++ i)
    //{
    //    lu_solver_.analyzePattern(L_[petal_id][i]);
    //    lu_solver_.factorize(L_[petal_id][i]);

    //    Eigen::VectorXd next_pos = lu_solver_.solve(b_[petal_id].row(i).transpose());

    //    PetalMatrix& petal_matrix = deform_petals_[petal_id]._petal_matrix;
    //    petal_matrix.row(i) = next_pos;
    //}

    return energy(petal_id);
}

double Solver::energy(int petal_id)
{
    DeformPetal& deform_petal = deform_petals_[petal_id];
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

    return (e1 + e2);
}

void Solver::e_step(int petal_id)
{
    std::cout << "E-Step:" << std::endl;

    CorresMatrix& corres_mat = deform_petals_[petal_id]._corres_matrix;
    WeightList& weight_list = deform_petals_[petal_id]._weight_list;

    for (size_t i = 0, i_end = corres_mat.cols(); i < i_end; ++ i)
    {
        for (size_t j = 0, j_end = corres_mat.rows(); j < j_end; ++ j)
        {
            corres_mat(j, i) = gaussian(petal_id, j, i) /** weight_list[j]*/;
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

double Solver::m_step(int petal_id)
{
    std::cout << "M-Step:" << std::endl;

    int iter = 0;
    double eps = 0;

    double e = 0;

    initbuild(petal_id);
    left_sys(petal_id);
    right_sys(petal_id);

    do {
        double e_n = solve(petal_id);
        eps = std::fabs((e_n - e) / e_n);
        e = e_n;

        projection(petal_id);
        update(petal_id);
        right_sys(petal_id); // the update only effects right side 

    }while(eps > eps_ && iter < iter_num_);

    return e;
}

void Solver::deform()
{
    std::cout << "Start Deforming..." << std::endl;
    for (size_t i = 0; i < petal_num_; ++ i)
    {
        std::cout << "Petal " << i << std::endl;
        deform(i);
    }
}

void Solver::deform(int petal_id)
{

    int iter_num = 0;
    double eps = 1;

    double e = 0;

    std::cout << "Start EM Iteration..." << std::endl;

    do 
    {
        e_step(petal_id);

        double e_n = m_step(petal_id);

        eps = std::fabs((e_n - e) / e_n);
        e = e_n;

        std::cout << "In EM Iteration \t" << "iter: " << ++ iter_num << "\tdelta: " << eps << std::endl;

    } while (iter_num < iter_num_ && eps > eps_ );

    // flower deformed
    deforming(petal_id);
}

void Solver::deforming(int petal_id)
{
    // vertices
    Petals& petals = flower_->getPetals();
    Petal& petal = petals.at(petal_id);
    PetalMatrix& pm = deform_petals_[petal_id]._petal_matrix;

    for (size_t j = 0, j_end = petal.getVertices()->size(); j < j_end; ++ j)
    {
        petal.getVertices()->at(j).x() = pm(0, j);
        petal.getVertices()->at(j).y() = pm(1, j);
        petal.getVertices()->at(j).z() = pm(2, j);
    }

    petal.updateNormals();

    // handles
    AffineMatrix& am = deform_petals_[petal_id]._affine_matrix;
    int handle_count = 0;
    osg::ref_ptr<Skeleton> skeleton = petal.getSkeleton();
    for (size_t i = 0; i < skeleton->getBranches().size(); ++ i)
    {
        Skeleton::Branch& branch = skeleton->getBranch(i);
        for (size_t j = 0; j < branch.size(); ++ j)
        {
            Point& p = branch.at(j);
            Eigen::MatrixXd T = am.block<4,3>(handle_count*4,0);
            Eigen::Vector3d np = T.transpose() * Eigen::Vector4d(p.x, p.y, p.z, 1);
            p.x = np(0);
            p.y = np(1);
            p.z = np(2);

            handle_count ++;
        }
    }
}

void Solver::lbs(int petal_id)
{
    AffineMatrix& am = deform_petals_[petal_id]._affine_matrix;
    PetalMatrix& pm = deform_petals_[petal_id]._petal_matrix;

    pm = (M_[petal_id] * am).transpose();
}

double Solver::zero_correction(double value)
{
    double min_double = std::numeric_limits<double>::min();
    if (min_double > value)
        return min_double;
    return value;
}

void Solver::initbuild(int petal_id)
{
    data_term_[petal_id].build();
    arap_term_[petal_id].build();
}

void Solver::left_sys(int petal_id)
{
    for (int i = 0; i < 3; ++ i)
    {
        L_[petal_id][i] = data_term_[petal_id].A()[i] + arap_term_[petal_id].A()[i];
        L_[petal_id][i].makeCompressed();
    }
}

void Solver::right_sys(int petal_id)
{
    b_[petal_id] = data_term_[petal_id].b() + arap_term_[petal_id].b();
}

void Solver::projection(int petal_id)
{
    data_term_[petal_id].projection();
    arap_term_[petal_id].projection();
}

void Solver::update(int petal_id)
{
    data_term_[petal_id].update();
    arap_term_[petal_id].update();
}

double Solver::gaussian(int petal_id, int m_id, int c_id)
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