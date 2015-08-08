#include "flower.h"
#include "point_cloud.h"
#include "solver.h"

int Solver::iter_num_ = 30;
double Solver::eps_ = 1e-3;
double Solver::lambda_data_fitting_ = 0.01;
double Solver::lambda_skel_smooth_ = 0.0;
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

    A_ = std::vector<std::vector<Eigen::MatrixXd>>(petal_num_, std::vector<Eigen::MatrixXd>(3));
    b_.resize(petal_num_);

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

    // init key region indices
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        KeyRegionIndices& kri = deform_petals_[i]._region_indices;
        kri = point_cloud_->getFittingMesh(i);
    }

    // init cloud matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        osg::ref_ptr<PointCloud> petal_cloud = point_cloud_->getFittingCloud(i);
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
        int ver_num = deform_petal._origin_petal.cols();

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
        Petal& petal = petals.at(i);
        CovMatrix& cov_matrix = deform_petals_[i]._cov_matrix;
        cov_matrix = petal.getGaussianSphere();
    }

    // init petal matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        PetalMatrix& petal_matrix = deform_petals_[i]._petal_matrix;
        petal_matrix = deform_petals_[i]._origin_petal;

        KeyRegionIndices& kri = deform_petals_[i]._region_indices;
        Petal& petal = petals.at(i);
        osg::ref_ptr<PointCloud> petal_cloud = point_cloud_->getFittingCloud(i);
        std::vector<int> knn_idx;
        petal.searchNearestIdx(petal_cloud, kri, knn_idx);

        for (size_t j = 0, j_end = kri.size(); j < j_end; ++ j)
        {
            petal_matrix.col(kri[j]) << 
                petal_cloud->at(knn_idx[j]).x, petal_cloud->at(knn_idx[j]).y, petal_cloud->at(knn_idx[j]).z;
        }
    }

    // init correspondence matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        CloudMatrix& cloud_mat = deform_petals_[i]._cloud_matrix;
        //PetalMatrix& petal_mat = deform_petals_[i]._petal_matrix;
        KeyRegionIndices& region_indices = deform_petals_[i]._region_indices;
        CorresMatrix corres_mat = CorresMatrix::Zero(region_indices.size(), cloud_mat.cols());
        deform_petals_[i]._corres_matrix = corres_mat;
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
        ConvertAffineMatrix& convert_affine = deform_petals_[i]._convert_affine;

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
        convert_affine = M;

        AffineMatrix& affine_matrix = deform_petals_[i]._affine_matrix;
        affine_matrix.resize(4*hdl_num, 3);
    }

    // init handle matrix and branch size
    for (size_t i = 0; i < petal_num_; ++ i)
    {
        Petal& petal = petals.at(i);
        osg::ref_ptr<Skeleton> skeleton = petal.getSkeleton();
        Skeleton::Joints joints = skeleton->getJoints();
        Skeleton::Branches branches = skeleton->getBranches();
        int joint_number = skeleton->getJointNumber();

        // form handle matrix by joints
        HandleMatrix& handle_matrix = deform_petals_[i]._handle_matrix;
        handle_matrix.resize(joint_number, 3);
        for (size_t j = 0; j < joint_number; ++ j)
        {
            handle_matrix.row(j) << joints[j].x, joints[j].y, joints[j].z;
        }

        // store branch sizes
        BranchList& branch_list = deform_petals_[i]._branch_list;
        for (auto& branch : branches)
            branch_list.push_back(branch);
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

    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        skel_term_.push_back(SkelSmoothTerm(i));
    }
}

double Solver::solve(int petal_id)
{
    // solve T
    for (size_t i = 0; i < 3; ++ i)
    {
        Eigen::MatrixXd A = A_[petal_id][i];
        Eigen::VectorXd b = b_[petal_id].row(i).transpose();

        Eigen::VectorXd next_values = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        // new T
        AffineMatrix& affine_matrix = deform_petals_[petal_id]._affine_matrix;
        affine_matrix.col(i) = next_values;
    }

    // update vertices
    lbs(petal_id);

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
    osg::ref_ptr<Skeleton> skeleton = petal.getSkeleton();
    Skeleton::Joints& joints = skeleton->getJoints();
    for (size_t i = 0; i < skeleton->getJointNumber(); ++ i)
    {
        auto& joint = joints[i];
        Eigen::MatrixXd T = am.block<4,3>(i*4,0);
        Eigen::Vector3d np = T.transpose() * Eigen::Vector4d(joint.x, joint.y, joint.z, 1);
        joint.x = np(0);
        joint.y = np(1);
        joint.z = np(2);
    }
}

void Solver::lbs(int petal_id)
{
    AffineMatrix& am = deform_petals_[petal_id]._affine_matrix;
    PetalMatrix& pm = deform_petals_[petal_id]._petal_matrix;
    ConvertAffineMatrix& cam = deform_petals_[petal_id]._convert_affine;

    pm = (cam * am).transpose();
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
    skel_term_[petal_id].build();
}

void Solver::left_sys(int petal_id)
{
    for (int i = 0; i < 3; ++ i)
    {
        A_[petal_id][i] = data_term_[petal_id].A()[i] + arap_term_[petal_id].A()[i] + skel_term_[petal_id].A()[i];
    }
}

void Solver::right_sys(int petal_id)
{
    b_[petal_id] = data_term_[petal_id].b() + arap_term_[petal_id].b() + skel_term_[petal_id].b();
}

void Solver::projection(int petal_id)
{
    data_term_[petal_id].projection();
    arap_term_[petal_id].projection();
    skel_term_[petal_id].projection();
}

void Solver::update(int petal_id)
{
    data_term_[petal_id].update();
    arap_term_[petal_id].update();
    skel_term_[petal_id].update();
}

double Solver::gaussian(int petal_id, int m_id, int c_id)
{
    double p;

    CovMatrix& cov_mat = deform_petals_[petal_id]._cov_matrix;
    CloudMatrix& cloud_mat = deform_petals_[petal_id]._cloud_matrix;
    PetalMatrix& petal_mat = deform_petals_[petal_id]._petal_matrix;
    KeyRegionIndices& kri = deform_petals_[petal_id]._region_indices;

    Eigen::Vector3d xu = cloud_mat.col(c_id) - petal_mat.col(kri[m_id]);
    p = pow(2*M_PI, -3/2.0) * pow((cov_mat.col(kri[m_id]).asDiagonal()).toDenseMatrix().determinant(), -1/2.0) * 
        exp((-1/2.0)*xu.transpose()*cov_mat.col(kri[m_id]).asDiagonal().inverse()*xu);

    return p;
}