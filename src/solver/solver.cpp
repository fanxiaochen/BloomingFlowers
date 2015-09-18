#include "flower.h"
#include "point_cloud.h"
#include "solver.h"
#include "main_window.h"
#include "parameters.h"

int Solver::iter_num_ = 50;
double Solver::eps_ = 1e-3;
double Solver::lambda_tip_fitting_ = 0.02;
double Solver::lambda_boundary_fitting_ = 0.02;
double Solver::lambda_inner_fitting_ = 0.01;
double Solver::lambda_skel_smooth_ = 0;
double Solver::lambda_collision_ = 1;
double Solver::lambda_arap_ = 1;
double Solver::lambda_closure_ = 1;
double Solver::noise_p_ = 0.0;
std::string Solver::closure_ids_ = "";
int Solver::closure_start_frame_ = 20;
bool Solver::is_forward_ = true;
std::vector<Solver::DeformPetal> Solver::deform_petals_;
std::vector<CollidingPoint> Solver::colliding_points_;

bool Solver::has_point_cloud_ = true;

PointCloud* Solver::closure_cloud_ = nullptr;

int Solver::cloud_frame_ = 0;

Solver::Solver(PointCloud* point_cloud, Flower* flower, int cloud_frame)
{
    point_cloud_ = point_cloud;
    flower_ = flower;
    cloud_frame_ = cloud_frame;

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
    Petals& petals = flower_->getPetals();
    petal_num_ = petals.size();

    A_ = std::vector<std::vector<Eigen::MatrixXd>>(petal_num_, std::vector<Eigen::MatrixXd>(3));
    b_.resize(petal_num_);

    FA_.resize(3);

    deform_petals_.resize(petal_num_);


    // init solver parameters
    iter_num_ = MainWindow::getInstance()->getParameters()->getIterNum();
    eps_ = MainWindow::getInstance()->getParameters()->getEps();
    lambda_tip_fitting_ = MainWindow::getInstance()->getParameters()->getTipFitting();
    lambda_boundary_fitting_ = MainWindow::getInstance()->getParameters()->getBoundaryFitting();
    lambda_inner_fitting_ = MainWindow::getInstance()->getParameters()->getInnerFitting();
    lambda_skel_smooth_ = MainWindow::getInstance()->getParameters()->getSkelSmooth();
    lambda_collision_ = MainWindow::getInstance()->getParameters()->getCollision();
    lambda_arap_ = MainWindow::getInstance()->getParameters()->getARAP();
    lambda_closure_ = MainWindow::getInstance()->getParameters()->getClosure();
    noise_p_ = MainWindow::getInstance()->getParameters()->getNoiseP();
    closure_ids_ = MainWindow::getInstance()->getParameters()->getClosureIds();
    closure_start_frame_ = MainWindow::getInstance()->getParameters()->getClosureStartFrame();

    flower_->setPetalRelation(MainWindow::getInstance()->getParameters()->getPetalRelation());

    closure_cloud_ = point_cloud_->getClosureCloud();
    if (closure_cloud_ != nullptr)
        closure_cloud_->buildSelfKdtree();
}

void Solver::boundary_inner_setting()
{
    initMeshParas();
    std::cout << "finish mesh initialization" << std::endl;

    initSkelParas();
    std::cout << "finish skeleton initialization" << std::endl;

    initFittingParas_later_stage();
    std::cout << "finish data fitting initialization" << std::endl;

    initTerms();

    std::cout << "finish solver initialization" << std::endl;
}

void Solver::trajectory_guided_setting()
{
    initMeshParas();
    std::cout << "finish mesh initialization" << std::endl;

    initSkelParas();
    std::cout << "finish skeleton initialization" << std::endl;

    initFittingParas_early_stage();
    std::cout << "finish data fitting initialization" << std::endl;

    initTerms();

    std::cout << "finish solver initialization" << std::endl;
}

void Solver::init_setting()
{
    if (has_point_cloud_)
    {
        boundary_inner_setting();
    }
    else
        trajectory_guided_setting();
}

void Solver::initFittingParas_early_stage()
{
    Petals& petals = flower_->getPetals();

    // init inner matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        osg::ref_ptr<PointCloud> petal_cloud = point_cloud_->getSamplingPetalCloud(i, 10);

        if (petal_cloud != NULL)
        {
            CloudMatrix cm(3, petal_cloud->size());

            for (size_t j = 0, j_end = petal_cloud->size(); j < j_end; ++ j)
            {
                cm.col(j) << petal_cloud->at(j).x, petal_cloud->at(j).y, petal_cloud->at(j).z;
            }

            deform_petals_[i]._inner_matrix = cm;
        }
        else deform_petals_[i]._inner_matrix = deform_petals_[i]._origin_petal;
    }

    // init petal matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        deform_petals_[i]._petal_matrix = deform_petals_[i]._origin_petal;
    }

    // init inner correspondence matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        CloudMatrix& cloud_mat = deform_petals_[i]._inner_matrix;
        PetalMatrix& petal_mat = deform_petals_[i]._petal_matrix;
        CorresMatrix corres_mat = CorresMatrix::Zero(petal_mat.cols(), cloud_mat.cols());
        deform_petals_[i]._inner_corres = corres_mat;
    }

    // init inner visible parts
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals[i];
        PetalMatrix& petal_mat = deform_petals_[i]._origin_petal;
        VisList& vis_list = deform_petals_[i]._inner_vis;
        vis_list = std::vector<int>(petal_mat.cols(), 1);
    }

    // init inner weight list
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals.at(i);
        PetalMatrix& petal_mat = deform_petals_[i]._origin_petal;
        deform_petals_[i]._inner_weights = std::vector<double>(petal_mat.cols(), 1.0 / petal_mat.cols());
    }
}

void Solver::initFittingParas_later_stage()
{
    Petals& petals = flower_->getPetals();

    // init cloud matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        osg::ref_ptr<PointCloud> petal_cloud = point_cloud_->getPetalCloud(i);
        if (petal_cloud != NULL)
        {
            CloudMatrix cm(3, petal_cloud->size());

            for (size_t j = 0, j_end = petal_cloud->size(); j < j_end; ++ j)
            {
                cm.col(j) << petal_cloud->at(j).x, petal_cloud->at(j).y, petal_cloud->at(j).z;
            }

            deform_petals_[i]._cloud_matrix = cm;
        }
        else deform_petals_[i]._cloud_matrix = deform_petals_[i]._origin_petal;
    }


    // init inner matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        osg::ref_ptr<PointCloud> petal_cloud = point_cloud_->getSamplingPetalCloud(i, 10);
        
        if (petal_cloud != NULL)
        {
            CloudMatrix cm(3, petal_cloud->size());
            for (size_t j = 0, j_end = petal_cloud->size(); j < j_end; ++ j)
            {
                cm.col(j) << petal_cloud->at(j).x, petal_cloud->at(j).y, petal_cloud->at(j).z;
            }
            deform_petals_[i]._inner_matrix = cm;
        }

        else deform_petals_[i]._inner_matrix = deform_petals_[i]._origin_petal;
    }

    // init petal matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        deform_petals_[i]._petal_matrix = deform_petals_[i]._origin_petal;
    }

    // init inner correspondence matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        CloudMatrix& cloud_mat = deform_petals_[i]._inner_matrix;
        PetalMatrix& petal_mat = deform_petals_[i]._petal_matrix;
        CorresMatrix corres_mat = CorresMatrix::Zero(petal_mat.cols(), cloud_mat.cols());
        deform_petals_[i]._inner_corres = corres_mat;
    }

    // init inner visible parts
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals[i];
        PetalMatrix& petal_mat = deform_petals_[i]._origin_petal;
        VisList& vis_list = deform_petals_[i]._inner_vis;
        vis_list = std::vector<int>(petal_mat.cols(), 1);
    }

    // init inner weight list
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals.at(i);
        deform_petals_[i]._inner_weights = petal.getWeights();
    }

   
    // init boundary cloud
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        osg::ref_ptr<PointCloud> boundary_cloud = point_cloud_->getBoundary(i);
        
        if (boundary_cloud != NULL)
        {
            CloudMatrix cm(3, boundary_cloud->size());

            for (size_t j = 0, j_end = boundary_cloud->size(); j < j_end; ++ j)
            {
                cm.col(j) << boundary_cloud->at(j).x, boundary_cloud->at(j).y, boundary_cloud->at(j).z;
            }

            deform_petals_[i]._boundary_cloud = cm;
        }

        // boundary cloud couldn't be empty here
    }

    // init boundary correspondence matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        CloudMatrix& boundary_cloud = deform_petals_[i]._boundary_cloud;
        PetalMatrix& petal_mat = deform_petals_[i]._petal_matrix;

        CorresMatrix corres_mat = CorresMatrix::Zero(petal_mat.cols(), boundary_cloud.cols());
        deform_petals_[i]._boundary_corres = corres_mat;
    }

    // init boundary visible parts
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals[i];
        PetalMatrix& petal_mat = deform_petals_[i]._origin_petal;
        VisList& vis_list = deform_petals_[i]._boundary_vis;
        std::vector<int> detected_boundary = petal.getDetectedBoundary();
        vis_list = std::vector<int>(petal_mat.cols(), 0);
        for (int index : detected_boundary)
            vis_list[index] = 1;
    }

    // init boundary weight list
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals.at(i);
        deform_petals_[i]._boundary_weights = petal.getWeights();
    }


    // init tip cloud
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        osg::ref_ptr<PointCloud> tip_cloud = point_cloud_->getTips(i);

        if (tip_cloud != NULL)
        {
            CloudMatrix cm(3, tip_cloud->size());

            for (size_t j = 0, j_end = tip_cloud->size(); j < j_end; ++ j)
            {
                cm.col(j) << tip_cloud->at(j).x, tip_cloud->at(j).y, tip_cloud->at(j).z;
            }

            deform_petals_[i]._tip_cloud = cm;
        }

        // tip cloud couldn't be empty here
    }



    // init tip correspondence matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        CloudMatrix& tip_cloud = deform_petals_[i]._tip_cloud;
        PetalMatrix& petal_mat = deform_petals_[i]._petal_matrix;

        CorresMatrix corres_mat = CorresMatrix::Zero(petal_mat.cols(), tip_cloud.cols());
        deform_petals_[i]._tip_corres = corres_mat;
    }



    // init tip visible parts
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals[i];
        PetalMatrix& petal_mat = deform_petals_[i]._origin_petal;
        VisList& vis_list = deform_petals_[i]._tip_vis;
        std::vector<int> detected_tip = petal.getDetectedTips();
        vis_list = std::vector<int>(petal_mat.cols(), 0);
        for (int index : detected_tip)
            vis_list[index] = 1;
    }



    // init tip weight list
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals.at(i);
        deform_petals_[i]._tip_weights = petal.getWeights();
    }
    

}

void Solver::initMeshParas()
{
    Petals& petals = flower_->getPetals();

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
        face_list.clear();
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

    // init hard constraints
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals[i];
        HardCtrsIdx& hc_idx = deform_petals_[i]._hc_idx;
        hc_idx = petal.getHardCtrsIndex();
        std::sort(hc_idx.begin(), hc_idx.end());
    }

}

void Solver::initSkelParas()
{
    Petals& petals = flower_->getPetals();

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
        branch_list.clear();
        for (auto& branch : branches)
            branch_list.push_back(branch);
    }
}

void Solver::initTerms()
{
    if (has_point_cloud_)
    {
        for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
        {
            boundary_term_.push_back(BoundaryFittingTerm(i));
        }

        for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
        {
            tip_term_.push_back(TipFittingTerm(i));
        }
    }

    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        inner_term_.push_back(DataFittingTerm(i));
    }

    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        arap_term_.push_back(ARAPTerm(i));
    }

    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        skel_term_.push_back(SkelSmoothTerm(i));
    }

    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        collision_term_.push_back(CollisionDetectionTerm(i));
    }

    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        closure_term_.push_back(ClosureTerm(i));
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

// need to be considered
double Solver::energy(int petal_id)
{
    DeformPetal& deform_petal = deform_petals_[petal_id];
    CorresMatrix& tip_corres = deform_petal._tip_corres;
    CorresMatrix& boundary_corres = deform_petal._boundary_corres;
    CorresMatrix& inner_corres = deform_petal._inner_corres;
    CloudMatrix& tip_cloud = deform_petal._tip_cloud;
    CloudMatrix& boundary_cloud = deform_petal._boundary_cloud;
    CloudMatrix& inner_cloud = deform_petal._inner_matrix;
    PetalMatrix& petal_matrix = deform_petal._petal_matrix;
    CovMatrix& cov_matrix = deform_petal._cov_matrix;
    PetalMatrix& origin_petal = deform_petal._origin_petal;
    RotList& rot_list = deform_petal._R_list;
    WeightMatrix& weight_matrix = deform_petal._weight_matrix;
    AdjList& adj_list = deform_petal._adj_list;
    IntersectList& intersect_list = deform_petal._intersect_list;


    double e1 = 0, e2 = 0, e3 = 0, e4 = 0, e5 = 0;

    for (size_t k = 0, k_end = petal_matrix.cols(); k < k_end; ++ k)
    {
        if (has_point_cloud_)
        {
            for (size_t n = 0, n_end = boundary_corres.cols(); n < n_end; ++ n)
            {
                Eigen::Vector3d cm = boundary_cloud.col(n) - petal_matrix.col(k);
                e1 += boundary_corres(k, n) * cm.transpose() * cov_matrix.col(k).asDiagonal().inverse() * cm;
            }

            for (size_t n = 0, n_end = tip_corres.cols(); n < n_end; ++ n)
            {
                Eigen::Vector3d cm = tip_cloud.col(n) - petal_matrix.col(k);
                e2 += tip_corres(k, n) * cm.transpose() * cov_matrix.col(k).asDiagonal().inverse() * cm;
            }
        }

        for (size_t n = 0, n_end = inner_corres.cols(); n < n_end; ++ n)
        {
            Eigen::Vector3d cm = inner_cloud.col(n) - petal_matrix.col(k);
            e3 += inner_corres(k, n) * cm.transpose() * cov_matrix.col(k).asDiagonal().inverse() * cm;
        }

        for (size_t j = 0, j_end = adj_list[k].size(); j < j_end; ++ j)
        {
            Eigen::Vector3d mkj = petal_matrix.col(k)-petal_matrix.col(adj_list[k][j]);
            Eigen::Vector3d pre_mkj = origin_petal.col(k)-origin_petal.col(adj_list[k][j]);
            e4 += weight_matrix.coeffRef(k, adj_list[k][j]) * (mkj-rot_list[k]*pre_mkj).squaredNorm();
        }
    }

    for (size_t i = 0, i_end = intersect_list.size(); i < i_end; ++ i)
    {
        CollidingPoint cp = getCollidingPoint(petal_id, intersect_list[i]);
        osg::Vec3 projection = computeProjection(cp);
        osg::Vec3 point = cp.p_;
        e5 += (point - projection).length2();
    }

    //std::cout << "e1:" << e1 << " e2:" << e2 << " e3:" << e3 << " e4:" << e4 << " e5:" << e5 << std::endl;

    return (Solver::lambda_boundary_fitting_ * e1 + Solver::lambda_tip_fitting_ * e2 + 
        Solver::lambda_inner_fitting_ * e3 + Solver::lambda_arap_ * e4 + Solver::lambda_collision_ * e5);
}

// both boundary and inner fitting
void Solver::e_step(int petal_id)
{
    std::cout << "E-Step:" << std::endl;

    if (has_point_cloud_)
    {
        // boundary corres
        {
            CorresMatrix& corres_mat = deform_petals_[petal_id]._boundary_corres;
            WeightList& weight_list = deform_petals_[petal_id]._boundary_weights;
            VisList& vis_list = deform_petals_[petal_id]._boundary_vis;

            for (size_t i = 0, i_end = corres_mat.cols(); i < i_end; ++ i)
            {
                for (size_t j = 0, j_end = corres_mat.rows(); j < j_end; ++ j)
                {
                    corres_mat(j, i) = boundary_gaussian(petal_id, j, i) 
                        * vis_list[j]  * weight_list[j];
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

        // tip corres
        {
            CorresMatrix& corres_mat = deform_petals_[petal_id]._tip_corres;
            WeightList& weight_list = deform_petals_[petal_id]._tip_weights;
            VisList& vis_list = deform_petals_[petal_id]._tip_vis;

            for (size_t i = 0, i_end = corres_mat.cols(); i < i_end; ++ i)
            {
                for (size_t j = 0, j_end = corres_mat.rows(); j < j_end; ++ j)
                {
                    corres_mat(j, i) = tip_gaussian(petal_id, j, i) 
                        * vis_list[j]  * weight_list[j];
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
        
    }

    // inner corres
    {
        CorresMatrix& corres_mat = deform_petals_[petal_id]._inner_corres;
        WeightList& weight_list = deform_petals_[petal_id]._inner_weights;
        VisList& vis_list = deform_petals_[petal_id]._inner_vis;

        for (size_t i = 0, i_end = corres_mat.cols(); i < i_end; ++ i)
        {
            for (size_t j = 0, j_end = corres_mat.rows(); j < j_end; ++ j)
            {
                corres_mat(j, i) = inner_gaussian(petal_id, j, i) * vis_list[j]  * weight_list[j];
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
    
}

double Solver::m_step(int petal_id)
{
    std::cout << "M-Step:" << std::endl;

     // update gmm's weights
    if (has_point_cloud_)
    {
        {
            CorresMatrix& corres_mat = deform_petals_[petal_id]._boundary_corres;
            WeightList& weight_list = deform_petals_[petal_id]._boundary_weights;
            for (size_t i = 0, i_end = weight_list.size(); i < i_end; ++ i)
            {
                weight_list[i] = corres_mat.row(i).sum() / corres_mat.cols();
            }
        }
        
        {
            CorresMatrix& corres_mat = deform_petals_[petal_id]._tip_corres;
            WeightList& weight_list = deform_petals_[petal_id]._tip_weights;
            for (size_t i = 0, i_end = weight_list.size(); i < i_end; ++ i)
            {
                weight_list[i] = corres_mat.row(i).sum() / corres_mat.cols();
            }
        }
    }

    {
        CorresMatrix& corres_mat = deform_petals_[petal_id]._inner_corres;
        WeightList& weight_list = deform_petals_[petal_id]._inner_weights;
        for (size_t i = 0, i_end = weight_list.size(); i < i_end; ++ i)
        {
            weight_list[i] = corres_mat.row(i).sum() / corres_mat.cols();
        }
    }
    

    // update expectation
    int iter = 0;
    double eps = 0;

    double e = 0;

    initBuild(petal_id);
    left_sys(petal_id);
    right_sys(petal_id);

    do {
        double e_n = solve(petal_id);
        eps = std::fabs((e_n - e) / e_n);
        e = e_n;

        projection(petal_id);
        update(petal_id);
        right_sys(petal_id); // the update only effects right side 

        iter ++;

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

void Solver::initBuild(int petal_id)
{
    if (has_point_cloud_)
    {
        boundary_term_[petal_id].build();
        tip_term_[petal_id].build();
    }

    inner_term_[petal_id].build();
    arap_term_[petal_id].build();
    skel_term_[petal_id].build();
}

void Solver::left_sys(int petal_id)
{
    for (int i = 0; i < 3; ++ i)
    {
        A_[petal_id][i] = inner_term_[petal_id].A()[i] + 
            arap_term_[petal_id].A()[i] + skel_term_[petal_id].A()[i];

        if (has_point_cloud_)
            A_[petal_id][i] += (boundary_term_[petal_id].A()[i] + tip_term_[petal_id].A()[i]);
    }
}

void Solver::right_sys(int petal_id)
{
    b_[petal_id] = inner_term_[petal_id].b() + 
        arap_term_[petal_id].b() + skel_term_[petal_id].b();

    if (has_point_cloud_)
        b_[petal_id] += (boundary_term_[petal_id].b() + tip_term_[petal_id].b());
}

void Solver::projection(int petal_id)
{
    if (has_point_cloud_)
    {
        boundary_term_[petal_id].projection();
        tip_term_[petal_id].projection();
    }

    inner_term_[petal_id].projection();
    arap_term_[petal_id].projection();
    skel_term_[petal_id].projection();
}

void Solver::update(int petal_id)
{
    if (has_point_cloud_)
    {
        boundary_term_[petal_id].update();
        tip_term_[petal_id].update();
    }

    inner_term_[petal_id].update();
    arap_term_[petal_id].update();
    skel_term_[petal_id].update();
}

double Solver::inner_gaussian(int petal_id, int m_id, int c_id)
{
    double p;

    CovMatrix& cov_mat = deform_petals_[petal_id]._cov_matrix;
    CloudMatrix& cloud_mat = deform_petals_[petal_id]._inner_matrix;
    PetalMatrix& petal_mat = deform_petals_[petal_id]._petal_matrix;

    Eigen::Vector3d xu = cloud_mat.col(c_id) - petal_mat.col(m_id);
    p = pow(2*M_PI, -3/2.0) * pow((cov_mat.col(m_id).asDiagonal()).toDenseMatrix().determinant(), -1/2.0) * 
        exp((-1/2.0)*xu.transpose()*cov_mat.col(m_id).asDiagonal().inverse()*xu);

    return p;
}

double Solver::boundary_gaussian(int petal_id, int m_id, int c_id)
{
    double p;

    CovMatrix& cov_mat = deform_petals_[petal_id]._cov_matrix;
    CloudMatrix& cloud_mat = deform_petals_[petal_id]._boundary_cloud;
    PetalMatrix& petal_mat = deform_petals_[petal_id]._petal_matrix;

    Eigen::Vector3d xu = cloud_mat.col(c_id) - petal_mat.col(m_id);
    p = pow(2*M_PI, -3/2.0) * pow((cov_mat.col(m_id).asDiagonal()).toDenseMatrix().determinant(), -1/2.0) * 
        exp((-1/2.0)*xu.transpose()*cov_mat.col(m_id).asDiagonal().inverse()*xu);

    return p;
}

double Solver::tip_gaussian(int petal_id, int m_id, int c_id)
{
    double p;

    CovMatrix& cov_mat = deform_petals_[petal_id]._cov_matrix;
    CloudMatrix& cloud_mat = deform_petals_[petal_id]._tip_cloud;
    PetalMatrix& petal_mat = deform_petals_[petal_id]._petal_matrix;

    Eigen::Vector3d xu = cloud_mat.col(c_id) - petal_mat.col(m_id);
    p = pow(2*M_PI, -3/2.0) * pow((cov_mat.col(m_id).asDiagonal()).toDenseMatrix().determinant(), -1/2.0) * 
        exp((-1/2.0)*xu.transpose()*cov_mat.col(m_id).asDiagonal().inverse()*xu);

    return p;
}


void Solver::full_deform()
{
    std::cout << "Start Deforming..." << std::endl;
    
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

    // save trans
    saveT();
}

void Solver::e_step()
{
    for (size_t i = 0; i < petal_num_; ++ i)
    {
        e_step(i);
    }
}

double Solver::m_step()
{
    std::cout << "M-Step:" << std::endl;

    // update gmm's weights
    for (size_t j = 0; j < petal_num_; ++ j)
    {
        if (has_point_cloud_)
        {
            {
                CorresMatrix& corres_mat = deform_petals_[j]._boundary_corres;
                WeightList& weight_list = deform_petals_[j]._boundary_weights;
                for (size_t i = 0, i_end = weight_list.size(); i < i_end; ++ i)
                {
                    weight_list[i] = corres_mat.row(i).sum() / corres_mat.cols();
                }
            }
            
            {
                CorresMatrix& corres_mat = deform_petals_[j]._tip_corres;
                WeightList& weight_list = deform_petals_[j]._tip_weights;
                for (size_t i = 0, i_end = weight_list.size(); i < i_end; ++ i)
                {
                    weight_list[i] = corres_mat.row(i).sum() / corres_mat.cols();
                }
            }
        }

        {
            CorresMatrix& corres_mat = deform_petals_[j]._inner_corres;
            WeightList& weight_list = deform_petals_[j]._inner_weights;
            for (size_t i = 0, i_end = weight_list.size(); i < i_end; ++ i)
            {
                weight_list[i] = corres_mat.row(i).sum() / corres_mat.cols();
            }
        }
    }


    // update expectation
    int iter = 0;
    double eps = 0;

    double e = 0;

    initBuild();

    do {
        collision_detection();
        projection();
        update();
        left_sys();
        right_sys(); 

        double e_n = solve();
        eps = std::fabs((e_n - e) / e_n);
        e = e_n;

        iter ++;

    }while(eps > eps_ && iter < iter_num_);

    return e;

}

void Solver::deforming()
{
    for (size_t i = 0; i < petal_num_; ++ i)
    {
        deforming(i);
    }
}

void Solver::initBuild()
{
    for (size_t i = 0; i < petal_num_; ++ i)
    {
        if(has_point_cloud_)
        {
            boundary_term_[i].build();
            tip_term_[i].build();
        }

        inner_term_[i].build();
        arap_term_[i].build();
        skel_term_[i].build();
        collision_term_[i].build();
        closure_term_[i].build();
    }
}

void Solver::left_sys()
{
    FA_.clear();
    FA_.resize(3);

    int num = 0;
    for (size_t j = 0; j < petal_num_; ++ j)
    {
        HandleMatrix& hm = deform_petals_[j]._handle_matrix;
        num += 4 * hm.rows();
    }

    // generate and merge
    for (int i = 0; i < 3; ++ i)
    {
        FA_[i].resize(num, num);
        FA_[i].setZero();

        int row_idx = 0, col_idx = 0;
        for (size_t j = 0; j < petal_num_; ++ j)
        {
            A_[j][i] = inner_term_[j].A()[i] + arap_term_[j].A()[i] + skel_term_[j].A()[i] + collision_term_[j].A()[i] + closure_term_[j].A()[i];

            if (has_point_cloud_)
                A_[j][i] += (boundary_term_[j].A()[i] + tip_term_[j].A()[i]);

            FA_[i].block(row_idx, col_idx, A_[j][i].rows(), A_[j][i].cols()) = A_[j][i];

            row_idx += A_[j][i].rows();
            col_idx += A_[j][i].cols();
        }
    }
}

void Solver::right_sys()
{
    int num = 0;
    for (size_t j = 0; j < petal_num_; ++ j)
    {
        HandleMatrix& hm = deform_petals_[j]._handle_matrix;
        num += 4 * hm.rows();
    }
    Fb_.resize(3, num);

    // generate and merge
    int col_idx = 0;
    for (size_t j = 0; j < petal_num_; ++ j)
    {
        b_[j] = inner_term_[j].b() + arap_term_[j].b() + skel_term_[j].b() + collision_term_[j].b() + closure_term_[j].b();

        if (has_point_cloud_)
            b_[j] += (boundary_term_[j].b() + tip_term_[j].b());

        Fb_.block(0, col_idx, b_[j].rows(), b_[j].cols()) = b_[j];
        col_idx += b_[j].cols();
    }
}

double Solver::solve()
{
    // solve T
    for (size_t i = 0; i < 3; ++ i)
    {
        Eigen::MatrixXd A = FA_[i];
        Eigen::VectorXd b = Fb_.row(i).transpose();

        Eigen::VectorXd next_values = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

        // new T
        int start_idx = 0;
        for (size_t j = 0; j < petal_num_; ++ j)
        {
            HandleMatrix& handle_matrix = deform_petals_[j]._handle_matrix;
            int handle_num = handle_matrix.rows();
            AffineMatrix& affine_matrix = deform_petals_[j]._affine_matrix;
            affine_matrix.col(i) = next_values.segment(start_idx, 4*handle_num);
            start_idx += 4*handle_num;
        }
    }

    // update vertices
    lbs();

    return energy();
}

void Solver::lbs()
{
    for (size_t j = 0; j < petal_num_; ++ j)
    {
        lbs(j);
    }
}

double Solver::energy()
{
    double e = 0;
    for (size_t j = 0; j < petal_num_; ++ j)
    {
        e += energy(j);
    }

    return e;
}

void Solver::projection()
{
    for (size_t j = 0; j < petal_num_; ++ j)
    {
        if (has_point_cloud_)
        {
            boundary_term_[j].projection();
            tip_term_[j].projection();
        }

        inner_term_[j].projection();
        arap_term_[j].projection();
        skel_term_[j].projection();
        collision_term_[j].projection();
        closure_term_[j].projection();
    }
}

void Solver::update()
{
    for (size_t j = 0; j < petal_num_; ++ j)
    {
        if(has_point_cloud_)
        {
            boundary_term_[j].update();
            tip_term_[j].update();
        }

        inner_term_[j].update();
        arap_term_[j].update();
        skel_term_[j].update();
        collision_term_[j].update();
        closure_term_[j].update();
    }
}

void Solver::collision_detection()
{
    PetalMatrix_to_Flower();

    CollisionDetector collision_detector;
    collision_detector.setFlower(flower_);
    collision_detector.checkCollision();
    collision_detector.resolveCollision();
    colliding_points_ = collision_detector.getCollidingPoints();

    Flower_to_PetalMatrix();
}

// before collision detection
// used in m step loop for collision detection
void Solver::PetalMatrix_to_Flower()
{
    for (size_t i = 0; i < deform_petals_.size(); ++ i)
    {
        // vertices
        Petals& petals = flower_->getPetals();
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

// after collision detection
// used in m step loop for collision detection
void Solver::Flower_to_PetalMatrix()
{
    Petals& petals = flower_->getPetals();
    for (size_t i = 0, i_end = petals.size(); i < i_end; ++ i)
    {
        IntersectList& intersect_list = deform_petals_[i]._intersect_list;
        intersect_list = petals[i].getIntersectedVertices();
    }
}

CollidingPoint Solver::getCollidingPoint(int petal_id, int ver_id)
{
    for (size_t i = 0, i_end = colliding_points_.size(); i < i_end; ++ i)
    {
        if (colliding_points_[i].petal_id_ == petal_id &&
            colliding_points_[i].vertex_id_ == ver_id)
            return colliding_points_[i];
    }

    CollidingPoint cp;
    cp.petal_id_ = -1;
    cp.vertex_id_ = -1;
    return cp;
}

osg::Vec3 Solver::computeProjection(CollidingPoint cp)
{

    float k = sqrt(cp.dis2_ / cp.normal_.length2());

    float r = MainWindow::getInstance()->getParameters()->getMovingRatio();

    osg::Vec3 projection;
    projection.x() = cp.p_.x() - cp.normal_.x() * k * r;
    projection.y() = cp.p_.y() - cp.normal_.y() * k * r;
    projection.z() = cp.p_.z() - cp.normal_.z() * k * r;

    return projection;
}


void Solver::saveT(int petal_id)
{
    std::string& flower_path = flower_->getTransFolder();

    std::string trans_file_ext = is_forward_ ? ".ft" : ".bt";

    DeformPetal& deform_petal = deform_petals_[petal_id];
    HandleMatrix& handle_matrix = deform_petal._handle_matrix;
    AffineMatrix& am = deform_petal._affine_matrix;

    Petal& petal = flower_->getPetals()[petal_id];
    std::string& obj_name = petal.getObjName();

    std::string trans_file = flower_path + "/" + obj_name + trans_file_ext;
    std::fstream fs(trans_file, std::ios_base::out);

    fs << handle_matrix.rows() << "\n";
    fs << am << "\n";
}

void Solver::saveT()
{
    for (size_t i = 0; i < deform_petals_.size(); ++ i)
    {
        saveT(i);
    }
}