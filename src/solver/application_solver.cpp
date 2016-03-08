#include <Eigen/SparseQR>
#include "application_solver.h"
#include "transfer.h"
#include "main_window.h"


ApplicationSolver::ApplicationSolver(PointCloud* point_cloud, Flower* flower, int flower_frame)
    :Solver(point_cloud, flower, flower_frame)
{
    ref_flower_frame_ = flower_frame;

    L_ = std::vector<std::vector<Eigen::MatrixXd>>(petal_num_, std::vector<Eigen::MatrixXd>(3));
    b_L_.resize(petal_num_);
}

void ApplicationSolver::init_setting()
{
    loadRefFrame();
    std::cout << "finish reference loading" << std::endl;

    initMeshParas();
    std::cout << "finish mesh initialization" << std::endl;

    initSkelParas();
    std::cout << "finish skeleton initialization" << std::endl;

    initTerms();
    std::cout << "finish solver initialization" << std::endl;
}

void ApplicationSolver::full_deform()
{
    std::cout << "Start application solver..." << std::endl;
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

        iter++;
    }while (eps > eps_ && iter < iter_num_);

    std::cout << "End application solver \t" << "iter: " << iter << "\tdelta: " << eps << std::endl;

    // flower deformed
    deforming();
}

void ApplicationSolver::initMeshParas()
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

    // init petal matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        deform_petals_[i]._petal_matrix = deform_petals_[i]._origin_petal;
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

    // init reference matrix
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petals& petals = ref_flower_.getPetals();
        Petal& petal = petals[i];
        int petal_size = petal.getVertices()->size();
        PetalMatrix pm(3, petal_size);

        for (size_t j = 0, j_end = petal_size; j < j_end; ++ j)
        {
            pm.col(j) << petal.getVertices()->at(j).x(), petal.getVertices()->at(j).y(), petal.getVertices()->at(j).z();
        }

        deform_petals_[i]._ref_matrix = pm;
    }
}

void ApplicationSolver::initTerms()
{
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++i)
    {
        arap_term_.push_back(ARAPTerm(i));
    }
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++i)
    {
        collision_term_.push_back(CollisionDetectionTerm(i));
    }
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++i)
    {
        interpolation_term_.push_back(InterpolationTerm(i));
    }
    for (size_t i = 0, i_end = petal_num_; i < i_end; ++i)
    {
        closure_term_.push_back(ClosureTerm(i));
    }
}

void ApplicationSolver::initBuild()
{
    for (size_t i = 0; i < petal_num_; ++i)
    {
        arap_term_[i].build();
        collision_term_[i].build();
        interpolation_term_[i].build();
        closure_term_[i].build();
    }
}

void ApplicationSolver::left_sys()
{
    FL_.clear();
    FL_.resize(3);

    int num = 0;
    for (size_t j = 0; j < petal_num_; ++ j)
    {
        PetalMatrix& pm = deform_petals_[j]._origin_petal;
        num += pm.cols();
    }

    // generate and merge
    for (int i = 0; i < 3; ++ i)
    {
        FL_[i].resize(num, num);
        FL_[i].setZero();

        int row_idx = 0, col_idx = 0;
        for (size_t j = 0; j < petal_num_; ++ j)
        {
            L_[j][i] = Eigen::MatrixXd(arap_term_[j].L()[i] + collision_term_[j].L()[i] + 
                interpolation_term_[j].L()[i] + closure_term_[j].L()[i]);

           // assignSparseMatrix(L_[j][i], FL_[i].block(row_idx, col_idx, L_[j][i].rows(), L_[j][i].cols()));
            FL_[i].block(row_idx, col_idx, L_[j][i].rows(), L_[j][i].cols()) = L_[j][i];

            row_idx += L_[j][i].rows();
            col_idx += L_[j][i].cols();
        }
    }
}

void ApplicationSolver::right_sys()
{
    int num = 0;
    for (size_t j = 0; j < petal_num_; ++ j)
    {
        PetalMatrix& pm = deform_petals_[j]._origin_petal;
        num += pm.cols();
    }

    Fb_L_.resize(3, num);

    // generate and merge
    int col_idx = 0;
    for (size_t j = 0; j < petal_num_; ++ j)
    {
        b_L_[j] = arap_term_[j].b_L() + collision_term_[j].b_L() + interpolation_term_[j].b_L() + closure_term_[j].b_L();

        Fb_L_.block(0, col_idx, b_L_[j].rows(), b_L_[j].cols()) = b_L_[j];
        col_idx += b_L_[j].cols();
    }
}

void ApplicationSolver::projection()
{
    for (size_t j = 0; j < petal_num_; ++j)
    {
        arap_term_[j].projection();
        collision_term_[j].projection();
        interpolation_term_[j].projection();
        closure_term_[j].projection();
    }
}

void ApplicationSolver::update()
{
    for (size_t j = 0; j < petal_num_; ++j)
    {
        collision_term_[j].update();
        arap_term_[j].update();
        interpolation_term_[j].update();
        closure_term_[j].update();
    }
}

double ApplicationSolver::solve()
{
    // solve x, y ,z independently
    for (size_t i = 0; i < 3; ++ i)
    {
        lu_solver_.analyzePattern(FL_[i].sparseView());
        lu_solver_.factorize(FL_[i].sparseView());

        Eigen::VectorXd next_pos = lu_solver_.solve(Fb_L_.row(i).transpose());

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

double ApplicationSolver::energy(int petal_id)
{
    DeformPetal& deform_petal = deform_petals_[petal_id];
    PetalMatrix& ref_matrix = deform_petal._ref_matrix;
    PetalMatrix& petal_matrix = deform_petal._petal_matrix;
    PetalMatrix& origin_petal = deform_petal._origin_petal;
    RotList& rot_list = deform_petal._R_list;
    WeightMatrix& weight_matrix = deform_petal._weight_matrix;
    AdjList& adj_list = deform_petal._adj_list;
    IntersectList& intersect_list = deform_petal._intersect_list;


    double e1 = 0, e2 = 0, e3 = 0;

    for (size_t k = 0, k_end = petal_matrix.cols(); k < k_end; ++ k)
    {
        for (size_t j = 0, j_end = adj_list[k].size(); j < j_end; ++ j)
        {
            Eigen::Vector3d mkj = petal_matrix.col(k)-petal_matrix.col(adj_list[k][j]);
            Eigen::Vector3d pre_mkj = origin_petal.col(k)-origin_petal.col(adj_list[k][j]);
            e1 += weight_matrix.coeffRef(k, adj_list[k][j]) * (mkj-rot_list[k]*pre_mkj).squaredNorm();
        }
    }

    for (size_t i = 0, i_end = intersect_list.size(); i < i_end; ++ i)
    {
        CollidingPoint cp = getCollidingPoint(petal_id, intersect_list[i]);
        osg::Vec3 projection = computeProjection(cp);
        osg::Vec3 point = cp.p_;
        e2 += (point - projection).length2();
    }

    // interpolation term
    for (size_t k = 0, k_end = petal_matrix.cols(); k < k_end; ++ k)
    {
        Eigen::Vector3d delta = (petal_matrix.col(k) - ref_matrix.col(k));
        e3 += delta.squaredNorm();
    }

    //std::cout << "e1:" << e1 << " e2:" << e2 << " e3:" << e3 << std::endl;

    return (Solver::lambda_arap_ * e1 + Solver::lambda_collision_ * e2 + Solver::lambda_interpolation_ * e3);
}

double ApplicationSolver::energy()
{
    double e = 0;
    for (size_t j = 0; j < petal_num_; ++ j)
    {
        e += energy(j);
    }

    return e;
}

void ApplicationSolver::assignSparseMatrix(Eigen::SparseMatrix<double>& source, Eigen::Block<Eigen::SparseMatrix<double>>& target)
{
    for (size_t i = 0, i_end = target.rows(); i < i_end; ++ i)
        for (size_t j = 0, j_end = target.rows(); j < j_end; ++ j)
            target.coeffRef(i, j) = source.coeffRef(i, j);
}

void ApplicationSolver::deforming(int petal_id)
{
    // vertices
    Petals& petals = flower_->getPetals();
    Petal& petal = petals.at(petal_id);
    PetalMatrix& pm = deform_petals_[petal_id]._petal_matrix;
    PetalMatrix& om = deform_petals_[petal_id]._origin_petal;
    HardCtrsIdx& hc_idx = deform_petals_[petal_id]._hc_idx;

    for (size_t j = 0, j_end = petal.getVertices()->size(); j < j_end; ++ j)
    {
        petal.getVertices()->at(j).x() = pm(0, j);
        petal.getVertices()->at(j).y() = pm(1, j);
        petal.getVertices()->at(j).z() = pm(2, j);
    }

    //// for root constraint
    //for (auto& idx : hc_idx)
    //{
    //    petal.getVertices()->at(idx).x() = om(0, idx);
    //    petal.getVertices()->at(idx).y() = om(1, idx);
    //    petal.getVertices()->at(idx).z() = om(2, idx);
    //}

    petal.updateNormals();
}

void ApplicationSolver::deforming()
{
    for (size_t i = 0; i < petal_num_; ++ i)
    {
        deforming(i);
    }
}

void ApplicationSolver::loadRefFrames()
{
    // hard code path
    std::string ref_flowers_folder = "D:/baidu disk/WorkSpace/Projects/BloomingFlower/BloomingFlowers/data/applications/new-transfer/transfered_flowers";
    std::string ref_flower_path = ref_flowers_folder + "/" + QString("frame_%1").arg(ref_flower_frame_, 5, 10, QChar('0')).toStdString();
    ref_flower_.load(ref_flower_path);
}

void ApplicationSolver::loadRefFrame()
{
    // hard code path
    std::string ref_flowers_folder = "D:/baidu disk/WorkSpace/Projects/BloomingFlower/BloomingFlowers/data/applications/new-transfer/petal sequences";
    Transfer t(ref_flowers_folder, flower_, cloud_frame_);
    ref_flower_ = *t.update();
}