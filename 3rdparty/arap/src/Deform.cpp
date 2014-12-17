#include "Deform.h"

using namespace std;
using namespace Eigen;

Deform::Deform(const float *P_data, int P_Num, const AdjList &adj_list, const TriangleList &triangle_list)
    :P_data(P_data), 
    P_Num(P_Num), 
    max_iter(10), 
    min_tol(1e-3), 
    lambda(5), 
    adj_list(adj_list)
{

    for (size_t i = 0, i_end = triangle_list.size(); i < i_end; ++ i)
    {
        const std::vector<int>& face = triangle_list[i];
        assert(face.size() == 3);
        face_list.push_back(Eigen::Vector3i(face[0], face[1], face[2]));
    }

    P.resize(3, P_Num);
    for (int i = 0; i != P_Num; ++i) {P.col(i) << P_data[3*i], P_data[3*i+1], P_data[3*i+2];}
    P_Prime = P;
    R = vector<Matrix3f>(P_Num, Matrix3f::Identity());

    // Weight
//    build_weight_matrix();
}

float *Deform::do_Deform()
{
    int iter = 0;
    double delta = 0;

    do {
        set_linear_sys();
        delta = update_P_Prime();
        update_Ri();
        cout << "iter: " << ++ iter << "\tdelta: " << delta << endl;

    }while(delta > min_tol && iter < max_iter);

    return P_Prime.data();
}

float *Deform::do_Deform(int iter_num)
{
    int iter = 0;
    double delta = 0;

    do {
        set_linear_sys();
        delta = update_P_Prime();
        update_Ri();
        cout << "iter: " << ++ iter << "\tdelta: " << delta << endl;

    }while(iter < iter_num);

    return P_Prime.data();
}

float *Deform::get_P_Prime()
{
    return P_Prime.data();
}

float* Deform::do_Deform_Iter(float &delta)
{
    set_linear_sys();
    delta = update_P_Prime();
    update_Ri();
    return P_Prime.data();
}

void Deform::set_tolerance(float tol)
{
    min_tol = tol;
}

void Deform::set_max_iteration(int iter_num)
{
    max_iter = iter_num;
}

void Deform::set_lambda(float lamdba)
{
    this->lambda = lamdba;
}

void Deform::set_hard_ctrs(const VectorF &T, const VectorI &idx_T)
{
    assert(T.size()/3 == idx_T.size());

    for (int i = 0, i_end = idx_T.size(); i < i_end; ++ i)
    {
        int idx = idx_T[i];
        P_Prime.col(idx) << T[3*i], T[3*i+1], T[3*i+2];
    }

    for (int i = 0, i_end = idx_T.size(); i < i_end; ++ i)
    {
        int cid = idx_T[i];
//        P_Prime.col(idx) << T[3*i], T[3*i+1], T[3*i+2];
        Eigen::Vector3i ctr; 
        ctr << T[3*i], T[3*i+1], T[3*i+2];

        hard_ctrs.push_back(Constraint(ctr, cid));
    }

    std::sort(hard_ctrs.begin(), hard_ctrs.end(), ConstraintCompare());
}

void Deform::set_soft_ctrs(const VectorF &T, const VectorI &idx_T)
{
    assert(T.size()/3 == idx_T.size());

    for (int i = 0, i_end = idx_T.size(); i < i_end; ++ i)
    {
        int idx = idx_T[i];
        P_Prime.col(idx) << T[3*i], T[3*i+1], T[3*i+2];
    }

    for (int i = 0, i_end = idx_T.size(); i < i_end; ++ i)
    {
        int cid = idx_T[i];
        //        P_Prime.col(idx) << T[3*i], T[3*i+1], T[3*i+2];
        Eigen::Vector3i ctr; 
        ctr << T[3*i], T[3*i+1], T[3*i+2];

        soft_ctrs.push_back(Constraint(ctr, cid));
    }

    std::sort(soft_ctrs.begin(), soft_ctrs.end(), ConstraintCompare());
}

float Deform::energy()
{
    // to do
    return 0;
}

void Deform::build_laplacian_matrix()
{
    vector<Triplet<float> > weight_list;
    weight_list.reserve(3*7*P_Num); // each vertex may have about 7 vertices connected

    vector<Triplet<float> > weight_sum;
    weight_sum.reserve(3*P_Num);

    for (int i = 0; i != P_Num; ++i) {
        
        float wi = 0;

        for (decltype(adj_list[i].size()) j = 0; j != adj_list[i].size(); ++j) {
            int id_j = adj_list[i][j];

            vector<int> share_vertex;
            find_share_vertex(i, id_j, share_vertex);

            float wij = 0;
            if (share_vertex.size()==2) wij = compute_wij(&P_data[3*i], &P_data[3*id_j], 
                &P_data[3*share_vertex[0]], &P_data[3*share_vertex[1]]);
            else wij = compute_wij(&P_data[3*i], &P_data[3*id_j], &P_data[3*share_vertex[0]]);

            wi += wij;

            weight_list.push_back(Triplet<float>(i, id_j, wij));
            weight_list.push_back(Triplet<float>(i+P_Num, id_j+P_Num, wij));
            weight_list.push_back(Triplet<float>(i+2*P_Num, id_j+2*P_Num, wij));
        }

        weight_sum.push_back(Triplet<float>(i, i, wi));
        weight_sum.push_back(Triplet<float>(i+P_Num, i+P_Num, wi));
        weight_sum.push_back(Triplet<float>(i+2*P_Num, i+2*P_Num, wi));
    }

    Weight.resize(3*P_Num, 3*P_Num);
    Weight.setFromTriplets(weight_list.begin(), weight_list.end());
}

bool Deform::is_hard_ctrs(int id)
{
    int k = 0, k_end = hard_idx.size();
    while (k < k_end)
    {
        if (id == hard_idx[k++])
            return true;
    }
    return false;
}

float Deform::compute_wij(const float *p1, const float *p2, const float *p3, const float *p4)
{
    float e1 = sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1])+(p1[2]-p2[2])*(p1[2]-p2[2]));
    float e2 = sqrt((p1[0]-p3[0])*(p1[0]-p3[0])+(p1[1]-p3[1])*(p1[1]-p3[1])+(p1[2]-p3[2])*(p1[2]-p3[2]));
    float e3 = sqrt((p3[0]-p2[0])*(p3[0]-p2[0])+(p3[1]-p2[1])*(p3[1]-p2[1])+(p3[2]-p2[2])*(p3[2]-p2[2]));
    float alpha_cos = (e3*e3+e2*e2-e1*e1)/(2*e3*e2);
    float beta_cos = 0;
    if (p4 != nullptr) {
        float e4 = sqrt((p1[0]-p4[0])*(p1[0]-p4[0])+(p1[1]-p4[1])*(p1[1]-p4[1])+(p1[2]-p4[2])*(p1[2]-p4[2]));
        float e5 = sqrt((p4[0]-p2[0])*(p4[0]-p2[0])+(p4[1]-p2[1])*(p4[1]-p2[1])+(p4[2]-p2[2])*(p4[2]-p2[2]));
        beta_cos = (e4*e4+e5*e5-e1*e1)/(2*e4*e5);
    }
    return ((alpha_cos/sqrt(1-alpha_cos*alpha_cos))+(beta_cos/sqrt(1-beta_cos*beta_cos)))/2;
}

void Deform::find_share_vertex(int pi, int pj, VectorI &share_vertex)
{
    vector<int> vertices;
    set_intersection(adj_list[pi].begin(), adj_list[pi].end(), adj_list[pj].begin(), adj_list[pj].end(), back_inserter(vertices));
    for (auto &i : vertices) {
        vector<int> f;
        f.push_back(pi);
        f.push_back(pj);
        f.push_back(i);
        sort(f.begin(), f.end());
        vector<Vector3i>::iterator it = find(face_list.begin(), face_list.end(), Map<Vector3i>(&f[0]));
        if (it != face_list.end()) {
            if ((*it)(0) != pi && (*it)(0) != pj) share_vertex.push_back((*it)(0));
            else if ((*it)(1) != pi && (*it)(1) != pj) share_vertex.push_back((*it)(1));
            else share_vertex.push_back((*it)(2));
        }
    }
    if (share_vertex.size() > 2) {
        cout << "share vertices number warning: " << share_vertex.size() << endl;
    }
}

void Deform::build_weight_matrix()
{
    
}

void Deform::build_laplacian_matrix()
{
    Eigen::SparseMatrix<float> weight = Weight;

    vector<Triplet<float> > weight_sum;
    weight_sum.reserve(3*P_Num);

    for (int i = 0; i != P_Num; ++i) {
        float wi = 0;

        int k = 0, k_end = hard_idx.size();
        while (k < k_end)
        {
            if (i == hard_idx[k])
            {
                for (int j = 0; j != P_Num; ++ j)
                {
                    weight.coeffRef(i, j) = 0;
                    weight.coeffRef(i+P_Num, j+P_Num) = 0;
                    weight.coeffRef(i+2*P_Num, j+2*P_Num) = 0;
                }

                wi = 1;
                weight_sum.push_back(Triplet<float>(i, i, wi));
                weight_sum.push_back(Triplet<float>(i+P_Num, i+P_Num, wi));
                weight_sum.push_back(Triplet<float>(i+2*P_Num, i+2*P_Num, wi));

                break;
            }
            k++;
        }

        if (k != k_end)
            continue;

        for (decltype(adj_list[i].size()) j = 0; j != adj_list[i].size(); ++j) {
            int id_j = adj_list[i][j];

            wi += Weight.coeffRef(i, id_j);
        }

        if (wi < 0.1) cout << "Edge Weight Sum Warning: " << wi << endl;
        weight_sum.push_back(Triplet<float>(i, i, wi));
        weight_sum.push_back(Triplet<float>(i+P_Num, i+P_Num, wi));
        weight_sum.push_back(Triplet<float>(i+2*P_Num, i+2*P_Num, wi));
    }

    SparseMatrix<float> Weight_sum(3*P_Num, 3*P_Num);
    Weight_sum.setFromTriplets(weight_sum.begin(), weight_sum.end());
    L =  Weight_sum - weight;
}

void Deform::update_Ri()
{
    Matrix3f Si;
    MatrixXf Di;
    Matrix3Xf Pi_Prime;
    Matrix3Xf Pi;
    for (int i = 0; i != P_Num; ++i) {
        Di = MatrixXf::Zero(adj_list[i].size(), adj_list[i].size());
        Pi_Prime.resize(3, adj_list[i].size());
        Pi.resize(3, adj_list[i].size());
        // if there is not any single unconnected point this for loop can have a more efficient representation
        for (decltype(adj_list[i].size()) j = 0; j != adj_list[i].size(); ++j) {
            Di(j, j) = Weight.coeffRef(i, adj_list[i][j]);
            Pi.col(j) = P.col(i) - P.col(adj_list[i][j]);
            Pi_Prime.col(j) = P_Prime.col(i) - P_Prime.col(adj_list[i][j]);
        }
        Si = Pi * Di * Pi_Prime.transpose();
        Matrix3f Ui;
        Vector3f Wi;
        Matrix3f Vi;
        wunderSVD3x3(Si, Ui, Wi, Vi);
        R[i] = Vi * Ui.transpose();

        if (R[i].determinant() < 0)
            std::cout << "determinant is negative!" << std::endl;
    }
}

float Deform::update_P_Prime()
{
    VectorXf d_vec(VectorXf::Map(d.data(), d.cols()*d.rows()));
    VectorXf P_Prime_vec = chol.solve(d_vec);
    P_Prime.row(0) = P_Prime_vec.segment(0, P_Num);
    P_Prime.row(1) = P_Prime_vec.segment(0+P_Num, P_Num);
    P_Prime.row(2) = P_Prime_vec.segment(0+2*P_Num, P_Num);

    return (P_Prime - P).norm() / P.norm();
}



void Deform::set_linear_sys()
{
    build_laplacian_matrix();

    chol.analyzePattern(L);
    chol.factorize(L);

    d = MatrixX3f::Zero(P_Num, 3);
    for (int i = 0; i != P_Num; ++i) {

        int k = 0, k_end = hard_idx.size();
        while (k < k_end)
        {
            if (i == hard_idx[k])
            {
                d.row(i) << hard_ctrs[3*k], hard_ctrs[3*k+1], hard_ctrs[3*k+2];
                break;
            }
            k++;
        }

        if (k != k_end)
            continue;

        // if there is not any single unconnected point this for loop can have a more efficient representation
        for (decltype(adj_list[i].size()) j = 0; j != adj_list[i].size(); ++j) {
            d.row(i) += ((Weight.coeffRef(i, adj_list[i][j])/2)*(R[i]+R[adj_list[i][j]])*(P.col(i) - P.col(adj_list[i][j]))).transpose();
        }
        
        /*for (int k = 0, k_end = fixed_idx.size(); k < k_end; ++ k)
        {
        d.row(i) += Weight.coeffRef(i, fixed_idx[k])*
        RowVector3f(fixed_pos[3*k], fixed_pos[3*k+1], fixed_pos[3*k+2]);
        }*/
    }
}

//void Deform::set_linear_sys(VectorF &T, VectorI &idx_T)
//{
//    decltype(adj_list.size()) P_Num = adj_list.size();
//    d = MatrixX3f::Zero(P_Num, 3);
//    SparseMatrix<float> L_cur = L;
//    
//    for (decltype(P_Num) i = 0; i != P_Num; ++i) {
//        // if there is not any single unconnected point this for loop can have a more efficient representation
//        for (decltype(adj_list[i].size()) j = 0; j != adj_list[i].size(); ++j) {
//            d.row(i) += ((Weight.coeffRef(i, adj_list[i][j])/2)*(R[i]+R[adj_list[i][j]])*(P.col(i) - P.col(adj_list[i][j]))).transpose();
//        }
//    }
//    
//    for (decltype(idx_T.size()) i = 0; i != idx_T.size(); ++i) {
//        d.row(idx_T[i]) += (lamd_deform/2)*RowVector3f(T[3*i], T[3*i+1], T[3*i+2]);
//        L_cur.coeffRef(idx_T[i], idx_T[i]) += lamd_deform/2;
//        L_cur.coeffRef(idx_T[i]+P_Num, idx_T[i]+P_Num) += lamd_deform/2;
//        L_cur.coeffRef(idx_T[i]+2*P_Num, idx_T[i]+2*P_Num) += lamd_deform/2;
//    }
//
//    chol.factorize(L_cur);
//
//    
//
//}
//
//void Deform::set_linear_sys(VectorF &T, VectorI &idx_T, VectorF &F, VectorI &idx_F)
//{
//    decltype(adj_list.size()) P_Num = adj_list.size();
//    d = MatrixX3f::Zero(P_Num, 3);
//    SparseMatrix<float> L_cur = L;
//    
//    
//    for (decltype(idx_T.size()) i = 0; i != idx_T.size(); ++i) {
//        d.row(idx_T[i]) += (lamd_deform/2)*RowVector3f(T[3*i], T[3*i+1], T[3*i+2]);
//        L_cur.coeffRef(idx_T[i], idx_T[i]) += lamd_deform/2;
//        L_cur.coeffRef(idx_T[i]+P_Num, idx_T[i]+P_Num) += lamd_deform/2;
//        L_cur.coeffRef(idx_T[i]+2*P_Num, idx_T[i]+2*P_Num) += lamd_deform/2;
//    }
//    
//    for (decltype(idx_F.size()) i = 0; i != idx_F.size(); ++i) {
//        d.row(idx_F[i]) += (lamd_hard/2)*RowVector3f(F[3*i], F[3*i+1], F[3*i+2]);
//        L_cur.coeffRef(idx_F[i], idx_F[i]) += lamd_hard/2;
//        L_cur.coeffRef(idx_F[i]+P_Num, idx_F[i]+P_Num) += lamd_hard/2;
//        L_cur.coeffRef(idx_F[i]+2*P_Num, idx_F[i]+2*P_Num) += lamd_hard/2;
//    }
//
//    chol.factorize(L_cur);
//}
//
