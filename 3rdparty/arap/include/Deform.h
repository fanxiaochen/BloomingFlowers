#ifndef _Deform_H
#define _Deform_H

#define ARAP_DLL_EXPORTS

#include <iostream>
#include <vector>
#include <cmath>

#include <Eigen\Eigen>
#include <Eigen\Sparse>

#include "WunderSVD3x3.h"

#include "arap_wrapper.h"

class ARAP_DLL_API Deform
{
public:
    typedef std::vector< std::vector<int> > AdjList;
    typedef std::vector< Eigen::Vector3i > FaceList;
    typedef std::vector< float > VectorF;
    typedef std::vector< int > VectorI;

public:
    Deform(){};
    ~Deform(){};
    Deform(float *P_data, int P_Num, AdjList &adj_list, FaceList &face_list);
    
    // old complete deform call
    float *do_Deform(VectorF &T, VectorI &idx_T);
    
    // get current P' point i stores from P_Prime[3*i + 0]
    float *get_P_Prime();
    
    // do deform one iterate, returned current P'
    float *do_Deform_Iter(float &delta);
    
    // set linear system. T is the target point, idx_T is the corresponding
    // indices of these points to model. You can tune lamd_deform for them
    void set_linear_sys(VectorF &T, VectorI &idx_T);
    
    // set linear system. F is another set of target point working 
    // similarly to T. You can tune lamd_hard for them
    void set_linear_sys(VectorF &T, VectorI &idx_T, VectorF &F, VectorI &idx_F);
    
    // set lamd. Default is 5 and 5
    void set_lamd(float lamd_deform, float lamd_hard);

private:
    void update_Ri();
    float update_P_Prime();
    float compute_wij(float *p1, float *p2, float *p3, float *p4 = nullptr);
    void find_share_Vertex(int i, int j, AdjList &adj_list, FaceList &face_list, VectorI &share_Vertex);

private:
    Eigen::Matrix3Xf P_Prime;
    Eigen::Matrix3Xf P;

    AdjList adj_list;
    Eigen::SparseMatrix<float> Weight;
    Eigen::SparseMatrix<float> L;
    std::vector<Eigen::Matrix3f> R;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<float>> chol;
    Eigen::MatrixX3f d;

    int max_iter;
    double min_delta;
    float lamd_deform;
    float lamd_hard;
};

#endif