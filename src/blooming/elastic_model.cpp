#include "elastic_model.h"
#include "WunderSVD3x3.h"

#include "point_cloud.h"
#include "flower.h"


class  LocalEdge 
{
public:
	LocalEdge( unsigned int f, unsigned int v, unsigned int local_v)
		:m_f(f)
		,m_v(v)
		,m_local_v(local_v)
	{

	}
public:
	unsigned int m_f;
	unsigned int m_v;
	unsigned int m_local_v;
};


#pragma region bending constraint
double ElasticModel::BasicAdjFacet::computeTheta( Eigen::VectorXd& X )
{
	EigenVector3 x0 = X.block_vector( m_v0 ); 
	EigenVector3 x1 = X.block_vector( m_v1 );
	EigenVector3 x2 = X.block_vector( m_v2 );
	EigenVector3 x3 = X.block_vector( m_v3 );

	EigenVector3 nA = (x2-x0).cross(x1-x0);
	EigenVector3 nA_n = nA.normalized();
	EigenVector3 nB = (x1-x3).cross(x2-x3);
	EigenVector3 nB_n = nB.normalized();
	EigenVector3 e = x1-x2;
	EigenVector3 e_n = e.normalized();
	ScalarType l_nA = nA.norm();
	ScalarType l_nB = nB.norm();
	ScalarType l_e = e.norm();
	ScalarType cos_theta = nA_n.dot(nB_n);
	ScalarType sin_theta = nA_n.cross(nB_n).dot(e_n);
	ScalarType theta = std::atan2( sin_theta, cos_theta );
	return theta;
}

double ElasticModel::BasicAdjFacet::computeStiffnessAndRestValue( Eigen::VectorXd& X )
{
	EigenVector3 x0 = X.block_vector( m_v0 ); 
	EigenVector3 x1 = X.block_vector( m_v1 );
	EigenVector3 x2 = X.block_vector( m_v2 );
	EigenVector3 x3 = X.block_vector( m_v3 );

	ScalarType area1 = ( (x2-x0).cross(x1-x0) ).norm();
	ScalarType area2 = ( (x2-x3).cross(x1-x3) ).norm();
	
	m_stiffness = (x1-x2).squaredNorm() /( area1 + area2 );

	m_rest_angle = computeTheta( X );

	return m_stiffness;
}

void ElasticModel::BasicAdjFacet::evaluateGradientAndHessian( const VectorX& x, 
															 VectorX& gradient, 
															 std::vector<SparseMatrixTriplet>& hessian_triplets )
{
	EigenVector3 x0 = x.block_vector( m_v0 );
	EigenVector3 x1 = x.block_vector( m_v1 );
	EigenVector3 x2 = x.block_vector( m_v2 );
	EigenVector3 x3 = x.block_vector( m_v3);

	EigenVector3 nA = (x2-x0).cross(x1-x0);
	EigenVector3 nA_n = nA.normalized();
	EigenVector3 nB = (x1-x3).cross(x2-x3);
	EigenVector3 nB_n = nB.normalized();
	EigenVector3 e = x1-x2;
	EigenVector3 e_n = e.normalized();
	ScalarType l_nA = nA.norm();
	ScalarType l_nB = nB.norm();
	ScalarType l_e = e.norm();
	ScalarType cos_theta = nA_n.dot(nB_n);
	ScalarType sin_theta = nA_n.cross(nB_n).dot(e_n);
	ScalarType theta = std::atan2( sin_theta,cos_theta );


	EigenMatrix3 I3 = EigenMatrix3::Identity();
	VectorX qA(12);
	qA.setZero();
	qA.block_vector(0) = x2-x1;
	qA.block_vector(1) = x0-x2;
	qA.block_vector(2) = x1-x0;
	VectorX qB(12);
	qB.setZero();
	qB.block_vector(1) = x2-x3;
	qB.block_vector(2) = x3-x1;
	qB.block_vector(3) = x1-x2;
	VectorX qe(4);
	qe << 0, 1, -1, 0;

	// graident
	// 临时存放这四个点的gradient
	VectorX grad_cos_all(4*3);
	VectorX grad_sin_all(4*3);
	VectorX grad_nA_norm_all(4*3*3);
	VectorX grad_nB_norm_all(4*3*3);
	VectorX grad_e_norm_all(4*3*3);
	VectorX grad_theta_all(4*3);

	// grad_grad_nA/(grad_x_m_s grad_x_n_t)
	for( int n = 0; n!= 4; ++n )
	{
		EigenVector3 qAn = qA.block_vector(n);
		EigenVector3 qBn = qB.block_vector(n);
		for( int t = 0; t != 3; ++t )
		{
			// grad
			EigenVector3 grad_nA = skewMatrixRow( qAn,t );
			EigenVector3 grad_nB = skewMatrixRow( qBn,t );
			EigenVector3 grad_e = identifyRow(t) * qe(n);
			// remove the assumption that the normal and edge vectors have constant length
			EigenVector3 grad_nA_norm = (I3- (nA_n*nA_n.transpose()) )*grad_nA/l_nA;
			EigenVector3 grad_nB_norm = (I3 - (nB_n*nB_n.transpose()) )*grad_nB/l_nB;
			EigenVector3 grad_e_norm = (I3 - (e_n*e_n.transpose()))*identifyRow(t)*qe(n)/l_e;
			ScalarType grad_cos = grad_nA_norm.dot(nB_n) + nA_n.dot( grad_nB_norm );
			ScalarType grad_sin = ( grad_nA_norm.cross(nB_n) +  (nA_n).cross(grad_nB_norm) ).dot(e_n)
				+ (nA_n.cross(nB_n)).dot(grad_e_norm) ;
			ScalarType grad_theta = cos_theta * grad_sin - sin_theta*grad_cos;
			int idx_local = 3*n+t;
			grad_cos_all(idx_local) = grad_cos;
			grad_sin_all(idx_local) = grad_sin;
			grad_theta_all(idx_local) = grad_theta;
			grad_nA_norm_all.block_vector(idx_local) =  grad_nA_norm ;
			grad_nB_norm_all.block_vector(idx_local) =  grad_nB_norm ;
			grad_e_norm_all.block_vector(idx_local) = grad_e_norm;

			int idx = 3*getIdx(n)+t;
			ScalarType grad_fb = m_stiffness*(theta- m_rest_angle)*grad_theta;
			gradient(idx) += grad_fb;
		}
	}


	Eigen::MatrixXd grad_qA( 12,12 );
	grad_qA.setZero();
	grad_qA.block_matrix(1,0) = I3;
	grad_qA.block_matrix(2,0) = I3;
	grad_qA.block_matrix(0,1) = I3;
	grad_qA.block_matrix(2,1) = I3;
	grad_qA.block_matrix(0,2) = I3;
	grad_qA.block_matrix(1,2) = I3;
	Eigen::MatrixXd grad_qB(12,12);
	grad_qB.setZero();
	grad_qB.block_matrix(2,1) = -I3;
	grad_qB.block_matrix(3,1) = I3;
	grad_qB.block_matrix(1,2) = I3;
	grad_qB.block_matrix(3,2) = -I3;
	grad_qB.block_matrix(1,3) = -I3;
	grad_qB.block_matrix(2,3) = I3;

	// grad_grad_nA/(grad_x_m_s grad_x_n_t)
	for( int n = 0; n!= 4; ++n )
	{
		EigenVector3 qAn = qA.block_vector(n);
		EigenVector3 qBn = qB.block_vector(n);
		for( int t = 0; t != 3; ++t )
		{
			int idx_local_nt = 3*n+t; 
			EigenVector3 gradnAnt = grad_nA_norm_all.block_vector(idx_local_nt);
			EigenVector3 gradnBnt = grad_nB_norm_all.block_vector(idx_local_nt);
			EigenVector3 gradent = grad_e_norm_all.block_vector(idx_local_nt);
			ScalarType gradcosnt = grad_cos_all(idx_local_nt);
			ScalarType gradsinnt = grad_sin_all(idx_local_nt);
			ScalarType gradThetant = grad_theta_all(idx_local_nt);
			// hession
			VectorX tmp_qA = grad_qA.col(idx_local_nt);
			VectorX tmp_qB = grad_qB.col(idx_local_nt);
			for( int m = 0; m!=4; ++m)
			{	
				EigenVector3 qAm = qA.block_vector(m);
				EigenVector3 qBm = qB.block_vector(m);
				for( int s = 0; s != 3; ++s )
				{
					int idx_local_ms = 3*m+s; 
					EigenVector3 gradnAms = grad_nA_norm_all.block_vector(idx_local_ms);
					EigenVector3 gradnBms = grad_nB_norm_all.block_vector(idx_local_ms);
					EigenVector3 gradems = grad_e_norm_all.block_vector(idx_local_ms);
					ScalarType gradcosms = grad_cos_all(idx_local_ms);
					ScalarType gradsinms = grad_sin_all(idx_local_ms);
					ScalarType gradThetams = grad_theta_all(idx_local_ms);
					EigenVector3 hess_nA = skewMatrixRow( tmp_qA.block_vector(m), s);
					EigenVector3 hess_nB = skewMatrixRow(tmp_qB.block_vector(m), s);
					ScalarType hess_e = 0;
					// remove the assumption that the normal and edge vectors have constant length
					EigenVector3 hess_nA_norm = -( (gradnAnt*nA_n.transpose()) + (nA_n*gradnAnt.transpose()) )*skewMatrixRow(qAm,s)/l_nA
						+ (I3- (nA_n*nA_n.transpose()) )/(l_nA*l_nA)*( hess_nA*l_nA 
						- skewMatrixRow(qAn,t).dot( nA_n) * skewMatrixRow(qAm,s) );
					EigenVector3 hess_nB_norm = -( (gradnBms*nB_n.transpose()) + (nB_n*gradnBms.transpose()) )*skewMatrixRow(qBm,s)/(l_nB)
						+ (I3- (nB_n*nB_n.transpose()) )/(l_nB*l_nB)*( hess_nB*l_nB 
						- skewMatrixRow(qBm,s) *(skewMatrixRow(qBn,t ).dot( nB_n) ) );
					EigenVector3 hess_e_norm = -( (gradent*nB_n.transpose()) + (nB_n*gradent.transpose()) )* skewMatrixRow(qBm,s)/l_nB
						+ (I3- (e_n*e_n.transpose()))/(l_e*l_e)*qe(m)*qe(n)*e_n(t)*identifyRow(s);

					ScalarType hess_cos = hess_nA_norm.dot(nB_n) + gradnBnt.dot(gradnAms) + gradnAnt.dot(gradnBms) + nA_n.dot( hess_nA_norm );
					ScalarType hess_sin = ( hess_nA_norm.cross(nB_n) + gradnAms.cross(gradnBnt) + gradnAnt.cross(gradnBms) + nA_n.cross(hess_nA_norm) ).dot(e_n)
						+ (gradnAms.cross(nB_n) + nA_n.cross(gradnBms)).dot(gradent) 
						+ (gradnAnt.cross(nB_n) + nA_n.cross(gradnBnt)).dot(gradems)
						+ (nA_n.cross(nB_n)).dot(hess_e_norm);
					ScalarType hess_theta = cos_theta*hess_sin - sin_theta*hess_cos
						+ (sin_theta*sin_theta - cos_theta*cos_theta)*(gradsinms*gradcosnt + gradcosms*gradsinnt)
						+ 2*sin_theta*cos_theta*(gradcosms*gradcosnt - gradsinms*gradsinnt );

					ScalarType hess_fb = m_stiffness* ( gradThetams*gradThetant  + (theta- m_rest_angle)*hess_theta );
					hessian_triplets.push_back( SparseMatrixTriplet( 3*getIdx(m)+s, 3*getIdx(n)+t, hess_fb) );
				}
			}
		}
	}
}

Eigen::Vector3d ElasticModel::BasicAdjFacet::skewMatrixRow( const Eigen::Vector3d& v, int s )
{
	assert( s<3 && s>=0);
	EigenVector3 new_v;
	switch (s)
	{
	case 0:
		new_v = EigenVector3( 0, -v.z(), v.y() );
		break;
	case 1:
		new_v = EigenVector3( v.z(), 0, -v.x() );
		break;
	default:
		new_v = EigenVector3( -v.y(), v.x(), 0);
		break;
	}
	return new_v;
}

Eigen::Vector3d ElasticModel::BasicAdjFacet::identifyRow( int s )
{
	assert( s<3 && s>=0);
	EigenVector3 new_v(0,0,0);
	new_v(s) = s;
	return new_v;
}

int ElasticModel::BasicAdjFacet::getIdx( int v_id )
{
	assert( v_id <=3 && v_id >=0 );
	int id = 0;
	switch (v_id)
	{
	case 0:
		id = m_v0;
		break;
	case 1:
		id = m_v1;
		break;
	case 2:
		id = m_v2;
		break;
	default:
		id = m_v3;
		break;
	}
	return id;
}


#pragma endregion

#pragma region stretching constraint
double ElasticModel::BasicEdge::computeStiffnessAndRestValue( Eigen::VectorXd& X )
{
	EigenVector3 x1 = X.block_vector( m_v1 );
	EigenVector3 x2 = X.block_vector( m_v2 );
	m_stiffness = 1/(x1-x2).squaredNorm();

	m_rest_length = (x1-x2).norm();

	return m_stiffness;

}

void ElasticModel::BasicEdge::evaluateGradientAndHessian( const VectorX& x, 
														 VectorX& gradient, 
														 std::vector<SparseMatrixTriplet>& hessian_triplets )
{
	EigenMatrix3 I3 = EigenMatrix3::Identity();
	IndexType idxi = m_v1;
	IndexType idxj = m_v2;
	EigenVector3 x_ij = x.block_vector(idxi) - x.block_vector(idxj);
	EigenVector3 x_ij_n = x_ij.normalized();
	ScalarType l_ij = x_ij.norm();

	// gradient
	EigenVector3 g_ij = x_ij.normalized()*(m_stiffness*(l_ij-m_rest_length));
	gradient.block_vector( idxi ) += g_ij; 
	gradient.block_vector( idxj ) -= g_ij; 

	// hessian
	EigenMatrix3 k = ( I3- (I3 - (x_ij_n*x_ij_n.transpose()))*m_rest_length/l_ij )*m_stiffness;
	for (int row = 0; row < 3; row ++)
	{
		for (int col = 0; col < 3; col ++)
		{
			double val = k(row,col);
			//Update the global hessian matrix
			hessian_triplets.push_back(SparseMatrixTriplet(3*idxi+row, 3*idxi+col, val));
			hessian_triplets.push_back(SparseMatrixTriplet(3*idxi+row, 3*idxj+col, -val));
			hessian_triplets.push_back(SparseMatrixTriplet(3*idxj+row, 3*idxi+col, -val));
			hessian_triplets.push_back(SparseMatrixTriplet(3*idxj+row, 3*idxj+col, val));
		}
	}
}
#pragma endregion




ElasticModel::ElasticModel()
	:petal_num_(0),
	iter_num_(30),
	eps_(1e-4),
	lambda_(1.0),
	noise_p_(0.0),
	weight_b(1),
	weight_s(100),
	h_(0.1)
{

}

ElasticModel::ElasticModel(PointCloud* point_cloud, Flower* flower)
	:petal_num_(0),
	iter_num_(30), 
	eps_(1e-2),
	lambda_(1),
	noise_p_(0.0),
	point_cloud_(point_cloud),
	flower_(flower),
	weight_b(0),
	weight_s(10),
	h_(1)
{

}


ElasticModel::~ElasticModel()
{

}

void ElasticModel::setIterNum(int iter_num)
{
	iter_num_ = iter_num;
}

void ElasticModel::setEps(double eps)
{
	eps_ = eps;
}

void ElasticModel::setPointCloud(PointCloud* point_cloud)
{
	point_cloud_ = point_cloud;
}

void ElasticModel::setFlower(Flower* flower)
{
	flower_ = flower;
}


void ElasticModel::deform()
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



void ElasticModel::e_step()
{
	std::cout << "E-Step: No explicit output" << std::endl;

	for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
	{
		e_step(i);
	}
}

void ElasticModel::e_step( int petal_id )
{
	CorresMatrix& corres_mat = deform_petals_[petal_id]._corres_matrix;
	VisList& vis_list = deform_petals_[petal_id]._vis_list;

	for (size_t i = 0, i_end = corres_mat.cols(); i < i_end; ++ i)
	{
		for (size_t j = 0, j_end = corres_mat.rows(); j < j_end; ++ j)
		{
			corres_mat(j, i) = gaussian(petal_id, j, i) * vis_list[j];
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


double ElasticModel::m_step()
{
	std::cout << "M-Step:" << std::endl;

	int iter = 0;
	double eps = 0;

	double e = energy();
	std::cout << "energy: " << e<< std::endl;


	updateSys();

	do {
		double e_n = solve();

		eps = std::abs( e - e_n );
		e = e_n;

		std::cout << "energy: " << e<< std::endl;
		++iter;


		updateSys();

	}while(eps > eps_ /*|| iter < iter_num_*/);

	return e;
}


//************************************
// Method:    initialize
// Returns:   void
// Function:  初始化
// Time:      2015/01/15： 能跑通
// Time:      2015/01/16:  将_petal_vector_的初始设成原始初值
// Author:    Qian
//************************************
void ElasticModel::initialize()
{
	Petals& petals = flower_->getPetals();
	petal_num_ = petals.size();

	deform_petals_.resize(petal_num_);

	// init origin petal
	for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
	{
		Petal& petal = petals.at(i);
		int petal_size = petal.getVertices()->size();
		PetalVector pm(3*petal_size);

		for (size_t j = 0, j_end = petal_size; j < j_end; ++ j)
		{
			pm.block_vector(j) = EigenVector3( petal.getVertices()->at(j).x(), petal.getVertices()->at(j).y(), petal.getVertices()->at(j).z() );
		}

		deform_petals_[i]._origin_petal = pm;
	}

	// init cloud matrix
	point_cloud_->flower_segmentation(flower_); // using knn to segment the point cloud to petals, and along with visibility determination

	for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
	{
		osg::ref_ptr<PointCloud> petal_cloud = point_cloud_->getPetalCloud(i);
		CloudVector cm(3*petal_cloud->size());
		if (petal_cloud != NULL)
		{
			for (size_t j = 0, j_end = petal_cloud->size(); j < j_end; ++ j)
			{
				cm.block_vector(j) =  EigenVector3( petal_cloud->at(j).x, petal_cloud->at(j).y, petal_cloud->at(j).z );
			}
		}

		deform_petals_[i]._cloud_vector = cm;
	}


	// init petal matrix  by point cloud and oriangle petal
	for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
	{
		Petal& petal = petals.at(i);
		osg::ref_ptr<PointCloud> petal_cloud = point_cloud_->getPetalCloud(i);
		std::vector<int> knn_idx;
		petal.searchNearestIdx(petal_cloud, knn_idx);

		int petal_size = petal.getVertices()->size();
		PetalVector pm(3*petal_size);

		double w_c = 0.5;
		for (size_t j = 0, j_end = petal_size; j < j_end; ++ j)
		{
			EigenVector3 v1( petal_cloud->at(knn_idx[j]).x, petal_cloud->at(knn_idx[j]).y, petal_cloud->at(knn_idx[j]).z );
			EigenVector3 v2( petal.getVertices()->at(j).x(), petal.getVertices()->at(j).y(), petal.getVertices()->at(j).z() );
			pm.block_vector(j) = w_c*v1 + (1-w_c)*v2;
		
		}
		deform_petals_[i]._petal_v = pm;
	}

	


	// init correspondence matrix (  )
	for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
	{
		CloudVector& cloud_v = deform_petals_[i]._cloud_vector;
		PetalVector& petal_v = deform_petals_[i]._petal_v;

		int point_num = cloud_v.size()/3;
		int vertex_num = petal_v.size()/3;

		CorresMatrix corres_mat = CorresMatrix::Zero(vertex_num, point_num);
		deform_petals_[i]._corres_matrix = corres_mat;
	}

	// init visibility list
	for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
	{
		Petal& petal = petals.at(i);
		deform_petals_[i]._vis_list = VisList( petal.getVertices()->size(), 1);
//		deform_petals_[i]._vis_list = petal.getVisibility();
		
	}

	// init adjacent list
	for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
	{
		Petal& petal = petals.at(i);
		deform_petals_[i]._adj_list = petal.getAdjList();
	}

	// init facets
	for( int  i = 0, i_end = petal_num_; i<i_end; ++i )
	{
		Petal& petal = petals.at(i);
		std::vector<std::vector<int> >& triangle_list = petal.getFaces(); 
		std::vector<BasicFacet>& facets = deform_petals_[i]._facets_;
		for (size_t i = 0, i_end = triangle_list.size(); i < i_end; ++ i)
		{
			std::vector<int>& face = triangle_list[i];
			assert( face.size() == 3);
			BasicFacet facet;
			facet.m_idx0 = face[0];
			facet.m_idx1 = face[1];
			facet.m_idx2 = face[2];
			facet.m_facet_idx = i;
			facets.push_back( facet );
		}
	}

	// init adjacent facets
	for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
	{
		extractAdjFacets( i );
	}

	// init vertex edges
	for( size_t i = 0, i_end = petal_num_; i < i_end; ++i )
	{
		extractVertexEdges(i);
	}

	// init index_range
	for( size_t i = 0, i_end = petal_num_; i < i_end; ++i )
	{
		IndexRange& id_range = deform_petals_[i]._index_range_;
		if( i== 0 )
		{
			id_range.m_min = 0;
			id_range.m_max = petals[i].getVertices()->size();
		}
		else
		{
			id_range.m_min = deform_petals_[i-1]._index_range_.m_max;
			id_range.m_max = id_range.m_min + petals[i].getVertices()->size();
		}
	}
	

	// init covariance matrix 
	for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
	{
		std::vector<std::vector<int>>& adj_list = deform_petals_[i]._adj_list;
		PetalVector& origin_petal = deform_petals_[i]._origin_petal;
		CovVector& cov_v = deform_petals_[i]._cov_v;
		cov_v.resize(origin_petal.size());

		for (size_t k = 0, k_end = adj_list.size(); k < k_end; ++ k)
		{
			Eigen::Vector3d c = origin_petal.block_vector(k);
			int adj_size = adj_list[k].size();
			EigenVector3 sv(0,0,0);

			for (size_t j = 0, j_end = adj_size; j < j_end; ++ j)
			{
				int id_j = adj_list[k][j];
				Eigen::Vector3d v = origin_petal.block_vector(id_j);

				sv += (c-v).cwiseAbs2();
			}

			cov_v.block_vector(k) = sv/adj_size; 
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

void ElasticModel::extractVertexEdges( int petal_id )
{
	ElasticPetal& elastic_petal = deform_petals_[petal_id];
	std::vector<BasicFacet>& facets = elastic_petal._facets_;
	std::vector<BasicEdge>& vertex_edges = elastic_petal._vertex_edges;

	unsigned int vertices_num = elastic_petal._origin_petal.size()/3;
	vertex_edges.clear();
	std::map<std::pair<int,int>, std::vector<LocalEdge>> mapAdjFacet;
	// Loop over faces
	for(int i = 0;i< facets.size();i++)
	{
		// Get indices of edge: s --> d
		BasicFacet& f = facets[i];
		int k1, k2;
		k1 = f.m_idx0 > f.m_idx1 ? f.m_idx1:f.m_idx0;
		k2 = (f.m_idx0+f.m_idx1) - k1;
		LocalEdge e0( f.m_facet_idx, f.m_idx2, 2);
		mapAdjFacet[ std::make_pair(k1,k2)].push_back( e0  );

		k1 = f.m_idx1 > f.m_idx2 ? f.m_idx2:f.m_idx1;
		k2 = (f.m_idx1+f.m_idx2) - k1;
		LocalEdge e1( f.m_facet_idx, f.m_idx1, 1);
		mapAdjFacet[ std::make_pair(k1,k2)].push_back( e1 );


		k1 = f.m_idx2 > f.m_idx0 ? f.m_idx0:f.m_idx2;
		k2 = (f.m_idx2+f.m_idx0) - k1;
		LocalEdge e2( f.m_facet_idx, f.m_idx0, 0);
		mapAdjFacet[ std::make_pair(k1,k2)].push_back( e2 );		
	}


	for( std::map<std::pair<int,int>, std::vector<LocalEdge>>::iterator mit = mapAdjFacet.begin();
		mit != mapAdjFacet.end(); ++mit )
	{
		BasicEdge e;
		e.m_v1 = mit->first.first;
		e.m_v2 = mit->first.second;
		e.m_f0 = (mit->second)[0].m_f;
		e.m_local_v0 = (mit->second)[0].m_local_v;
		e.m_count = mit->second.size();

		if( mit->second.size() == 2) 
		{
			e.m_f1 = (mit->second)[1].m_f;
			e.m_local_v1 = (mit->second)[1].m_local_v;
		}
		vertex_edges.push_back( e );
	}

	for( int i = 0 ;i!= vertex_edges.size(); ++i )
	{
		vertex_edges[i].computeStiffnessAndRestValue( elastic_petal._origin_petal );
	}
}

void ElasticModel::extractAdjFacets( int petal_id )
{
	ElasticPetal& elastic_petal = deform_petals_[petal_id];
	std::vector<BasicFacet>& facets = elastic_petal._facets_;
	std::vector<BasicAdjFacet>& adj_facets = elastic_petal._adj_facets_;

	unsigned int vertices_num = elastic_petal._origin_petal.size()/3;
	adj_facets.clear();
	std::map<std::pair<int,int>, std::vector<LocalEdge>> mapAdjFacet;
	// Loop over faces
	for(int i = 0;i< facets.size();i++)
	{
		// Get indices of edge: f.m_idx0 --> f.m_idx1
		BasicFacet& f = facets[i];
		int k1, k2;
		k1 = f.m_idx0 > f.m_idx1 ? f.m_idx1:f.m_idx0;
		k2 = (f.m_idx0+f.m_idx1) - k1;
		LocalEdge e0( f.m_facet_idx, f.m_idx2, 2);
		mapAdjFacet[ std::make_pair(k1,k2)].push_back( e0  );

		// Get indices of edge: f.m_idx1 --> f.m_idx2
		k1 = f.m_idx1 > f.m_idx2 ? f.m_idx2:f.m_idx1;
		k2 = (f.m_idx1+f.m_idx2) - k1;
		LocalEdge e1( f.m_facet_idx, f.m_idx0, 0);
		mapAdjFacet[ std::make_pair(k1,k2)].push_back( e1 );

		// Get indices of edge: f.m_idx2 --> f.m_idx0
		k1 = f.m_idx2 > f.m_idx0 ? f.m_idx0:f.m_idx2;
		k2 = (f.m_idx2+f.m_idx0) - k1;
		LocalEdge e2( f.m_facet_idx, f.m_idx1, 1);
		mapAdjFacet[ std::make_pair(k1,k2)].push_back( e2 );		
	}
	for( std::map<std::pair<int,int>, std::vector<LocalEdge>>::iterator mit = mapAdjFacet.begin();
		mit != mapAdjFacet.end(); ++mit )
	{
		if( mit->second.size() == 2) 
		{
			BasicAdjFacet adj_facet;
			adj_facet.m_v0 = (mit->second)[0].m_v;
			adj_facet.m_v1 = mit->first.first;
			adj_facet.m_v2 = mit->first.second;
			adj_facet.m_v3 = (mit->second)[1].m_v;

			adj_facet.m_f0 = (mit->second)[0].m_f;
			adj_facet.m_f1 = (mit->second)[1].m_f;
			adj_facet.m_local_v0 = (mit->second)[0].m_local_v;
			adj_facet.m_local_v1 = (mit->second)[1].m_local_v;
			adj_facets.push_back( adj_facet );
		}
	}


	// 设置stiffness & rest length
	for( int i = 0; i != adj_facets.size(); ++i )
	{
		BasicAdjFacet& adj_facet = adj_facets[i];
		adj_facet.computeStiffnessAndRestValue( elastic_petal._origin_petal );
	}
}



double ElasticModel::gaussian( int petal_id, int m_id, int c_id )
{
	double p;

	CovVector& cov_v = deform_petals_[petal_id]._cov_v;
	CloudVector& cloud_v = deform_petals_[petal_id]._cloud_vector;
	PetalVector& petal_v = deform_petals_[petal_id]._petal_v;

	Eigen::Vector3d xu = cloud_v.block_vector(c_id) - petal_v.block_vector(m_id);
	p = pow(2*M_PI, -3/2.0) * pow((cov_v.block_vector(m_id).asDiagonal()).toDenseMatrix().determinant(), -1/2.0) * 
		exp((-1/2.0)*xu.transpose()*cov_v.block_vector(m_id).asDiagonal().inverse()*xu);

	return p;
}


void ElasticModel::updateSys()
{
	if( petal_num_ ==0 )
		return;

	int dim_num = 3* deform_petals_[petal_num_-1]._index_range_.m_max;
	L_ = Eigen::SparseMatrix<double>( dim_num, dim_num );
	d_ = VectorX(dim_num);
	d_.setZero();

	for (size_t i = 0; i < petal_num_; ++ i)
	{
		// 似乎不对,改用Newton's method
		// updateSys1(i);
		updateSys2(i);
	}
	L_.makeCompressed();
}



double ElasticModel::energy()
{
	double J = 0;
	for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
	{
		J += evaluateEnergy(i);
	}
	return J;
}


double ElasticModel::evaluateEnergy( int petal_id )
{
	ElasticPetal& deform_petal = deform_petals_[petal_id];
	PetalVector& x = deform_petal._petal_v;
	CovVector& cov_v = deform_petal._cov_v;
	CloudVector& cloud_v = deform_petal._cloud_vector;
	
	// stretch
	double e_s = 0;
	VertexEdgeList& vertex_edges = deform_petals_[petal_id]._vertex_edges;
	// return 0.5*( ||x_i-x_j|| - l_0) )^2/(L)^2
	for( int i = 0;i!= vertex_edges.size(); ++i )
	{
		BasicEdge& v_e = vertex_edges[i];
		double l_ij = ( x.block_vector( v_e.m_v1) - x.block_vector( v_e.m_v2 ) ).norm();
		double stiffness = v_e.m_stiffness;
		double rest_length = v_e.m_rest_length;
		// cost
		double J = stiffness/2*std::pow((l_ij-rest_length),2);
		e_s += J;
	}

	double e_b = 0;
// 	// bending
// 	AdjFacetList& adj_facets = deform_petals_[petal_id]._adj_facets_;
// 	for( int i = 0; i!= adj_facets.size(); ++i )
// 	{
// 		BasicAdjFacet& adj_facet = adj_facets[i];
// 		double stiffness = adj_facet.m_stiffness;
// 		double rest_angle = adj_facet.m_rest_angle;
// 		ScalarType theta = adj_facet.computeTheta( x );
// 		ScalarType J = stiffness/2 * std::pow(theta - rest_angle,2 );
// 
// 		e_b += J;
// 	}

	double e_o = 0; // observation
	CorresMatrix& corres_matrix = deform_petal._corres_matrix;
	int point_num = corres_matrix.cols();
	int vertex_num = corres_matrix.rows();
	for (size_t k = 0, k_end = vertex_num; k < k_end; ++ k)
	{
		EigenMatrix3 cov_m = cov_v.block_vector(k).asDiagonal().inverse();
		for (size_t n = 0, n_end = corres_matrix.cols(); n < n_end; ++ n)
		{
			EigenVector3 cm = cloud_v.block_vector(n) - x.block_vector(k);
			e_o += corres_matrix(k, n) * cm.transpose() * cov_m * cm;
		}
	}

	double J = lambda_*e_o + ( weight_b*e_b + weight_s*e_s );
	return J;
}


void ElasticModel::evaluateGradientAndHessian( int petal_id, const VectorX& x, 
											  VectorX& gradient, std::vector<SparseMatrixTriplet>& hessian_triplets )
{
	int dim_num = x.rows();
	gradient.resize(dim_num);
	gradient.setZero();
	hessian_triplets.clear();

	ElasticPetal& deformed_petal = deform_petals_[petal_id];
	
	// stretching
	VectorX graident_s(dim_num);
	graident_s.setZero();
	std::vector<SparseMatrixTriplet> hessian_s;
	VertexEdgeList& springs = deformed_petal._vertex_edges;
	for( int i= 0; i!= springs.size(); ++i )
	{
		BasicEdge& spring = springs[i];
		spring.evaluateGradientAndHessian( x, graident_s, hessian_s);
	}
	// weight
	graident_s *= weight_s;
	for( int i = 0; i!= hessian_s.size(); ++i )
	{
		SparseMatrixTriplet triple(hessian_s[i].row(), hessian_s[i].col(), hessian_s[i].value()*weight_s); 
		hessian_s[i] = SparseMatrixTriplet( triple );
	}

	// bending
	VectorX graident_b(dim_num);
	graident_b.setZero();
	std::vector<SparseMatrixTriplet> hessian_b;
// 	AdjFacetList& adj_facets = deformed_petal._adj_facets_;
// 	for( int i = 0; i!= adj_facets.size(); ++i )
// 	{
// 		BasicAdjFacet& adj_facet = adj_facets[i];
// 		adj_facet.evaluateGradientAndHessian( x, graident_b, hessian_b );
// 	}
// 	// weight
// 	graident_b *= weight_b;
// 	for( int i = 0; i!=hessian_b.size(); ++i )
// 	{
// 		SparseMatrixTriplet triple(hessian_b[i].row(), hessian_b[i].col(), hessian_b[i].value()*weight_b); 
// 		hessian_b[i] = SparseMatrixTriplet( triple );
// 	}

	// observation
	CorresMatrix& corres_matrix = deformed_petal._corres_matrix;
	CovVector& cov_v = deformed_petal._cov_v;
	CloudVector& cloud_v = deformed_petal._cloud_vector;
	int vertex_num = dim_num/3;
	int point_num = cloud_v.rows()/3;
	VectorX graident_o(dim_num);
	graident_o.setZero();
	std::vector<SparseMatrixTriplet> hessian_o;
	for( int i = 0 ;i!= vertex_num; ++i )
	{
		EigenVector3 xi = x.block_vector(i);
		EigenVector3 cov_i = cov_v.block_vector(i);
		EigenVector3 g_i(0,0,0);
		for( int j = 0; j!= point_num; ++j )
		{
			EigenVector3 cm =  xi - cloud_v.block_vector(j);
			g_i += 2*corres_matrix(i,j)*cm;
		}
		graident_o.block_vector(i) += g_i.cwiseQuotient(cov_i);
		EigenVector3 hv = cov_i* 2* ( cov_v.row(i).sum() );
		hessian_o.push_back( SparseMatrixTriplet( 3*i+0, 3*i+0, lambda_*hv(0) ) );
		hessian_o.push_back( SparseMatrixTriplet( 3*i+1, 3*i+1, lambda_*hv(1) ) );
		hessian_o.push_back( SparseMatrixTriplet( 3*i+2, 3*i+2, lambda_*hv(2) ) );
	}
	graident_o *= lambda_;

	gradient += graident_s;
	gradient += graident_b;
	gradient += graident_o;
	hessian_triplets.insert( hessian_triplets.end(), hessian_s.begin(), hessian_s.end() );
	hessian_triplets.insert( hessian_triplets.end(), hessian_b.begin(), hessian_b.end() );
	hessian_triplets.insert( hessian_triplets.end(), hessian_o.begin(), hessian_o.end() );

}


void ElasticModel::updateSys1( int petal_id )
{
	ElasticPetal& deform_petal = deform_petals_[petal_id];
	CovVector& cov_v = deform_petal._cov_v;
	CorresMatrix& corres_matrix = deform_petal._corres_matrix;
	AdjList& adj_list = deform_petal._adj_list;
	HardCtrsIdx& hc_idx = deform_petal._hc_idx;
	IndexRange id_range = deform_petal._index_range_;

	PetalVector& petal_v = deform_petal._petal_v;
	int ver_num = petal_v.rows()/3;

 	VectorX gradient;
	std::vector<SparseMatrixTriplet> hessian_triples;
	evaluateGradientAndHessian( petal_id, petal_v,  gradient, hessian_triples );

	// update the left matrix L_
	int sym_dim = 3*deform_petals_.rbegin()->_index_range_.m_max;
	SparseMatrix hessian_mat( sym_dim, sym_dim);
	int delta_dim = 3*id_range.m_min;
	for( int i = 0 ;i!= hessian_triples.size(); ++i )
	{
		SparseMatrixTriplet triplet( hessian_triples[i].row()+delta_dim, hessian_triples[i].col()+delta_dim, hessian_triples[i].value() );
		hessian_triples[i] = triplet;
	}
	for( int i = 0;i!= 3*ver_num; ++i )
	{
		hessian_triples.push_back( SparseMatrixTriplet( delta_dim + i, delta_dim+i, 1/h_) );
	}
	hessian_mat.setFromTriplets( hessian_triples.begin(), hessian_triples.end() );
	L_ += hessian_mat;

	// update the right column d_
	d_.block( delta_dim, delta_dim, 3*ver_num, 1) -= gradient;	
}


void ElasticModel::updateSys2( int petal_id )
{
	ElasticPetal& deform_petal = deform_petals_[petal_id];
	CovVector& cov_v = deform_petal._cov_v;
	CorresMatrix& corres_matrix = deform_petal._corres_matrix;
	AdjList& adj_list = deform_petal._adj_list;
	HardCtrsIdx& hc_idx = deform_petal._hc_idx;
	IndexRange id_range = deform_petal._index_range_;

	PetalVector& petal_v = deform_petal._petal_v;
	int ver_num = petal_v.rows()/3;

	VectorX gradient;
	std::vector<SparseMatrixTriplet> hessian_triples;
	evaluateGradientAndHessian( petal_id, petal_v,  gradient, hessian_triples );

	// update the left matrix L_
	int sym_dim = 3*deform_petals_.rbegin()->_index_range_.m_max;
	SparseMatrix hessian_mat( sym_dim, sym_dim);
	int delta_dim = 3*id_range.m_min;
	for( int i = 0 ;i!= hessian_triples.size(); ++i )
	{
		SparseMatrixTriplet triplet( hessian_triples[i].row()+delta_dim, hessian_triples[i].col()+delta_dim, hessian_triples[i].value() );
		hessian_triples[i] = triplet;
	}
	hessian_mat.setFromTriplets( hessian_triples.begin(), hessian_triples.end() );
	L_ += hessian_mat;

	// update the right column d_
	d_.block( delta_dim, delta_dim, 3*ver_num, 1) -= gradient;	
}



double ElasticModel::solve()
{
	std::string warning_msg;
	VectorX delta_pos;
	conjugateGradientSolver( L_, d_, delta_pos,  warning_msg);
	//赋值
	for( int i = 0; i!= petal_num_; ++i )
	{
		PetalVector& petal_v = deform_petals_[i]._petal_v;
		IndexRange& id_range = deform_petals_[i]._index_range_;
		petal_v += delta_pos.block( 3*id_range.m_min, 0, 3*(id_range.m_max-id_range.m_min), 1);
	}

	return energy();
}

void ElasticModel::deforming()
{
	Petals& petals = flower_->getPetals();

	for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
	{
		Petal& petal = petals.at(i);
		PetalVector& petal_v = deform_petals_[i]._petal_v;

		for (size_t j = 0, j_end = petal.getVertices()->size(); j < j_end; ++ j)
		{
			EigenVector3 v = petal_v.block_vector(j);
			petal.getVertices()->at(j).x() = v.x();
			petal.getVertices()->at(j).y() = v.y();
			petal.getVertices()->at(j).z() = v.z();
		}
	}
}

bool ElasticModel::conjugateGradientSolver( const SparseMatrix& A, const VectorX& b, VectorX& x, std::string& warning_msg )
{
	Eigen::ConjugateGradient<SparseMatrix> cg;
	cg.compute(A);
	x = cg.solve(b);

	std::stringstream ss;
	ss <<  "#iterations:     " << cg.iterations() << "\n" << "estimated error: " << cg.error()      << "\n";
	warning_msg = ss.str();
	if( cg.info() != Eigen::Success )
	{
		return false;
	}
	else
	{
		return true;
	}
	warning_msg = ss.str();

}


double ElasticModel::zero_correction(double value)
{
	double min_double = std::numeric_limits<double>::min();
	if (min_double > value)
		return min_double;
	return value;
}






