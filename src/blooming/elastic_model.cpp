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



double ElasticModel::BasicAdjFacet::computeTheta( Eigen::Vector3d& x0, Eigen::Vector3d& x1, Eigen::Vector3d& x2, Eigen::Vector3d& x3 )
{
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

Eigen::Vector3d ElasticModel::BasicAdjFacet::skewMatrixRow( const Eigen::Vector3d& v, int s )
{
	assert( s<3 && s>0);
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
	assert( s<3 && s>0);
	EigenVector3 new_v(0,0,0);
	new_v(s) = s;
	return new_v;
}




ElasticModel::ElasticModel()
	:petal_num_(0),
	iter_num_(10),
	eps_(1e-2),
	lambda_(1.0),
	noise_p_(0.0),
	vertex_num_(0)
{

}

ElasticModel::ElasticModel(PointCloud* point_cloud, Flower* flower)
	:petal_num_(0),
	iter_num_(10), 
	eps_(1e-2),
	lambda_(0.01),
	noise_p_(0.0),
	point_cloud_(point_cloud),
	flower_(flower),
	vertex_num_(0)
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

}


double ElasticModel::m_step()
{
	std::cout << "M-Step:" << std::endl;

	int iter = 0;
	double eps = 0;

	double e = 0;

	updateLeftSys();
	updateRightSys();

	do {
		double e_n = solve();
		eps = std::fabs((e_n - e) / e_n);
		e = e_n;

		updateRightSys();

	}while(eps > eps_ && iter < iter_num_);

	return e;
}


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

		deform_petals_[i]._cloud_v = cm;
	}


	// init petal matrix
	for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
	{
		Petal& petal = petals.at(i);
		osg::ref_ptr<PointCloud> petal_cloud = point_cloud_->getPetalCloud(i);
		std::vector<int> knn_idx;
		petal.searchNearestIdx(petal_cloud, knn_idx);

		int petal_size = petal.getVertices()->size();
		PetalVector pm(3*petal_size);

		for (size_t j = 0, j_end = petal_size; j < j_end; ++ j)
		{
			pm.block_vector(j) = EigenVector3( petal_cloud->at(knn_idx[j]).x, petal_cloud->at(knn_idx[j]).y, petal_cloud->at(knn_idx[j]).z );
		}

		deform_petals_[i]._petal_v = pm;
	}


	// init correspondence matrix
	for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
	{
		CloudVector& cloud_mat = deform_petals_[i]._cloud_v;
		PetalVector& petal_mat = deform_petals_[i]._petal_v;
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

	// init facets
	for( int  i = 0, i_end = petal_num_; i<i_end; ++i )
	{
		Petal& petal = petals.at(i);
		std::vector<std::vector<int> >& triangle_list = petal.getFaces(); 
		std::vector<BasicFacet>& facets = deform_petals_[i]._facets_;
		for (size_t i = 0, i_end = triangle_list.size(); i < i_end; ++ i)
		{
			std::vector<int> face = triangle_list[i];
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
	

	// init covariance matrix 
	for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
	{
		std::vector<std::vector<int>>& adj_list = deform_petals_[i]._adj_list;
		PetalVector& origin_petal = deform_petals_[i]._origin_petal;
		CovVector& cov_v = deform_petals_[i]._cov_v;
		cov_v.resize(3, origin_petal.cols());

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

			cov_v.block_vector(k) = EigenVector3( s_x / adj_size, s_y / adj_size, s_z / adj_size ); 
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

	// µÃµ½¶¥µã
	vertex_num_ = 0;
	for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
	{
		ElasticPetal& elastic_petal = deform_petals_[i];
		vertex_num_ += elastic_petal._petal_v.rows();
	}


}

void ElasticModel::extractVertexEdges( int petal_id )
{
	ElasticPetal& elastic_petal = deform_petals_[petal_id];
	std::vector<BasicFacet>& facets = elastic_petal._facets_;
	std::vector<BasicEdge>& vertex_edges = elastic_petal._vertex_edges;

	unsigned int vertices_num = elastic_petal._petal_v.rows();
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
}

void ElasticModel::extractAdjFacets( int petal_id )
{
	ElasticPetal& elastic_petal = deform_petals_[petal_id];
	std::vector<BasicFacet>& facets = elastic_petal._facets_;
	std::vector<BasicAdjFacet>& adj_facets = elastic_petal._adj_facets_;

	unsigned int vertices_num = elastic_petal._petal_v.rows();
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
}



double ElasticModel::gaussian( int petal_id, int m_id, int c_id )
{
	double p;

	CovVector& cov_mat = deform_petals_[petal_id]._cov_v;
	CloudVector& cloud_mat = deform_petals_[petal_id]._cloud_v;
	PetalVector& petal_mat = deform_petals_[petal_id]._petal_v;

	Eigen::Vector3d xu = cloud_mat.col(c_id) - petal_mat.col(m_id);
	p = pow(2*M_PI, -3/2.0) * pow((cov_mat.col(m_id).asDiagonal()).toDenseMatrix().determinant(), -1/2.0) * 
		exp((-1/2.0)*xu.transpose()*cov_mat.col(m_id).asDiagonal().inverse()*xu);

	return p;
}


void ElasticModel::updateLeftSys()
{
	L_ = Eigen::SparseMatrix<double>();
	for (size_t i = 0; i < petal_num_; ++ i)
	{
		updateLeftSys(i);
	}
	L_.makeCompressed();
}

void ElasticModel::updateLeftSys( int petal_id )
{
// 	ElasticPetal& deform_petal = deform_petals_[petal_id];
// 	CovVector& cov_matrix = deform_petal._cov_matrix;
// 	CorresMatrix& corres_matrix = deform_petal._corres_matrix;
// 	WeightMatrix& weight_matrix = deform_petal._weight_matrix;
// 	AdjList& adj_list = deform_petal._adj_list;
// 	HardCtrsIdx& hc_idx = deform_petal._hc_idx;
// 	int ver_num = deform_petal._petal_matrix.cols();
// 
// 	std::vector<std::vector<Eigen::Triplet<double> > > weight_sums;
// 	weight_sums.resize(3); // for x,y,z coordinates
// 
// 	for (int i = 0; i < ver_num; ++i) 
// 	{
// 		double wi_x = 0, wi_y = 0, wi_z = 0;
// 
// 		for (size_t j = 0, j_end = deform_petal._adj_list[i].size(); j < j_end; ++j)
// 		{
// 			int id_j = deform_petal._adj_list[i][j];
// 			wi_x += weight_matrix.coeffRef(i, id_j);
// 			wi_y += weight_matrix.coeffRef(i, id_j);
// 			wi_z += weight_matrix.coeffRef(i, id_j);
// 		}
// 
// 		wi_x += zero_correction(lambda_*(2/cov_matrix.col(i)[0])*corres_matrix.row(i).sum());
// 		wi_y += zero_correction(lambda_*(2/cov_matrix.col(i)[1])*corres_matrix.row(i).sum());
// 		wi_z += zero_correction(lambda_*(2/cov_matrix.col(i)[2])*corres_matrix.row(i).sum());
// 
// 		weight_sums[0].push_back(Eigen::Triplet<double>(i, i, wi_x));
// 		weight_sums[1].push_back(Eigen::Triplet<double>(i, i, wi_y));
// 		weight_sums[2].push_back(Eigen::Triplet<double>(i, i, wi_z));
// 	}
// 
// 	Eigen::SparseMatrix<double> diag_coeff_x(ver_num, ver_num);
// 	Eigen::SparseMatrix<double> diag_coeff_y(ver_num, ver_num);
// 	Eigen::SparseMatrix<double> diag_coeff_z(ver_num, ver_num);
// 	diag_coeff_x.setFromTriplets(weight_sums[0].begin(), weight_sums[0].end());
// 	diag_coeff_y.setFromTriplets(weight_sums[1].begin(), weight_sums[1].end());
// 	diag_coeff_z.setFromTriplets(weight_sums[2].begin(), weight_sums[2].end());
// 
// 	// expand L to fill in new petal vertices
// 	int row_idx = L_[0].rows(), col_idx = L_[0].cols();  // rows and cols should be the same for x, y, z
// 
// 	L_[0].conservativeResize(row_idx+ver_num, col_idx+ver_num);
// 	L_[1].conservativeResize(row_idx+ver_num, col_idx+ver_num);
// 	L_[2].conservativeResize(row_idx+ver_num, col_idx+ver_num);
// 
// 	// block assignment is not supported by SparseMatrix...I have to update one by one
// 	Eigen::SparseMatrix<double> L_p_x = diag_coeff_x - weight_matrix;
// 	Eigen::SparseMatrix<double> L_p_y = diag_coeff_y - weight_matrix;
// 	Eigen::SparseMatrix<double> L_p_z = diag_coeff_z - weight_matrix;
// 
// 
// 	for (size_t i = 0; i < ver_num; ++ i)
// 	{
// 		int hc_id = deform_petal.isHardCtrs(i);
// 
// 		if ( hc_id != -1)
// 		{
// 			L_[0].coeffRef(row_idx+i, col_idx+i) = 1;
// 			L_[1].coeffRef(row_idx+i, col_idx+i) = 1;
// 			L_[2].coeffRef(row_idx+i, col_idx+i) = 1;
// 		}
// 		else
// 		{
// 			L_[0].coeffRef(row_idx+i, col_idx+i) = L_p_x.coeffRef(i, i);
// 			L_[1].coeffRef(row_idx+i, col_idx+i) = L_p_y.coeffRef(i, i);
// 			L_[2].coeffRef(row_idx+i, col_idx+i) = L_p_z.coeffRef(i, i);
// 
// 			for (size_t j = 0, j_end = adj_list[i].size(); j < j_end; ++ j)
// 			{
// 				int id_j = adj_list[i][j];
// 				L_[0].coeffRef(row_idx+i, col_idx+id_j) = L_p_x.coeffRef(i, id_j);
// 				L_[1].coeffRef(row_idx+i, col_idx+id_j) = L_p_y.coeffRef(i, id_j);
// 				L_[2].coeffRef(row_idx+i, col_idx+id_j) = L_p_z.coeffRef(i, id_j);
// 			}
// 		}
// 	}
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
	CloudVector& cloud_v = deform_petal._cloud_v;
	
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
	// bending
	AdjFacetList& adj_facets = deform_petals_[petal_id]._adj_facets_;
	for( int i = 0; i!= adj_facets.size(); ++i )
	{
		BasicAdjFacet& adj_facet = adj_facets[i];
		EigenVector3 x0 = x.block_vector( adj_facet.m_v0 ); 
		EigenVector3 x1 = x.block_vector( adj_facet.m_v1 );
		EigenVector3 x2 = x.block_vector( adj_facet.m_v2 );
		EigenVector3 x3 = x.block_vector( adj_facet.m_v3 );

		double stiffness = adj_facet.m_stiffness;
		double rest_angle = adj_facet.m_rest_angle;
		ScalarType theta = adj_facet.computeTheta( x0, x1, x2, x3 );
		ScalarType J = stiffness/2 * std::pow(theta - rest_angle,2 );

		e_b += J;
	}

	double e_o = 0; // observation
	CorresMatrix& corres_matrix = deform_petal._corres_matrix;
	PetalVector& origin_petal = deform_petal._origin_petal;
	for (size_t k = 0, k_end = corres_matrix.rows(); k < k_end; ++ k)
	{
		for (size_t n = 0, n_end = corres_matrix.cols(); n < n_end; ++ n)
		{
			EigenVector3 cm = cloud_v.block_vector(n) - x.block_vector(k);
			e_o += corres_matrix(k, n) * cm.transpose() * cov_v.block_vector(k).asDiagonal().inverse() * cm;
		}
	}

	double J = e_o + lambda_*( elastic_b*e_b + elastic_s*e_s );
	return J;

}

void ElasticModel::updateRightSys()
{

}

double ElasticModel::solve()
{
	return 0;
}

void ElasticModel::deforming()
{

}

