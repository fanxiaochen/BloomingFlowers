#ifndef ELASTIC_MODEL_H
#define ELASTIC_MODEL_H


class PointCloud;
class Flower;

#include <Eigen/Dense>
#include <Eigen/Sparse>

// eigen vector accessor
#define block_vector(a) block<3,1>(3*(a), 0)
// eigen matrix accessor
#define  block_matrix(a, b)  block<3,3>(3*(a), 3*(b))


class ElasticModel
{
private:
	typedef double ScalarType;
	typedef int    IndexType;
	typedef Eigen::VectorXd  VectorX;
	typedef Eigen::MatrixXd  CorresMatrix;
	typedef Eigen::VectorXd  CloudVector;
	typedef Eigen::VectorXd  PetalVector;
	typedef Eigen::VectorXd  CovVector;

	typedef Eigen::Vector3d  EigenVector3;      // x,y,z
	typedef Eigen::Matrix3d  EigenMatrix3;
	typedef Eigen::Triplet<double>   SparseMatrixTriplet;
	typedef Eigen::SparseMatrix<double> SparseMatrix;
	typedef std::vector<int> VisList;
	typedef std::vector<std::vector<int> > AdjList;
	typedef std::vector<Eigen::Vector3i > FaceList;
	typedef Eigen::SparseMatrix<double> WeightMatrix;
	typedef std::vector<Eigen::Matrix3d> RotList;
	typedef std::vector<int> HardCtrsIdx;

	// 基本类型
	struct IndexRange
	{
		IndexType m_min;   // 最小范围
		IndexType m_max;   // 最大范围
	};

	class BasicFacet
	{
	public:
		unsigned int  m_idx0;   //0
		unsigned int  m_idx1;   //1
		unsigned int  m_idx2;   //2
		unsigned int  m_facet_idx;  // facet_idx
	};
	class BasicAdjFacet
	{
	public:
		BasicAdjFacet()
			:m_stiffness(0)
		{

		}
		double computeTheta( Eigen::VectorXd& X  );
		double computeStiffness( Eigen::VectorXd& X );
		void evaluateGradientAndHessian( const VectorX& x, VectorX& gradient, std::vector<SparseMatrixTriplet>& hessian_triplets );

	private:
		// S(v) = (0, -v_z, v_y; v_z, 0, -v_x, -v_y, v_x, 0)
		inline Eigen::Vector3d  skewMatrixRow( const Eigen::Vector3d& v, int s);
		inline Eigen::Vector3d  identifyRow( int s );
		inline int  getIdx(int v_id);

	public:
		unsigned int m_v0, m_v1, m_v2, m_v3;  // 相邻的两个三角形（m_i0,m_i1,m_i2; m_i1,m_i2,m_i3）
		unsigned int m_f0, m_local_v0;        // (m_i0,m_i1,m_i2)对应的三角形id 以及 m_i0对应的局部id
		unsigned int m_f1, m_local_v1;        // (m_i1,m_i2,m_i3)对应的三角形id 以及 m_i3对应的局部id

		// 		double m_sq_L;    // the square of the edge length
		// 		double m_area;    // the sum of the areas of the two triangles sharing the edge e;
		double m_stiffness;
		double m_rest_angle;

	};
	class BasicEdge
	{
	public:
		BasicEdge()
			:m_stiffness(0)
			,m_rest_length(0)
		{

		}
		double computeStiffness( Eigen::VectorXd& X );
		void evaluateGradientAndHessian( const VectorX& x, VectorX& gradient, std::vector<SparseMatrixTriplet>& hessian_triplets );

		unsigned int m_v1, m_v2; // indices of endpoint vertices/faces
		unsigned int m_count;    // 次数
		unsigned int m_f0, m_local_v0;   // (m_i0,m_i1,m_i2)对应的三角形id 以及 m_i0对应的局部id
		unsigned int m_f1, m_local_v1;   // (m_i1,m_i2,m_i3)对应的三角形id 以及 m_i3对应的局部id
		/*		double   m_inv_sq_L;     // the invert of the square of the edge length*/
		double   m_stiffness;
		double   m_rest_length;  // 当前的rest length
	};
	typedef std::vector<BasicAdjFacet>   AdjFacetList;
	typedef std::vector<BasicEdge>       VertexEdgeList;
	typedef std::vector<BasicFacet>      FacetList;
	

	struct ElasticPetal
	{
		PetalVector     _origin_petal;
		PetalVector     _petal_v;
		CloudVector     _cloud_vector;    // 3n*1;
		CorresMatrix    _corres_matrix;   // m*n (m: vertex num; n: point num);
		CovVector       _cov_v;
		VisList         _vis_list;
		AdjList			 _adj_list;  // vertex's one-rings
		AdjFacetList	 _adj_facets_;
		VertexEdgeList	 _vertex_edges;
		FacetList        _facets_;
		IndexRange       _index_range_;   // 顶点的值的范围

		HardCtrsIdx     _hc_idx;

	};

public:
	ElasticModel();
	ElasticModel(PointCloud* point_cloud, Flower* flower);
	virtual ~ElasticModel();

	void setPointCloud(PointCloud* point_cloud);
	void setFlower(Flower* flower);

	void setIterNum(int iter_num);
	void setEps(double eps);
	void setLambda(double lambda);

	inline PointCloud* getPointCloud(){ return point_cloud_; }
	inline Flower* getFlower(){ return flower_; }

	void deform();

protected:
	void e_step();
	double m_step();

	void e_step(int petal_id);

	void visibility();

	void initialize();

	void extractVertexEdges( int petal_id );
	void extractAdjFacets( int petal_id );

	double gaussian(int petal_id, int m_id, int c_id);

	void updateSys();

	double solve();  // m_step时，执行一步隐式梯度下降迭代

	void updateSys(int petal_id);

	double energy();
	double zero_correction(double value);

	void deforming();

	int systemDimension();


	double evaluateEnergy(int petal_id );
	void evaluateGradientAndHessian(int petal_id, const VectorX& x, 
		VectorX& gradient, std::vector<SparseMatrixTriplet>& hessian_matrix);
	bool conjugateGradientSolver( const SparseMatrix& A, const VectorX& b,  VectorX& x, std::string& warning_msg );


private:
	PointCloud* point_cloud_;
	Flower* flower_;

	std::vector<ElasticPetal> deform_petals_;

	Eigen::SparseMatrix<double>  L_;
	Eigen::Matrix3Xd d_;
	Eigen::SparseLU<Eigen::SparseMatrix<double> > lu_solver_;
	//    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double> > lu_solver_;

	int petal_num_;

	int iter_num_;
	double eps_;

	double lambda_;
	double noise_p_;

	double weight_b;
	double weight_s;

	double h_;
         
};


#endif