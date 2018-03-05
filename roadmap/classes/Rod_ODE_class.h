#pragma once
#include <iostream>
#include <fstream>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <ctime>
#include <math.h>

namespace bnu = boost::numeric::ublas;
using namespace boost::numeric::odeint;
using namespace std;

typedef vector< double > State;
typedef vector<vector< double >> Matrix;
typedef vector<vector<vector< double >>> MatrixList;
typedef vector<vector<vector<vector< double >>>> MatrixListList;

// ODE solvers
typedef runge_kutta_dopri5< State > stepper_type; //
typedef runge_kutta_cash_karp54< State > stepper_type1;

void ode_function_solve(State, State &, double);
void ode_function_feasibility(State, State &, double);
void ode_function_A_path_solve(State x, State &dxdt, double t);
State dxdt_func(State x, State u, State mu, State M, State J);

class rod_ode
{
private:
	// Properties
	int points_on_rod; // Discretization size of the rod
	double L; // Length of rod
	//q0; // For now assume identity
	double solution_time;
	State c; // Stiffness of the rod
	
	// Solution data
	MatrixList T; // Transformation matrices along the rod 4x4
	MatrixList J; // Jacobian matrices along the rod 6x6
	Matrix P; // Discretization coordinates along the rod
	State t_length; // Discretization length

	// Solution data for A_path
	MatrixListList Jlist; // Transformation matrices along the rod 4x4 for each conf. along a set of a's
	MatrixList Plist; // Discretization coordinates along the rod for each conf. along a set of a's
	MatrixListList Tlist; // Transformation matrices along the rod 4x4 for each conf. along a set of a's
	State path_conjugate_points; // Stores the conjugate points for a set of a's

	int collision_point;
	const double rod_collision_tol = 16;

public:
	// Constructor
	rod_ode();
	// Destructor
	~rod_ode() {};

	void rod_solve(State); // Solve rod for one 'a'
	void rod_path_solve(Matrix); // Solve rod for a path of a's = A_path
	void LogSolutionData(State, const double, int); // Log solution data through the computation
	void LogSolutionDataG(State, const double, int); // Log solution data through the computation - for the solution with only \mu, M and J.
	void LogSolutionDataPath(State, const double, int); // Log solution data through the computation during solution of a path of a's

	// Is rod configuration feasible
	bool isRodFeasible(State a); // Check if conf. 'a' is feasible (stability and collision free) - discrete geometry version
	bool isRodFeasible_old(State a); // Check if conf. 'a' is feasible (stability and collision free) - full ode version
	bool Rod_Collision(); // Check if rod is in self-collision for one 'a' conf., checks all points on the rod
	int conjugate_point(State a); // Find the first conjugate point along the rod - discrete geometry version
	int conjugate_point_old(State a); // Find the first conjugate point along the rod - full ode version
	void conjugate_point_path(Matrix A); // Finds the first conjugate points for a set of a's

	// Setters
	void setDiscretizationSize(int); // Sets number of points on the rod
	void setC(double); // Set stiffness of rod
	void setL(double); // Set length of rod

	// Getters
	State getP(int);
	Matrix getPMatrix();
	Matrix getT(int);
	Matrix getJ(int);
	double get_t_length(int);
	int get_Points_on_Rod();
	State get_conjugate_points_path();
	Matrix getJ_from_Jlist(int conf, int index);
	Matrix getT_from_Tlist(int conf, int index);

	// Misc
	template <class T>
	void initMatrix(T &M, int n, int m) {
		M.resize(n);
		for (int i = 0; i < n; ++i)
			M[i].resize(m);
	}
	void initMatrixList(MatrixList &, int, int, int);
	void initMatrixListList(MatrixListList &, int, int, int, int);
	void initPathLogLists(int);

	double det(int n, Matrix mat); // Compute the determinant of a 6x6 matrix - NOT IN USE - Replaced by 'determinant'
	double determinant(Matrix mat) const; // Compute the determinant of a 6x6 matrix with boost - LU decomposition
	
	void print_A_State(State) const;
	int min(int, int);

	// Performance parameters
	int odes_counter;
	int valid_odes_counter;
	double odes_time;
	int get_odes_counter() {
		return odes_counter;
	}
	void set_odes_counter(int num) {
		odes_counter = num;
	}
	int get_valid_odes_counter() {
		return valid_odes_counter;
	}
	void set_valid_odes_counter(int num) {
		valid_odes_counter = num;
	}
	double get_odes_time() {
		return odes_time;
	}
	void set_odes_time(double num) {
		odes_time = num;
	}

	// Stream the integrate_**** to m_out (file or cout)
	struct streaming_observer
	{
		std::ostream &m_out;
		streaming_observer(std::ostream &out) : m_out(out) {}

		void operator()(const State &x, double t) const
		{
			m_out << t << " ";
			for (size_t i = 3; i <= 11; i += 4)
				m_out << " " << x[i];
			m_out << "\n";
		}

		// Should add this in the solve function
		//ofstream myfile("PointsOnRod.txt");
		//integrate_const(stepper_type(), ode_function, x, 0.0, L, dt, streaming_observer(myfile));
		//myfile.close();
	};
	
};


