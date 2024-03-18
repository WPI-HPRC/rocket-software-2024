#include <vector>
#include <functional>

class gradDescent
{
	public:
		// Constructor (default) / destructor.
		gradDescent();
	
		// Function to add a pointer to the object function.
		void SetObjectFcn(std::function<double(std::vector<double>*)> objectFcn);
	
		// Function to perform the optimization.
		std::vector<double> Optimize(std::vector<double> *funcLoc, double *funcVal);
	
	// Private functions.
	private:
		// Function to compute the gradient in the specified dimension.
		double ComputeGradient(int dim);
		
		// Function to compute the gradient vector.
		std::vector<double> ComputeGradientVector();
		
		// Function to compute the gradient magnitude.
		double ComputeGradientMagnitude(std::vector<double> gradientVector);
	
	// Private member variables.
	private:
		// The number of degrees of freedom of the system.
		int m_nDims;
		
		// The step size.
		double m_stepSize;
		
		// The maximum number of iterations.
		int m_maxIter;
		
		// The gradient step size (h).
		double m_h;
		
		// The gradient magnitude threshold (stopping condition).
		double m_gradientThresh;
		
		// The start point.
		std::vector<double> m_startPoint;
		
		// The current point.
		std::vector<double> m_currentPoint;

		// Function-pointer to the object function.
		std::function<double(std::vector<double>*)> m_objectFcn;

};