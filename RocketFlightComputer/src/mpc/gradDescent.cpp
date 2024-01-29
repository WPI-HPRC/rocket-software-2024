#include "gradDescent.h"
#include <iostream>
#include <fstream>
#include <math.h>

// Constructor.
gradDescent::gradDescent()
{
    // These are for the rocket optimization, payload will need something else 
	m_nDims = 1;
	m_stepSize = 0.2;
	m_maxIter = 25;
	m_h = 0.001;
	m_gradientThresh = 10.0;
}

// Function to set the object function.
void gradDescent::SetObjectFcn(std::function<double(std::vector<double>*)> objectFcn)
{
    // Set the object function.
	m_objectFcn = objectFcn;
}

// Function to perform the actual optimization.
std::vector<double> gradDescent::Optimize(std::vector<double> *funcLoc, double *funcVal)
{

    // set the start point to the previous control input
    m_startPoint = *funcLoc;
	
	// Loop up to max iterations or until threshold reached.
	int iterCount = 0;
	double gradientMagnitude = std::numeric_limits<double>::max();
	while ((iterCount < m_maxIter) && (gradientMagnitude > m_gradientThresh))
	{
		// Compute the gradient vector.
		std::vector<double> gradientVector = ComputeGradientVector();
		gradientMagnitude = ComputeGradientMagnitude(gradientVector);
	
		// Compute the new point.
		std::vector<double> newPoint = m_currentPoint;
		for (int i=0; i<m_nDims; ++i)
		{
			newPoint[i] += -(gradientVector[i] * m_stepSize);
		}
		
		// Update the current point.
		m_currentPoint = newPoint;
		
		// Increment the iteration counter.
		iterCount++;
	}
	
	// Return the results.
	*funcLoc = m_currentPoint;
	*funcVal = m_objectFcn(&m_currentPoint);
	
	return *funcLoc;
}

/* Function to compute the gradient of the object function in the
	specified dimension. */
double gradDescent::ComputeGradient(int dim)
{
	// Make a copy of the current location.
	std::vector<double> newPoint = m_currentPoint;
	
	// Modify the copy, according to h and dim.
	newPoint[dim] += m_h;
	
	// Compute the two function values for these points.
	double funcVal1 = m_objectFcn(&m_currentPoint);
	double funcVal2 = m_objectFcn(&newPoint);
	
	// Compute the approximate numerical gradient.
	return (funcVal2 - funcVal1) / m_h;
}

// Function to compute the gradient vector.
std::vector<double> gradDescent::ComputeGradientVector()
{
	std::vector<double> gradientVector = m_currentPoint;
	for (int i=0; i<m_nDims; ++i)
		gradientVector[i] = ComputeGradient(i);
		
	return gradientVector;
}

// Function to compute the gradient magnitude.
double gradDescent::ComputeGradientMagnitude(std::vector<double> gradientVector)
{
	double vectorMagnitude = 0.0;
	for (int i=0; i<m_nDims; ++i)
		vectorMagnitude += gradientVector[i] * gradientVector[i];
		
	return sqrt(vectorMagnitude);
}