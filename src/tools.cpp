#include <iostream>
#include "tools.h"


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() 
{
	MaxAcceleration = 0;
	previousetimeStamp = 0; 
	previousVelocity = 0;
	isInit = false;
}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth) {

	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	if (estimations.size() == 0 || estimations.size() != ground_truth.size())
		return rmse;
	VectorXd residuals;
	//accumulate squared residuals
	for (int i = 0; i < estimations.size(); ++i) {
		residuals = ((estimations[i] - ground_truth[i]).array())*((estimations[i] - ground_truth[i]).array());
		rmse += residuals;
	}

	//calculate the mean
	rmse = (1.0 / estimations.size())*(rmse);
	//calculate the squared root
	for (double j = 0; j < rmse.size(); j++) rmse(j) = sqrt(rmse(j));

	//return the result
	return rmse;
}
#pragma optimize( "", off )
double Tools::GetMaxAcceleration(double currentvelocity, double currentimeStamp)
{
	
	if (!isInit)
	{
		previousVelocity = currentvelocity;
		previousetimeStamp = currentimeStamp;
		isInit = true;
		return 0;
	}
	double diftime = abs(previousetimeStamp - currentimeStamp) / pow(10,6);
	double difVelocity = abs(previousVelocity - currentvelocity);
	previousVelocity = currentvelocity;
	previousetimeStamp = currentimeStamp;
	double acc = difVelocity / diftime;
	if (abs(acc) > MaxAcceleration && abs(acc)<9)
	{
		MaxAcceleration = abs(acc);
	}
	
	return MaxAcceleration;
}