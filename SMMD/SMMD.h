#pragma once
#include "GeoPoint.h"
#include "Map.h"
#include "Integral.h"
#include <math.h>
#include <iostream>
#include <thread>
#include <mutex>
using namespace std;
#define PI 3.141592653



class SMMD
{
public:
	friend class Integral;
	SMMD(Map& roadNetwork_);
	SMMD();
	Map* roadNetwork;
	//double gamma_d = 0.7812; //likelihood_d的指数分布的参数
	double gamma_d = 0.727843;//0.738174; //likelihood_d的指数分布的参数
	int doSMMD(GeoPoint* x, GeoPoint* d, double(SMMD::*p)(Edge*, GeoPoint*, GeoPoint*)); //done

	double prob(Edge* r, GeoPoint* x, GeoPoint* d); //done, need test
	double prob_Simple(Edge* r, GeoPoint* x, GeoPoint* d); //done, need test
	double probSMM(Edge* r, GeoPoint* x, GeoPoint* d);
	double prob_Ex(Edge* r, GeoPoint* x, GeoPoint* d);

	//private:
	double likelihood_x_Ex(Edge* r, GeoPoint* x); //
	double likelihood_x(Edge* r, GeoPoint* x); //done, need test
	double likelihood_d(Edge* r, GeoPoint* d); //done
	double likelihood_d_Beta(Edge* r, GeoPoint* d);

	double prior_r(Edge* r);

	double likelihood_x_Simple(Edge* r, GeoPoint* x); //done
	

	//functions for integral
	static double funcInGamma(double x, double* args, int argLen); //done
	static double funcFor2DInt(double mu, double tau, double *args, int argLen); //done
	//functions
	static double distM_signed(Edge* r, GeoPoint* x); //done
	static double gamma(double alpha); //done
	static double cosAngle(GeoPoint* pt1, GeoPoint* pt2, GeoPoint* pt3); //done
};
