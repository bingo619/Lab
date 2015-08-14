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
	int doSMMD_old(GeoPoint* x, GeoPoint* d, double(SMMD::*p)(Edge*, GeoPoint*, GeoPoint*)); //done
	int doSMMD(GeoPoint* x, GeoPoint* d, double(SMMD::*p)(Edge*, GeoPoint*, GeoPoint*));

	double probSMMD(Edge* r, GeoPoint* x, GeoPoint* d); //done, need test
	double probSMM(Edge* r, GeoPoint* x, GeoPoint* d);

	//private:
	double likelihood_d(Edge* r, GeoPoint* d); //done
	double likelihood_d_Beta(Edge* r, GeoPoint* d);

	double prior_r(Edge* r);

	double likelihood_x_Simple(Edge* r, GeoPoint* x); //done
	
	//functions
	static double distM_signed(Edge* r, GeoPoint* x); //done
	static double distM_signed(vector<GeoPoint*> polyline, GeoPoint* x);
	static double cosAngle(GeoPoint* pt1, GeoPoint* pt2, GeoPoint* pt3); //done
};
