#include "SMMD.h"

SMMD::SMMD()
{
	roadNetwork = NULL;
}

SMMD::SMMD(Map& roadNetwork_)
{
	roadNetwork = &roadNetwork_;
}

int SMMD::doSMMD_old(GeoPoint* x, GeoPoint* d, double (SMMD::*p)(Edge*, GeoPoint*, GeoPoint*))
{
	double thresholdM = 50.0;
	vector<Edge*> candidateRoads;
	//roadNetwork->getNearEdges(x->lat, x->lon, thresholdM, candidateRoads);

	roadNetwork->getNearEdges(x->lat, x->lon, 2, candidateRoads);
	if (candidateRoads[1] == NULL || candidateRoads[0] == NULL)
	{
		system("pause");
	}
	/*
	if (SMMD::distM_signed(candidateRoads[0], x) > 0)
		return candidateRoads[0]->id;
	else
		return candidateRoads[1]->id;
		*/


	double maxProb = -1;
	int returnId = -1;

	for each(Edge* r in candidateRoads)
	{
		if (r->r_hat.size() == 0)
			continue;
		double tempProb = (this->*p)(r, x, d);
		if (tempProb > maxProb)
		{
			maxProb = tempProb;
			returnId = r->id;
		}
	}
	return returnId;
}

int SMMD::doSMMD(GeoPoint* x, GeoPoint* d, double(SMMD::*p)(Edge*, GeoPoint*, GeoPoint*))
{
	double thresholdM = 50.0;
	vector<Edge*> candidateRoads;
	roadNetwork->getNearEdges(x->lat, x->lon, thresholdM, candidateRoads);
	//roadNetwork->getNearEdges(x->lat, x->lon, 2, candidateRoads);
	
	double maxProb = -1;
	int returnId = -1;

	for each(Edge* r in candidateRoads)
	{
		if (r->r_hat.size() == 0)
			continue;
		double tempProb = (this->*p)(r, x, d);
		if (tempProb > maxProb)
		{
			maxProb = tempProb;
			returnId = r->id;
		}
	}
	return returnId;
}

double SMMD::probSMMD(Edge* r, GeoPoint* x, GeoPoint* d)
{
	return likelihood_x_Simple(r, x) * likelihood_d(r, d) * (r->prior + 1);
}

double SMMD::probSMM(Edge* r, GeoPoint* x, GeoPoint* d)
{
	//return likelihood_x_Simple(r, x) * (r->prior + 1);
	//printf("likelihood:%lf, prior:%d\n", likelihood_x_Simple(r, x), (r->trainData.size() + 1));
	//system("pause");
	return likelihood_x_Simple(r, x) * (r->trainData.size()+1);
}

double SMMD::likelihood_x_Simple(Edge* r, GeoPoint* x)
{
	double dist = distM_signed(r, x);
	int slotId = r->getSlotId(x);
	double mu = r->thetas[slotId].mu;
	double sigma = r->thetas[slotId].sigma;
	return 1 / (2.506628275 * sigma) * exp(-(dist - mu) * (dist - mu) / (2 * sigma * sigma));
}

double SMMD::likelihood_d(Edge* r, GeoPoint* d)
{
	//////////////////////////////////////////////////////////////////////////
	///计算p(d|r)
	//////////////////////////////////////////////////////////////////////////
	double phi = cosAngle(roadNetwork->nodes[r->startNodeId], roadNetwork->nodes[r->endNodeId], d);
	//printf("phi = %lf\n", acos(phi) / PI * 180);
	//cout << gamma_d / (exp(gamma_d) - exp(-gamma_d)) * exp(gamma_d * phi) << endl;
	//system("pause");
	return gamma_d / (exp(gamma_d) - exp(-gamma_d)) * exp(gamma_d * phi);
}

double SMMD::likelihood_d_Beta(Edge* r, GeoPoint* d)
{
	double phi = cosAngle(roadNetwork->nodes[r->startNodeId], roadNetwork->nodes[r->endNodeId], d);
	double beta_fenzi = 0.2323 * pow(phi, 0.5891-1) * pow(1-phi, (0.2916 - 1));
	double beta_fenmu = 0.3118 * pow(phi, 0.4961 - 1) * pow(1 - phi, (0.4890 - 1));
	return beta_fenzi / beta_fenmu;
}

double SMMD::distM_signed(Edge* r, GeoPoint* x)
{
	//////////////////////////////////////////////////////////////////////////
	///返回x到r_hat的投影距离，有正负号
	///1)对于x匹配到的子线段(a,b)，符号为(x-a)叉积(b-a)
	///2)对于x匹配到顶点的情况，顶点认为是b，往前走一个认为是a
	///3)对于匹配到顶点，且顶点是r_hat的第一个顶点a0的情况，认为a是a0,b是a1
	//////////////////////////////////////////////////////////////////////////
	double minDist = INF;
	int a = 0;
	//遍历端点距离
	for (int i = 0; i < r->r_hat.size(); i++)
	{
		double tmpDist = GeoPoint::distM(x, r->r_hat[i]);
		if (tmpDist < minDist)
		{
			minDist = tmpDist;
			if (i != 0)
				a = i - 1; //condition (2)
			else
				a = 0; //condition (3)
		}
	}
	//遍历投影距离
	for (int i = 0; i < r->r_hat.size() - 1; i++)
	{
		
		/**********************************************************/
		/*test code starts from here*/
		if (i+1 >= r->r_hat.size())
		{
			printf("i+1 = %d, size = %d\n", i + 1, r->r_hat.size());
			system("pause");
		}
		/*test code ends*/
		/**********************************************************/
		
		if (cosAngle(x, r->r_hat[i], r->r_hat[i + 1]) <= 0 && cosAngle(x, r->r_hat[i + 1], r->r_hat[i]) <= 0) //确认x投影落在r_hat[i~i+1]上
		{
			double A = (r->r_hat[i + 1]->lat - r->r_hat[i]->lat);
			double B = -(r->r_hat[i + 1]->lon - r->r_hat[i]->lon);
			double C = r->r_hat[i]->lat * (r->r_hat[i + 1]->lon - r->r_hat[i]->lon)
				- r->r_hat[i]->lon * (r->r_hat[i + 1]->lat - r->r_hat[i]->lat);
			/*double rate = 1000;
			A *= rate;
			B *= rate;
			C *= rate;*/
			double tmpDist = abs(A * x->lon + B * x->lat + C) / sqrt(A * A + B * B);
			tmpDist *= GeoPoint::geoScale;
			if (minDist > tmpDist)
			{
				minDist = tmpDist;
				a = i; //condition (1)
			}
		}
	}
	//计算符号
	int b = a + 1;
	/**********************************************************/
	/*test code starts from here*/
	//异常情况
	if (a < 0 || b < 0 || b >= r->r_hat.size())
	{
		cout << "[异常][in dist_M_signed()] = " << a << ", b = " << b << endl;
		system("pause");
		exit(0);
	}
	/*test code ends*/
	/**********************************************************/
	//(x-a)叉积(b-a)
	GeoPoint p(x->lon - r->r_hat[a]->lon, x->lat - r->r_hat[a]->lat);   //x-a
	GeoPoint q(r->r_hat[b]->lon - r->r_hat[a]->lon, r->r_hat[b]->lat - r->r_hat[a]->lat); //b-a
	if (p.lon * q.lat - q.lon * p.lat > 0)
		return minDist;
	else
		return -minDist;
}

double SMMD::distM_signed(vector<GeoPoint*> polyline, GeoPoint* x)
{
	//////////////////////////////////////////////////////////////////////////
	///返回x到r_hat的投影距离，有正负号
	///1)对于x匹配到的子线段(a,b)，符号为(x-a)叉积(b-a)
	///2)对于x匹配到顶点的情况，顶点认为是b，往前走一个认为是a
	///3)对于匹配到顶点，且顶点是r_hat的第一个顶点a0的情况，认为a是a0,b是a1
	//////////////////////////////////////////////////////////////////////////
	double minDist = INF;
	int a = 0;
	//遍历端点距离
	for (int i = 0; i < polyline.size(); i++)
	{
		double tmpDist = GeoPoint::distM(x, polyline[i]);
		if (tmpDist < minDist)
		{
			minDist = tmpDist;
			if (i != 0)
				a = i - 1; //condition (2)
			else
				a = 0; //condition (3)
		}
	}
	//遍历投影距离
	for (int i = 0; i < polyline.size() - 1; i++)
	{

		/**********************************************************/
		/*test code starts from here*/
		if (i + 1 >= polyline.size())
		{
			printf("i+1 = %d, size = %d\n", i + 1, polyline.size());
			system("pause");
		}
		/*test code ends*/
		/**********************************************************/

		if (cosAngle(x, polyline[i], polyline[i + 1]) <= 0 && cosAngle(x, polyline[i + 1], polyline[i]) <= 0) //确认x投影落在r_hat[i~i+1]上
		{
			double A = (polyline[i + 1]->lat - polyline[i]->lat);
			double B = -(polyline[i + 1]->lon - polyline[i]->lon);
			double C = polyline[i]->lat * (polyline[i + 1]->lon - polyline[i]->lon)
				- polyline[i]->lon * (polyline[i + 1]->lat - polyline[i]->lat);
			/*double rate = 1000;
			A *= rate;
			B *= rate;
			C *= rate;*/
			double tmpDist = abs(A * x->lon + B * x->lat + C) / sqrt(A * A + B * B);
			tmpDist *= GeoPoint::geoScale;
			if (minDist > tmpDist)
			{
				minDist = tmpDist;
				a = i; //condition (1)
			}
		}
	}
	//计算符号
	int b = a + 1;
	/**********************************************************/
	/*test code starts from here*/
	//异常情况
	if (a < 0 || b < 0 || b >= polyline.size())
	{
		cout << "[异常][in dist_M_signed()] = " << a << ", b = " << b << endl;
		system("pause");
		exit(0);
	}
	/*test code ends*/
	/**********************************************************/
	//(x-a)叉积(b-a)
	GeoPoint p(x->lon - polyline[a]->lon, x->lat - polyline[a]->lat);   //x-a
	GeoPoint q(polyline[b]->lon - polyline[a]->lon, polyline[b]->lat - polyline[a]->lat); //b-a
	if (p.lon * q.lat - q.lon * p.lat > 0)
		return minDist;
	else
		return -minDist;
}

double SMMD::cosAngle(GeoPoint* pt1, GeoPoint* pt2, GeoPoint* pt3)
{
	//////////////////////////////////////////////////////////////////////////
	///cos<p1->p2, p2->p3>
	//////////////////////////////////////////////////////////////////////////	
	double v1x = pt2->lon - pt1->lon;
	double v1y = pt2->lat - pt1->lat;
	double v2x = pt3->lon - pt2->lon;
	double v2y = pt3->lat - pt2->lat;
	return (v1x * v2x + v1y * v2y) / sqrt((v1x * v1x + v1y * v1y)*(v2x * v2x + v2y * v2y));
}
