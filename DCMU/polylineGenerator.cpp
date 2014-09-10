/* 
 * Last Updated at [2014/3/4 15:47] by wuhao
 */
#include "PolylineGenerator.h"

double dist(Pt& pt1, Pt& pt2)
{
	return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
}

double dist(double x1, double y1, double x2, double y2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

double cosAngle(Pt& pt1, Pt& pt2, Pt& pt3)
{
	//////////////////////////////////////////////////////////////////////////
	///���ء�pt1_pt2_pt3��cosֵ
	//////////////////////////////////////////////////////////////////////////
	double v1x = pt1.x - pt2.x;
	double v1y = pt1.y - pt2.y;
	double v2x = pt3.x - pt2.x;
	double v2y = pt3.y - pt2.y;
	return (v1x * v2x + v1y * v2y) / sqrt((v1x * v1x + v1y * v1y)*(v2x * v2x + v2y * v2y));
}

void PolylineGenerator::sampling(int sampleNum)
{
	//////////////////////////////////////////////////////////////////////////
	///ʹ�����������pts�����������sampleNum����
	//////////////////////////////////////////////////////////////////////////
	if (pts.size() < sampleNum)
		return;
	list<Pt> tempPts;
	vector<Pt> pts_vec;
	for (list<Pt>::iterator iter = pts.begin(); iter != pts.end(); ++iter)
		pts_vec.push_back(*iter);
	while (tempPts.size() < sampleNum)
	{
		int vitimId = int(((double)rand()) / RAND_MAX * (pts_vec.size() - 1));
		tempPts.push_back(pts_vec[vitimId]);
	}
	pts = tempPts;
}

pair<double, double> PolylineGenerator::central_difference(double (PolylineGenerator::*f)(double, double, int), int vi)
{
	//////////////////////////////////////////////////////////////////////////
	///��df/dx@x, df/dy@y
	//////////////////////////////////////////////////////////////////////////
	double preIterValue, currentIterValue;
	int k;
	//int max_it = 100;
	//double eps = 1e-5;
	double h = 0.01;
	preIterValue = 0.0;
	pair<double, double> ans;
	double x = polyline[vi].x;
	double y = polyline[vi].y;
	
	//��df/dx
	for (k = 0; k < max_it; k++)
	{
		currentIterValue = ((this->*f)(x + h, y, vi) - (this->*f)(x - h, y, vi)) / (h + h);
		if (fabs(currentIterValue - preIterValue) < stopEps)
		{
			ans.first = currentIterValue;
			break;
		}
		h *= 0.5;
		preIterValue = currentIterValue;
	}
	if (k == max_it)
	{
		printf("<df/dx>δ�ܴﵽ����Ҫ��,�������������!\n");
		printf("preIter = %.8lf, currentIter = %.8lf\n", preIterValue, currentIterValue);
		system("pause");
	}
	ans.first = currentIterValue;

	//��df/dy
	preIterValue = 0.0;
	h = 0.01;
	for (k = 0; k < max_it; k++)
	{
		currentIterValue = ((this->*f)(x, y + h, vi) - (this->*f)(x, y - h, vi)) / (h + h);
		if (fabs(currentIterValue - preIterValue) < stopEps)
		{
			ans.second = currentIterValue;
			break;
		}
		h *= 0.5;
		preIterValue = currentIterValue;
	}
	if (k == max_it)
	{
		printf("<df/dy>δ�ܴﵽ����Ҫ��,�������������!\n");
		printf("preIter = %.8lf, currentIter = %.8lf\n", preIterValue, currentIterValue);
		system("pause");
	}
	ans.second = currentIterValue;
	return ans;
}

double PolylineGenerator::d2f_dx2(int x, int y, int i)
{
	double preIterAns = 0.0, currentIterAns;
	//int max_it = 1000; //����������
	//double eps = 1e-5;
	double h = 1;
	double ans;
	int k;

	//��d2f/dx2
	for (k = 0; k < max_it; k++)
	{
		currentIterAns = (calculate(x + h, y, i) - 2 * calculate(x, y, i) + calculate(x - h, y, i)) / (h * h);
		if (fabs(currentIterAns - preIterAns) < stopEps)
		{
			ans = currentIterAns;
			break;
		}
		h *= 0.5;
		preIterAns = currentIterAns;
	}
	if (k == max_it)
	{
		printf("<d2f/dx2>δ�ܴﵽ����Ҫ��,�������������!");
		system("pause");
	}
	return ans;
}

double PolylineGenerator::d2f_dy2(int x, int y, int i)
{
	double preIterAns = 0.0, currentIterAns;
	//int max_it = 1000; //����������
	//double eps = 1e-5;
	double h = 1;
	double ans;
	int k;

	//��d2f/dy2
	for (k = 0; k < max_it; k++)
	{
		currentIterAns = (calculate(x, y + h, i) - 2 * calculate(x, y, i) + calculate(x, y - h, i)) / (h * h);
		if (fabs(currentIterAns - preIterAns) < stopEps)
		{
			ans = currentIterAns;
			break;
		}
		h *= 0.5;
		preIterAns = currentIterAns;
	}
	if (k == max_it)
	{
		printf("<d2f/dy2>δ�ܴﵽ����Ҫ��,�������������!");
		system("pause");
	}
	return ans;
}

double PolylineGenerator::d2f_dxdy(int x, int y, int i)
{
	double preIterAns = 0.0, currentIterAns;
	//int max_it = 1000; //����������
	//double eps = 1e-5;
	double h = 1;
	double ans;
	int k;

	//��d2f/dxdy
	for (k = 0; k < max_it; k++)
	{
		currentIterAns = (calculate(x + h / 2, y + h / 2, i) - calculate(x + h / 2, y - h / 2, i)
			- calculate(x - h / 2, y + h / 2, i) + calculate(x - h / 2, y - h / 2, i)) / (h * h);
		if (fabs(currentIterAns - preIterAns) < stopEps)
		{
			ans = currentIterAns;
			break;
		}
		h *= 0.5;
		preIterAns = currentIterAns;
	}
	if (k == max_it)
	{
		printf("<d2f/dxdy>δ�ܴﵽ����Ҫ��,�������������!");
		system("pause");
	}
	return ans;
}

double PolylineGenerator::calcStep(int x, int y, int i)
{
	double (PolylineGenerator::* pCalcFunc)(double, double, int); //һ�����Ա����ָ�����pmf�Ķ���
	pCalcFunc = &PolylineGenerator::calculate;
	pair<double, double> gradient = central_difference(pCalcFunc, i);
	Matrix<double> gradF(2, 1);
	gradF.at(0, 0) = gradient.first;
	gradF.at(1, 0) = gradient.second;
	//����Hesse����
	Matrix<double> hesseMat(2, 2);
	hesseMat.at(0, 0) = d2f_dx2(x, y, i);
	hesseMat.at(0, 1) = hesseMat.at(1, 0) = d2f_dxdy(x, y, i);
	hesseMat.at(1, 1) = d2f_dy2(x, y, i);
	
	//����gradT*H*grad
	Matrix<double> fengmu = gradF.traspose() * hesseMat * gradF;

	return sqrt(gradient.first * gradient.first + gradient.second * gradient.second) / fengmu.at(0, 0);
}

double PolylineGenerator::calculate(double x, double y, int i)
{
	double Gvi = 0;
	//����ǰ�벿��
	double delta_nvi = 0;
	double vvi = 0, sigma_posi_vi = 0, sigma_nega_vi = 0;
	int k = polyline.size() - 1;
	//��vvi, sigma_posi_vi, sigma_nega_vi
	for each (Pt pt in (*vSet[i]))
	{
		vvi += dist(x, y, pt.x, pt.y) * dist(x, y, pt.x, pt.y);
	}
	//printf("vvi = %lf\n", vvi);
	if (i < k)
	{
		for each (Pt pt in (*sSet[i]))
		{
			double x0 = pt.x, y0 = pt.y;
			double _x = polyline[i + 1].x;
			double _y = polyline[i + 1].y;
			double cos = ((x - x0) * (x - _x) + (y - y0) * (y - _y)) /
				sqrt(((x - x0) * (x - x0) + (y - y0) * (y - y0)) * ((x - _x) * (x - _x) + (y - _y) * (y - _y)));
			double sin_square = 1 - cos * cos;
			if (sin_square < 0) //��ֹdouble������sin_square = -0.00000001�������
				sin_square = 0;
			double sin = sqrt(sin_square);
			double d = dist(x, y, x0, y0) * sin;
			sigma_posi_vi += d * d;
		}
		//printf("sigma_posi_vi = %lf\n", sigma_posi_vi);
	}
	if (i > 0)
	{
		for each (Pt pt in (*sSet[i - 1]))
		{
			double x0 = pt.x, y0 = pt.y;
			double _x = polyline[i - 1].x;
			double _y = polyline[i - 1].y;
			double cos = ((x - x0) * (x - _x) + (y - y0) * (y - _y)) / 
				sqrt(((x - x0) * (x - x0) + (y -y0) * (y - y0)) * ((x - _x) * (x - _x) + (y - _y) * (y - _y)));
			double sin_square = 1 - cos * cos;
			if (sin_square < 0) //��ֹdouble������sin_square = -0.00000001�������
				sin_square = 0;
			double sin = sqrt(sin_square);
			double d = dist(x, y, x0, y0) * sin;
			sigma_nega_vi += d * d;
			
			/**********************************************************/
			/*test code starts from here*/
			if (isnan(sigma_nega_vi))
			{
				printf("d = %lf\n dist(x,y,x0,y0) = %lf\n x = %lf, y = %lf, x0 = %lf, y0 = %lf\n", d, dist(x, y, x0, y0), x, y, x0, y0);
				printf("sin = %lf, cos = %lf", sin, cos);
				system("pause");
			}
			/*test code ends*/
			/**********************************************************/
			
		}
		//printf("sigma_nega_vi = %lf\n", sigma_nega_vi);
	}
	//��delta_nvi
	if (i == 0)
	{
		delta_nvi = vvi + sigma_posi_vi;
	}
	else if (i == k)
	{
		delta_nvi = vvi + sigma_nega_vi;
	}
	else
	{
		delta_nvi = vvi + sigma_nega_vi + sigma_posi_vi;
	}
	Gvi += delta_nvi / n;

	//�����벿��,��ǳͷ�
	double pvi = 0;
	//�����������
	if (k == 1)
		return Gvi;
	else if (k == 2 && i == 1)
	{
		pvi += dist(x, y, polyline[i - 1].x, polyline[i - 1].y);
		pvi += r * r * (1 + cosAngle(polyline[i - 1], Pt(x, y), polyline[i + 1]));
		pvi += dist(x, y, polyline[i + 1].x, polyline[i + 1].y);
	}
	else
	{
		if (i == 0)
		{
			pvi += dist(x, y, polyline[i + 1].x, polyline[i + 1].y); //��+(vi)
			pvi += r * r * (1 + cosAngle(Pt(x, y), polyline[i + 1], polyline[i + 2])); //��(vi+1)
		}
		else if (i == 1)
		{
			pvi += dist(x, y, polyline[i - 1].x, polyline[i - 1].y); //��-(vi)
			pvi += r * r * (1 + cosAngle(polyline[i - 1], Pt(x, y), polyline[i + 1])); //��(vi)
			pvi += r * r * (1 + cosAngle(Pt(x, y), polyline[i + 1], polyline[i + 2])); //��(vi+1)
		}
		else if (i == k - 1)
		{
			pvi += r * r * (1 + cosAngle(polyline[i - 2], polyline[i - 1], Pt(x, y))); //��(vi-1)
			pvi += r * r * (1 + cosAngle(polyline[i - 1], Pt(x, y), polyline[i + 1])); //��(vi)
			pvi += dist(x, y, polyline[i + 1].x, polyline[i + 1].y); //��+(vi)
		}
		else if (i == k)
		{
			pvi += r * r * (1 + cosAngle(polyline[i - 2], polyline[i - 1], Pt(x, y))); //��(vi-1)
			pvi += dist(x, y, polyline[i - 1].x, polyline[i - 1].y); //��-(vi)
		}
		else
		{
			pvi += r * r * (1 + cosAngle(polyline[i - 2], polyline[i - 1], Pt(x, y))); //��(vi-1)
			pvi += r * r * (1 + cosAngle(polyline[i - 1], Pt(x, y), polyline[i + 1])); //��(vi)
			pvi += r * r * (1 + cosAngle(Pt(x, y), polyline[i + 1], polyline[i + 2])); //��(vi+1)
		}
	}
	//cout << "angle penalty = " << lambdaP * 1 / (k + 1) * pvi << endl;
	Gvi += lambdaP * 1 / (k + 1) * pvi;
	if (isnan(Gvi))
	{
		cout << Gvi << endl;
		printf("vvi = %lf\n", vvi);
		printf("sigma_posi_vi = %.8lf\n", sigma_posi_vi);
		printf("sigma_nega_vi = %.8lf\n", sigma_nega_vi);
		cout << "angle penalty = " << lambdaP * 1 / (k + 1) * pvi << endl;
		printf("lambdaP = %.8lf, pvi = %.8lf\n", lambdaP, pvi);
		printf("k = %d i = %d\n", k, i);
		system("pause");
	}
	return Gvi;
}

PolylineGenerator::PolylineGenerator()
{
}

void PolylineGenerator::genPolyline(list<Pt>& pts)
{
	for each(Pt pt in pts)
	{
		this->pts.push_back(pt);
	}
	if (pts.size() <= 1)
	{
		cout << "������̫�٣��˳���" << endl;
		system("pause");
		exit(0);
	}
	initialization();
	//����7�����Ƶ��polyline
	for (int i = 0; i < 9; i++)
	{
		optimizationEx();
		//optimization();
	}
}

double PolylineGenerator::getRadius()
{
	//////////////////////////////////////////////////////////////////////////
	///�������ݵ�İ뾶
	//////////////////////////////////////////////////////////////////////////                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
	double medianY = 0, medianX = 0;
	for (list<Pt>::iterator ptIter = pts.begin(); ptIter != pts.end(); ptIter++)
	{
		medianX += ptIter->x;
		medianY += ptIter->y;
	}
	medianY /= pts.size();
	medianX /= pts.size();
	double radius = 0;
	for (list<Pt>::iterator ptIter = pts.begin(); ptIter != pts.end(); ptIter++)
	{
		double d = dist(ptIter->x, ptIter->y, medianX, medianY);
		if (d > radius)
			radius = d;
	}
	return radius;
}

void PolylineGenerator::initialization()
{
	sampling(1000);
	polyline.clear();
	n = pts.size();
	r = getRadius();
	//��ʼ��,���������˵�,ȡlat�����������������lon���������������
	double minX = 99999999, maxX = 0;
	double minY = 99999999, maxY = 0;
	Pt maxXpt, minXpt, maxYpt, minYpt;
	for each (Pt pt in pts)
	{
		if (pt.x < minX)
		{
			minX = pt.x;
			minXpt.x = pt.x;
			minXpt.y = pt.y;
		}
		if (pt.x > maxX)
		{
			maxX = pt.x;
			maxXpt.x = pt.x;
			maxXpt.y = pt.y;
		}
		if (pt.y < minY)
		{
			minY = pt.y;
			minYpt.x = pt.x;
			minYpt.y = pt.y;
		}
		if (pt.y > maxY)
		{
			maxY = pt.y;
			maxYpt.x = pt.x;
			maxYpt.y = pt.y;
		}
	}
	//printf("(%lf, %lf) , (%lf, %lf)\n", minX, minY, maxX, maxY);
	Pt startPt, endPt;
	if (dist(minXpt, maxXpt) > dist(minYpt, maxYpt))
	{
		startPt = minXpt;
		endPt = maxXpt;
	}
	else
	{
		startPt = minYpt;
		endPt = maxYpt;
	}
	polyline.push_back(startPt);
	polyline.push_back(endPt);
	vSet.push_back(new list<Pt>);
	vSet.push_back(new list<Pt>);
	sSet.push_back(new list<Pt>);
	vSet[0]->splice(vSet[0]->end(), pts);
	doProject();
	//printf("(%lf, %lf) , (%lf, %lf)\n", polyline[0].x, polyline[0].y, polyline[1].x, polyline[1].y);
}

void PolylineGenerator::doProject(int index)
{
	//////////////////////////////////////////////////////////////////////////
	///TODO:
	///�Ե�i����ͶӰ��,��Ҫ���µ�ΪSi-2,Si-1,Si,Si+1,Vi-1,Vi,Vi+1��Щ����
	//////////////////////////////////////////////////////////////////////////
	//update S
/*	list<PrjPoint> tempPtds;

	for (int i = -2; i <= 1; i++)
	{
		if (index + i < 0 || index + 1 >= polyline.size() - 1)
			continue;
		tempPtds.splice(tempPtds.end(), polyline[index]->vSet);
		tempPtds.splice(tempPtds.end(), polyline[index]->sSet);

	}
	//update V
	for (int i = -1; i <= 1; i++)
	{
		if (index + i < 0 || index + 1 >= polyline.size());
	}
	*/
}

void PolylineGenerator::doProject()
{
	//////////////////////////////////////////////////////////////////////////
	///1.���vSet��sSet
	///2.��������ͶӰ�󵽸���vSet��sSet��
	///3.����lambdaP
	//////////////////////////////////////////////////////////////////////////
	list<Pt> tempPts;
	//�������е�
	for (int i = 0; i < polyline.size() - 1; i++)
	{
		tempPts.splice(tempPts.end(), (*vSet[i]));
		tempPts.splice(tempPts.end(), (*sSet[i]));
	}
	tempPts.splice(tempPts.end(), (*vSet[polyline.size() - 1]));
	//���·���
	while (tempPts.size() != 0)
	{
		Pt top = tempPts.back();
		reProject(top);
		tempPts.pop_back();
	}
	//����lambdaP
	double delta_n_f = 0;
	for (int i = 0; i < vSet.size(); i++)
	{
		for each (Pt pt in (*vSet[i]))
		{
			delta_n_f += pt.dist * pt.dist;
		}
	}
	for (int i = 0; i < sSet.size(); i++)
	{
		for each (Pt pt in (*sSet[i]))
		{
			delta_n_f += pt.dist * pt.dist;
		}
	}
	lambdaP = anglePenalty * (polyline.size()-1) * pow(n, -1 / 3) * pow(delta_n_f, 0.5) / r;
}

void PolylineGenerator::optimization()
{
	int k = polyline.size() - 1;
	double convergeThreshold = 1;
	//��ÿ�������
	cout << "poly size " << polyline.size() << endl;
	for (int _i = 0; _i < 10; _i++)
	{
		for (int i = 0; i < polyline.size(); i++)
		{
			//ֱ�������ĵ������ȴﵽ������ֵ
			//TEST: �ȶ�ÿ�������3��
			double step = 0.2;
			for (int iter = 0; iter < 100; iter++)
			{
				double Gvi = calculate(polyline[i].x, polyline[i].y, i);
				//printf("Gvi = %lf\n", Gvi);
				//���ݶ�
				double (PolylineGenerator::* pCalcFunc)(double, double, int); //һ�����Ա����ָ�����pmf�Ķ���
				pCalcFunc = &PolylineGenerator::calculate;
				pair<double, double> gradient = central_difference(pCalcFunc, i);
				double gradient_x = gradient.first;
				double gradient_y = gradient.second;
				//��һ��
				double direction_x = -gradient_x / sqrt(gradient_x * gradient_x + gradient_y * gradient_y);
				double direction_y = -gradient_y / sqrt(gradient_x * gradient_x + gradient_y * gradient_y);
				//test code start
				//printf("v%d gradient(%lf, %lf)\n", i, gradient_x, gradient_y);
				//printf("v%d direction(%lf, %lf)\n", i, direction_x, direction_y);
				//double step = gradient_x * gradient_x + gradient_y * gradient_y;
				//double step = sqrt(gradient_x * gradient_x + gradient_y * gradient_y);
				//step = step * 14 / 15;
				double newX = polyline[i].x + step * direction_x;
				double newY = polyline[i].y + step * direction_y;
				double delta = (newX - polyline[i].x) * (newX - polyline[i].x) + (newY - polyline[i].y) * (newY - polyline[i].y);
				//if (delta < convergeThreshold)
				//		break;
				//else
				{
					polyline[i].x = newX;
					polyline[i].y = newY;
					doProject();
				}
			}
		}
	}
	//add a new point
	double maxLength = 0;
	int candidateV;
	/*for (int i = 0; i < polyline.size() - 1; i++)
	{
		double tempDist = dist(polyline[i], polyline[i + 1]);
		if (tempDist > maxLength)
		{
			maxLength = tempDist;
			candidateV = i;
		}
	}*/
	double maxNum = -1;
	for (int i = 0; i < polyline.size() - 1; i++)
	{
		if (sSet[i]->size() > maxNum)
		{
			maxNum = sSet[i]->size();
			candidateV = i;
		}
	}
	Pt newPt;
	newPt.x = (polyline[candidateV].x + polyline[candidateV + 1].x) / 2;
	newPt.y = (polyline[candidateV].y + polyline[candidateV + 1].y) / 2;
	polyline.insert(polyline.begin() + candidateV + 1, newPt);
	vSet.push_back(new list<Pt>);
	sSet.push_back(new list<Pt>);
	doProject();
}

void PolylineGenerator::optimizationEx()
{
	int k = polyline.size() - 1;
	double convergeThreshold = 0.001;
	double preGvi = 9999999999;
	//��ÿ�������
	cout << "poly size " << polyline.size() << endl;
	for (int _i = 0; _i < 20; _i++)
	{
		for (int i = 0; i < polyline.size(); i++)
		{
			//ֱ�������ĵ������ȴﵽ������ֵ
			//TEST: �ȶ�ÿ�������3��
			for (int iter = 0; iter < 100; iter++)
			{
				double Gvi = calculate(polyline[i].x, polyline[i].y, i);
				//if (abs(Gvi - preGvi) / Gvi < convergeThreshold)
				//{
			//		break;
			//	}
				//printf("Gvi = %lf\n", Gvi);
				//���ݶ�
				double (PolylineGenerator::* pCalcFunc)(double, double, int); //һ�����Ա����ָ�����pmf�Ķ���
				pCalcFunc = &PolylineGenerator::calculate;
				pair<double, double> gradient = central_difference(pCalcFunc, i);
				double gradient_x = gradient.first;
				double gradient_y = gradient.second;				
				//��һ��
				double direction_x = -gradient_x / sqrt(gradient_x * gradient_x + gradient_y * gradient_y);
				double direction_y = -gradient_y / sqrt(gradient_x * gradient_x + gradient_y * gradient_y);				
				//�󲽳�
				//double step = calcStep(polyline[i].x, polyline[i].y, i);
				double step = 0.2;
				//printf("step = %lf, gv[%d] = %lf\n", step, i, Gvi);
				//system("pause");
				//�ƶ���
				double newX = polyline[i].x + step * direction_x;
				double newY = polyline[i].y + step * direction_y;
				//double delta = (newX - polyline[i].x) * (newX - polyline[i].x) + (newY - polyline[i].y) * (newY - polyline[i].y);
				//if (delta < convergeThreshold)
				//		break;
				//else
				{
					polyline[i].x = newX;
					polyline[i].y = newY;
					doProject();
					preGvi = Gvi;
				}
			}
		}
	}
	//add a new point
	double maxLength = 0;
	int candidateV;
	/*for (int i = 0; i < polyline.size() - 1; i++)
	{
	double tempDist = dist(polyline[i], polyline[i + 1]);
	if (tempDist > maxLength)
	{
	maxLength = tempDist;
	candidateV = i;
	}
	}*/
	double maxNum = -1;
	for (int i = 0; i < polyline.size() - 1; i++)
	{
		if (sSet[i]->size() > maxNum)
		{
			maxNum = sSet[i]->size();
			candidateV = i;
		}
	}
	Pt newPt;
	newPt.x = (polyline[candidateV].x + polyline[candidateV + 1].x) / 2;
	newPt.y = (polyline[candidateV].y + polyline[candidateV + 1].y) / 2;
	polyline.insert(polyline.begin() + candidateV + 1, newPt);
	vSet.push_back(new list<Pt>);
	sSet.push_back(new list<Pt>);
	doProject();
}

void PolylineGenerator::reProject(Pt& pt)
{
	//////////////////////////////////////////////////////////////////////////
	///��ptͶӰ����Ӧ��vSet��sSet��
	//////////////////////////////////////////////////////////////////////////
	double minDist = 9999999.0;
	list<Pt>* dest = NULL;
	//�����˵����
	for (int i = 0; i < polyline.size(); i++)
	{
		double tmpDist = dist(polyline[i], pt);
		if (tmpDist < minDist)
		{
			minDist = tmpDist;
			dest = vSet[i];
		}
	}
	//test code starts
	if (dest == NULL)
	{
		printf("v0 (%lf, %lf)\n", polyline[0].x, polyline[0].y);
		cout << "dist to v0" << dist(polyline[0], pt) << endl;
		cout << "dist to v1" << dist(polyline[1], pt) << endl;
		cout << "polyline size: " << polyline.size() << endl;
		printf("pt (%lf, %lf)\n", pt.x, pt.y);
		cout << "dest should not bu null! " << endl;
		system("pause");
		exit(0);
	}
	//test code ends

	//����ͶӰ����
	for (int i = 0; i < polyline.size() - 1; i++)
	{
		//��ͶӰ
		if (cosAngle(pt, polyline[i], polyline[i + 1]) >= 0 && cosAngle(pt, polyline[i + 1], polyline[i]) >= 0)
		{
			double A = polyline[i + 1].y - polyline[i].y;
			double B = -(polyline[i + 1].x - polyline[i].x);
			double C = polyline[i].y * (polyline[i + 1].x - polyline[i].x)
				- polyline[i].x * (polyline[i + 1].y - polyline[i].y);
			double tmpDist1 = abs(A * pt.x + B * pt.y + C) / sqrt(A * A + B * B);
			double cos = cosAngle(pt, polyline[i], polyline[i + 1]);
			double sin = sqrt(1 - cos*cos);
			double tmpDist2 = dist(pt, polyline[i]) * sin;
			if (abs(tmpDist1 - tmpDist2)> 10e-7)
			{
				cout << "not equal!" << endl;
				printf("using line func = %.9lf, using cos = %.9lf\n", tmpDist1, tmpDist2);
			}
			if (minDist > tmpDist2)
			{
				minDist = tmpDist2;
				dest = sSet[i];
			}
		}
	}
	pt.dist = minDist;
	if (dest == NULL)
	{
		cout << "NULL! " << endl;
		system("pause");
		exit(0);
	}
	dest->push_back(pt);
}