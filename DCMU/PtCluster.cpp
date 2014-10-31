#include "PtCluster.h"

//////////////////////////////////////////////////////////////////////////
///public part
//////////////////////////////////////////////////////////////////////////

void PtCluster::drawClusters(MapDrawer& md)
{
	for (int i = 0; i < clusters.size(); i++)
	{
		Gdiplus::Color color = randomColor();
		for each (GeoPoint* pt in clusters[i]->pts)
		{
			md.drawPoint(color, pt->lat, pt->lon);
		}
	}
}

void PtCluster::run(PointGridIndex* _ptIndex)
{
	this->ptIndex = _ptIndex;
	calPtsDirs();
	doDirCluster();
	pts.clear();
}

//////////////////////////////////////////////////////////////////////////
///private part
//////////////////////////////////////////////////////////////////////////
void PtCluster::calPtsDirs()
{
	//double d = 4.0; //���ο��
	double angleStep = PI / 24; //�Ƕ�ö������
	//�����е����pts
	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for (list<GeoPoint*>::iterator iter = ptIndex->grid[row][col]->begin(); iter != ptIndex->grid[row][col]->end(); iter++)
			{
				pts.push_back(*iter);
			}
		}
	}
	int tenPercent = pts.size() / 10;
	int count = 1;
	for each (GeoPoint* pt in pts)
	{
		if (count % tenPercent == 0)
			printf("����㷽�� %d0%% �����\n", count / tenPercent);
		calDir(pt, angleStep, d, l);
		count++;
	}
	printf("����㷽�������\n");
}

void PtCluster::count(GeoPoint* p0, GeoPoint* p, vector<int>& countVec, double d, double l, double angleStep)
{
	if (p == p0)
		return;
	double theta_1, theta_2;
	pair<double, double> vec_p0p;
	GeoPoint tempMirrorPt;
	if (p->lat <= p0->lat)
	{
		tempMirrorPt.lon = 2 * p0->lon - p->lon;
		tempMirrorPt.lat = 2 * p0->lat - p->lat;
		p = &tempMirrorPt;
	}
	vec_p0p = make_pair(p->lon - p0->lon, p->lat - p0->lat);
	theta_1 = acos((1 * vec_p0p.first + 0 * vec_p0p.second)
		/ sqrt((1 * 1 + 0 * 0) * (vec_p0p.first * vec_p0p.first + vec_p0p.second * vec_p0p.second)));
	theta_2 = acos((d / 2) / GeoPoint::distM(p0, p));

	double theta_s, theta_e;
	theta_s = theta_1 + theta_2 - PI / 2;
	theta_e = theta_1 - theta_2 + PI / 2;
	/*printf("p0 = (%.8lf, %.8lf), p = (%.8lf, %.8lf)\n", p0->lon, p0->lat, p->lon, p->lat);
	printf("p0p(%.8lf, %.8lf)\n", vec_p0p.first, vec_p0p.second);
	printf("t1 = %lf, t2 = %lf, ts = %lf, te = %lf\n", theta_1, theta_2, theta_s, theta_e);
	system("pause");*/
	for (double dir = 0, i = 0; dir < PI; dir += angleStep, i++)
	{
		if (dir >= theta_s && dir <= theta_e)
			countVec[i]++;
	}
}

void PtCluster::calDir(GeoPoint* p0, double angleStep, double d, double l)
{
	//////////////////////////////////////////////////////////////////////////
	///����p0�ĵ㷽��
	///angleStepΪ��С�Ƕ����ȣ�����ΧΪ[0,��)
	///�������p0��direction����
	//////////////////////////////////////////////////////////////////////////
	
	/**********************************************************/
	/*test code starts from here*/
//	if (p0->mmRoadId != roadId)
	//	return;
	/*test code ends*/
	/**********************************************************/
	
	vector<GeoPoint*> nearPts;
	ptIndex->getNearPts(p0, sqrt(d / 2 * d / 2 + l*l), nearPts);
	vector<int> countVec;
	for (double dir = 0; dir < PI; dir += angleStep)
	{
		countVec.push_back(0);
	}
	for each (GeoPoint* p in nearPts)
	{
		count(p0, p, countVec, d, l, angleStep);
	}
	double bestDir = countVec[0];
	int maxCount = -1;
	for (int i = 0; i < countVec.size(); i++)
	{
		if (countVec[i] > maxCount)
		{
			maxCount = countVec[i];
			bestDir = i * angleStep;
		}
	}
	p0->direction = bestDir;
}

void PtCluster::getPtsNearCluster(Cluster* cluster, double distThresM, double angleThres, vector<GeoPoint*>& nearPts)
{
	//////////////////////////////////////////////////////////////////////////
	///��cluster��mbr������չһȦ������������Ƕ���cluster�Ƕ�ֻ��С��angleThres�ĵ����nearPts��
	//////////////////////////////////////////////////////////////////////////
	nearPts.clear();
	pair<int, int> rowCol_s = ptIndex->getRowCol(new GeoPoint(cluster->area.minLat, cluster->area.minLon));
	pair<int, int> rowCol_e = ptIndex->getRowCol(new GeoPoint(cluster->area.maxLat, cluster->area.maxLon));
	rowCol_s.first--;
	rowCol_s.second--;
	rowCol_e.first++;
	rowCol_e.second++;
	if (rowCol_s.first < 0) rowCol_s.first = 0;
	if (rowCol_e.first >= ptIndex->gridHeight) rowCol_e.first = ptIndex->gridHeight - 1;
	if (rowCol_s.second < 0) rowCol_s.second = 0;
	if (rowCol_e.second >= ptIndex->gridWidth) rowCol_e.second = ptIndex->gridWidth - 1;

	for (int row = rowCol_s.first; row <= rowCol_e.first; row++)
	{
		for (int col = rowCol_s.second; col <= rowCol_e.second; col++)
		{
			for each (GeoPoint* pt in *(ptIndex->grid[row][col]))
			{
				if (pt->clusterId == -1 &&
					(abs(pt->direction - cluster->direction) < angleThres ||
					PI - abs(pt->direction - cluster->direction) < angleThres))
				{
					nearPts.push_back(pt);
				}
			}
		}
	}
}

bool PtCluster::fetch(Cluster* cluster, double distThresM, double angleThres)
{
	//////////////////////////////////////////////////////////////////////////
	///��cluster����������������ĵ������������û���������false
	//////////////////////////////////////////////////////////////////////////
	bool canFetch = false;
	vector<GeoPoint*> nearPts;
	getPtsNearCluster(cluster, distThresM, angleThres, nearPts);
	for (int i = 0; i < nearPts.size(); i++)
	{
		bool validation = false;
		GeoPoint* currentPt = nearPts[i];
		vector<GeoPoint*> ptsNearCurrentPt;
		ptIndex->getNearPts(currentPt, 1, ptsNearCurrentPt);
		//����currentPt��������������cluster�еĵ�
		for (int j = 0; j < ptsNearCurrentPt.size(); j++)
		{
			if (ptsNearCurrentPt[j]->clusterId != cluster->clusterId)
				continue;
			if (GeoPoint::distM(ptsNearCurrentPt[j], currentPt) < distThresM)
			{
				validation = true;
				break;
			}
		}
		if (validation)
		{
			cluster->add(currentPt);
			canFetch = true;
		}
	}
	return canFetch;
}

Cluster* PtCluster::genOneCluster(int clusterId, GeoPoint* seedPt, double distThresM, double angleThres)
{
	//////////////////////////////////////////////////////////////////////////
	///�������ӵ�seedPt����һ��cluster
	///[TODO]:���ӵ�һ�����Ƕ���ȽϺã����ӵ�ѡȡ�Ƿ��н�����
	//////////////////////////////////////////////////////////////////////////
	Cluster* cluster = new Cluster;
	cluster->clusterId = clusterId;
	cluster->add(seedPt);
	while (fetch(cluster, distThresM, angleThres)){}
	return cluster;
}

void PtCluster::doDirCluster()
{
	//double distThresM = 15;
	//double angleThres = 20.0 / 180.0 * PI;
	int currentClusterId = clusters.size();
	int count = 1;
	int tenPercent = pts.size() / 10;
	for each (GeoPoint* pt in pts)
	{
		//if (count % tenPercent == 0)
		//	printf("������� %d0%% �����\n", count / tenPercent);
		count++;
		if (pt->clusterId != -1)
		{
			continue;
		}
		Cluster* cluster = genOneCluster(currentClusterId, pt, distThresM, angleThres);
		clusters.push_back(cluster);
		currentClusterId++;
	}	
	cout << "�㷽��������" << endl;
}
