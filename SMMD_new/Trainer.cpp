#include "Trainer.h"


Trainer::Trainer(Map* roadNetwork_, Area* area_)
{
	this->roadNetwork = roadNetwork_;
	this->area = area_;
}

void Trainer::loadPrior(string priorDataPath)
{
	ifstream ifs(priorDataPath);
	double currentId = -1;
	while (!ifs.eof())
	{
		currentId++;
		if (currentId >= roadNetwork->edges.size())
			break;
		int count;
		ifs >> count;
		if (ifs.fail())
			break;
		if (roadNetwork->edges[currentId] == NULL)
			continue;
		roadNetwork->edges[currentId]->prior = count;
	}
}

void Trainer::loadTrainData(string trainDataPath, int count /* = INF */)
{
	//////////////////////////////////////////////////////////////////////////
	///1����ȡѵ��������
	///2�������ݷַ�����Ӧ��edge��
	///ע����ȡ�����ݶ������趨area�е�
	//////////////////////////////////////////////////////////////////////////
	TrajReader tReader(trainDataPath);
	tReader.readGeoPoints(trainDataSet, area, count);
	//�ַ�����
	for each (GeoPoint* pt in trainDataSet)
	{
		if (pt == NULL || pt->mmRoadId == -1)
			continue;
		if (roadNetwork->edges[pt->mmRoadId] != NULL)
			roadNetwork->edges[pt->mmRoadId]->trainData.push_back(pt);
	}
}

void Trainer::trainSMMD()
{
	//////////////////////////////////////////////////////////////////////////
	///ѵ��������area�е�r�Ĳ���
	//////////////////////////////////////////////////////////////////////////


	for (int i = 0; i < roadNetwork->edges.size(); i++)
	{
		if (roadNetwork->edges[i] == NULL || roadNetwork->edges[i]->r_hat.size() == 0)
			continue;
		trainOneRoad_GaussianParam(roadNetwork->edges[i], roadNetwork->edges[i]->trainData);
	}
}

void Trainer::trainSimple(double intervalM /* = 50.0 */)
{
	//////////////////////////////////////////////////////////////////////////
	///ѵ��������area�е�r�Ĳ���
	//////////////////////////////////////////////////////////////////////////
	for (int i = 0; i < roadNetwork->edges.size(); i++)
	{
		if (roadNetwork->edges[i] == NULL || roadNetwork->edges[i]->r_hat.size() == 0)
			continue;
		roadNetwork->edges[i]->cut(intervalM);
		trainOneRoad_GaussianParam(roadNetwork->edges[i], roadNetwork->edges[i]->trainData);
	}
}

void Trainer::trainOneRoad_GaussianParam(Edge* r, list<GeoPoint*>& pts)
{
	//////////////////////////////////////////////////////////////////////////
	///ʹ��һ���˹ģ��ѵ��һ��·��ÿһ��segment�Ĳ���mu�Լ�sigma
	//////////////////////////////////////////////////////////////////////////
	//�ַ���ÿһ��
	vector<list<GeoPoint*> > slots;
	for (int i = 0; i < r->thetas.size(); i++)
	{
		list<GeoPoint*> emptyList;
		slots.push_back(emptyList);
	}
	for each(GeoPoint* pt in pts)
	{
		int slotId = r->getSlotId(pt);
		slots[slotId].push_back(pt);
	}

	//����mu
	for (int i = 0; i < r->thetas.size(); i++) //for each r
	{
		double mu = 0; //��ֵ
		for each(GeoPoint* pt in slots[i])
		{
			mu += SMMD::distM_signed(r, pt);
		}
		if (slots[i].size() > 0)
		{
			mu /= slots[i].size();
			r->thetas[i].mu = mu;
			double sigma = 0; //��׼��
			for each(GeoPoint* pt in slots[i])
			{
				double dist = SMMD::distM_signed(r, pt);
				sigma += (dist - mu) * (dist - mu);
			}
			sigma /= slots[i].size();
			sigma = sqrt(sigma);
			r->thetas[i].sigma = sigma;

			//ע�⵽�����N=1������£�sigma = 0�� �����޷������˹�ֲ�����Ϊ��ĸ��0��
			if (abs(sigma) < eps)
				slots[i].clear();
			/**********************************************************/
			/*test code starts from here*/
			//printf("mu = %lf, sigma = %lf, N = %d\n", mu, sigma, slots[i].size());
			//	system("pause");
			/*test code ends*/
			/**********************************************************/		
		}
		if (slots[i].size() == 0) //slot����û��ѵ���������� //ע�⣬�������else����Ϊǰ������п��ܻ�ʹ����Щslot����ǿ�����Ϊ����ִ�����´���
		{
			if (i > 0) //������ǵ�һ�����Ǿͳ�ǰһ��
			{
				r->thetas[i] = r->thetas[i - 1];
			}
			else //����ǵ�һ�������ڶ���,�ڶ���Ҳû�еĻ���������,�Դ�����ֱ������Ϊֹ
			{
				//�����ʲôҲ�����������
			}
		}
		//����ʼ����û�е����
		int slotId = 0;
		while (slots[slotId].size() == 0)
		{
			slotId++;
		}
		for (int i = 0; i < slotId; i++)
		{
			r->thetas[i] = r->thetas[slotId]; //��ǰ��һ��û�е�ȫ����thetas[slotId]
		}
	}
}

void Trainer::genCenterline(string outPath)
{
	//////////////////////////////////////////////////////////////////////////
	///����area�ڵĵ�·����r_hat
	///��Ҫ�ȵ���loadTrainData()
	//////////////////////////////////////////////////////////////////////////
	ofstream ofs(outPath);
	ofs << fixed << showpoint << setprecision(8);

	for (int i = 0; i < roadNetwork->edges.size(); i++)
	{
		if (roadNetwork->edges[i] == NULL)
			continue;
		if (roadNetwork->edges[i]->trainData.size() <= 5)
			continue;
		PolylineGenerator pg;
		pg.bigIterTimes = 3; //����һ���ֵ��ѭ������
		pg.smallIterTimes = 50; //����һ����ĵ�������
		pg.step = 0.5; //ÿһ�ε����Ĳ���
		pg.sampleSize = 1000; //sampling������
		pg.anglePenalty = 0.003; 

		//��cluster��Ĺ켣��ȫ������pts��
		list<Pt> pts;

		//��������ת���������ռ�����ӳ�䵽5000���ط�Χ�ϱȽϺ���
		MapDrawer mdTempForCoordThrans;
		mdTempForCoordThrans.setArea(area);
		mdTempForCoordThrans.setResolution(5000);

		for each (GeoPoint* gPt in roadNetwork->edges[i]->trainData)
		{
			Pt tmpPt;
			Gdiplus::Point gdiPt = mdTempForCoordThrans.geoToScreen(gPt->lat, gPt->lon);
			tmpPt.x = (double)gdiPt.X;
			tmpPt.y = (double)gdiPt.Y;
			pts.push_back(tmpPt);
		}
		pg.genPolyline(pts);

		//��polylineת�ؿռ�����
		vector<GeoPoint*> centerLine;
		for (int i = 0; i < pg.polyline.size(); i++)
		{
			GeoPoint* pt = new GeoPoint(mdTempForCoordThrans.screenToGeo((int)pg.polyline[i].x, (int)pg.polyline[i].y));
			centerLine.push_back(pt);
		}

		//���
		ofs << i << " " << centerLine.size() << endl;
		for (int j = 0; j < centerLine.size(); j++)
		{
			ofs << centerLine[j]->lat << " " << centerLine[j]->lon << " ";
		}
		ofs << endl;

		//plot pts
		/*for each (GeoPoint* pt in roadNetwork->edges[i]->trainData)
		{
			md.drawPoint(Color::Green, pt->lat, pt->lon);
		}

		//draw polyline
		for (int i = 0; i < centerLine.size() - 1; i++)
		{
			md.drawLine(Color::Red, centerLine[i]->lat, centerLine[i]->lon, centerLine[i + 1]->lat, centerLine[i + 1]->lon);
			md.drawBigPoint(Color::Black, centerLine[i]->lat, centerLine[i]->lon);
			md.drawBigPoint(Color::Black, centerLine[i + 1]->lat, centerLine[i + 1]->lon);
		}*/
	}
	ofs.close();
}

void Trainer::genFakeCenterline(string outPath)
{
	//////////////////////////////////////////////////////////////////////////
	///��ԭ��ͼ�ĵ�·��״���ɼٵ�centerline
	//////////////////////////////////////////////////////////////////////////
	ofstream ofs(outPath);
	ofs << fixed << showpoint << setprecision(8);

	for (int i = 0; i < roadNetwork->edges.size(); i++)
	{
		if (roadNetwork->edges[i] == NULL)
			continue;
		if (roadNetwork->edges[i]->trainData.size() <= 5)
			continue;
		
		vector<GeoPoint*> centerLine;
		for each(GeoPoint* pt in *roadNetwork->edges[i]->figure)
		{
			centerLine.push_back(pt);
		}

		//���
		ofs << i << " " << centerLine.size() << endl;
		for (int j = 0; j < centerLine.size(); j++)
		{
			ofs << centerLine[j]->lat << " " << centerLine[j]->lon << " ";
		}
		ofs << endl;
	}
	ofs.close();
}

void Trainer::drawCenterline(string centerlinePath, MapDrawer& md)
{
	roadNetwork->loadPolylines(centerlinePath);
	for each (Edge* r in roadNetwork->edges)
	{
		if (r == NULL || r->r_hat.size() == 0)
			continue;
		for (int i = 0; i < r->r_hat.size() - 1; i++)
		{
			md.drawLine(Gdiplus::Color::Black, r->r_hat[i]->lat, r->r_hat[i]->lon, r->r_hat[i + 1]->lat, r->r_hat[i + 1]->lon);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////
///KNN
////////////////////////////////////////////////////////////////////////////////////////////
KNNClassifier::KNNClassifier(Map* roadNetwork_, Area* area_)
{
	this->roadNetwork = roadNetwork_;
	this->area = area_;
	ptGridIndex = new PointGridIndex();
}

void KNNClassifier::loadTrainData(string trainDataPath, int count /* = INF */)
{
	//////////////////////////////////////////////////////////////////////////
	///1����ȡѵ��������
	///2�������ݷ�������������
	///ע����ȡ�����ݶ������趨area�е�
	//////////////////////////////////////////////////////////////////////////
	TrajReader tReader(trainDataPath);
	tReader.readGeoPoints(trainDataSet, area, count);
	double gridSizeM = 20.0;
	int gridWidth = (area->maxLon - area->minLon) / (gridSizeM / GeoPoint::geoScale);
	ptGridIndex->createIndex(trainDataSet, area, gridWidth);
}

int KNNClassifier::classify(GeoPoint* pt, int k)
{
	//////////////////////////////////////////////////////////////////////////
	///��pt��kNN��trainingData��ѡ����ǩ�����Ǹ���ǩ��Ϊ���
	//////////////////////////////////////////////////////////////////////////
	vector<GeoPoint*> kNNPts;
	ptGridIndex->kNN_exact(pt, k, kNNPts);
	vector<int> votes;
	for (int i = 0; i < roadNetwork->edges.size(); i++)
		votes.push_back(0);
	for (int i = 0; i < kNNPts.size(); i++)
		votes[kNNPts[i]->mmRoadId]++;
	int max = 0;
	int maxEdgeId = 0;
	for (int i = 0; i < roadNetwork->edges.size(); i++)
	{
		if (votes[i] > max)
		{
			max = votes[i];
			maxEdgeId = i;
		}
	}
	return maxEdgeId;
}