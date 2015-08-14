#include "Trainer.h"


Trainer::Trainer(Map* roadNetwork_, Area* area_)
{
	this->roadNetwork = roadNetwork_;
	this->area = area_;
}

void Trainer::setPriorParams(double u0_, double lambda0_, double alpha0_, double beta0_)
{
	u0 = u0_;
	lambda0 = lambda0_;
	alpha0 = alpha0_;
	beta0 = beta0_;
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
		trainOneRoad(roadNetwork->edges[i], roadNetwork->edges[i]->trainData);
	}
}

void Trainer::trainSMMDEx(double intervalM /* = 50.0 */)
{
	//////////////////////////////////////////////////////////////////////////
	///ѵ��������area�е�r�Ĳ���
	//////////////////////////////////////////////////////////////////////////

	for (int i = 0; i < roadNetwork->edges.size(); i++)
	{
		if (roadNetwork->edges[i] == NULL || roadNetwork->edges[i]->r_hat.size() == 0)
			continue;
		roadNetwork->edges[i]->cut(intervalM);
		trainOneRoadEx(roadNetwork->edges[i], roadNetwork->edges[i]->trainData);
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
		trainOneRoadSimple(roadNetwork->edges[i], roadNetwork->edges[i]->trainData);
	}
}

void Trainer::trainOneRoadSimple(Edge* r, list<GeoPoint*>& pts)
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

void Trainer::trainOneRoad(Edge* r, list<GeoPoint*>& pts)
{
	//////////////////////////////////////////////////////////////////////////
	///ѵ��һ��·r�Ĳ�����ptsΪƥ�䵽r�ϵ���ʷ�켣��
	//////////////////////////////////////////////////////////////////////////
	int N = pts.size();
	//sigma xi
	double sigma = 0, sigma2 = 0, variance = 0;
	int i;

	double dist = 0;
	for (list<GeoPoint*>::iterator ptIter = pts.begin(); ptIter != pts.end(); ptIter++)
	{
		dist = SMMD::distM_signed(r, (*ptIter));
		sigma += dist;
		sigma2 += dist * dist;
	}
	//iterate
	double convergence = 0.00001;
	int iterateCount = 0;
	r->alpha = alpha0 + N / 2.0; //done
	r->beta = beta0;
	r->lambda = lambda0;
	r->u = (u0 * lambda0 + sigma) / (lambda0 + N); //done
	//training
	double mean_mu = r->u, mean_mu2, mean_tau = 1;

	double preLambda = 0, preBeta = 0;
	do
	{
		preLambda = r->lambda;
		preBeta = r->beta;
		mean_mu2 = r->u * r->u + 1 / r->lambda;
		r->beta = beta0 + ((N + lambda0) * mean_mu2 - (2 * lambda0 + sigma) * mean_mu + lambda0 * u0 * u0 + sigma2) / 2;
		mean_tau = r->alpha / r->beta;
		r->lambda = (lambda0 + N) * mean_tau;
	} while (abs(r->beta - preBeta) > convergence && abs(r->lambda - preLambda) > convergence);

	//printf("rId = %d\n u = %lf, lambda = %lf\n alpha = %lf, beta = %lf\n E_tau = %lf\n", r->id, r->u, r->lambda, r->alpha, r->beta, mean_tau);
	//system("pause");
}

void Trainer::trainOneRoadEx(Edge* r, list<GeoPoint*>& pts)
{
	//////////////////////////////////////////////////////////////////////////
	///ѵ��һ��·r�Ĳ�����ptsΪƥ�䵽r�ϵ���ʷ�켣��
	//////////////////////////////////////////////////////////////////////////
	//�ַ�pts������slots
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
	//�������
	for (int i = 0; i < r->thetas.size(); i++) //for each slot
	{
		int N = slots[i].size();

		if (N > 0)
		{
			//sigma xi
			double sigma = 0, sigma2 = 0, variance = 0;

			double dist = 0;
			for (list<GeoPoint*>::iterator ptIter = slots[i].begin(); ptIter != slots[i].end(); ptIter++)
			{
				dist = SMMD::distM_signed(r, (*ptIter));
				sigma += dist;
				sigma2 += dist * dist;
			}
			//iterate
			double convergence = 0.00001;
			int iterateCount = 0;
			r->thetas[i].alpha = alpha0 + N / 2.0; //done
			r->thetas[i].beta = beta0;
			r->thetas[i].lambda = lambda0;
			r->thetas[i].u = (u0 * lambda0 + sigma) / (lambda0 + N); //done
			//training
			double mean_mu = r->thetas[i].u, mean_mu2, mean_tau = 1;

			double preLambda = 0, preBeta = 0;
			do
			{
				preLambda = r->thetas[i].lambda;
				preBeta = r->thetas[i].beta;
				mean_mu2 = r->thetas[i].u * r->thetas[i].u + 1 / r->thetas[i].lambda;
				r->thetas[i].beta = beta0 + ((N + lambda0) * mean_mu2 - (2 * lambda0 + sigma) * mean_mu + lambda0 * u0 * u0 + sigma2) / 2;
				mean_tau = r->thetas[i].alpha / r->thetas[i].beta;
				r->thetas[i].lambda = (lambda0 + N) * mean_tau;
			} while (abs(r->thetas[i].beta - preBeta) > convergence && abs(r->thetas[i].lambda - preLambda) > convergence);

			//printf("rId = %d\n u = %lf, lambda = %lf\n alpha = %lf, beta = %lf\n E_tau = %lf\n", r->id, r->thetas[i].u, r->thetas[i].lambda, r->thetas[i].alpha, r->thetas[i].beta, mean_tau);
			//system("pause");
		}
		else //slot����û��ѵ����������
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