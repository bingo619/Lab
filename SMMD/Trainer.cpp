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
	///1）读取训练集数据
	///2）将数据分发到相应的edge上
	///注：读取的数据都是在设定area中的
	//////////////////////////////////////////////////////////////////////////
	TrajReader tReader(trainDataPath);
	tReader.readGeoPoints(trainDataSet, area, count);
	//分发数据
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
	///训练所有在area中的r的参数
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
	///训练所有在area中的r的参数
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
	///训练所有在area中的r的参数
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
	///使用一层高斯模型训练一条路上每一个segment的参数mu以及sigma
	//////////////////////////////////////////////////////////////////////////
	//分发到每一段
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

	//计算mu
	for (int i = 0; i < r->thetas.size(); i++) //for each r
	{
		double mu = 0; //均值
		for each(GeoPoint* pt in slots[i])
		{
			mu += SMMD::distM_signed(r, pt);
		}
		if (slots[i].size() > 0)
		{
			mu /= slots[i].size();
			r->thetas[i].mu = mu;
			double sigma = 0; //标准差
			for each(GeoPoint* pt in slots[i])
			{
				double dist = SMMD::distM_signed(r, pt);
				sigma += (dist - mu) * (dist - mu);
			}
			sigma /= slots[i].size();
			sigma = sqrt(sigma);
			r->thetas[i].sigma = sigma;

			//注意到，如果N=1的情况下，sigma = 0， 导致无法计算高斯分布，因为分母变0了
			if (abs(sigma) < eps)
				slots[i].clear();
			/**********************************************************/
			/*test code starts from here*/
			//printf("mu = %lf, sigma = %lf, N = %d\n", mu, sigma, slots[i].size());
			//	system("pause");
			/*test code ends*/
			/**********************************************************/		
		}
		if (slots[i].size() == 0) //slot里面没有训练集点的情况 //注意，这儿不用else是因为前面代码有可能会使得有些slot里面强制清空为了来执行以下代码
		{
			if (i > 0) //如果不是第一个，那就抄前一个
			{
				r->thetas[i] = r->thetas[i - 1];
			}
			else //如果是第一个，抄第二个,第二个也没有的话抄第三个,以此类推直到抄到为止
			{
				//这儿先什么也不做，最后处理
			}
		}
		//处理开始几个没有的情况
		int slotId = 0;
		while (slots[slotId].size() == 0)
		{
			slotId++;
		}
		for (int i = 0; i < slotId; i++)
		{
			r->thetas[i] = r->thetas[slotId]; //把前面一串没有的全部抄thetas[slotId]
		}
	}
}

void Trainer::trainOneRoad(Edge* r, list<GeoPoint*>& pts)
{
	//////////////////////////////////////////////////////////////////////////
	///训练一条路r的参数，pts为匹配到r上的历史轨迹点
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
	///训练一条路r的参数，pts为匹配到r上的历史轨迹点
	//////////////////////////////////////////////////////////////////////////
	//分发pts到各个slots
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
	//计算参数
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
		else //slot里面没有训练集点的情况
		{
			if (i > 0) //如果不是第一个，那就抄前一个
			{
				r->thetas[i] = r->thetas[i - 1];
			}
			else //如果是第一个，抄第二个,第二个也没有的话抄第三个,以此类推直到抄到为止
			{
				//这儿先什么也不做，最后处理
			}
		}
		//处理开始几个没有的情况
		int slotId = 0;
		while (slots[slotId].size() == 0)
		{
			slotId++;
		}
		for (int i = 0; i < slotId; i++)
		{
			r->thetas[i] = r->thetas[slotId]; //把前面一串没有的全部抄thetas[slotId]
		}
	}
}

void Trainer::genCenterline(string outPath)
{
	//////////////////////////////////////////////////////////////////////////
	///对于area内的道路生成r_hat
	///需要先调用loadTrainData()
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
		pg.bigIterTimes = 3; //调整一整轮点的循环次数
		pg.smallIterTimes = 50; //调整一个点的迭代次数
		pg.step = 0.5; //每一次迭代的步长
		pg.sampleSize = 1000; //sampling的数量
		pg.anglePenalty = 0.003; 

		//将cluster里的轨迹点全部倒到pts里
		list<Pt> pts;

		//设置坐标转换器，将空间坐标映射到5000像素范围上比较合适
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

		//将polyline转回空间坐标
		vector<GeoPoint*> centerLine;
		for (int i = 0; i < pg.polyline.size(); i++)
		{
			GeoPoint* pt = new GeoPoint(mdTempForCoordThrans.screenToGeo((int)pg.polyline[i].x, (int)pg.polyline[i].y));
			centerLine.push_back(pt);
		}

		//输出
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
	///用原地图的道路形状生成假的centerline
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

		//输出
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