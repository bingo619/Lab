/* 
 * Last Updated at [2014/9/24 11:08] by wuhao
 */
#include "DCMU.h"

//////////////////////////////////////////////////////////////////////////
///public part
//////////////////////////////////////////////////////////////////////////

void DCMU::run()
{
	//网格聚类
	gridClustering();
	cout << ptIndexes.size() << endl;
	
	/*//code for TCC
	extractForTC1();
	return;*/

	//对每一个block上色
	for (int i = 0; i < ptIndexes.size(); i++)
	{
		Gdiplus::Color color = MapDrawer::randomColor();
		PointGridIndex* ptIndex = ptIndexes[i];
		for (int row = 0;  row < ptIndex->gridHeight; row++)
		{
			for (int col = 0; col < ptIndex->gridWidth; col++)
			{
				for each (GeoPoint* pt in *ptIndex->grid[row][col])
				{
					md.drawPoint(color, pt->lat, pt->lon);
				}
			}
		}
	}
	//降噪，25m匹配距离的数据可以不用
 	//for (int i = 0; i < ptIndexes.size(); i++)
 	//{
		///*MapDrawer md;
		//md.setArea(&area);
		//md.setResolution(1000);
		//md.newBitmap();
		//md.lockBits();
		//denoiser.runEx(ptIndexes[i]);
		//denoiser.drawPts(md);
		//md.unlockBits();
		//char id[10];
		//_itoa(i, id, 10);
		//string id_str = id;
		//md.saveBitmap("partition_" + id_str + ".png");*/
		//denoiser.runEx(ptIndexes[i]);
		////denoiser.drawPts(md);
 	//}

	//方向聚类
	for (int i = 0; i < ptIndexes.size(); i++)
	{
		cout << "output i = " << i << endl;
		ptCluster.run(ptIndexes[i]);
		ptCluster.outputPtsDir(StringOperator::intToString(i) + ".txt");
		ptCluster.pts.clear();
		//ptCluster.drawClusters(md);
		//cout << "!!!!!!!!!!!" << ptCluster.pts.size();
		//用来画点方向，
		/*for each (GeoPoint* pt in ptCluster.pts)
		{
			GeoPoint* ptEnd = new GeoPoint;
			double length = 3 / GeoPoint::geoScale; //箭头长度为3m
			ptEnd->lat = pt->lat + length * sin(pt->direction);
			ptEnd->lon = pt->lon + length * cos(pt->direction);
			md.drawLine(Gdiplus::Color::Blue, pt->lat, pt->lon, ptEnd->lat, ptEnd->lon);
		}*/
	}
	
	//生成路段
	RoadGenerator rg;
	rg.run(&allPtIndex, ptCluster.clusters);
}

void DCMU::extractForTC1()
{
	//////////////////////////////////////////////////////////////////////////
	///目标区域：港口区
	///目的：将partition到partition[0]的所有点的clusterId标记成1
	///然后将所有连续的两个点的clusterId都为1的segment选出来
	//////////////////////////////////////////////////////////////////////////
	ofstream ofs("port.txt");
	ofs << fixed << showpoint << setprecision(8);
	int partId = 1;
	for (int row = 0; row < ptIndexes[partId]->gridHeight; row++)
	{
		for (int col = 0; col < ptIndexes[partId]->gridWidth; col++)
		{
			for each (GeoPoint* pt in *ptIndexes[partId]->grid[row][col])
			{
				pt->clusterId = 1;
			}
		}
	}
	for each(Traj* traj in trajs)
	{
		Traj::iterator currentIter = traj->begin();
		Traj::iterator nextIter = traj->begin(); nextIter++;
		for (; nextIter != traj->end(); currentIter++, nextIter++)
		{
			if ((*currentIter)->clusterId == 1 && (*nextIter)->clusterId == 1)
			{
				ofs << (*currentIter)->time << " " << (*currentIter)->lat << " " << (*currentIter)->lon <<
					" " << (*nextIter)->time << " " << (*nextIter)->lat << " " << (*nextIter)->lon << endl;
				md.drawLine(Gdiplus::Color::Red, (*currentIter)->lat, (*currentIter)->lon, (*nextIter)->lat, (*nextIter)->lon);
			}
		}
	}
	ofs.close();
}

void DCMU::run1()
{
	//////////////////////////////////////////////////////////////////////////
	///只测试一个block，也就是一个ptIndexes[i]
	//////////////////////////////////////////////////////////////////////////
	allPtIndex.drawGridLine(Gdiplus::Color::Green, md);
	//网格聚类
	gridClustering();

	/**********************************************************/
	/*test code starts from here*/
	//用来看通过gridClustering后聚出来的每个block里面有多少点
	//可以通过挑里面点的数量对应的index来做专项测试
	for (int i = 0; i < ptIndexes.size(); i++)
	{
		int count = 0;
		for (int row = 0; row < ptIndexes[i]->gridHeight; row++)
		{
			for (int col = 0; col < ptIndexes[i]->gridWidth; col++)
			{
				count += ptIndexes[i]->grid[row][col]->size();
			}
		}
		if (count > 10000)
		{
			cout << i << ": " << count << endl;
			//system("pause");
		}
	}
	/*test code ends*/
	/**********************************************************/
	
	int index = 0; //这里填上想要专门测试的blockID
	denoiser.run(ptIndexes[index]);

	//方向聚类
	ptCluster.run(ptIndexes[index]);
	ptCluster.drawClusters(md);

	//生成路段
	RoadGenerator rg;
	rg.run(ptIndexes[index], ptCluster.clusters);
}

void DCMU::run2()
{
	//////////////////////////////////////////////////////////////////////////
	///专为降噪设计
	//////////////////////////////////////////////////////////////////////////
	denoiser.runEx(&allPtIndex);
	denoiser.drawPts(md);
	denoiser.outputJSON();
}

double cosAngle(GeoPoint* pt1, GeoPoint* pt2, GeoPoint* pt3)
{
	double v1x = pt2->lon - pt1->lon;
	double v1y = pt2->lat - pt1->lat;
	double v2x = pt3->lon - pt2->lon;
	double v2y = pt3->lat - pt2->lat;
	return (v1x * v2x + v1y * v2y) / sqrt((v1x * v1x + v1y * v1y)*(v2x * v2x + v2y * v2y));
}

double getRoadDir(GeoPoint* pt, Edge* edge)
{
	//////////////////////////////////////////////////////////////////////////
	///测试降噪厉害的版本用的，现在用不到
	//////////////////////////////////////////////////////////////////////////
	double minDist = INF;
	double roadDir;
	//遍历端点距离
	for (Figure::iterator iter = edge->figure->begin(); iter != edge->figure->end(); iter++)
	{
		double tmpDist = GeoPoint::distM(pt->lat, pt->lon, (*iter)->lat, (*iter)->lon);
		if (tmpDist < minDist)
		{
			minDist = tmpDist;
			if (*iter != edge->figure->back())
			{
				Figure::iterator tempIter = iter;
				tempIter++;
				double vec_x = (*tempIter)->lon - (*iter)->lon;
				double vec_y = (*tempIter)->lat - (*iter)->lat;
				if (vec_y < 0)
				{
					vec_x = -vec_x;
					vec_y = -vec_y;
				}
				roadDir = acos(vec_x / sqrt(vec_x * vec_x + vec_y * vec_y));
			}
			else
			{
				Figure::iterator tempIter = iter;
				tempIter--;
				double vec_x = (*iter)->lon - (*tempIter)->lon;
				double vec_y = (*iter)->lat - (*tempIter)->lat;
				if (vec_y < 0)
				{
					vec_x = -vec_x;
					vec_y = -vec_y;
				}
				roadDir = acos(vec_x / sqrt(vec_x * vec_x + vec_y * vec_y));
			}
		}
	}
	//遍历投影距离
	Figure::iterator iter = edge->figure->begin();
	Figure::iterator nextIter = edge->figure->begin();
	nextIter++;
	while (nextIter != edge->figure->end())
	{
		//有投影
		if (cosAngle(pt, (*iter), (*nextIter)) <= 0 && cosAngle(pt, (*nextIter), (*iter)) <= 0)
		{
			double A = ((*nextIter)->lat - (*iter)->lat);
			double B = -((*nextIter)->lon - (*iter)->lon);
			double C = (*iter)->lat * ((*nextIter)->lon - (*iter)->lon)
				- (*iter)->lon * ((*nextIter)->lat - (*iter)->lat);
			double tmpDist = abs(A * pt->lon + B * pt->lat + C) / sqrt(A * A + B * B);
			tmpDist *= GeoPoint::geoScale;
			if (minDist > tmpDist)
			{
				minDist = tmpDist;
				double vec_x = (*nextIter)->lon - (*iter)->lon;
				double vec_y = (*nextIter)->lat - (*iter)->lat;
				if (vec_y < 0)
				{
					vec_x = -vec_x;
					vec_y = -vec_y;
				}
				roadDir = acos(vec_x / sqrt(vec_x * vec_x + vec_y * vec_y));
			}
		}
		iter++;
		nextIter++;
	}
	return roadDir;
}

void DCMU::run3(int roadId, double rectW, double rectL)
{
	//////////////////////////////////////////////////////////////////////////
	///测试降噪的厉害的版本用的，暂时现在不用
	//////////////////////////////////////////////////////////////////////////
	ptCluster.d = rectW;
	ptCluster.l = rectL;
	ptCluster.run(&allPtIndex);
	//drawPts
	for each (GeoPoint* pt in ptCluster.pts)
	{
		md.drawPoint(Gdiplus::Color::Green, pt->lat, pt->lon);
	}

	//drawRoadId
	for (int i = 0; i < roadNetwork.edges.size(); i++)
	{
		if (roadNetwork.edges[i] == NULL)
		{
			continue;
		}
		int  deltaX = (double)rand() / RAND_MAX *70;
		int  deltaY = (double)rand() / RAND_MAX * 50;
		int x = md.geoToScreen(roadNetwork.edges[i]->figure->front()->lat, roadNetwork.edges[i]->figure->front()->lon).X + deltaX;
		int y = md.geoToScreen(roadNetwork.edges[i]->figure->front()->lat, roadNetwork.edges[i]->figure->front()->lon).Y + deltaY;
		md.drawInt(Gdiplus::Color::Red, x, y, i);
	}
	//return;
	
	/*double roadDir;
	double vec_x = roadNetwork.edges[roadId]->figure->back()->lon - roadNetwork.edges[roadId]->figure->front()->lon;
	double vec_y = roadNetwork.edges[roadId]->figure->back()->lat - roadNetwork.edges[roadId]->figure->front()->lat;
	if (vec_y < 0)
	{
		vec_x = -vec_x;
		vec_y = -vec_y;
	}*/
	
	//vec_x *= 1000;
	//vec_y *= 1000;
	//printf("vec_X = %lf, vec_y = %lf\n", vec_x, vec_y);
	//printf("acos = %.8lf\n", vec_x / sqrt(vec_x * vec_x + vec_y * vec_y));
	//roadDir = acos(vec_x / sqrt(vec_x * vec_x + vec_y * vec_y));
	//cout << "roadDir = " << roadDir << endl;
	//system("pause");
	//ptCluster.calPtsDirs();
	double error = 0;
	//cal road Dir

	int count = 0;
	for (int row = 0; row < ptCluster.ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptCluster.ptIndex->gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptCluster.ptIndex->grid[row][col]))
			{
				if (pt->mmRoadId != roadId)
					continue;
				count++;

				double roadDir = getRoadDir(pt, roadNetwork.edges[roadId]);

				GeoPoint* ptEnd = new GeoPoint;
				double length = 3 / GeoPoint::geoScale;
				ptEnd->lat = pt->lat + length * sin(pt->direction);
				ptEnd->lon = pt->lon + length * cos(pt->direction);
				md.drawLine(Gdiplus::Color::Blue, pt->lat, pt->lon, ptEnd->lat, ptEnd->lon);

				

				double deltaAngle = 0;
				if (abs(roadDir - pt->direction) < PI / 2)
					deltaAngle = abs(roadDir - pt->direction);
				else
					deltaAngle = PI - abs(roadDir - pt->direction);
				if (deltaAngle > ptCluster.angleThres)
				{
					error += (deltaAngle - ptCluster.angleThres);// *(deltaAngle - ptCluster.angleThres);
// 					GeoPoint* ptEnd = new GeoPoint;
// 					double length = 3 / GeoPoint::geoScale;
// 					ptEnd->lat = pt->lat + length * sin(pt->direction);
// 					ptEnd->lon = pt->lon + length * cos(pt->direction);
// 					md.drawLine(Gdiplus::Color::Red, pt->lat, pt->lon, ptEnd->lat, ptEnd->lon);
				}
			}
		}		
	}
	error /= count;
	//error /= 2;
	printf("w = %lf, l = %lf, error = %.8lf\n", rectW, rectL, error);
}


//////////////////////////////////////////////////////////////////////////
///private part
//////////////////////////////////////////////////////////////////////////
void DCMU::gridClustering()
{
	//////////////////////////////////////////////////////////////////////////
	///网格粗糙聚类，实际上就是个dfs
	///将每个聚类中的点输出到ptIndexes[i]
	///如果联通分支过小则忽略，不输出
	//////////////////////////////////////////////////////////////////////////
	bool** dfsState = new bool*[allPtIndex.gridHeight];
	for (int i = 0; i < allPtIndex.gridHeight; i++)
		dfsState[i] = new bool[allPtIndex.gridWidth];
	for (int i = 0; i < allPtIndex.gridHeight; i++)
	{
		for (int j = 0; j < allPtIndex.gridWidth; j++)
		{
			dfsState[i][j] = false;
		}
	}
	int count = 0; //连通分支编号

	for (int row = 0; row < allPtIndex.gridHeight; row++)
	{
		for (int col = 0; col < allPtIndex.gridWidth; col++)
		{
			if (dfsState[row][col])
				continue;
			else
			{
				vector<pair<int, int>> connectingCompnt; //记录当前dfs的连通分支
				dfs(row, col, dfsState, connectingCompnt);
				if (connectingCompnt.size() < 10) //连通分支过小，忽略
					continue;
				else
				{
					//根据连通分支去索引里把点输出
					PointGridIndex* tempPtIndex = new PointGridIndex;
					list<GeoPoint*> tempPts;
					for (int i = 0; i < connectingCompnt.size(); i++)
					{						
						for each (GeoPoint* pt in *(allPtIndex.grid[connectingCompnt[i].first][connectingCompnt[i].second]))
						{
							tempPts.push_back(pt);
						}
					}
					tempPtIndex->createIndex(tempPts, &area, gridWidth);
					ptIndexes.push_back(tempPtIndex);
					count++;
				}
			}
		}
	}
}

void DCMU::dfs(int row, int col, bool** dfsState, vector<pair<int, int>>& connectingCompnt)
{
	//////////////////////////////////////////////////////////////////////////
	///对网格[row][col]进行dfs遍历，其中标记访问为此函数职责，dfsState为访问状态，true为已访问
	///将自己的索引编号存入connectingCompnt以记录连通分支信息
	///ignoreThres 为设定的忽略网格的阈值，当网格内点数量小于该阈值则视为该网格为空
	//////////////////////////////////////////////////////////////////////////
	int ignoreThres = 3;
	if (row >= allPtIndex.gridHeight || row < 0 || col >= allPtIndex.gridWidth || col < 0)
		return;
	if (dfsState[row][col])
		return;
	else
	{  
		dfsState[row][col] = true;
		if (allPtIndex.grid[row][col]->size() <= ignoreThres) //网格里点数量小于一定程度的忽略
			return;
		connectingCompnt.push_back(make_pair(row, col));
		dfs(row + 1, col, dfsState, connectingCompnt);
		dfs(row - 1, col, dfsState, connectingCompnt);
		dfs(row, col + 1, dfsState, connectingCompnt);
		dfs(row, col - 1, dfsState, connectingCompnt);
	}
}