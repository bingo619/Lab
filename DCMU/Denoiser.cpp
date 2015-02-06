/* 
 * Last Updated at [2015/2/6 10:59] by wuhao
 */
#include "Denoiser.h"

//////////////////////////////////////////////////////////////////////////
///public part
//////////////////////////////////////////////////////////////////////////
void Denoiser::run(PointGridIndex* _ptIndex)
{
	//////////////////////////////////////////////////////////////////////////
	///老版本，废弃
	//////////////////////////////////////////////////////////////////////////
	this->ptIndex = _ptIndex;
	int k = 20;
	double kNNThresholdM = 3;
	int gridRange = 2;
	double supportRatio = 4.0;

	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptIndex->grid[row][col]))
			{
				calLMD(pt, k, kNNThresholdM);
			}
		}
		printf("row %d计算完成,共 %d 行\n", row, ptIndex->gridHeight);
	}
	printf("LMD计算完成\n");

	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptIndex->grid[row][col]))
			{
				outlierValidation(pt, gridRange, supportRatio);
			}
		}
		printf("row %d计算完成,共 %d 行\n", row, ptIndex->gridHeight);
	}
	printf("outlier点排除完成\n");
	return;

	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptIndex->grid[row][col]))
			{
				outlierReValidation(pt);
			}
		}
		printf("row %d计算完成,共 %d 行\n", row, ptIndex->gridHeight);
	}

	updatePtIndex(ptIndex);
}

void Denoiser::drawPts(MapDrawer& md)
{
	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptIndex->grid[row][col]))
			{
				if (pt->isOutlier == 1)
					//continue;
					md.drawPoint(Gdiplus::Color::Red, pt->lat, pt->lon);
				else
					md.drawPoint(Gdiplus::Color::Green, pt->lat, pt->lon);
			}
		}
	}
}

void Denoiser::outputJSON()
{
	string path = "json_denoise.js";
	ofstream ofs(path);
	if (!ofs)
	{
		cout << "open " << path << " error!" << endl;
		system("pause");
	}
	ofs << fixed << showpoint << setprecision(8);
	ofs << "data = [" << endl;
	vector<GeoPoint*> outputPts;
	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptIndex->grid[row][col]))
			{
				outputPts.push_back(pt);
			}
		}
	}
	for (int i = 0; i < outputPts.size() - 2; i++)
	{
		//{"x":1.29539,"y":103.78579,"edge":37618},
		GeoPoint* pt = outputPts[i];
		ofs << "{\"x\":" << pt->lat << ",\"y\":" << pt->lon << ",\"edge\":";
		if (pt->isOutlier == true)
			ofs << -1 << "}," << endl;
		else
			ofs << 1 << "}," << endl;
	}
	//last line
	ofs << "{\"x\":" << outputPts.back()->lat << ",\"y\":" << outputPts.back()->lon << ",\"edge\":";
	if (outputPts.back()->isOutlier == true)
		ofs << -1 << "}]" << endl;
	else
		ofs << 1 << "}]" << endl;
	ofs.close();
}

//////////////////////////////////////////////////////////////////////////
///private part
//////////////////////////////////////////////////////////////////////////
void Denoiser::calLMD(GeoPoint* pt, int k, double kNNThresholdM)
{
	//////////////////////////////////////////////////////////////////////////
	///计算pt到它的knn的平均距离
	//////////////////////////////////////////////////////////////////////////
	vector<GeoPoint*> kNNSet;
	ptIndex->kNN(pt, k, kNNThresholdM, kNNSet);
	double lmd = 0;
	for (int i = 0; i < kNNSet.size(); i++)
	{
		lmd += kNNSet[i]->dist;
	}
	lmd /= kNNSet.size();
	pt->lmd = lmd;
	
	/**********************************************************/
	/*test code starts from here*/
	double maxDist = -1;
	for (int i = 0; i < kNNSet.size(); i++)
	{
		if (kNNSet[i]->dist > maxDist);
		maxDist = kNNSet[i]->dist;
	}
	pt->lmd = maxDist;
	/*test code ends*/
	/**********************************************************/
	
}

void Denoiser::outlierValidation(GeoPoint* pt, int gridRange, double supportRatio)
{

	/**********************************************************/
	/*test code starts from here*/
	/*if (pt->extendField1 > 20.0)
	{
	pt->extendField2 = 1;
	return;
	}*/
	if (pt->lmd < 3.0)
	{
		pt->isOutlier = 0;
		return;
	}
	//else
	//	pt->extendField2 = 1;
	//return;
	/*test code ends*/
	/**********************************************************/

	/*vector<GeoPoint*> nearPts;
	ptIndex.getNearPts(pt, gridRange, nearPts);
	for (int i = 0; i < nearPts.size(); i++)
	{
	if (nearPts[i]->extendField1 / pt->extendField1 > supportRatio)
	{
	pt->extendField2 = 1;
	return;
	}
	}*/
	pair<int, int> rolCol = ptIndex->getRowCol(pt);
	int rowPt = rolCol.first;
	int colPt = rolCol.second;
	int row1 = rowPt - gridRange;
	int col1 = colPt - gridRange;
	int row2 = rowPt + gridRange;
	int col2 = colPt + gridRange;
	if (row1 < 0) row1 = 0;
	if (row2 >= ptIndex->gridHeight) row2 = ptIndex->gridHeight - 1;
	if (col1 < 0) col1 = 0;
	if (col2 >= ptIndex->gridWidth) col2 = ptIndex->gridWidth - 1;
	int currentGridCount = ptIndex->grid[rowPt][colPt]->size();
	for (int row = row1; row <= row2; row++)
	{
		for (int col = col1; col <= col2; col++)
		{
			if (ptIndex->grid[row][col]->size() / currentGridCount > supportRatio)
			{
				pt->isOutlier = 1;
				return;
			}
		}
	}
}

void Denoiser::outlierReValidation(GeoPoint* pt)
{
	if (pt->isOutlier == 0)
		return;
	pair<int, int> rolCol = ptIndex->getRowCol(pt);
	int rowPt = rolCol.first;
	int colPt = rolCol.second;
	int row1 = rowPt - 1;
	int col1 = colPt - 1;
	int row2 = rowPt + 1;
	int col2 = colPt + 1;
	if (row1 < 0) row1 = 0;
	if (row2 >= ptIndex->gridHeight) row2 = ptIndex->gridHeight - 1;
	if (col1 < 0) col1 = 0;
	if (col2 >= ptIndex->gridWidth) col2 = ptIndex->gridWidth - 1;
	int innerPtCount = 0;
	int outlierCount = 0;
	for (int row = row1; row <= row2; row++)
	{
		for (int col = col1; col <= col2; col++)
		{
			if (row == rowPt && col == colPt)
				continue;
			for each (GeoPoint* pt in *(ptIndex->grid[row][col]))
			{
				if (pt->isOutlier == 1)
					outlierCount++;
				else
					innerPtCount++;
			}
		}
	}
	if (outlierCount == 0)
	{
		if (innerPtCount >= 30)
		{
			pt->isOutlier = 0;
			return;
		}
		else
			return;
	}
	if (innerPtCount / outlierCount > 10)
	{
		pt->isOutlier = 0;
	}
}

void Denoiser::updatePtIndex(PointGridIndex* ptIndex)
{
	//////////////////////////////////////////////////////////////////////////
	///降噪后用来更新ptIndex里面的点，被认为是噪声的点将会被自动从中删除
	///需要注意的是删除的只是点的指针，点被开辟空间如果需要释放的话，还是得手动加入代码删除
	//////////////////////////////////////////////////////////////////////////
	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for (list<GeoPoint*>::iterator iter = ptIndex->grid[row][col]->begin(); iter != ptIndex->grid[row][col]->end();)
			{
				if ((*iter)->isOutlier == 0)
				{
					iter = ptIndex->grid[row][col]->erase(iter);
				}
				else
				iter++;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
//kNN降噪法需要推算的参数

double computePhi(PointGridIndex* ptIndex)
{
	double gridSizeM = 3.0;
	PointGridIndex tempIndex;
	list<GeoPoint*> pts;
	//get area
	double minLat = 999;
	double maxLat = -1;
	double minLon = 999;
	double maxLon = -1;
	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for each (GeoPoint* pt in *ptIndex->grid[row][col])
			{
				pts.push_back(pt);
				if (pt->lat < minLat) minLat = pt->lat;
				if (pt->lat > maxLat) maxLat = pt->lat;
				if (pt->lon < minLon) minLon = pt->lon;
				if (pt->lon > maxLon) maxLon = pt->lon;
			}
		}
	}
	double delta = 0.0001;
	Area tempArea(minLat - delta, maxLat + delta, minLon - delta, maxLon + delta);
	
	/**********************************************************/
	/*test code starts from here*/
	printf("square = %lf\n", (maxLon - minLon) * (maxLat - minLat) * GeoPoint::geoScale * GeoPoint::geoScale);
	/*test code ends*/
	/**********************************************************/
	
	double widthM = (maxLon - minLon) * GeoPoint::geoScale;
	int gridWidth = widthM / gridSizeM;
	tempIndex.createIndex(pts, &tempArea, gridWidth);

	//cal square
	double square = 0;
	for (int row = 0; row < tempIndex.gridHeight; row++)
	{
		for (int col = 0; col < tempIndex.gridWidth; col++)
		{
			if (tempIndex.grid[row][col]->size() > 0)
			{
				square++;
			}
		}
	}
	square = square * gridSizeM * gridSizeM;
	
	/**********************************************************/
	/*test code starts from here*/
	printf("square = %lf\n", square);
	/*test code ends*/
	/**********************************************************/
	
	

	//cal phi
	double roadWidthM = 10.7918;
	//tempIndex.drawGridLine(Gdiplus::Color::Green, md);
	return pts.size() / (square / (2 * roadWidthM));
}

double computeR(PointGridIndex* ptIndex, int k)
{
	double phi = computePhi(ptIndex);
	printf("phi = %lf\n", phi);
	double sigma = 15.4307;
	double l = 15;
	double p_l = exp(-l*l / (2 * sigma*sigma)) / (sqrt(2 * PI) * sigma);
	return sqrt(k / (phi * p_l)) / 2;
}

//////////////////////////////////////////////////////////////////////////
//这部分代码暂时不用
int Denoiser::calDensity(GeoPoint* pt)
{
	vector<GeoPoint*> nearPts;
	ptIndex->getNearPts(pt, r, nearPts);
	return (nearPts.size() + 1);
}

int Denoiser::calDensity(double lat, double lon)
{
	GeoPoint tempPt(lat, lon);
	vector<GeoPoint*> nearPts;
	ptIndex->getNearPts(&tempPt, r, nearPts);
	return nearPts.size();
}

pair<int, double> Denoiser::findMaxDensity(GeoPoint* pt)
{
	double currentL = 0, currentArc = 0;
	int maxDensity = -1;
	double maxL;
	while (currentL <= L)
	{
		currentArc = 0;
		double angleStep = arcStep / L;
		double currentAngle = 0;
		while (currentArc < 2 * PI * currentL)
		{
			double deltaX = r * cos(currentAngle) / GeoPoint::geoScale;
			double deltaY = r * sin(currentAngle) / GeoPoint::geoScale;
			int density = calDensity(pt->lat + deltaY, pt->lon + deltaX);
			//printf("lat = %lf, lon = %lf\n", pt->lat + deltaY, pt->lon + deltaX);
			//pt->print();
			//printf("density = %d\n", density);
			//system("pause");
			if (maxDensity < density)
			{
				maxDensity = density;
				maxL = currentL;
			}
			currentAngle += angleStep;
			currentArc += arcStep;
		}
		currentL += LStep;
	}
	if (maxDensity == -1)
	{
		cout << "density = -1!" << endl;
		system("pause");
	}
	//cout << "maxdensity = " << maxDensity << endl;
	//system("pause");
	return make_pair(maxDensity, maxL);
}

//////////////////////////////////////////////////////////////////////////
//新版本
void Denoiser::outlierValidationEx(GeoPoint* pt, double kNNThresholdM)
{
	if (pt->lmd < kNNThresholdM)
	{
		pt->isOutlier = 0;
		return;
	}
	pt->isOutlier = 1;
	return;
	int density = calDensity(pt);
	pair<int, double> tempPair = findMaxDensity(pt);
	double maxL = tempPair.second;
	//printf("density = %d\n", density);
	//system("pause");
	int maxDensity = tempPair.first;
	
	/*double ratio = (double)maxDensity / (double)density;
	if (ratio > 5)
	{
		pt->isOutlier = 1;
	}
	return;*/
	if (density < 3)
	{
		pt->isOutlier = 1;
	}
	if (maxL > 15)
	{
		pt->isOutlier = 1;
	}
	return;

	double prob = 1 - pow((double)density / (double)maxDensity, (maxL - 10.0) / 2.0);
	cout << "prob = " << prob << endl;
	cout << "maxL = " << maxL << endl;
	cout << "density = " << density << endl;
	cout << "maxDensity = " << maxDensity << endl;
	cout << (double)density / (double)maxDensity << endl;
	system("pause");
	double randVal = ((double)rand()) / (double)RAND_MAX;
	if (randVal < prob)
	{
		pt->isOutlier = 1;
	}
}

void Denoiser::runEx(PointGridIndex* _ptIndex)
{
	this->ptIndex = _ptIndex;
	int count = 0;
	int k = 10;
	double kNNThresholdM = computeR(this->ptIndex, k);
	kNNThresholdM *= relaxRatio;//or k = 10, ratio = 1.9
	cout << "kNNThresholdM = " << kNNThresholdM << endl;
	//return;
//	double kNNThresholdM = 2.3;

	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptIndex->grid[row][col]))
			{
				calLMD(pt, k, kNNThresholdM);
			}
		}
	}
	printf("LMD计算完成\n");
	for (int row = 0; row < ptIndex->gridHeight; row++)
	{
		for (int col = 0; col < ptIndex->gridWidth; col++)
		{
			for each (GeoPoint* pt in *(ptIndex->grid[row][col]))
			{
				if (count % 1000 == 0)
				{
					cout << "已处理" << count << endl;
				}
				outlierValidationEx(pt, kNNThresholdM);
				count++;
			}
		}
	}
}

