#include <iostream>
#include <math.h>
#include "GeoPoint.h"
#include "Map.h"
#include "MapDrawer.h"
#include "MapMatching.h"
#include "TrajReader.h"
#include "TrajDrawer.h"
#include "Integral.h"
#include "StringOperator.h"
#include "SMMD.h"
#include "Trainer.h"
#include <string>
#include <time.h>
#include <thread>
#include "Evaluation.h"
using namespace std;
using namespace Gdiplus;

//string workspaceFolder = "C:\\Downloads\\TrajData\\SMMD\\";
string workspaceFolder = "D:\\trajectory\\singapore_data\\experiments\\SMMD\\";
//string mapFileFolder = "C:\\Downloads\\TrajData\\map\\old\\";
string mapFileFolder = "D:\\trajectory\\singapore_data\\singapore_map\\old\\";
string testDataFolder = workspaceFolder + "testData\\";
string trainDataFolder = workspaceFolder + "trainData\\beijing\\";


int datasetID = 3;
string trainDataPath = trainDataFolder + "area" + StringOperator::intToString(datasetID) +"_50m.txt"; // +"area1_50m.txt";
string testDataPath = testDataFolder + "testData3_area" + StringOperator::intToString(datasetID) +".txt";// "testData_area1.txt";
//string testDataPath = testDataFolder + "testData3.txt";
string polylineFilePath = trainDataFolder + "polyline1_new.txt";
string fakeCenterlineFilePath = trainDataFolder + "fakeCenterline" + StringOperator::intToString(datasetID) +".txt";
string priorFilePath = trainDataFolder + "SMMD_prior3.txt";

//Area area(1.342433, 1.353236, 103.864837, 103.875921); //area1
//Area area(1.3290, 1.3444, 103.8377, 103.8598); //area2
//Area area(1.3280, 1.3947, 103.8245, 103.9114); //area3
//Area area(1.22, 1.5, 103.62, 104); //singapore half
//Area area(39.398073, 40.366082, 115.770605, 117.079043); //beijing full
//Area area(40.0310, 40.0764, 116.4001, 116.4710); //beijing area1
//Area area(39.9308, 40.0168, 116.3042, 116.4228); //beijing area2
Area area(39.9651, 39.9899, 116.3711, 116.4134); //beijing area3
/*double minLat = 1.22;
double maxLat = 1.5;
double minLon = 103.620;
double maxLon = 104.0;*/


double noiseScaleM = 0;


Map roadNetwork;
Trainer trainer(&roadNetwork, &area);
MapDrawer md;

MapMatcher mm(&roadNetwork);

vector<SampleType> testDataSet;
list<GeoPoint*> testSMMDataSet;
list<GeoPoint*> trainDataSet;

void loadTestData(double threshM_for_intersection, int count  = INF, bool draw = false );

//////////////////////////////////////////////////////////////////////////
///只跑一遍
//////////////////////////////////////////////////////////////////////////
void trans(int i, ofstream& ofs)
{
	//////////////////////////////////////////////////////////////////////////
	///处理测试数据，数据格式为raw data
	//////////////////////////////////////////////////////////////////////////
	ifstream ifs_trip(testDataFolder + "raw\\trip" + StringOperator::intToString(i) + ".txt");
	ifstream ifs_result(testDataFolder + "raw\\new_match_result" + StringOperator::intToString(i) + ".txt");
	if (!ifs_trip)
	{
		cout << "error!" << endl;
		system("pause");
	}
	while (!ifs_trip.eof())
	{
		string carId;
		double xLat, xLon, dLat, dLon;
		int dummy;
		int rId;
		ifs_trip >> carId >> xLat >> xLon >> dLat >> dLon >> dummy >> dummy;
		//printf("carId = %s, xlat = %lf, xlon = %lf, dlat = %lf, dlon = %lf\n", carId.c_str(), xLat, xLon, dLat, dLon);
		//if (ifs_trip.fail())
		//	break;
		ifs_result >> rId;
		//printf("rId = %d\n", rId);
		//system("pause");
		if (rId != -1)
		{
			ifs_result >> dummy >> dummy >> dummy >> dummy >> dummy;
		}
		else
			continue;
		ofs << xLat << " " << xLon << " " << dLat << " " << dLon << " " << rId << endl;
	}
	ifs_trip.close();
	ifs_result.close();
}

void outputTestData()
{
	ofstream ofs("testData.txt");
	ofs << fixed << showpoint << setprecision(8);
	for (int i = 0; i < 10; i++)
	{
		trans(i, ofs);
	}
	ofs.close();
}

void testForDistM()
{
	loadTestData(25.0);
	SMMD smmd(roadNetwork);
	roadNetwork.loadPolylines(trainDataFolder + "polyline1.txt");
	//md.lockBits();
	//先画polyline
	int specialId = 19868;
	for each (Edge* r in roadNetwork.edges)
	{
		if (r == NULL || r->r_hat.size() == 0)
			continue;
		if (r->id != specialId)
			continue;
		for (int i = 0; i < r->r_hat.size() - 1; i++)
		{
			md.drawLine(Color::Black, r->r_hat[i]->lat, r->r_hat[i]->lon, r->r_hat[i + 1]->lat, r->r_hat[i + 1]->lon);
		}
		md.drawBigPoint(Color::Blue, r->r_hat[0]->lat, r->r_hat[0]->lon); //画起点
		r->alpha = 0;
	}
	
	//再画点
	for each (SampleType td in testDataSet)
	{
		if (td.rId != specialId)
			continue;
		if (roadNetwork.edges[td.rId] == NULL || roadNetwork.edges[td.rId]->r_hat.size() == 0)
			continue;
		if (smmd.distM_signed(roadNetwork.edges[td.rId], td.x) < 0)
			md.drawBigPoint(Color::Red, td.x->lat, td.x->lon);
		else
			md.drawBigPoint(Color::Green, td.x->lat, td.x->lon);
		roadNetwork.edges[td.rId]->alpha++;
		/*if (roadNetwork.edges[td.rId]->alpha > 100)
		{
			cout << td.rId << endl;
			system("pause");
		}*/
	}
	md.unlockBits();
	md.saveBitmap("testDistM.png");
}

void filterTrainData()
{
	//////////////////////////////////////////////////////////////////////////
	///将训练集数据过滤，只输出area内的数据
	//////////////////////////////////////////////////////////////////////////
	string outpath = "area2_50m.txt";
	string inpath = trainDataFolder + "newMMTrajs_50m.txt";
	TrajReader tReader(inpath);
	list<GeoPoint*> pts;
	tReader.readGeoPoints(pts, &area);//, 1000);
	tReader.close();
	tReader.outputPts(pts, outpath);
}

void filterTestData()
{
	ifstream ifs(testDataFolder + "testData3.txt");
	ofstream ofs("testData3_area1.txt");
	ofs << fixed << showpoint << setprecision(8);
	int currentCount = 0;
	if (!ifs)
	{
		cout << "error!" << endl;
		system("pause");
	}
	while (!ifs.eof())
	{
		double xLat, xLon, dLat, dLon;
		int rId;
		ifs >> xLat >> xLon >> dLat >> dLon >> rId;
		if (!area.inArea(xLat, xLon))
			continue;
		if (roadNetwork.edges[rId] == NULL)
			continue;
		ofs << xLat << " " << xLon << " " << dLat << " " << dLon << " " << rId << endl;
		currentCount++;
	}
	ofs.close();
	ifs.close();
}

void drawTrainData(string filePath)
{
	TrajReader tReader(filePath);
	list<GeoPoint*> pts;
	tReader.readGeoPoints(pts, &area, 2000000);
	tReader.close();

	for each (GeoPoint* pt in pts)
	{
		md.drawBigPoint(Color::Green, pt->lat, pt->lon);
	}

	trainer.drawCenterline(fakeCenterlineFilePath, md);
}

//////////////////////////////////////////////////////////////////////////
///噪声相关
//////////////////////////////////////////////////////////////////////////

void addNoise(double noiseScaleM, vector<SampleType>& tests)
{
	//////////////////////////////////////////////////////////////////////////
	///对测试集加上噪声
	//////////////////////////////////////////////////////////////////////////

	for (int i = 0; i < tests.size(); i++)
	{
		double deltaX = (((double)rand() / (double)RAND_MAX) * 2 - 1) * noiseScaleM / GeoPoint::geoScale;
		double deltaY = (((double)rand() / (double)RAND_MAX) * 2 - 1) * noiseScaleM / GeoPoint::geoScale;
		if (!area.inArea(tests[i].x->lat + deltaY, tests[i].x->lon + deltaX))
		{
			continue;
		}

		tests[i].x->lat += deltaY;
		tests[i].x->lon += deltaX;
	}
}

void addNoise(double noiseScaleM, list<GeoPoint*>& trainDataSet)
{
	//////////////////////////////////////////////////////////////////////////
	///对训练集加上噪声
	//////////////////////////////////////////////////////////////////////////
	for  each (GeoPoint* pt in trainDataSet)
	{
		double deltaX = (((double)rand() / (double)RAND_MAX) * 2 - 1) * noiseScaleM / GeoPoint::geoScale;
		double deltaY = (((double)rand() / (double)RAND_MAX) * 2 - 1) * noiseScaleM / GeoPoint::geoScale;
		if (!area.inArea(pt->lat + deltaY, pt->lon + deltaX))
		{
			continue;
		}

		pt->lat += deltaY;
		pt->lon += deltaX;
	}
}

////////////////////////////////////////////////////////////////////////

vector<int> correctCounts;
vector<int> allCounts;

void loadTestData(double threshM_for_intersection, int count /* = INF */, bool draw /* = false */)
{
	//////////////////////////////////////////////////////////////////////////
	///读入SMMD测试数据
	//////////////////////////////////////////////////////////////////////////
	ifstream ifs(testDataPath);
	int currentCount = 0;
	if (!ifs)
	{
		cout << "error!" << endl;
		system("pause");
	}
	while (!ifs.eof())
	{
		double xLat, xLon, dLat, dLon;
		int rId;
		ifs >> xLat >> xLon >> dLat >> dLon >> rId;
		if (!area.inArea(xLat, xLon))
		{
			cout << "not in" << endl;
			continue;
		}
		if (currentCount >= count)
			break;
		if (roadNetwork.edges[rId] == NULL)
			continue;
		GeoPoint* x = new GeoPoint(xLat, xLon);
		GeoPoint* d = new GeoPoint(dLat, dLon);
		SampleType td(x, d, rId);

		//滤去在边界的点
		if (roadNetwork.edges[rId]->r_hat.size() == 0)
		{
			continue;
		}
		//滤去路口的点
		vector<GeoPoint*> oneNN;
		roadNetwork.ptIndex.kNN_exact(x, 1, oneNN);
		if (GeoPoint::distM(oneNN[0], x) < threshM_for_intersection)
			continue;
		if (roadNetwork.edges[rId]->r_hat.size() == 0)
			continue;
		testDataSet.push_back(td);
		currentCount++;
	}
	cout << "测试集读入完成， 共读入 " << testDataSet.size() << "个测试样本" << endl;
	ifs.close();

	if (draw)
	{
		for each (SampleType testData in testDataSet)
		{
			md.drawPoint(Color::Red, testData.x->lat, testData.x->lon);
		}
	}
}

void trainSMM(double intervalM, int count = INF)
{
	TrajReader tReader(trainDataPath);
	tReader.readGeoPoints(trainDataSet, &area, count);
	tReader.close();
	//add noise
	addNoise(noiseScaleM, trainDataSet);
	////trainer.loadTrainData(trainDataSet);
	//trainer.loadTrainData(trainDataPath, count);
	////trainer.loadPrior(priorFilePath);
	//roadNetwork.loadPolylines(polylineFilePath);
	//roadNetwork.loadPolylines(fakeCenterlineFilePath);
	////trainer.trainSimple(intervalM);
	cout << "参数训练完成" << endl;
}

void trainSMMD(double intervalM, int count = INF)
{
	for each (SampleType testData in testDataSet)
	{
		trainer.trainDataSet.push_back(new GeoPoint(testData.x->lat, testData.x->lon, 0, testData.rId));
	}

	for each (GeoPoint* pt in trainer.trainDataSet)
	{
		if (pt == NULL || pt->mmRoadId == -1)
			continue;
		if (roadNetwork.edges[pt->mmRoadId] != NULL)
			roadNetwork.edges[pt->mmRoadId]->trainData.push_back(pt);
	}
	trainer.loadPrior(priorFilePath);
	//roadNetwork.loadPolylines(polylineFilePath);
	//roadNetwork.loadPolylines(fakeCenterlineFilePath);
	trainer.trainSimple(intervalM);
	cout << "参数训练完成" << endl;
}

void evalSMMD(vector<SampleType>& tests)
{
	//system("pause");
	double (SMMD::* pFunc)(Edge*, GeoPoint*, GeoPoint*); //一个类成员函数指针变量pFunc的定义
	

	bool smmdSwitch = false;
	if (smmdSwitch)
	{
		pFunc = &SMMD::probSMMD;
		//读入SMMD testdata
	}
	else
	{
		pFunc = &SMMD::probSMM;
		//构造SMM testData
		list <GeoPoint*> testSMMDataSet;
		TrajReader tReader(trainDataPath);
		tReader.readGeoPoints(testSMMDataSet, &area, 100000);
		tReader.close();
		tests.clear();
		for each (GeoPoint* pt in testSMMDataSet)
		{
			tests.push_back(SampleType(pt, NULL, pt->mmRoadId));
		}
	}
	cout << "test sample size = " << tests.size() << endl;


	SMMD smmd(roadNetwork);
	int baselineCorrectCount = 0;
	int correctCount = 0;
	int allCount = tests.size();

	int crossCount = 0;
	int biasCount = 0;
	int probCount = 0;
	int twoNNNoCount = 0;
	int invalidCount = 0;

	int smmCorrectCount = 0;

	for (int i = 0; i < tests.size(); i++)
	{
		if (i % 1000 == 0)
			cout << i << endl;
		SampleType testData = tests[i];

		if (testData.rId == -1)
		{
			allCount--;
			continue;
		}

		if (roadNetwork.edges[testData.rId] == NULL)
		{
			allCount--;
			continue;
		}

		if (roadNetwork.edges[testData.rId]->r_hat.size()== 0 || roadNetwork.distM(testData.x->lat, testData.x->lon, roadNetwork.edges[testData.rId]) >= 50.0)
		{
			allCount--;
			invalidCount++;
			//md.drawBigPoint(Color::Black, testData.x->lat, testData.x->lon);
			continue;
		}

		/*int slotId = roadNetwork.edges[testData.rId]->getSlotId(testData.x);
		if (abs(roadNetwork.edges[testData.rId]->thetas[slotId].mu) < 5)
		{
			allCount--;
			continue;
		}*/
		/*
		if (abs(roadNetwork.edges[testData.rId]->u) > 5)
		{
			allCount--;
			continue;
		}*/

		//滤去路口测试数据点
		vector<GeoPoint*> oneNN;
		roadNetwork.ptIndex.kNN_approx(testData.x, 1, 100.0, oneNN);
		if (GeoPoint::distM(oneNN[0], testData.x) < -50.0)
		{
			allCount--;
			crossCount++;
			continue;
		}

		vector<Edge*> nearEdges;
		roadNetwork.getNearEdges(testData.x->lat, testData.x->lon, 2, nearEdges);
		if (nearEdges[1] == NULL || nearEdges[0] == NULL)
		{
			system("pause");
		}
		if (nearEdges[0]->r_hat.size() == 0 || nearEdges[1]->r_hat.size() == 0)
		{
			allCount--;
			continue;
		}

		//最近两条之中都没的话排除
		if (nearEdges[0]->id != testData.rId && nearEdges[1]->id != testData.rId)
		{
			allCount--;
			twoNNNoCount++;
			continue;
		}

		//偏的不厉害的排除
		int slotId0 = nearEdges[0]->getSlotId(testData.x);
		int slotId1 = nearEdges[1]->getSlotId(testData.x);
		if (abs(roadNetwork.edges[testData.rId]->thetas[slotId0].mu - roadNetwork.edges[testData.rId]->thetas[slotId1].mu) < 0)
		{
			allCount--;
			biasCount++;
			continue;
		}

		//如果最近的就一条
		if (abs(roadNetwork.distM(testData.x->lat, testData.x->lon, nearEdges[1]) - roadNetwork.distM(testData.x->lat, testData.x->lon, nearEdges[0])) > eps)
		{
			allCount--;
			//biasCount++;
			continue;
		}

		double ratio;
		int ans;
		int ans_smm;
		if (smmdSwitch)
		{
			ratio = smmd.probSMMD(nearEdges[0], testData.x, testData.d) / smmd.probSMMD(nearEdges[1], testData.x, testData.d);
			ans = smmd.probSMMD(nearEdges[0], testData.x, testData.d) > smmd.probSMMD(nearEdges[1], testData.x, testData.d) ? nearEdges[0]->id : nearEdges[1]->id;
			ans_smm = smmd.probSMM(nearEdges[0], testData.x, testData.d) > smmd.probSMM(nearEdges[1], testData.x, testData.d) ? nearEdges[0]->id : nearEdges[1]->id;
		}
		else
		{
			ratio = smmd.probSMM(nearEdges[0], testData.x, testData.d) / smmd.probSMM(nearEdges[1], testData.x, testData.d);
			ans = smmd.probSMM(nearEdges[0], testData.x, testData.d) > smmd.probSMM(nearEdges[1], testData.x, testData.d) ? nearEdges[0]->id : nearEdges[1]->id;
		}	

		if (ratio < 1)
			ratio = 1 / ratio;
		

		//只留下高概率的
		if (ratio >=-1)
		{
			if (testData.rId == ans)
				correctCount++;
			if (testData.rId == ans_smm)
				smmCorrectCount++;
		}
		else
		{
			allCount--;
			probCount++;
			continue;
		}

		md.drawBigPoint(Color::Red, testData.x->lat, testData.x->lon);

		//base line
		int ans_b;
		if (SMMD::distM_signed(nearEdges[0], testData.x) > 0) // <0 北京 >0新加坡
			ans_b = nearEdges[0]->id;
		else
			ans_b = nearEdges[1]->id;
		if (testData.rId == ans_b)
			baselineCorrectCount++;
		else
			md.drawBigPoint(Color::Black, testData.x->lat, testData.x->lon);
		

		//if (testData.rId == smmd.doSMMD(testData.x, testData.d, pFunc))
		//	correctCount++;
	}
	printf("SMMD_acc = %lf\n", (double)correctCount / (double)allCount);
	if (smmdSwitch)
		printf("SMM_acc = %lf\n", (double)smmCorrectCount / (double)allCount);
	printf("base_acc = %lf\n", (double)baselineCorrectCount / (double)allCount);
	printf("base_correctCount = %d, allCount = %d\n", baselineCorrectCount, allCount);
	printf("SMMDcorrectCount = %d, allCount = %d\n", correctCount, allCount);
	printf("cross = %d, bias = %d, 2NN = %d, prob = %d, invalid = %d\n", crossCount, biasCount, twoNNNoCount, probCount, invalidCount);
}

void evalSMMD_2(vector<SampleType>& tests)
{
	double (SMMD::* pFunc)(Edge*, GeoPoint*, GeoPoint*); //一个类成员函数指针变量pFunc的定义

	bool smmdSwitch = false;
	if (smmdSwitch)
	{
		pFunc = &SMMD::probSMMD;
	}
	else
	{
		pFunc = &SMMD::probSMM;
		//构造SMM testData
		list <GeoPoint*> testSMMDataSet;
		TrajReader tReader(trainDataPath);
		tReader.readGeoPoints(testSMMDataSet, &area, 400000);
		tReader.close();
		tests.clear();
		for each (GeoPoint* pt in testSMMDataSet)
		{
			tests.push_back(SampleType(pt, NULL, pt->mmRoadId));
		}
	}
	cout << "test sample size = " << tests.size() << endl;


	SMMD smmd(roadNetwork);
	int baselineCorrectCount = 0;
	int correctCount = 0;
	int allCount = 0;
	int wrongCount_our = 0;
	int wrongCount_both = 0;
	int wrongCount_naive = 0;

	int ourWnaiveR = 0;
	int ourRnaiveW = 0;

	int crossCount = 0;
	int biasCount = 0;
	int probCount = 0;
	int twoNNNoCount = 0;
	int invalidCount = 0;
	int tooFarCount = 0;

	for (int i = 0; i < tests.size(); i++)
	{
		if (i % 1000 == 0)
			cout << i << endl;
		SampleType testData = tests[i];
		
		/**********************************************************/
		/*test code starts from here*/

		if (testData.rId == 48067)
		{
			//allCount--;
			continue;
			md.drawBigPoint(Color::Black, testData.x->lat, testData.x->lon);
		}
		/*test code ends*/
		/**********************************************************/
		if (roadNetwork.edges[testData.rId] == NULL)
		{

			continue;
		}

		if (/*roadNetwork.edges[testData.rId] == NULL || */roadNetwork.edges[testData.rId]->r_hat.size() == 0)// || roadNetwork.distM(testData.x->lat, testData.x->lon, roadNetwork.edges[testData.rId]) >= 50.0)
		{
			invalidCount++;
			continue;
		}

		if (roadNetwork.distM(testData.x->lat, testData.x->lon, roadNetwork.edges[testData.rId]) >= 50.0)
		{
			md.drawBigPoint(Color::Green, testData.x->lat, testData.x->lon);
			tooFarCount++;
			continue;
		}

		/*int slotId = roadNetwork.edges[testData.rId]->getSlotId(testData.x);
		if (abs(roadNetwork.edges[testData.rId]->thetas[slotId].mu) < 5)
		{
		allCount--;
		continue;
		}*/
		/*
		if (abs(roadNetwork.edges[testData.rId]->u) > 5)
		{
		allCount--;
		continue;
		}*/

		//滤去路口测试数据点
		vector<GeoPoint*> oneNN;
		roadNetwork.ptIndex.kNN_approx(testData.x, 1, 100.0, oneNN);
		if (GeoPoint::distM(oneNN[0], testData.x) < -50.0) //负数则不滤点
		{
			crossCount++;
			continue;
		}

		vector<Edge*> nearEdges;
		roadNetwork.getNearEdges(testData.x->lat, testData.x->lon, 2, nearEdges);
		if (nearEdges[1] == NULL || nearEdges[0] == NULL)
		{
			system("pause");
		}
		if (nearEdges[0]->r_hat.size() == 0 || nearEdges[1]->r_hat.size() == 0)
		{
			continue;
		}

		//最近两条之中都没的话排除 (75%->72%)
		if (nearEdges[0]->id != testData.rId && nearEdges[1]->id != testData.rId)
		{
			twoNNNoCount++;
			//continue;
		}

		//偏的不厉害的排除
		int slotId0 = nearEdges[0]->getSlotId(testData.x);
		int slotId1 = nearEdges[1]->getSlotId(testData.x);
		if (abs(roadNetwork.edges[testData.rId]->thetas[slotId0].mu - roadNetwork.edges[testData.rId]->thetas[slotId1].mu) < 0) //负数不过滤
		{
			biasCount++;
			continue;
		}

		//如果最近的就一条
		if (abs(roadNetwork.distM(testData.x->lat, testData.x->lon, nearEdges[1]) - roadNetwork.distM(testData.x->lat, testData.x->lon, nearEdges[0])) > eps)
		{
			cout << "最近的就一条" << endl;
			system("pause");
		}

		double ratio;
		int ans;

		if (smmdSwitch)
		{
			ratio = smmd.probSMMD(nearEdges[0], testData.x, testData.d) / smmd.probSMMD(nearEdges[1], testData.x, testData.d);
			//ans = smmd.probSMMD(nearEdges[0], testData.x, testData.d) > smmd.probSMMD(nearEdges[1], testData.x, testData.d) ? nearEdges[0]->id : nearEdges[1]->id;
			ans = smmd.doSMMD(testData.x, testData.d, pFunc);
		}
		else
		{
			ratio = smmd.probSMM(nearEdges[0], testData.x, testData.d) / smmd.probSMM(nearEdges[1], testData.x, testData.d);
			//ans = smmd.probSMM(nearEdges[0], testData.x, testData.d) > smmd.probSMM(nearEdges[1], testData.x, testData.d) ? nearEdges[0]->id : nearEdges[1]->id;
			ans = smmd.doSMMD(testData.x, testData.x, pFunc);
		}
		if (ratio < 1)
			ratio = 1 / ratio;

		//只留下高概率的
		if (ratio >= 3)
		{
			//base line
			int ans_b;
			if (SMMD::distM_signed(nearEdges[0], testData.x) > 0)
				ans_b = nearEdges[0]->id;
			else
				ans_b = nearEdges[1]->id;
			
			if (testData.rId == ans && testData.rId == ans_b)
				correctCount++;
			if (testData.rId != ans && testData.rId != ans_b)
				wrongCount_both++;
			if (testData.rId != ans)
				wrongCount_our++;
			if (testData.rId != ans_b)
				wrongCount_naive++;
			if (ans == ans_b)
				allCount++;
			if (testData.rId != ans && testData.rId == ans_b)
				ourWnaiveR++;
			if (testData.rId == ans && testData.rId != ans_b)
				ourRnaiveW++;
		}
		else
		{
			probCount++;
			continue;
		}
		md.drawBigPoint(Color::Red, testData.x->lat, testData.x->lon);

		//if (testData.rId == smmd.doSMMD(testData.x, testData.d, pFunc))
		//	correctCount++;
	}
	printf("wrong ourWnaiveR = %lf\n", (double)ourWnaiveR / (double)wrongCount_our);
	printf("wrong: %lf\n", (double)wrongCount_both / (double)wrongCount_our);
	printf("SMMD_acc = %lf\n", (double)correctCount / (double)allCount);
	printf("SMMDcorrectCount = %d, allCount = %d\n", correctCount, allCount);
	printf("allCount = %d, cross = %d, bias = %d, 2NN = %d, prob = %d, invalid = %d, tooFar = %d\n", allCount, crossCount, biasCount, twoNNNoCount, probCount, invalidCount, tooFarCount);
}

void loadSMMTestData(int trainCount, int testCount, vector<SampleType>& tests, list<GeoPoint*>& testSMMDataSet)
{
	list <GeoPoint*> dummyDataSet;
	TrajReader tReader(trainDataPath);
	tReader.readGeoPoints(dummyDataSet, &area, trainCount);
	tReader.readGeoPoints(testSMMDataSet, &area, testCount);
	tReader.close();
	tests.clear();
	for each (GeoPoint* pt in testSMMDataSet)
	{
		//注意到有时候在边界的点会匹配到一条不在area中的道路上，这样的sample要丢掉
		//但是testSMMDataSet中没有丢掉，如果需要直接用testSMMDataSet里面的数据做测试，则需要手动丢弃
		if (roadNetwork.edges[pt->mmRoadId] == NULL)
		{
			continue;
		}
		tests.push_back(SampleType(pt, NULL, pt->mmRoadId));
	}
}

void evalSMMD_new(vector<SampleType>& tests)
{
	double (SMMD::* pFunc)(Edge*, GeoPoint*, GeoPoint*); //一个类成员函数指针变量pFunc的定义

	bool smmdSwitch = true;
	if (smmdSwitch)
	{
		pFunc = &SMMD::probSMMD;
	}
	else
	{
		pFunc = &SMMD::probSMM;
		//构造SMM testData
		
	}
	cout << "test sample size = " << tests.size() << endl;
	addNoise(noiseScaleM, tests);
	//draw testData
	for each (SampleType testData in testDataSet)
	{
		md.drawPoint(Color::Red, testData.x->lat, testData.x->lon);
	}
	

	SMMD smmd(roadNetwork);
	int baselineCorrectCount = 0;
	int correctCount = 0;
	int allCount = 0;
	int wrongCount_our = 0;
	int wrongCount_both = 0;
	int wrongCount_naive = 0;

	int ourWnaiveR = 0;
	int ourRnaiveW = 0;

	int crossCount = 0;
	int biasCount = 0;
	int probCount = 0;
	int twoNNNoCount = 0;
	int invalidCount = 0;
	int tooFarCount = 0;

	for (int i = 0; i < tests.size(); i++)
	{
		if (i % 10000 == 0)
			cout << "testing " << i << endl;
		SampleType testData = tests[i];

		if (roadNetwork.edges[testData.rId] == NULL)
		{
			continue;
		}

		if (/*roadNetwork.edges[testData.rId] == NULL || */roadNetwork.edges[testData.rId]->r_hat.size() == 0)// || roadNetwork.distM(testData.x->lat, testData.x->lon, roadNetwork.edges[testData.rId]) >= 50.0)
		{
			invalidCount++;
			continue;
		}

		/*if (roadNetwork.distM(testData.x->lat, testData.x->lon, roadNetwork.edges[testData.rId]) >= 50.0) //匹配距离太远
		{
			md.drawBigPoint(Color::Green, testData.x->lat, testData.x->lon);
			tooFarCount++;
			continue;
		}*/

		/*int slotId = roadNetwork.edges[testData.rId]->getSlotId(testData.x);
		if (abs(roadNetwork.edges[testData.rId]->thetas[slotId].mu) < 5)
		{
		allCount--;
		continue;
		}*/
		/*
		if (abs(roadNetwork.edges[testData.rId]->u) > 5)
		{
		allCount--;
		continue;
		}*/

		//滤去路口测试数据点
		vector<GeoPoint*> oneNN;
		roadNetwork.ptIndex.kNN_exact(testData.x, 1,oneNN);
		if (GeoPoint::distM(oneNN[0], testData.x) < -50.0) //负数则不滤点
		{
			crossCount++;
			continue;
		}

		vector<Edge*> nearEdges;
		roadNetwork.getNearEdges(testData.x->lat, testData.x->lon, 2, nearEdges);
		if (nearEdges[1] == NULL || nearEdges[0] == NULL)
		{
			system("pause");
		}
		if (nearEdges[0]->r_hat.size() == 0 || nearEdges[1]->r_hat.size() == 0)
		{
			continue;
		}

		//最近两条之中都没的话排除 (75%->72%)
		if (nearEdges[0]->id != testData.rId && nearEdges[1]->id != testData.rId)
		{
			twoNNNoCount++;
			//continue;
		}

		//偏的不厉害的排除
		/*int slotId0 = nearEdges[0]->getSlotId(testData.x);
		int slotId1 = nearEdges[1]->getSlotId(testData.x);
		if (abs(roadNetwork.edges[testData.rId]->thetas[slotId0].mu - roadNetwork.edges[testData.rId]->thetas[slotId1].mu) < 0) //负数不过滤
		{
			biasCount++;
			continue;
		}*/

		//如果最近的就一条
		if (abs(roadNetwork.distM(testData.x->lat, testData.x->lon, nearEdges[1]) - roadNetwork.distM(testData.x->lat, testData.x->lon, nearEdges[0])) > eps)
		{
			cout << "最近的就一条" << endl;
			system("pause");
		}

		double ratio;
		int ans;

		if (smmdSwitch)
		{
			//printf("nearEdges[0].id = %d, nearEdges[1].id = %d\n", nearEdges[0]->id, nearEdges[1]->id);
			ratio = smmd.probSMMD(nearEdges[0], testData.x, testData.d) / smmd.probSMMD(nearEdges[1], testData.x, testData.d);
			//ans = smmd.probSMMD(nearEdges[0], testData.x, testData.d) > smmd.probSMMD(nearEdges[1], testData.x, testData.d) ? nearEdges[0]->id : nearEdges[1]->id;
			ans = smmd.doSMMD(testData.x, testData.d, pFunc);
		}
		else
		{
			ratio = smmd.probSMM(nearEdges[0], testData.x, testData.d) / smmd.probSMM(nearEdges[1], testData.x, testData.d);
			//ans = smmd.probSMM(nearEdges[0], testData.x, testData.d) > smmd.probSMM(nearEdges[1], testData.x, testData.d) ? nearEdges[0]->id : nearEdges[1]->id;
			ans = smmd.doSMMD(testData.x, testData.x, pFunc);
		}
		if (ratio < 1)
			ratio = 1 / ratio;


		/**********************************************************/
		/*test code starts from here*/
		if (ans == testData.rId)
		{
			//correctCount++;
		}
		//allCount++;
		/*test code ends*/
		/**********************************************************/

		//只留下高概率的
		if (ratio >= 1)//(ratio >= 1)
		{
			//base line
			int ans_b;
			if (SMMD::distM_signed(nearEdges[0], testData.x) > 0)
				ans_b = nearEdges[0]->id;
			else
				ans_b = nearEdges[1]->id;

			if (testData.rId == ans && testData.rId == ans_b)
				correctCount++;
			if (testData.rId != ans && testData.rId != ans_b)
				wrongCount_both++;
			if (testData.rId != ans)
				wrongCount_our++;
			if (testData.rId != ans_b)
				wrongCount_naive++;
			if (ans == ans_b)
				allCount++;
			if (testData.rId != ans && testData.rId == ans_b)
				ourWnaiveR++;
			if (testData.rId == ans && testData.rId != ans_b)
				ourRnaiveW++;
		}
		else
		{
			probCount++;
			continue;
		}
		md.drawBigPoint(Color::Red, testData.x->lat, testData.x->lon);



		//if (testData.rId == smmd.doSMMD(testData.x, testData.d, pFunc))
		//	correctCount++;
	}
	printf("wrong ourWnaiveR = %lf\n", (double)ourWnaiveR / (double)wrongCount_our);
	printf("wrong both: %lf\n", (double)wrongCount_both / (double)wrongCount_our);
	printf("SMMDcorrectCount = %d, allCount = %d\n", correctCount, allCount);
	printf("allCount = %d, cross = %d, bias = %d, 2NN = %d, prob = %d, invalid = %d, tooFar = %d\n", allCount, crossCount, biasCount, twoNNNoCount, probCount, invalidCount, tooFarCount);
	printf("SMMD_acc = %lf\n", (double)correctCount / (double)allCount);
}

void evalSMM()
{
	double (SMMD::* pFunc)(Edge*, GeoPoint*, GeoPoint*); //一个类成员函数指针变量pFunc的定义
	pFunc = &SMMD::probSMM;

	SMMD smmd(roadNetwork);
	int correctCount = 0;

	//构造SMM testData
	list <GeoPoint*> testSMMDataSet;
	TrajReader tReader(trainDataPath);
	//TrajReader tReader(testDataPath);
	tReader.readGeoPoints(testSMMDataSet, &area, 200000);
	tReader.close();
	int allCount = testSMMDataSet.size();

	for each (GeoPoint* testData in testSMMDataSet)
	{
		
		/**********************************************************/
		/*test code starts from here*/
		if (testData->mmRoadId == 48067)
		{

			allCount--;
			continue;
			md.drawBigPoint(Color::Black, testData->lat, testData->lon);
		}
		/*test code ends*/
		/**********************************************************/

		if (roadNetwork.edges[testData->mmRoadId] == NULL || roadNetwork.edges[testData->mmRoadId]->r_hat.size() == 0 || roadNetwork.distM(testData->lat, testData->lon, roadNetwork.edges[testData->mmRoadId]) >= 50.0)
		{
			allCount--;
			continue;
		}
		/*int slotId = roadNetwork.edges[testData->mmRoadId]->getSlotId(testData);
		if (abs(roadNetwork.edges[testData->mmRoadId]->thetas[slotId].mu) < 5)
		{
			allCount--;
			continue;
		}*/
		//过滤路口
		/*vector<GeoPoint*> oneNN;
		roadNetwork.ptIndex.kNN_approx(testData, 1, 100.0, oneNN);
		if (GeoPoint::distM(oneNN[0], testData) < 50.0)
		{
			allCount--;
			continue;
		}*/

		md.drawBigPoint(Color::Red, testData->lat, testData->lon);
		
		
		
		if (roadNetwork.edges[testData->mmRoadId] == NULL)
			allCount--;
		if (testData->mmRoadId == smmd.doSMMD_old(testData, testData, pFunc))
			correctCount++;
	}
	printf("acc = %lf\n", (double)correctCount / (double)allCount);
}

void baseline2_SMMD()
{
	//roadNetwork.loadPolylines(fakeCenterlineFilePath);
	/*for (int i = 0; i < roadNetwork.edges.size(); i++)
	{
		if (roadNetwork.edges[i] == NULL)
			continue;
		vector<GeoPoint*> centerLine;
		for each(GeoPoint* pt in *roadNetwork.edges[i]->figure)
		{
			roadNetwork.edges[i]->r_hat.push_back(pt);
		}
	}*/
	
	SMMD smmd(roadNetwork);
	double correctCount = 0;
	int allCount = testDataSet.size();
	for (int i = 0; i < testDataSet.size(); i++)
	{
		int ans = 0;
		if (i % 100 == 0)
			cout << i << endl;
		SampleType testData = testDataSet[i];
		if (testData.rId == 48067)
		{

			allCount--;
			continue;
			md.drawBigPoint(Color::Black, testData.x->lat, testData.x->lon);
		}

		if (roadNetwork.edges[testData.rId] == NULL || roadNetwork.edges[testData.rId]->r_hat.size() == 0 || roadNetwork.distM(testData.x->lat, testData.x->lon, roadNetwork.edges[testData.rId]) >= 50.0)
		{
			allCount--;
			continue;
		}
		md.drawBigPoint(Color::Black, testData.x->lat, testData.x->lon);

		vector<GeoPoint*> oneNN;
		roadNetwork.ptIndex.kNN_approx(testData.x, 1, 100.0, oneNN);
		if (GeoPoint::distM(oneNN[0], testData.x) < 50.0)
		{
			allCount--;
			continue;
		}
		int slotId = roadNetwork.edges[testData.rId]->getSlotId(testData.x);
		if (roadNetwork.edges[testData.rId]->thetas.size() <= slotId)
		{
			printf("slotId = %d, thetas_size = %d, rId = %d\n", slotId, roadNetwork.edges[testData.rId]->thetas.size(), testData.rId);
			system("pause");
			continue;
		}
		if (abs(roadNetwork.edges[testData.rId]->thetas[slotId].mu) < 5)
		{
			allCount--;
			continue;
		}
		/*if (abs(roadNetwork.edges[testData.rId]->u) > 5)
		{
			allCount--;
			continue;
		}*/
		md.drawBigPoint(Color::Red, testData.x->lat, testData.x->lon);

		//找最近的
		vector<Edge*> nearEdges;
		roadNetwork.getNearEdges(testData.x->lat, testData.x->lon, 2, nearEdges);
		if (nearEdges[1] == NULL || nearEdges[0] == NULL)
		{
			system("pause");
		}
		//如果最近的就一条
		if (abs(roadNetwork.distM(testData.x->lat, testData.x->lon, nearEdges[1]) - roadNetwork.distM(testData.x->lat, testData.x->lon, nearEdges[0])) > eps)
		{
			ans = nearEdges[0]->id;
			cout << "1" << endl;
			system("pause");
		}
		else
		{
			if (SMMD::distM_signed(nearEdges[0], testData.x) > 0)
				ans = nearEdges[0]->id;
			else
				ans = nearEdges[1]->id;
			/*if (testData.rId == nearEdges[0]->id || testData.rId == nearEdges[1]->id)
			{
			correctCount += 0.5;
			ans = -999;
			}*/
		}
		//check
		if (testData.rId == ans)
			correctCount++;
	}
	printf("acc = %lf\n", (double)correctCount / (double)allCount);
}

void baseline2_SMM()
{
	//构造SMM testData
	list <GeoPoint*> testSMMDataSet;
	TrajReader tReader(trainDataPath);
	tReader.readGeoPoints(testSMMDataSet, &area, 200000);
	tReader.close();

	//roadNetwork.loadPolylines(fakeCenterlineFilePath);
	for (int i = 0; i < roadNetwork.edges.size(); i++)
	{
		if (roadNetwork.edges[i] == NULL)
			continue;
		vector<GeoPoint*> centerLine;
		for each(GeoPoint* pt in *roadNetwork.edges[i]->figure)
		{
			roadNetwork.edges[i]->r_hat.push_back(pt);
		}
	}
	SMMD smmd(roadNetwork);
	double correctCount = 0;
	int allCount = testSMMDataSet.size();
	for each (GeoPoint* testData in testSMMDataSet)
	{
		if (roadNetwork.edges[testData->mmRoadId] == NULL)
			allCount--;
		int ans = 0;
		//找最近的
		vector<Edge*> nearEdges;
		roadNetwork.getNearEdges(testData->lat, testData->lon, 2, nearEdges);
		if (nearEdges[1] == NULL || nearEdges[0] == NULL)
		{
			system("pause");
		}
		//如果最近的就一条
		if (abs(roadNetwork.distM(testData->lat, testData->lon, nearEdges[1]) - roadNetwork.distM(testData->lat, testData->lon, nearEdges[0])) > eps)
		{
			ans = nearEdges[0]->id;
		}
		else
		{
			if (SMMD::distM_signed(nearEdges[0], testData) > 0)
				ans = nearEdges[0]->id;
			else
				ans = nearEdges[1]->id;
			/*if (testData.rId == nearEdges[0]->id || testData.rId == nearEdges[1]->id)
			{
			correctCount += 0.5;
			ans = -999;
			}*/
		}
		//check
		if (testData->mmRoadId == ans)
			correctCount++;
	}
	printf("acc = %lf\n", (double)correctCount / (double)allCount);
}

///////////////////////////////////////////////////////////////////////////////
void genCenterline()
{
	trainer.loadTrainData(trainDataPath);
	trainer.genFakeCenterline("fakeCenterline2.txt");
	//draw	
	for each (GeoPoint* pt in trainer.trainDataSet)
	{
		//md.drawBigPoint(Color::Green, pt->lat, pt->lon);
	}

	//trainer.drawCenterline("fakeCenterline3.txt", md);
}

void matchOneTraj(ofstream& fout, Traj* traj, MapMatcher& mm)
{
	if (traj == NULL)
		return;
	
	list<Edge*> result;
	mm.MapMatching(*traj, result, 50.0); //MapMatching
	//对每一个点
	Traj::iterator ptIter = traj->begin();
	list<Edge*>::iterator edgeIter = result.begin();
	while (ptIter != traj->end())
	{
		if ((*edgeIter) != NULL) //匹配成功
		{
			(*ptIter)->mmRoadId = (*edgeIter)->id;
			fout << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << (*edgeIter)->id << endl;
		}
		else //匹配失败
		{
			(*ptIter)->mmRoadId = -1;
			fout << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << -1 << endl;
		}
		ptIter++;
		edgeIter++;
	}
	fout << -1 << endl;
}

void mapmatching()
{
	ofstream fout("newMMTrajs_50m.txt");
	ifstream trajIfs(trainDataFolder + "newMMTrajs.txt");
	MapMatcher mm(&roadNetwork);
	fout << fixed << showpoint << setprecision(8);
	bool isStart = true;
	int time, mmRoadId;
	double lat, lon;
	Traj* tmpTraj = NULL;
	int currentCount = 0;
	//int count ;
	while (trajIfs)
	{
		int time;
		trajIfs >> time;
		if (trajIfs.fail())
		{
			break;
		}
		if (time == -1)
		{
			isStart = true;
			if (tmpTraj != NULL && tmpTraj->size() > 1)
			{
				matchOneTraj(fout, tmpTraj, mm); //map matching and output
				//清除内存
				for each (GeoPoint* pt in *tmpTraj)
				{
					delete pt;
				}
				delete tmpTraj;
				currentCount++;
				if (currentCount % 100000 == 0)
				{
					cout << currentCount << " finished" << endl;
				}
			}
			continue;
		}
		else
		{
			trajIfs >> lat >> lon >> mmRoadId;
			GeoPoint* pt = new GeoPoint(lat, lon, time, mmRoadId);
			if (isStart)
			{
				tmpTraj = new Traj();
				tmpTraj->push_back(pt);
				isStart = false;
			}
			else
			{
				tmpTraj->push_back(pt);
			}
		}
	}
	trajIfs.close();
	fout.close();	
}

void genSMMDPrior(vector<SampleType>& testDataSet)
{
	//////////////////////////////////////////////////////////////////////////
	///输出SMMD的prior
	//////////////////////////////////////////////////////////////////////////
	ofstream ofs("SMMD_prior2.txt");
	int edgesSize = roadNetwork.edges.size();
	int* prior = new int[edgesSize];	
	for (int i = 0; i < edgesSize; i++)
		prior[i] = 0;
	for each (SampleType testData in testDataSet)
	{
		prior[testData.rId]++;
	}
	for (int i = 0; i < edgesSize; i++)
	{
		ofs << prior[i] << endl;
	}
	ofs.close();
}

void outputJSON(string trajFilePath)
{
	string path = "json_area3.js";
	ofstream ofs(path);
	if (!ofs)
	{
		cout << "open " << path << " error!" << endl;
		system("pause");
	}
	ofs << fixed << showpoint << setprecision(8);
	ofs << "data = [" << endl;
	TrajReader tr(trajFilePath);
	list<GeoPoint*> ptsList;
	tr.readGeoPoints(ptsList, &area, 300000);
	tr.close();
	vector<GeoPoint*> outputPts;
	for each (GeoPoint* pt in ptsList)
	{
		outputPts.push_back(pt);
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

void getBeijingArea()
{
	string filePath = "D:\\trajectory\\singapore_data\\experiments\\SMMD\\trainData\\Beijing\\all.txt";
	TrajReader tr(filePath);
	list<GeoPoint*> pts;
	Area beijinArea;
	tr.readGeoPoints(pts);
	tr.close();
	beijinArea.getArea(pts);
	beijinArea.print();
	roadNetwork.setArea(&beijinArea);
	roadNetwork.open("D:\\trajectory\\beijing_data\\beijing_map\\new\\");
	md.setArea(&beijinArea);
	md.setResolution(15000);
	md.newBitmap();
	md.lockBits();
	
	for each (GeoPoint* pt in pts)
	{
		md.drawBigPoint(Color::Red, pt->lat, pt->lon);
	}
	roadNetwork.drawMap(Color::Blue, md);
	md.unlockBits();
	md.saveBitmap("beijing.png");
}
//////////////////////////////////////////////////////////////////////////
///training & evaluation
//////////////////////////////////////////////////////////////////////////

void simpleBaseline(vector<SampleType>& testDataSet, vector<int>& prediction)
{
	//////////////////////////////////////////////////////////////////////////
	///最简单的baseline
	///如果最近的只有一条返回最近的，否则返回双向路的随机一条
	//////////////////////////////////////////////////////////////////////////
	prediction.clear();
	for each (SampleType testData in testDataSet)
	{
		//找2NN
		vector<Edge*> nearEdges;
		roadNetwork.getNearEdges(testData.x->lat, testData.x->lon, 2, nearEdges);
		if (nearEdges[1] == NULL || nearEdges[0] == NULL)
		{
			system("pause");
		}
		//如果最近的就一条
		if (abs(roadNetwork.distM(testData.x->lat, testData.x->lon, nearEdges[1]) - roadNetwork.distM(testData.x->lat, testData.x->lon, nearEdges[0])) > eps)
		{
			prediction.push_back(nearEdges[0]->id);
		}
		else
		{
			if ( (double)rand()/ (double)RAND_MAX > 0.5) // 随机一条
				prediction.push_back(nearEdges[0]->id);
			else
				prediction.push_back(nearEdges[1]->id);
		}
	}

}

void count_no2NN(vector<SampleType>& testDataSet)
{
	//////////////////////////////////////////////////////////////////////////
	///统计有多少比例testsample答案不是2NN内
	//////////////////////////////////////////////////////////////////////////
	int correctCount = 0;
	int no2NN = 0;
	for each(SampleType testData in testDataSet)
	{
		vector<Edge*> nearEdges;
		roadNetwork.getNearEdges(testData.x->lat, testData.x->lon, 2, nearEdges);
		if (testData.rId != nearEdges[0]->id && testData.rId != nearEdges[1]->id)
			no2NN++;
	}
	printf("not2NN ratio = %lf\n", (double)no2NN / (double)testSMMDataSet.size());
}

void baseline(vector<SampleType>& testDataSet, vector<int>& prediction)
{
	//////////////////////////////////////////////////////////////////////////
	///baseline
	///如果最近的只有一条返回最近的，否则返回落在道路哪侧的那条路
	//////////////////////////////////////////////////////////////////////////
	prediction.clear();
	for each (SampleType testData in testDataSet)
	{
		//找2NN
		vector<Edge*> nearEdges;
		roadNetwork.getNearEdges(testData.x->lat, testData.x->lon, 2, nearEdges);
		if (nearEdges[1] == NULL || nearEdges[0] == NULL)
		{
			system("pause");
		}
		//如果最近的就一条
		if (abs(roadNetwork.distM(testData.x->lat, testData.x->lon, nearEdges[1]) - roadNetwork.distM(testData.x->lat, testData.x->lon, nearEdges[0])) > eps)
		{
			prediction.push_back(nearEdges[0]->id);
		}
		else
		{
			if (nearEdges[0]->r_fig.size() == 0 && nearEdges[1]->r_fig.size() == 0)
			{
				prediction.push_back(0);
			}
			else if (nearEdges[0]->r_fig.size() == 0 && nearEdges[1]->r_fig.size() != 0)
			{
				prediction.push_back(nearEdges[1]->id);
			}
			else if (nearEdges[1]->r_fig.size() == 0 && nearEdges[0]->r_fig.size() != 0)
			{
				prediction.push_back(nearEdges[0]->id);
			}
			else
			{
				if (SMMD::distM_signed(nearEdges[0]->r_fig, testData.x) > 0) // <0 北京 >0新加坡
					prediction.push_back(nearEdges[0]->id);
				else
					prediction.push_back(nearEdges[1]->id);
			}			
		}
	}
}

void kNN(int k, vector<SampleType>& testDataSet, vector<int>& prediction, int trainCount)
{
	prediction.clear();
	KNNClassifier kNNClassifier(&roadNetwork, &area);
	kNNClassifier.loadTrainData(trainDataPath, trainCount);
	for each (SampleType testData in testDataSet)
	{
		prediction.push_back(kNNClassifier.classify(testData.x, k));
	}
}

void SMM(vector<SampleType>& testDataSet, vector<int>& prediction)
{
	prediction.clear();
	SMMD smm(roadNetwork);
	trainSMM(25.0, 500000);
	double (SMMD::* pFunc)(Edge*, GeoPoint*, GeoPoint*); //一个类成员函数指针变量pFunc的定义
	pFunc = &SMMD::probSMM;
	for each (SampleType testData in testDataSet)
	{
		prediction.push_back(smm.doSMMD(testData.x, testData.x, pFunc));
	}
}

void Smmd(vector<SampleType>& testDataSet, vector<int>& prediction)
{
	prediction.clear();
	SMMD smm(roadNetwork);
	trainSMM(25.0, 20000);
	double (SMMD::* pFunc)(Edge*, GeoPoint*, GeoPoint*); //一个类成员函数指针变量pFunc的定义
	pFunc = &SMMD::probSMMD;
	for each (SampleType testData in testDataSet)
	{
		prediction.push_back(smm.doSMMD(testData.x, testData.x, pFunc));
	}
}

double evaluate(vector<SampleType>& testDataSet, vector<int>& prediction)
{ 
	int correct = 0, all = testDataSet.size();
	for (int i = 0; i < testDataSet.size(); i++)
	{
		if (testDataSet[i].rId == prediction[i])
			correct++;
	}
	return (double)correct / (double)all;
}

void shiftMap(double deltaXM, double deltaYM)
{
	//////////////////////////////////////////////////////////////////////////
	///对edge->figure进行平移
	//////////////////////////////////////////////////////////////////////////
	
	for (int i = 0; i < roadNetwork.edges.size(); i++)
	{
		Edge* edge = roadNetwork.edges[i];
		if (edge == NULL)
			continue;
		for each (GeoPoint* pt in *edge->figure)
		{
			pt->lat += deltaYM / GeoPoint::geoScale;
			pt->lon += deltaXM / GeoPoint::geoScale;
		}
	}
	roadNetwork.createGridIndex();
	cout << "shift map finished" << endl;
}

void trembleMap(double rangeM)
{
	//////////////////////////////////////////////////////////////////////////
	///对figure进行扰动
	//////////////////////////////////////////////////////////////////////////

	for (int i = 0; i < roadNetwork.edges.size(); i++)
	{
		Edge* edge = roadNetwork.edges[i];
		if (edge == NULL)
			continue;
		Figure::iterator iter = edge->figure->begin();
		iter++; //顶点不要动
		//double deltaX = (((double)rand() / (double)RAND_MAX) * 2 - 1) * rangeM;
		//double deltaY = (((double)rand() / (double)RAND_MAX) * 2 - 1) * rangeM;
		for (; iter != edge->figure->end(); iter++)
		{
			if ((*iter) == edge->figure->back())
			continue;
			(*iter)->lat += (((double)rand() / (double)RAND_MAX) * 2 - 1) * rangeM / GeoPoint::geoScale;
			(*iter)->lon += (((double)rand() / (double)RAND_MAX) * 2 - 1) * rangeM / GeoPoint::geoScale;
		}
		/*{
			if ((*iter) == edge->figure->back())
				continue;
			(*iter)->lat += deltaY / GeoPoint::geoScale;
			(*iter)->lon += deltaX / GeoPoint::geoScale;
		}*/
	}
	roadNetwork.createGridIndex();
	cout << "shift map finished" << endl;
}

void check_r_hat()
{
	//check r_hat
	for (int i = 0; i < roadNetwork.edges.size(); i++)
	{
		if (roadNetwork.edges[i] == NULL || roadNetwork.edges[i]->r_hat.size() == 0)
			continue;
		int index = 0;
		Figure::iterator iter = roadNetwork.edges[i]->figure->begin();
		if (roadNetwork.edges[i]->figure->size() != roadNetwork.edges[i]->r_hat.size()) //size check
		{
			cout << "check 1: size error!" << endl;
			printf("edgeId = %d, figSize = %d, r_hat size = %d\n", i, roadNetwork.edges[i]->figure->size(), roadNetwork.edges[i]->r_hat.size());
			system("pause");
		}
		if (roadNetwork.edges[i]->r_fig.size() != roadNetwork.edges[i]->r_hat.size())
		{
			printf("r_fig size = %d, r_hat size = %d\n", roadNetwork.edges[i]->r_fig.size(), roadNetwork.edges[i]->r_hat.size());
			system("pause");
		}
		
		
		for (; iter != roadNetwork.edges[i]->figure->end(); iter++)
		{
			GeoPoint* pt = *iter;
			if (abs(pt->lat - roadNetwork.edges[i]->r_hat[index]->lat) > eps || abs(pt->lon - roadNetwork.edges[i]->r_hat[index]->lon) > eps)
			{
				//printf("something wrong with r_hat\n");
				cout << "figure ";
				pt->print();
				cout << "r_hat ";
				roadNetwork.edges[i]->r_hat[index]->print();
				system("pause");
			}
			index++;
		}
	}
	cout << "check_r_hat clear" << endl;
}

vector<int> mapper_OldToNew;
vector<int> mapper_NewToOld;
void reMap(list<GeoPoint*>& trainDataSet, list<GeoPoint*>& testDataSet)
{
	//////////////////////////////////////////////////////////////////////////
	///对label进行重新映射，映射成连续标号，从0开始标
	//////////////////////////////////////////////////////////////////////////
	
	mapper_OldToNew.clear();
	mapper_NewToOld.clear();
	vector<int> allOldLabels;
	for each (GeoPoint* pt in trainDataSet)
	{
		allOldLabels.push_back(pt->mmRoadId);
	}
	for each (GeoPoint* pt in testDataSet)
	{
		allOldLabels.push_back(pt->mmRoadId);
	}
	sort(allOldLabels.begin(), allOldLabels.end());
	
	for (int i = 0; i < roadNetwork.edges.size(); i++)
	{
		mapper_OldToNew.push_back(-1);
	}
	int preOldLabel = -1;
	int currentNewLabel = 0;
	for (int i = 0; i < allOldLabels.size(); i++)
	{
		if (roadNetwork.edges[allOldLabels[i]] == NULL) //注意到有时候在边界的点会匹配到一条不在area中的道路上，这样的sample要丢掉
			continue;		
		if (allOldLabels[i] != preOldLabel)
		{
			mapper_OldToNew[allOldLabels[i]] = currentNewLabel;
			mapper_NewToOld.push_back(allOldLabels[i]);
			currentNewLabel++;
		}
		preOldLabel = allOldLabels[i];
	}
	cout << "共有" << currentNewLabel << "个新label"<< endl;
	cout << "mapper_newtoold.size = " << mapper_NewToOld.size() << endl;
	system("pause");	
}

void genCSV(list<GeoPoint*>& dataset, string featureFileName, string labelFileName)
{
	//////////////////////////////////////////////////////////////////////////
	///输出成csv文件供python调用
	///feature文件行数对应feature维度，列数对应sample个数
	///label文件就一列
	//////////////////////////////////////////////////////////////////////////
	ofstream featureOFS(featureFileName);
	ofstream labelOFS(labelFileName);
	featureOFS << fixed << showpoint << setprecision(8);
	labelOFS << fixed << showpoint << setprecision(8);
	
	list<GeoPoint*>::iterator iter = dataset.begin();
	for (; iter != dataset.end(); iter++)
	{
		if (roadNetwork.edges[(*iter)->mmRoadId] == NULL)
			continue;
		if ((*iter) == dataset.back())
			featureOFS << (*iter)->lat << endl;
		else
			featureOFS << (*iter)->lat << ",";
	}
	iter = dataset.begin();
	for (; iter != dataset.end(); iter++)
	{
		if (roadNetwork.edges[(*iter)->mmRoadId] == NULL)
			continue;
		if ((*iter) == dataset.back())
			featureOFS << (*iter)->lon << endl;
		else
			featureOFS << (*iter)->lon << ",";
	}

	//这个for输出测试样本到每条候选路的dist
	//将这个for注释掉后则只输出二维坐标feature
	/*for (int i = 0; i < mapper_NewToOld.size(); i++)
	{
		iter = dataset.begin();
		for (; iter != dataset.end(); iter++)
		{
			if (roadNetwork.edges[(*iter)->mmRoadId] == NULL)
				continue;
			double dist = roadNetwork.distM((*iter)->lat, (*iter)->lon, roadNetwork.edges[mapper_NewToOld[i]]);
			if ((*iter) == dataset.back())
				featureOFS << dist << endl;
			else
				featureOFS << dist << ",";
		}
	}*/
	
	featureOFS.close();
	
	iter = dataset.begin();
	for (; iter != dataset.end(); iter++)
	{
		if (roadNetwork.edges[(*iter)->mmRoadId] == NULL)
			continue;
		labelOFS << mapper_OldToNew[(*iter)->mmRoadId]<<endl;
	}
	labelOFS.close();
	cout << "输出csv文件成功" << endl;
}

void genCSV_nomap(list<GeoPoint*>& dataset, string featureFileName, string labelFileName)
{
	//////////////////////////////////////////////////////////////////////////
	///输出成csv文件供python调用
	///feature文件行数对应feature维度，列数对应sample个数
	///label文件就一列
	//////////////////////////////////////////////////////////////////////////
	ofstream featureOFS(featureFileName);
	ofstream labelOFS(labelFileName);
	featureOFS << fixed << showpoint << setprecision(8);
	labelOFS << fixed << showpoint << setprecision(8);

	list<GeoPoint*>::iterator iter = dataset.begin();
	for (; iter != dataset.end(); iter++)
	{
		if (roadNetwork.edges[(*iter)->mmRoadId] == NULL)
			continue;
		if ((*iter) == dataset.back())
			featureOFS << (*iter)->lat << endl;
		else
			featureOFS << (*iter)->lat << ",";
	}
	iter = dataset.begin();
	for (; iter != dataset.end(); iter++)
	{
		if (roadNetwork.edges[(*iter)->mmRoadId] == NULL)
			continue;
		if ((*iter) == dataset.back())
			featureOFS << (*iter)->lon << endl;
		else
			featureOFS << (*iter)->lon << ",";
	}

	//这个for输出测试样本到每条候选路的dist
	//将这个for注释掉后则只输出二维坐标feature
	/*for (int i = 0; i < mapper_NewToOld.size(); i++)
	{
	iter = dataset.begin();
	for (; iter != dataset.end(); iter++)
	{
	if (roadNetwork.edges[(*iter)->mmRoadId] == NULL)
	continue;
	double dist = roadNetwork.distM((*iter)->lat, (*iter)->lon, roadNetwork.edges[mapper_NewToOld[i]]);
	if ((*iter) == dataset.back())
	featureOFS << dist << endl;
	else
	featureOFS << dist << ",";
	}
	}*/

	featureOFS.close();

	iter = dataset.begin();
	for (; iter != dataset.end(); iter++)
	{
		if (roadNetwork.edges[(*iter)->mmRoadId] == NULL)
			continue;
		labelOFS << (*iter)->mmRoadId << endl;
	}
	labelOFS.close();
	cout << "输出csv文件成功" << endl;
}

void statPrjDist(int roadId, Map& roadNetwork, list<GeoPoint*>& pts, int intervalCount, string fileName)
{
	//////////////////////////////////////////////////////////////////////////
	///default: intervalCount = 40
	//////////////////////////////////////////////////////////////////////////
	double intervalM = 100.0 / intervalCount;
	vector<GeoPoint*> shape;
	for each (GeoPoint* pt in *roadNetwork.edges[roadId]->figure)
	{
		shape.push_back(pt);
	}
	vector<int> result;
	for (int i = 0; i < intervalCount; i++)
	{
		result.push_back(0);
	}
	for each (GeoPoint* pt in pts)
	{
		double dist = SMMD::distM_signed(shape, pt);
		int index = (dist + 50.0) / intervalM;
		if (index < 0) index = 0;
		if (index >= intervalCount) index = intervalCount - 1;
		result[index]++;
	}
	ofstream ofs(fileName);
	for (int i = 0; i < intervalCount; i++)
	{
		ofs << result[i] << endl;
	}
	ofs.close();
}


void matchForOneTraj(ofstream& oldOFS, ofstream& newOFS, Traj* traj, MapMatcher& mm)
{
	//////////////////////////////////////////////////////////////////////////
	///为traj进行地图匹配
	///传统按点匹配结果输出到oldOFS
	///新的按路段匹配结果输出到newOFS
	///1)中间有点匹配失败则用前后最短路代替
	///2)如果整条轨迹匹配失败则跳过
	///3)如果只有一个点匹配成功其余都失败则跳过
	///4)全局变量currentId控制轨迹id, oldOFS newOFS输出的轨迹id保持一致
	//////////////////////////////////////////////////////////////////////////
	
	
	
}

void mapmatching(list<Traj*>& trajs, string fileName, double maxDist = 50.0)
{
	ofstream ofs(fileName);
	ofs << fixed << showpoint << setprecision(8);
	int count = 0;
	for each (Traj* traj in trajs)
	{
		if (traj == NULL)
			continue;
		if (traj->size() <= 2)
			continue;
		if (count % 10000 == 0)
			printf("matching %d trajs\n", count);
		list<Edge*> result;
		mm.MapMatching(*traj, result, 50.0); //MapMatching
		//////////////////////////////////////////////
		//输出
		list<Edge*>::iterator edgeIter = result.begin();
		//对每一个点
		Traj::iterator ptIter = traj->begin();
		edgeIter = result.begin();
		while (ptIter != traj->end())
		{
			if ((*edgeIter) != NULL) //匹配成功
			{
				//(*ptIter)->mmRoadId = (*edgeIter)->id;
				//输入文件的mmRoadId域实际上是speed，所以(*ptIter)->mmRoadId输出的是speed
				ofs << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << (*edgeIter)->id << endl;
			}
			else //匹配失败
			{
				//(*ptIter)->mmRoadId = -1;
				ofs << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << -1 << endl;
			}
			ptIter++;
			edgeIter++;
		}
		ofs << -1 << endl;
		count++;
	}
	ofs.close();
}

void mmForBJdata()
{
	list<Traj*> trajs;
	TrajReader tr("D:\\trajectory\\singapore_data\\experiments\\SMMD\\trainData\\beijing\\newMMTrajs_50m.txt");
	tr.readTrajs(trajs);
	mapmatching(trajs, "mmTrajs_newmap_50m.txt");
}

void main()
{
	
	/*code for map matching*/
	/*double minLat = 0.99999;
	double maxLat = 1.6265;
	double minLon = 103.548;
	double maxLon = 104.1155;
	area.setArea(minLat, maxLat, minLon, maxLon);*/

	double gridSizeM = 25.0;
	int gridWidth = (area.maxLon - area.minLon) * GeoPoint::geoScale / gridSizeM;
	roadNetwork.setArea(&area);
	//roadNetwork.openOld("D:\\trajectory\\singapore_data\\singapore_map\\old\\", gridWidth);
	//roadNetwork.open("D:\\trajectory\\beijing_data\\beijing_map\\new\\", gridWidth);
	roadNetwork.open("D:\\trajectory\\beijing_data\\beijing_map\\new20151015\\", gridWidth);
	//roadNetwork.openOld(mapFileFolder, gridWidth);
	//roadNetwork.loadPolylines(fakeCenterlineFilePath);
	cout << "load map finished" << endl;

	//将figure复制到r_hat与r_fig，其中r_fig可以变化，来进行对baseline影响的实验
	/*for (int i = 0; i < roadNetwork.edges.size(); i++)
	{
		if (roadNetwork.edges[i] == NULL || roadNetwork.edges[i]->r_hat.size() == 0)
			continue;
		roadNetwork.edges[i]->r_fig.clear();
		roadNetwork.edges[i]->r_hat.clear();
		for each (GeoPoint* figurePt in *roadNetwork.edges[i]->figure)
		{
			GeoPoint* r_fig_pt = new GeoPoint(figurePt->lat, figurePt->lon);
			GeoPoint* r_hat_pt = new GeoPoint(figurePt->lat, figurePt->lon);
			roadNetwork.edges[i]->r_fig.push_back(r_fig_pt);
			roadNetwork.edges[i]->r_hat.push_back(r_hat_pt);
		}
	}*/

	shiftMap(0, 0);

	md.setArea(&area);
	md.setResolution(10000);
	md.newBitmap();
	md.lockBits();
	
	//loadTestData(-1);// INF, true);
	
	roadNetwork.drawMap(Color::Blue, md);
	/*list<GeoPoint*> pts;
	cout << "draw training data..." << endl;
	//TrajReader tr(trainDataPath);
	TrajReader tr("D:\\trajectory\\singapore_data\\experiments\\SMMD\\trainData\\beijing\\mmTrajs_newmap_50m.txt");
	tr.readGeoPoints(pts, &area);
	tr.outputPts(pts, "area3.txt");
	tr.close();
	for each(GeoPoint* pt in pts)
	{
		if (pt->mmRoadId == 68904)//43086)//49773)
			md.drawBigPoint(Gdiplus::Color::Blue, pt->lat, pt->lon);
	}
	//roadNetwork.ptIndex.drawGridLine(Color::Green, md);
	md.unlockBits();
	md.saveBitmap("BJ_plot.png");
	return;
	*/
	/**********************************************************/
	/*test code starts from here*/
	noiseScaleM = 0;
	int trainCount = 40000;
	int testCount = 10000;
	
	loadSMMTestData(trainCount, testCount, testDataSet, testSMMDataSet); //forSMM
	//loadTestData(-99, testCount); //forSMMD
	int startTime = clock();
	trainSMM(25.0, trainCount);
	int endTime = clock();
	printf("training time: %d\n", endTime - startTime);
	//system("pause");
	//reMap(trainDataSet, testSMMDataSet);
	genCSV_nomap(testSMMDataSet, "testFeature.csv", "testLabel.csv");
	genCSV_nomap(trainDataSet, "trainFeature.csv", "trainLabel.csv");
	return;
	check_r_hat();

	////evalSMMD_new(testDataSet);
	/*test code ends*/
	/**********************************************************/
	
	trembleMap(0);
	roadNetwork.drawMap(Color::Blue, md);
	roadNetwork.ptIndex.drawGridLine(Color::Green, md);
	if (0)
	{
		vector<int> ans_knn;
		int k = 5;
		kNN(k, testDataSet, ans_knn, trainCount);
		double acc_knn = evaluate(testDataSet, ans_knn);
		printf("acc_knn = %lf\n", acc_knn);
	}

	if (0)
	{
		vector<int> ans_smm;
		SMM(testDataSet, ans_smm);
		double acc_smm = evaluate(testDataSet, ans_smm);
		printf("acc_smm = %lf\n", acc_smm);
	}

	if (1)
	{
		vector<int> ans_simpleBaseline;
		simpleBaseline(testDataSet, ans_simpleBaseline);
		double acc_simpleBaseline = evaluate(testDataSet, ans_simpleBaseline);
		printf("acc_simpleBaseline = %lf\n", acc_simpleBaseline);
		count_no2NN(testDataSet);
	}

	if (1)
	{
		vector<int> ans_baseline;
		baseline(testDataSet, ans_baseline);
		double acc_baseline = evaluate(testDataSet, ans_baseline);
		printf("acc_baseline = %lf\n", acc_baseline);
	}

	md.unlockBits();
	md.saveBitmap("area2_SG.png");
	system("pause");
}