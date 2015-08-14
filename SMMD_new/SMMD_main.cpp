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

string workspaceFolder = "D:\\trajectory\\singapore_data\\experiments\\SMMD\\";
string testDataFolder = workspaceFolder + "testData\\";
string trainDataFolder = workspaceFolder + "trainData\\";


int datasetID = 2;
string trainDataPath = trainDataFolder + "area" + StringOperator::intToString(datasetID) +"_50m.txt"; // +"area1_50m.txt";
string testDataPath = testDataFolder + "testData3_area" + StringOperator::intToString(datasetID) +".txt";// "testData_area1.txt";
//string testDataPath = testDataFolder + "testData3.txt";
string polylineFilePath = trainDataFolder + "polyline1_new.txt";
string fakeCenterlineFilePath = trainDataFolder + "fakeCenterline" + StringOperator::intToString(datasetID) +".txt";
string priorFilePath = trainDataFolder + "SMMD_prior3.txt";

//Area area(1.342433, 1.353236, 103.864837, 103.875921); //area1
Area area(1.3290, 1.3444, 103.8377, 103.8598); //area2
//Area area(1.3280, 1.3947, 103.8245, 103.9114); //area3
//Area area(1.22, 1.5, 103.62, 104); //singapore half
//Area area(39.398073, 40.366082, 115.770605, 117.079043); //beijing full
//Area area(40.0310, 40.0764, 116.4001, 116.4710); //beijing area1
//Area area(39.9308, 40.0168, 116.3042, 116.4228); //beijing area2
/*double minLat = 1.22;
double maxLat = 1.5;
double minLon = 103.620;
double maxLon = 104.0;*/



Map roadNetwork;
Trainer trainer(&roadNetwork, &area);
MapDrawer md;


vector<SampleType> testDataSet;

void loadTestData(int count  = INF, bool draw = false );

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
	loadTestData();
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

	for each (GeoPoint* pt in pts)
	{
		md.drawBigPoint(Color::Green, pt->lat, pt->lon);
	}

	trainer.drawCenterline(fakeCenterlineFilePath, md);
}

////////////////////////////////////////////////////////////////////////

vector<int> correctCounts;
vector<int> allCounts;

void loadTestData(double threshM_for_intersection, int count /* = INF */, bool draw /* = false */)
{
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
			continue;
		if (currentCount >= count)
			break;
		if (roadNetwork.edges[rId] == NULL)
			continue;
		GeoPoint* x = new GeoPoint(xLat, xLon);
		GeoPoint* d = new GeoPoint(dLat, dLon);
		SampleType td(x, d, rId);

		//滤去路口的点
		vector<GeoPoint*> oneNN;
		roadNetwork.ptIndex.kNN_exact(x, 1, oneNN);
		if (GeoPoint::distM(oneNN[0], x) < threshM_for_intersection)
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
	trainer.loadTrainData(trainDataPath, count);
	trainer.loadPrior(priorFilePath);
	//roadNetwork.loadPolylines(polylineFilePath);
	roadNetwork.loadPolylines(fakeCenterlineFilePath);
	trainer.trainSimple(intervalM);
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
	roadNetwork.loadPolylines(fakeCenterlineFilePath);
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
	}
	else
	{
		pFunc = &SMMD::probSMM;
		//构造SMM testData
		list <GeoPoint*> testSMMDataSet;
		TrajReader tReader(trainDataPath);
		tReader.readGeoPoints(testSMMDataSet, &area, 100000);
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

		/**********************************************************/
		/*test code starts from here*/
		
		if (testData.rId == 48067)
		{
			allCount--;
			continue;
			md.drawBigPoint(Color::Black, testData.x->lat, testData.x->lon);
		}
		/*test code ends*/
		/**********************************************************/
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
		if (ratio >=2)
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
		if (SMMD::distM_signed(nearEdges[0], testData.x) < 0) // <0 北京 >0新加坡
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

		bool smmdSwitch = true;
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
			//allCount--;
			continue;
		}

		if (/*roadNetwork.edges[testData.rId] == NULL || */roadNetwork.edges[testData.rId]->r_hat.size() == 0)// || roadNetwork.distM(testData.x->lat, testData.x->lon, roadNetwork.edges[testData.rId]) >= 50.0)
		{
			//allCount--;
			invalidCount++;
			//md.drawBigPoint(Color::Black, testData.x->lat, testData.x->lon);
			continue;
		}

		if (roadNetwork.distM(testData.x->lat, testData.x->lon, roadNetwork.edges[testData.rId]) >= 50.0)
		{
			//allCount--;
			md.drawBigPoint(Color::Green, testData.x->lat, testData.x->lon);
			//TrajDrawer::drawOneTraj(roadNetwork.edges[37958]->figure, md, Color::Red, true);
			//printf("%d: %d\n", i, testData.rId);
			//testData.x->print();			
			//system("pause");
			//return;
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
		if (GeoPoint::distM(oneNN[0], testData.x) < 50.0)
		{
			//allCount--;
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
			//allCount--;
			continue;
		}

		//最近两条之中都没的话排除
		if (nearEdges[0]->id != testData.rId && nearEdges[1]->id != testData.rId)
		{
			//allCount--;
			twoNNNoCount++;
			continue;
		}

		//偏的不厉害的排除
		int slotId0 = nearEdges[0]->getSlotId(testData.x);
		int slotId1 = nearEdges[1]->getSlotId(testData.x);
		if (abs(roadNetwork.edges[testData.rId]->thetas[slotId0].mu - roadNetwork.edges[testData.rId]->thetas[slotId1].mu) < 0)
		{
			//allCount--;
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
			ans = smmd.probSMMD(nearEdges[0], testData.x, testData.d) > smmd.probSMMD(nearEdges[1], testData.x, testData.d) ? nearEdges[0]->id : nearEdges[1]->id;
		}
		else
		{
			ratio = smmd.probSMM(nearEdges[0], testData.x, testData.d) / smmd.probSMM(nearEdges[1], testData.x, testData.d);
			ans = smmd.probSMM(nearEdges[0], testData.x, testData.d) > smmd.probSMM(nearEdges[1], testData.x, testData.d) ? nearEdges[0]->id : nearEdges[1]->id;
		}
		if (ratio < 1)
			ratio = 1 / ratio;

		//只留下高概率的
		if (ratio >= 1)
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
			//allCount--;
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
	printf("cross = %d, bias = %d, 2NN = %d, prob = %d, invalid = %d\n", crossCount, biasCount, twoNNNoCount, probCount, invalidCount);
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
	tReader.readGeoPoints(testSMMDataSet, &area, 200000);
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
		int slotId = roadNetwork.edges[testData->mmRoadId]->getSlotId(testData);
		if (abs(roadNetwork.edges[testData->mmRoadId]->thetas[slotId].mu) < 5)
		{
			allCount--;
			continue;
		}
		vector<GeoPoint*> oneNN;
		roadNetwork.ptIndex.kNN_approx(testData, 1, 100.0, oneNN);
		if (GeoPoint::distM(oneNN[0], testData) < 50.0)
		{
			allCount--;
			continue;
		}

		md.drawBigPoint(Color::Red, testData->lat, testData->lon);
		
		
		
		if (roadNetwork.edges[testData->mmRoadId] == NULL)
			allCount--;
		if (testData->mmRoadId == smmd.doSMMD(testData, testData, pFunc))
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
			if (SMMD::distM_signed(nearEdges[0], testData.x) > 0) // <0 北京 >0新加坡
				prediction.push_back(nearEdges[0]->id);
			else
				prediction.push_back(nearEdges[1]->id);
		}
	}
}

void kNN(vector<SampleType>& testDataSet, vector<int>& prediction)
{
	prediction.clear();
	int k = 10;
	KNNClassifier kNNClassifier(&roadNetwork, &area);
	kNNClassifier.loadTrainData(trainDataPath, 500000);
	for each (SampleType testData in testDataSet)
	{
		prediction.push_back(kNNClassifier.classify(testData.x, k));
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
	roadNetwork.open("D:\\trajectory\\beijing_data\\beijing_map\\new\\", gridWidth);
	for (int i = 0; i < roadNetwork.edges.size(); i++)
	{
		for each (GeoPoint* figurePt in *roadNetwork.edges[i]->figure)
		{
			roadNetwork.edges[i]->r_fig.push_back(figurePt);
		}
	}

	md.setArea(&area);
	md.setResolution(8000);
	md.newBitmap();
	md.lockBits();
	
	loadTestData(25.0);// INF, true);
	roadNetwork.drawMap(Color::Blue, md);
	roadNetwork.ptIndex.drawGridLine(Color::Green, md);

	if (1)
	{
		vector<int> ans_simpleBaseline;
		simpleBaseline(testDataSet, ans_simpleBaseline);
		double acc_simpleBaseline = evaluate(testDataSet, ans_simpleBaseline);
		printf("acc_simpleBaseline = %lf\n", acc_simpleBaseline);
	}

	if (1)
	{
		vector<int> ans_baseline;
		baseline(testDataSet, ans_baseline);
		double acc_baseline = evaluate(testDataSet, ans_baseline);
		printf("acc_baseline = %lf\n", acc_baseline);
	}

	if (1)
	{
		vector<int> ans_knn;
		kNN(testDataSet, ans_knn);
		double acc_knn = evaluate(testDataSet, ans_knn);
		printf("acc_knn = %lf\n", acc_knn);
	}
	md.unlockBits();
	md.saveBitmap("area2_SG.png");
	system("pause");
}