/* 
 * Last Updated at [2014/10/25 15:07] by wuhao
 */
#pragma once
#include <iostream>
#include <string>
#include <stdlib.h>
#include <fstream>
#include <iomanip>
#include "GeoPoint.h"
#include "TrajReader.h"
#include "MapMatching.h"
#include "TrajDrawer.h"

using namespace std;

extern Map roadNetwork;
extern Map originalRoadNetwork;
extern MapDrawer md;

class ExpGenerator
{
public:
	Area* area;
	string inputFolder = "G:\\workspace\\1\\";// = "D:\\trajectory\\singapore_data\\201202\\every day\\";
	vector<string> inputFileNames;
	string outputFolder = "G:\\workspace\\1\\"; // "D:\\trajectory\\singapore_data\\experiments\\big area\\"; //����·�������޸�
	string newMMTrajsFileName = "newMMTrajs.txt";
	string newMMTrajsFileName_unmatched = "newMMTrajs_unmatched.txt";
	ofstream newMMTrajsFile;

	list<Traj*> rawTrajs;
	list<Traj*> trajsInArea;
	list<Traj*> doneTrajs;

	
	//driver func	
	void genExpData_1MM();
	void genExpData_2MM();
	void genSubSampledData(int interval, string folder, string fileName);
	void deleteForGeo();
	void deleteType1();
	void deleteType2();
	void deleteType3();


	void setArea(Area* area);
	//void dumpToFile(list<GeoPoint*>& source, string filename);
	
	

private:
	void genExpData_1MM(string rawTrajFilePath);
	void genExpData_2MM(string rawTrajFilePath);
	void extractUnmatchedTrajs();
	void readRawTrajs(string rawTrajFilePath);
	void doSplit();
	void doMM(Map* roadNetwork, list<Traj*>& trajs, double thresM = 50.0);
	void outputNewTrajs(list<Traj*>& trajs);
	bool overDistLimit(GeoPoint* pt1, GeoPoint* pt2);
	void dumpTo(list<Traj*>& source, list<Traj*>& dest);
	void deleteList(list<Traj*>& victimList);
	//parameters in split func
	double limitSpeed = 50.0;
	double limitDist = 800;
	double limitTime = 100;
};