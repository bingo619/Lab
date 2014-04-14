/* 
 * Last Updated at [2014/1/24 22:23] by wuhao
 */
#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <time.h>
#include "MapDrawer.h"
#include "Map.h"
#include <list>
#include <vector>
#include "GeoPoint.h"
#include "StringOperator.h"
#include "IndexedTraj.h"
#include "MapMatching.h"
#include <iomanip>
#include "PolylineGenerator.h"

#define eps 1e-8
#define INFINITE 999999999
using namespace std;

double minLat = 1.22;
double maxLat = 1.5;
double minLon = 103.620;
double maxLon = 104.0;
//double minLat = 0.99999;
//double maxLat = 1.6265;
//double minLon = 103.548;
//double maxLon = 104.1155;
int size = 15000;
//some switches
bool zoomed = true;
bool doExtendAndOutput = true;

typedef list<GeoPoint*> Traj;
Map roadNetwork;
MapDrawer md;

double limitSpeed = 50; //�������33m/s��prune��
double theta = 22.5;
double limitCosTheta = cos(theta/180*3.141592653); //ת��Ǵ���theta��prune��
double limitDist = 400; //�켣��֮�䳬��300m��prune��
double limitTime = 100; //�����������60���prune��

//grid index
int gridWidth = 500;
int gridHeight;
double gridSizeDeg;
list<IndexedTraj*> **grid = NULL;
int gridSearchRange = 1;

//cluster
double clusterDist = 50;
int supportThreshold = 6;
typedef list<IndexedTraj*> Cluster;

list<Traj*> rawTrajs;
list<Traj*> tempTrajs;
vector<IndexedTraj*> trajs;
vector<Cluster*> clusters;

//split
int outputFileIndex = 0;

/**********************************************************/
/*test code starts from here*/
vector<string> MMOutputFileName;
list<Figure*> gennedEgdes;
/*test code ends*/
/**********************************************************/


//��������
bool overDistLimit(GeoPoint* pt1, GeoPoint* pt2);
void drawOneTraj(Color color, Traj* traj);
bool smallerInX(simplePoint& pt1, simplePoint& pt2);

string getMMFileName(string fileName)
{
	//////////////////////////////////////////////////////////////////////////
	///���ݹ켣�ļ�����"input_000011.txt"����MM�ļ���"output_000011.txt"
	//////////////////////////////////////////////////////////////////////////
	string str = "000000";
	for (int i = 0; i < 6; i++)
		str[i] = fileName[i + 6];
	return "output_" + str + ".txt";
}

void readOneTrajectoryFail(std::string folderDir, std::string fileName)
{
	//////////////////////////////////////////////////////////////////////////
	///��һ��"input_000011.txt"(fileName), ͬʱ��һ��"output_000011.txt"
	///ֻ��ȡƥ��ʧ�ܵĹ켣��
	//////////////////////////////////////////////////////////////////////////
	FILE *fpIn, *fpOut;
	double lat, lon;
	int time, roadId;
	double confidence;
	bool isStart = true;
	fpIn = fopen((folderDir + "//input//" + fileName).c_str(), "r");
	fpOut = fopen((folderDir + "//output//" + getMMFileName(fileName)).c_str(), "r");
	bool startFlag = true;
	Traj* traj = NULL;
	while (!feof(fpIn))
	{
		int flag = fscanf(fpIn, "%d,%lf,%lf", &time, &lat, &lon);
		fscanf(fpOut, "%d,%d,%lf", &time, &roadId, &confidence);
		//��ֹĩ�ж�������
		if (flag == -1)
			break;
		if (roadId == -1) //map matchingʧ��
		{
			if (!md.inArea(lat, lon)) //����������
			{
				if (traj != NULL && traj->size() > 1)
				{
					startFlag = true;
					tempTrajs.push_back(traj);
				}
				continue;
			}
			if (startFlag)
			{
				traj = new Traj();
				GeoPoint* pt = new GeoPoint(lat, lon, time);
				traj->push_back(pt);
				startFlag = false;
			}
			else
			{
				//ԭ�ز���
				if (abs(lat - traj->back()->lat) < eps && abs(lon - traj->back()->lon) < eps)
					continue;
				GeoPoint* pt = new GeoPoint(lat, lon, time);
				traj->push_back(pt);
			}
		}
		else //map matching�ɹ�
		{
			if (!startFlag)
			{
				startFlag = true;
				if (traj->size() > 1)
				{
					tempTrajs.push_back(traj);
				}
				//else
					//delete(traj);
			}
		}
	}
	if (traj != NULL && traj->size() > 1)
	{
		tempTrajs.push_back(traj);
		//printf("traj made\n");
	}
	fclose(fpIn);
	fclose(fpOut);
}

Traj* readOneSRCTraj(std::string folderDir, std::string fileName, bool readMMFile)
{
	//////////////////////////////////////////////////////////////////////////
	///��һ���켣�ļ�,��"input_000011.txt",readMMFileΪtrueʱͬʱ����output_000011.txt
	//////////////////////////////////////////////////////////////////////////
	FILE *fpIn, *fpOut = NULL;
	double lat, lon;
	int time, roadId;
	double confidence;
	bool isStart = true;
	fpIn = fopen((folderDir + "input//" + fileName).c_str(), "r");
	if (readMMFile)
		fpOut = fopen((folderDir + "output//" + getMMFileName(fileName)).c_str(), "r");
	if (!fpIn)
	{
		cout << "open " + (folderDir + "input//" + fileName) + " error!\n";
		system("pause");
		exit(0);
	}
	Traj* traj = new Traj();
	while (!feof(fpIn))
	{
		int flag = fscanf(fpIn, "%d,%lf,%lf", &time, &lat, &lon);
		if (readMMFile)
			fscanf(fpOut, "%d,%d,%lf", &time, &roadId, &confidence);
		//��ֹĩ�ж�������
		if (flag == -1)
			break;
		GeoPoint* pt;
		if (readMMFile)
			pt = new GeoPoint(lat, lon, time, roadId);
		else
			pt = new GeoPoint(lat, lon, time);
		traj->push_back(pt);
	}
	fclose(fpIn);
	if (readMMFile)
		fclose(fpOut);
	return traj;
}

void readSRCTrajs(string folderDir, string fileName)
{
	rawTrajs.push_back(readOneSRCTraj(folderDir, fileName, true));
	//MMOutputFileName.push_back(getMMFileName(fileName));
}

void genStdTrajFile(list<Traj*>& sourceTrajs, string fileName)
{
	//////////////////////////////////////////////////////////////////////////
	///���켣�����Ա�׼��ʽ�����fileName�ļ���
	//////////////////////////////////////////////////////////////////////////
	ofstream ofs(fileName);
	ofs << fixed << showpoint << setprecision(8);
	for each (Traj* traj in sourceTrajs)
	{
		for (Traj::iterator ptIter = traj->begin(); ptIter != traj->end(); ptIter++)
		{
			ofs << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << (*ptIter)->mmRoadId << endl;
		}
		ofs << "-1" << endl;
	}
	ofs.close();
}

double trajDistM(Traj* traj, double thresholdM = INFINITE)
{
	//////////////////////////////////////////////////////////////////////////
	///����켣traj�ĳ��Ȳ����زض�,��λΪM
	///�����㳤��ʱ���ֵ�ǰ�����Ѿ�>thresholdM�Ļ��򲻼�����ȥֱ�ӷ���
	//////////////////////////////////////////////////////////////////////////
	double lengthM = 0;
	Traj::iterator ptIter = traj->begin(), nextPtIter = ptIter;
	nextPtIter++;
	while (1)
	{
		if (nextPtIter == traj->end())
			break;
		lengthM += GeoPoint::distM((*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
		if (lengthM > thresholdM)
			return lengthM;
		ptIter++;
		nextPtIter++;
	}
	return lengthM;
}

void doExtendForOneMMTraj(Traj* traj, list<Traj*>& dest, double extendDistM, double minTrajLength)
{
	//////////////////////////////////////////////////////////////////////////
	///��һ���Ѿ�MM���Ĺ켣�����и�,����ƥ��ʧ�ܵ��ӹ켣����dest,��������������extendDist��Χ����·
	///����ӹ켣ֻ��һ��������
	///[����] ���켣����С��minTrajLength�Ļ��Ͷ���
	//////////////////////////////////////////////////////////////////////////
	dest.clear();
	Traj* rawTraj = traj;
	Traj* tmpTraj = NULL;
	//��ÿ����
	Traj::iterator currentPtIter = rawTraj->begin();
	GeoPoint* currentPt = (*currentPtIter), *prePt = currentPt;
	bool startFlag = true;
	while (currentPtIter != rawTraj->end())
	{
		currentPt = (*currentPtIter);
		if (overDistLimit(prePt, currentPt)) //��ǰһ�������̫Զ���п�
		{
			if (tmpTraj != NULL && tmpTraj->size() > 1 && trajDistM(tmpTraj) > minTrajLength)
			{
				dest.push_back(tmpTraj);
				tmpTraj = NULL;
			}
			startFlag = true;
		}
		if (currentPt->mmRoadId == -1) //map matchingʧ��
		{
			if (!md.inArea(currentPt->lat, currentPt->lon)) //�������������п�
			{
				if (tmpTraj != NULL && tmpTraj->size() > 1 && trajDistM(tmpTraj) > minTrajLength)
				{
					dest.push_back(tmpTraj);
					tmpTraj = NULL;
				}
				startFlag = true;
			}
			else //����������
			{
				if (startFlag)
				{
					tmpTraj = new Traj();
					//tmpTraj->push_front(currentPt);
					//��ǰ��չ
					GeoPoint* extendPt = currentPt, *preExtendPt = currentPt;
					Traj::iterator extendPtIter = currentPtIter;
					while (md.inArea(extendPt->lat, extendPt->lon)//��չ����������
						&& !overDistLimit(preExtendPt, extendPt)) //��չ����ǰһ����չ����벻��
						
					{
						tmpTraj->push_front(extendPt);
						if (roadNetwork.getNearEdges(extendPt->lat, extendPt->lon, extendDistM).size() > 0)
						//[ע��]�������ܻ�����ڴ�й¶			
							break;
						preExtendPt = extendPt;
						if (extendPtIter != rawTraj->begin())
						{
							extendPtIter--;
							extendPt = (*extendPtIter);
						}
						else
							break;
					} //while end
					startFlag = false;
				} //if (startFlag) end
				else
				{
					tmpTraj->push_back(currentPt);
				}
			}
		}
		else //map matching�ɹ�
		{
			if (!startFlag)
			{
				//�����չ
				GeoPoint* extendPt = currentPt, *preExtendPt = tmpTraj->back();
				Traj::iterator extendPtIter = currentPtIter;
				while (extendPtIter != rawTraj->end() //��չ��û��β
					&& md.inArea(extendPt->lat, extendPt->lon) //��չ����������
					&& !overDistLimit(preExtendPt, extendPt)) //��չ����ǰһ����չ����벻��
				{
					tmpTraj->push_back(extendPt);
					if (roadNetwork.getNearEdges(extendPt->lat, extendPt->lon, extendDistM).size() > 0)
						break;
					preExtendPt = extendPt;
					extendPtIter++;
					extendPt = (*extendPtIter);
				}
				if (tmpTraj->size() > 1)
				{
					dest.push_back(tmpTraj);
					tmpTraj = NULL;
				}
				else
					tmpTraj->clear();
				startFlag = true;
			}
		}
		prePt = currentPt;
		currentPtIter++;
	} //while end
	if (tmpTraj != NULL)
	{
		if (tmpTraj->size() > 1 && trajDistM(tmpTraj) > minTrajLength)
		{
			dest.push_back(tmpTraj);
			tmpTraj = NULL;
		}
		else
			tmpTraj->clear();
	}
}

void doExtend(list<Traj*>& src, list<Traj*>& dest, double extendDistM, bool doOutput = false)
{
	//////////////////////////////////////////////////////////////////////////
	///����src, src����ΪMM��Ĺ켣����
	///�Բ�ƥ����ӹ켣������������extendDistM������·λ��,���ҹ켣��֮�䲻��̫Զ
	///�����dest,doOutputΪtrueʱͬʱ������ļ�
	///[����]���켣���볤�ȵ���minTrajLengthM�Ļ�����
	//////////////////////////////////////////////////////////////////////////
	ofstream ofs;
	double minTrajLength = 100;
	if (doOutput)
	{
		ofs.open("extended_unmatched_trajs.txt");
		ofs << fixed << showpoint << setprecision(8);
	}
	//����ÿ���켣
	for (list<Traj*>::iterator trajIter = src.begin(); trajIter != src.end(); trajIter++)
	{
		list<Traj*> extendSubTrajs;
		doExtendForOneMMTraj(*trajIter, extendSubTrajs, extendDistM, minTrajLength);
		dest.splice(dest.end(), extendSubTrajs);
	}

	cout << ">> extracting and extending unmatchied trajs finished" << endl;
	if (doOutput)
	{
		cout << ">> start output to file" << endl;
		for (list<Traj*>::iterator trajIter = dest.begin(); trajIter != dest.end(); trajIter++)
		{
			for (Traj::iterator ptIter = (*trajIter)->begin(); ptIter != (*trajIter)->end(); ptIter++)
			{
				ofs << (*ptIter)->time << " "
					<< (*ptIter)->lat << " "
					<< (*ptIter)->lon << " "
					<< (*ptIter)->mmRoadId << endl;
			}
			ofs << "-1" << endl;
		}
		cout << ">> output to file finished" << endl;
		ofs.close();
	}
}

void scanTrajFolder(string folderDir, void (*func)(string, string))
{
	//////////////////////////////////////////////////////////////////////////
	///����folderDir�µ����й켣�ļ�,�켣�ļ�Ŀ¼�и�ʽҪ�� 
	//////////////////////////////////////////////////////////////////////////
	/*�ļ�Ŀ¼�ṹΪ
	* folderDir
	* |-input
	*   |-input_000011.txt ...
	* |-output
	*   |-output_000011.txt ...
	*/
	cout << ">> start scanning " << folderDir << endl;

	string tempS = folderDir + "\\input\\" + "*.txt";
	const char* dir = tempS.c_str();
	_finddata_t file;
	long lf;
	if ((lf = _findfirst(dir, &file)) == -1l)
		return;
	else
	{
		do
		{
			//readOneTrajectory(folderDir, file.name);
			//tempTrajs.push_back(readOneTrajectory(folderDir + "input\\" + file.name));
			func(folderDir, file.name);
		} while (_findnext(lf, &file) == 0);
		_findclose(lf);
		cout << ">> scanning " << folderDir << " finished" << endl;
		return;
	}
}

//���ֹ켣
void readOneSRCTrajAndSplit(string folderDir, string fileName, ofstream& ofs)
{
	//////////////////////////////////////////////////////////////////////////
	///����һ��input_XXXXXX.txt�켣�ļ�,�������ֳ��������ӹ켣(�������̫��ʱ��̫�����ж�)
	//////////////////////////////////////////////////////////////////////////
	FILE *fpIn;
	double lat, lon;
	int time;
	bool isStart = true;
	fpIn = fopen((folderDir + "//input//" + fileName).c_str(), "r");
	GeoPoint prePt, currentPt;
	bool startFlag = true;

	while (!feof(fpIn))
	{
		int flag = fscanf(fpIn, "%d,%lf,%lf", &time, &lat, &lon);
		//��ֹĩ�ж�������
		if (flag == -1)
			break;
		if (startFlag)
		{
			if (md.inArea(lat, lon))
			{
				ofs << time << " " << lat << " " << lon << " -1" << endl;
				prePt.lat = lat;
				prePt.lon = lon;
				prePt.time = time;
				startFlag = false;
			}
			else
				continue;
		}
		else
		{
			currentPt.lat = lat;
			currentPt.lon = lon;
			currentPt.time = time;
			if (md.inArea(lat, lon))
			{
				if (!overDistLimit(&prePt, &currentPt))
				{
					ofs << time << " " << lat << " " << lon << " -1" << endl;
					prePt.lat = lat;
					prePt.lon = lon;
					prePt.time = time;
					startFlag = false;
				}
				else
				{
					ofs << -1 << endl;
					ofs << time << " " << lat << " " << lon << " -1" << endl;
					prePt.lat = lat;
					prePt.lon = lon;
					prePt.time = time;
					startFlag = false;
				}
			}
			else
			{
				ofs << -1 << endl;
				startFlag = true;
			}
		}
	}
	if (startFlag == false)
		ofs << -1 << endl;
	fclose(fpIn);
}
void splitSRCTrajFiles(string folderDir)
{
	//////////////////////////////////////////////////////////////////////////
	///����folderDir\input�µ����й켣�ļ�,�켣�ļ�Ŀ¼�и�ʽҪ�� 
	///��̫Զ̫���Ĺ켣�ж�,Ȼ�������folderDir�µ�splitedTrajs.txt
	//////////////////////////////////////////////////////////////////////////
	/*�ļ�Ŀ¼�ṹΪ
	* folderDir
	* |-input
	*   |-input_000011.txt ...
	*/
	cout << ">> start scanning " << folderDir << endl;
	ofstream ofs(folderDir + "splitedTrajs.txt");
	ofs << fixed << showpoint << setprecision(8);
	string tempS = folderDir + "input\\" + "*.txt";
	const char* dir = tempS.c_str();
	_finddata_t file;
	long lf;
	if ((lf = _findfirst(dir, &file)) == -1l)
		return;
	else
	{
		do
		{
			readOneSRCTrajAndSplit(folderDir, file.name, ofs);
		} while (_findnext(lf, &file) == 0);
		_findclose(lf);
		cout << ">> scanning " << folderDir << " finished" << endl;
		return;
	}
	ofs.close();
}

//�����׼�켣�ļ�,��ʽ:time lat lon mmRoadId
void readStdTrajs(string path, list<Traj*>& dest, int num = INFINITE)
{
	//////////////////////////////////////////////////////////////////////////
	///num�������켣������Ĭ��ΪINFINITE������ȫ������
	//////////////////////////////////////////////////////////////////////////
	cout << ">> start reading std trajs" << endl;
	ifstream ifs(path);
	if (!ifs)
	{
		cout << "open traj file error" << endl;
		system("pause");
		exit(0);
	}
	bool isStart = true;
	int time, mmRoadId;
	double lat, lon;
	Traj* tmpTraj = NULL;
	int count = 0;
	while (ifs)
	{
		if (count == num)
		{
			break;
		}
		int time;
		ifs >> time;
		if (ifs.fail())
		{
			break;
		}
		if (time == -1)
		{
			isStart = true;
			if (tmpTraj != NULL && tmpTraj->size() > 1)
			{
				dest.push_back(tmpTraj);
				count++;
			}
			continue;
		}
		else
		{
			ifs >> lat >> lon >> mmRoadId;
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
	cout << ">> reading std trajs finished" << endl;
}

void readStdTrajs(string path, vector<IndexedTraj*>& dest, int num = INFINITE)
{
	cout << ">> start reading std trajs" << endl;
	ifstream ifs(path);
	if (!ifs)
	{
		cout << "open traj file error" << endl;
		system("pause");
		exit(0);
	}
	bool isStart = true;
	int time, mmRoadId;
	double lat, lon;
	Traj* tmpTraj = NULL;
	IndexedTraj* tmpITraj = NULL;
	int count = 0;
	while (ifs)
	{
		if (count == num)
		{
			break;
		}
		int time;
		ifs >> time;
		if (ifs.fail())
		{
			break;
		}
		if (time == -1)
		{
			isStart = true;
			if (tmpTraj != NULL && tmpTraj->size() > 1)
			{
				tmpITraj = new IndexedTraj(tmpTraj);
				dest.push_back(tmpITraj);
				count++;
			}
			continue;
		}
		else
		{
			ifs >> lat >> lon >> mmRoadId;
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
	cout << ">> reading std trajs finished" << endl;
}

//raw data mapmatching
void rawMapMatching(list<Traj*>& trajs, string outputPath)
{
	cout << ">> start map matching" << endl;
	ofstream ofs(outputPath);
	ofs << fixed << showpoint << setprecision(8);
	//��ÿһ���켣
	for (list<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		list<Edge*> result = MapMatching(*(*trajIter), 50.0); //MapMatching
		Traj* traj = (*trajIter);
		if (traj == NULL)
			continue;
		//��ÿһ����
		Traj::iterator ptIter = traj->begin();
		list<Edge*>::iterator edgeIter = result.begin();
		while (ptIter != traj->end())
		{
			if ((*edgeIter) != NULL) //ƥ��ɹ�
			{
				ofs << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << (*edgeIter)->id << endl;
			}
			else //ƥ��ʧ��
			{
				ofs << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << "-1" << endl;
			}
			ptIter++;
			edgeIter++;
		}
		ofs << "-1" << endl;
		ofs.close();
		ofs << "output MM file to " << outputPath << endl;
	}
}

double cosAngle(GeoPoint* pt1, GeoPoint* pt2, GeoPoint* pt3)
{
	//////////////////////////////////////////////////////////////////////////
	///������(pt1->pt2)��(pt2->pt3)�нǵ�����
	//////////////////////////////////////////////////////////////////////////
	double v1x = pt2->lon - pt1->lon;
	double v1y = pt2->lat - pt1->lat;
	double v2x = pt3->lon - pt2->lon;
	double v2y = pt3->lat - pt2->lat;
	return (v1x * v2x + v1y * v2y) / sqrt((v1x * v1x + v1y * v1y)*(v2x * v2x + v2y * v2y));
}

bool overDistLimit(GeoPoint* pt1, GeoPoint* pt2)
{
	//////////////////////////////////////////////////////////////////////////
	///ʱ��������limitTime�����ٶȴ���limitSpeed����֮��������limitDist����false
	//////////////////////////////////////////////////////////////////////////
	double  dist = GeoPoint::distM(*pt1, *pt2);
	double speed;
	if (pt2->time - pt1->time != 0)
	{
		speed = dist / abs(pt2->time - pt1->time);
	}
	else
		speed = 0;
	return dist > limitDist || abs(pt2->time - pt1->time) > limitTime || (speed > limitSpeed);
}

bool overAngle(GeoPoint* pt1, GeoPoint* pt2, GeoPoint* pt3)
{
	return (cosAngle(pt1, pt2, pt3) < limitCosTheta);
}

void trajRefinement(Traj* traj)
{
	//////////////////////////////////////////////////////////////////////////
	///��һ��ƥ��ʧ�ܵĹ켣����ɸѡ,����"������"�Ĺ켣��
	//////////////////////////////////////////////////////////////////////////
	if (traj->size() == 2)
	{
		if (!overDistLimit(traj->front(), traj->back()))
		{
			IndexedTraj* iTraj = new IndexedTraj(traj);
			trajs.push_back(iTraj);
		}
	}
	else if (traj->size() == 3)
	{
		Traj::iterator iter = traj->begin();
		GeoPoint* pt1 = (*iter);
		iter++;
		GeoPoint* pt2 = (*iter);
		iter++;
		GeoPoint* pt3 = (*iter);
		if (!overAngle(pt1, pt2, pt3))
		{
			if (!overDistLimit(pt1, pt2) && !overDistLimit(pt2, pt3))
			{
				Traj* tempTraj = new Traj();
				tempTraj->push_back(pt1);
				tempTraj->push_back(pt2);
				tempTraj->push_back(pt3);
				IndexedTraj* iTraj = new IndexedTraj(tempTraj);
				trajs.push_back(iTraj);
			}
		}
		else
		{
			if (!overDistLimit(pt1, pt2))
			{
				Traj* tempTraj = new Traj();
				tempTraj->push_back(pt1);
				tempTraj->push_back(pt2);
				IndexedTraj* iTraj = new IndexedTraj(tempTraj);
				trajs.push_back(iTraj);
			}
			if (overDistLimit(pt1, pt2) && !overDistLimit(pt2, pt3))
			{
				Traj* tempTraj = new Traj();
				tempTraj->push_back(pt2);
				tempTraj->push_back(pt3);
				IndexedTraj* iTraj = new IndexedTraj(tempTraj);
				trajs.push_back(iTraj);
			}
		}
	}
	else
	{
		Traj::iterator currentIter = traj->begin();
		Traj* tempTraj =  new Traj();
		while (currentIter != traj->end())
		{
			if (tempTraj->size() == 0)
			{
				Traj::iterator tempIter = currentIter;
				Traj::iterator nextIter = ++currentIter;
				if (nextIter == traj->end())
					break;
				if (!overDistLimit((*tempIter), (*nextIter)))
				{
					tempTraj->push_back((*tempIter));
					tempTraj->push_back((*nextIter));
				}
				else
					continue;
			}
			else
			{
				Traj::iterator iter = tempTraj->end();
				iter--;
				Traj::iterator preIter = iter;
				iter--;
				Traj::iterator prepreIter = iter;
				if (!overDistLimit((*preIter), (*currentIter)) && !overAngle((*prepreIter),(*preIter),(*currentIter)))
				{
					tempTraj->push_back((*currentIter));
				}
				else
				{
					IndexedTraj* iTraj = new IndexedTraj(tempTraj);
					trajs.push_back(iTraj);
					/*if (tempTraj->size() != 2 && tempTraj->size() != 3 && tempTraj->size() != 4)
					{
						cout << "size:" << tempTraj->size() << endl;
					}*/					
					tempTraj = new Traj();
					continue;
				}
			}									
			currentIter++;
		}
		//delete(traj);
	}
	
}

void refinement()
{
	//////////////////////////////////////////////////////////////////////////
	///�����й켣����ɸѡ,Դ�켣����ΪtempTrajs,Ŀ��켣����Ϊtrajs
	//////////////////////////////////////////////////////////////////////////
	cout << ">> start refinement" << endl;
	for (list<Traj*>::iterator iter = tempTrajs.begin(); iter != tempTrajs.end(); iter++)
	{
		trajRefinement(*iter);
	}
	cout << ">> refinement finished" << endl;
}
//grid index
void createGridIndex(void(*pInsertFunc)(IndexedTraj*))
{
	//////////////////////////////////////////////////////////////////////////
	///�����й켣������������
	//////////////////////////////////////////////////////////////////////////
	cout << ">> start creating grid index" << endl;
	//initialization
	gridHeight = int((maxLat - minLat) / (maxLon - minLon) * double(gridWidth)) + 1;
	gridSizeDeg = (maxLon - minLon) / double(gridWidth);
	grid = new list<IndexedTraj*>*[gridHeight];
	
	for (int i = 0; i < gridHeight; i++)
		grid[i] = new list<IndexedTraj*>[gridWidth];
	
	printf("gridWidth = %d, gridHeight = %d\n", gridWidth, gridHeight);
	cout << "gridSize = " << gridSizeDeg * GeoPoint::geoScale << "m" << endl;
	for (vector<IndexedTraj*>::iterator iTrajIter = trajs.begin(); iTrajIter != trajs.end(); iTrajIter++)
	{
		pInsertFunc((*iTrajIter));
	}
	int count = 0;
	for (int row = 0; row < gridHeight; row++)
	{
		for (int col = 0; col < gridWidth; col++)
		{
			count += grid[row][col].size();
		}
	}
	cout << "grid traj count: " << count << endl;
	cout << ">> grid index created" << endl;
}
	//Lite edition
void createGridIndexForOneTraj_Lite(IndexedTraj* iTraj)
{
	//////////////////////////////////////////////////////////////////////////
	///��һ���켣������������,�������켣�ϵĵ㾭������������
	///TODO:
	///		�Ƿ��б�Ҫʵ�־������е���������������?
	//////////////////////////////////////////////////////////////////////////
	Traj* traj = iTraj->traj;
	for (Traj::iterator iter = traj->begin(); iter != traj->end(); iter++)
	{
		int row = ((*iter)->lat - minLat) / gridSizeDeg;
		int col = ((*iter)->lon - minLon) / gridSizeDeg;
		if (row >= gridHeight || row < 0 || col >= gridWidth || col < 0)
		{
			printf("pt(%lf, %lf), row = %d, col = %d\n", (*iter)->lat, (*iter)->lon, row, col);
			system("pause");
		}
		//�ù켣�Ѿ������ĳ���������ټ�
		if (grid[row][col].size() > 0 && grid[row][col].back() == iTraj)
			continue;
		else
			grid[row][col].push_back(iTraj);
	}
}

	//Full edition
void insertTrajIntoGrid(IndexedTraj* iTraj, int row, int col)
{
	//////////////////////////////////////////////////////////////////////////
	///���켣traj����grid[row][col]������������Ѿ�����������
	//////////////////////////////////////////////////////////////////////////
	
	if (grid[row][col].size() > 0 && grid[row][col].back() == iTraj)
		return;
	else
		grid[row][col].push_back(iTraj);
}

void createGridIndexForSegment(IndexedTraj *iTraj, GeoPoint* fromPT, GeoPoint* toPt)
{
	//////////////////////////////////////////////////////////////////////////
	///��traj�е�fromPt->toPt�β����������������������񶼼�����ָ��
	///����������ཻ����������߳�֮��С��strictThreshold�򲻼�������
	//////////////////////////////////////////////////////////////////////////
	if (iTraj == NULL)
		return;
	bool crossRow;
	bool strictThreshold = 0.1;
	GeoPoint* pt1 = fromPT;
	GeoPoint* pt2 = toPt;
	double x1 = pt1->lon - minLon;
	double y1 = pt1->lat - minLat;
	double x2 = pt2->lon - minLon;
	double y2 = pt2->lat - minLat;
	int row1 = y1 / gridSizeDeg;
	int row2 = y2 / gridSizeDeg;
	int col1 = x1 / gridSizeDeg;
	int col2 = x2 / gridSizeDeg;
	double A = y2 - y1;
	double B = -(x2 - x1);
	double C = -B * y1 - A * x1;
	int i, j;
	//pt1,pt2����һ��cell��
	if (row1 == row2 && col1 == col2)
	{
		insertTrajIntoGrid(iTraj, row1, col1);
		return;
	}
	//ֻ��Խ�������
	if (row1 == row2)
	{
		//ͷ
		double headDist = ((min(col1, col2) + 1) * gridSizeDeg - min(x1, x2)) / gridSizeDeg;
		if (headDist / gridSizeDeg > strictThreshold)
			insertTrajIntoGrid(iTraj, row1, min(col1, col2));
		//�м�
		for (i = min(col1, col2) + 1; i < max(col1, col2); i++)
		{
			insertTrajIntoGrid(iTraj, row1, i);
		}
		//β
		double tailDist = (max(x1, x2) - max(col1, col2) * gridSizeDeg) / gridSizeDeg;
		if (tailDist / gridSizeDeg > strictThreshold)
			insertTrajIntoGrid(iTraj, row1, max(col1, col2));
		return;
	}
	//ֻ��Խ�������
	if (col1 == col2)
	{
		//ͷ
		double headDist = ((min(row1, row2) + 1) * gridSizeDeg - min(y1, y2)) / gridSizeDeg;
		if (headDist / gridSizeDeg > strictThreshold)
			insertTrajIntoGrid(iTraj, min(row1, row2), col1);
		//�м�
		for (i = min(row1, row2) + 1; i < max(row1, row2); i++)
		{
			insertTrajIntoGrid(iTraj, i, col1);
		}
		//β
		double tailDist = (max(y1, y2) - max(row1, row2) * gridSizeDeg) / gridSizeDeg;
		if (tailDist / gridSizeDeg > strictThreshold)
			insertTrajIntoGrid(iTraj, max(row1, row2), col1);
		return;
	}
	simplePoint pts[1000];
	int n_pts = 0;
	for (i = min(row1, row2) + 1; i <= max(row1, row2); i++)
	{
		pts[n_pts++] = std::make_pair((-C - B*i*gridSizeDeg) / A, i*gridSizeDeg);
	}
	for (i = min(col1, col2) + 1; i <= max(col1, col2); i++)
	{
		pts[n_pts++] = std::make_pair(i*gridSizeDeg, (-C - A*i*gridSizeDeg) / B);
	}
	std::sort(pts, pts + n_pts, smallerInX);

	GeoPoint* leftPt, *rightPt;
	if (x1 < x2)
	{
		leftPt = pt1;
		rightPt = pt2;
	}
	else
	{
		leftPt = pt2;
		rightPt = pt1;
	}
	double xL = leftPt->lon - minLon;
	double xR = rightPt->lon - minLon;
	double yL = leftPt->lat - minLat;
	double yR = rightPt->lat - minLat;

	//ͷ
	double headDist = sqrt((xL - pts[0].first)*(xL - pts[0].first) + (yL - pts[0].second)*(yL - pts[0].second));
	if (headDist / gridSizeDeg > strictThreshold)
		insertTrajIntoGrid(iTraj, (int)(yL / gridSizeDeg), (int)(xL / gridSizeDeg));
	//�м�
	for (i = 0; i < n_pts - 1; i++)
	{
		double dist = sqrt((pts[i].first - pts[i + 1].first)*(pts[i].first - pts[i + 1].first) + (pts[i].second - pts[i + 1].second)*(pts[i].second - pts[i + 1].second));
		if (dist / gridSizeDeg > strictThreshold)
			//insertEdgeIntoGrid(edge, getRowId(pts[i], pts[i + 1]), getColId(pts[i], pts[i + 1]));
		{
			//��1e-9��Ϊ�˽��double�ľ������,����ԭ��rowӦ����13��,��Ϊ�����������12.99999999,ȡ������12
			int pts_i_row = (int)(pts[i].second / gridSizeDeg + 1e-9);
			int pts_i_col = (int)(pts[i].first / gridSizeDeg + 1e-9);
			int pts_i_plus_1_row = (int)(pts[i + 1].second / gridSizeDeg + 1e-9);
			int pts_i_plus_1_col = (int)(pts[i + 1].first / gridSizeDeg + 1e-9);
			int row = min(pts_i_row, pts_i_plus_1_row);
			int col = min(pts_i_col, pts_i_plus_1_col);
			insertTrajIntoGrid(iTraj, row, col);
		}
	}
	//β
	double tailDist = sqrt((xR - pts[n_pts - 1].first)*(xR - pts[n_pts - 1].first) + (yR - pts[n_pts - 1].second)*(yR - pts[n_pts - 1].second));
	if (tailDist / gridSizeDeg > strictThreshold)
		insertTrajIntoGrid(iTraj, (int)(yR / gridSizeDeg), (int)(xR / gridSizeDeg));
	return;
}

void createGridIndexForOneTraj(IndexedTraj *iTraj)
{
	//////////////////////////////////////////////////////////////////////////
	///���켣iTraj������������
	///��������Ϊ: �Թ켣��������������ӹ켣ָ��,�����������ĳ��ȹ�С�����
	//////////////////////////////////////////////////////////////////////////
	if (iTraj == NULL)
		return;
	Traj::iterator ptIter = iTraj->traj->begin();
	Traj::iterator nextPtIter = iTraj->traj->begin(); nextPtIter++;
	while (nextPtIter != iTraj->traj->end())
	{
		createGridIndexForSegment(iTraj, *ptIter, *nextPtIter);
		ptIter++;
		nextPtIter++;
	}
}

//drawer
Color genColor(int i)
{
	switch (i % 10)
	{
	case 1: return Color::Red;
	case 2: return Color::Green;
	case 3: return Color::Blue;
	case 4: return Color::Yellow;
	case 5: return Color::Aqua;
	case 6: return Color::Pink;
	case 7: return Color::Gray;
	case 8: return Color::Brown;
	case 9: return Color::Gold;
	case 10: return Color::Purple;
	default:
		break;
	}
}

Color randomColor()
{
	int r = int(((double)rand()) / RAND_MAX * 255);
	int g = int(((double)rand()) / RAND_MAX * 255);
	int b = int(((double)rand()) / RAND_MAX * 255);
	Color color((byte)r, (byte)g, (byte)b);
	return color;
}

void drawGridLine(Gdiplus::Color color)
{
	//////////////////////////////////////////////////////////////////////////
	///��ͼƬ�ϻ��������� 
	//////////////////////////////////////////////////////////////////////////
	ARGB argb = Color::MakeARGB(90, color.GetR(), color.GetG(), color.GetB());
	color.SetValue(argb);
	double delta = 0.0000001;
	for (int i = 0; i < gridHeight; i++)
	{
		double lat = minLat + gridSizeDeg * i;
		md.drawLine(color, lat, minLon + delta, lat, maxLon - delta);
	}
	for (int i = 0; i < gridWidth; i++)
	{
		double lon = minLon + gridSizeDeg * i;
		md.drawLine(color, minLat + delta, lon, maxLat - delta, lon);
	}
}

void drawAllTrajs(std::string folderDir, std::string fileName)
{
	FILE *fpIn, *fpOut;
	double lat, lon;
	int time, roadId;
	double confidence;
	bool isStart = true;
	fpIn = fopen((folderDir + "//input//" + fileName).c_str(), "r");
	//fpOut = fopen((folderDir + "//output//" + getMMFileName(fileName)).c_str(), "r");
	bool startFlag = true;
	Traj* traj = NULL;
	double preLat, preLon;
	while (!feof(fpIn))
	{
		int flag = fscanf(fpIn, "%d,%lf,%lf", &time, &lat, &lon);
		//fscanf(fpOut, "%d,%d,%lf", &time, &roadId, &confidence);

		//��ֹĩ�ж�������
		if (flag == -1)
			break;
		if (startFlag)
		{
			preLat = lat;
			preLon = lon;
			startFlag = false;
			continue;
		}
		else
		{
			md.drawLine(Gdiplus::Color::Green, lat, lon, preLat, preLon);
			md.drawPoint(Gdiplus::Color::Red, preLat, preLon);
			if (md.inArea(lat, lon) && md.inArea(preLat, preLon) && abs(lat - preLat) > 0.15)
			{
				cout << fileName << " time:" << time << endl;
				system("pause");
			}
			preLat = lat;
			preLon = lon;
		}

	}
	if (traj != NULL && traj->size() > 1)
		tempTrajs.push_back(traj);
	fclose(fpIn);
	//fclose(fpOut);
}

void drawTrajs(Color color, list<Traj*> trajs, bool drawInterPt, bool boldLine)
{
	//////////////////////////////////////////////////////////////////////////
	///<����> ��trajs�е����й켣,����Ϊlist����,Ԫ��ΪTraj*
	///drawInterPtΪtrue�����м��,��Ϊ��ɫʮ�ֵ�
	///boldLineΪtrue�򻭴���
	//////////////////////////////////////////////////////////////////////////
	for (list<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		if (*trajIter == NULL)
			continue;
		Traj::iterator ptIter = (*trajIter)->begin(), nextPtIter = ptIter;
		nextPtIter++;
		while (1)
		{
			if (nextPtIter == (*trajIter)->end())
				break;
			if (boldLine)
				md.drawBoldLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			else
				md.drawLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			if (drawInterPt)
			{
				md.drawBigPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
				md.drawBigPoint(Gdiplus::Color::Black, (*nextPtIter)->lat, (*nextPtIter)->lon);
			}
			ptIter++;
			nextPtIter++;
		}
	}
}

void drawTrajs(Color color, vector<IndexedTraj*> trajs, bool drawInterPt, bool boldLine)
{
	//////////////////////////////////////////////////////////////////////////
	///<����> ��trajs�е����й켣,����Ϊvector����,Ԫ��ΪIndexedTraj*
	///drawInterPtΪtrue�����м��,��Ϊ��ɫʮ�ֵ�
	///boldLineΪtrue�򻭴���
	//////////////////////////////////////////////////////////////////////////
	for (vector<IndexedTraj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		Traj::iterator ptIter = (*trajIter)->traj->begin(), nextPtIter = ptIter;
		nextPtIter++;
		while (1)
		{
			if (nextPtIter == (*trajIter)->traj->end())
				break;
			if (boldLine)
				md.drawBoldLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			else
				md.drawLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			if (drawInterPt)
			{
				md.drawBigPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
				md.drawBigPoint(Gdiplus::Color::Black, (*nextPtIter)->lat, (*nextPtIter)->lon);
			}
			ptIter++;
			nextPtIter++;
		}
	}
}

void drawTrajs(Color color, list<IndexedTraj*> trajs, bool drawInterPt, bool boldLine)
{
	//////////////////////////////////////////////////////////////////////////
	///<����> ��trajs�е����й켣,����Ϊlist����,Ԫ��ΪIndexedTraj*
	///drawInterPtΪtrue�����м��,��Ϊ��ɫʮ�ֵ�
	///boldLineΪtrue�򻭴���
	//////////////////////////////////////////////////////////////////////////	
	for (list<IndexedTraj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		Traj::iterator ptIter = (*trajIter)->traj->begin(), nextPtIter = ptIter;
		nextPtIter++;
		while (1)
		{
			if (nextPtIter == (*trajIter)->traj->end())
				break;
			if (boldLine)
				md.drawBoldLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			else
				md.drawLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			if (drawInterPt)
			{
				md.drawBigPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
				md.drawBigPoint(Gdiplus::Color::Black, (*nextPtIter)->lat, (*nextPtIter)->lon);
			}
			ptIter++;
			nextPtIter++;
		}
	}
}

void drawOneTraj(Color color, Traj* traj)
{
	/************************************************************************/
	/*��colorɫ��һ���켣,�켣����Ϊlist<GeoPoint*>*                           */
	/************************************************************************/
	if (traj == NULL)
		return;
	Traj::iterator ptIter = traj->begin(), nextPtIter = ptIter;
	md.drawPoint(color, (*ptIter)->lat, (*ptIter)->lon);
	nextPtIter++;
	while (1)
	{
		if (nextPtIter == traj->end())
			break;
		md.drawLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
		//cout << "draw: " << md.geoToScreen((*ptIter)->lat, (*ptIter)->lon).X << ", " <<
		//	md.geoToScreen((*ptIter)->lat, (*ptIter)->lon).Y << endl;
		if (!md.inArea((*ptIter)->lat, (*ptIter)->lon))
			system("pause");
		md.drawBigPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
		//md.drawBigPoint(Gdiplus::Color::Black, (*nextPtIter)->lat, (*nextPtIter)->lon);
		ptIter++;
		nextPtIter++;
	}
	//md.drawBigPoint(Gdiplus::Color::Black, traj->back()->lat, traj->back()->lon); //���յ�
}

void drawClusteredTrajs()
{
	int count = 0;
	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i] == NULL)
			continue;
		if (clusters[i]->size() > 0)
		{
			Color color = randomColor();
			for (Cluster::iterator iTrajIter = clusters[i]->begin(); iTrajIter != clusters[i]->end(); iTrajIter++)
			{
				drawOneTraj(color, (*iTrajIter)->traj);
			}
			count += clusters[i]->size();
		}
	}
	cout << "cluster size: " << count << endl;
}

//core
	//new road generator
void drawPolyline(PolylineGenerator pg)
{
	cout << "polysize before drawing " << pg.polyline.size() << endl;
	for (int i = 0; i < pg.polyline.size() - 1; i++)
	{
		md.drawLine(Color::Green, (int)pg.polyline[i].x, (int)pg.polyline[i].y, (int)pg.polyline[i + 1].x, (int)pg.polyline[i + 1].y);
	}
	for (int i = 0; i < pg.polyline.size(); i++)
	{
		md.drawBigPoint(Color::Blue, (int)pg.polyline[i].x, (int)pg.polyline[i].y);
	}
}

void drawAllGennedEdges()
{
	drawTrajs(Color::Black, gennedEgdes, true, true);
}

int extendAndSplitEdge(GeoPoint* prePt, GeoPoint* succPt, double threshold, bool& onExtend)
{
	//////////////////////////////////////////////////////////////////////////
	///��prePt->succPt��������,�ҵ���һ���ཻ��·��,���ؽ������ɵ�nodeId,onExtendΪtrue
	///����ཻ·�ν���prePt<->succPt֮��,��onExtendΪfalse
	///���threshold���ڶ�û������Ҫ���·���򷵻�-1
	//////////////////////////////////////////////////////////////////////////
	double A = succPt->lat - prePt->lat;
	double B = -(succPt->lon - prePt->lon);
	double C = prePt->lat * (succPt->lon - prePt->lon)
		- prePt->lon * (succPt->lat - prePt->lat);
	vector<Edge*> nearEdges;
	roadNetwork.getNearEdges(succPt->lat, succPt->lon, threshold, nearEdges);
	int candidateEdgeId = -1;
	double candidateIntersectX = -1.0;
	double candidateIntersectY = -1.0;
	double minDist = INFINITE;
	for each (Edge* edge in nearEdges)
	{
		Figure::iterator ptIter = edge->figure->begin(), nextPtIter = ptIter;
		nextPtIter++;
		while (nextPtIter != edge->figure->end())
		{
			GeoPoint* edgePt = *ptIter;
			GeoPoint* nextEdgePt = *nextPtIter;
			if ((A * edgePt->lon + B * edgePt->lat + C) * (A * nextEdgePt->lon + B * nextEdgePt->lat + C) < 0) //����н���
			{
				//�󽻵�
				double _A = nextEdgePt->lat - edgePt->lat;
				double _B = -(nextEdgePt->lon - edgePt->lon);
				double _C = edgePt->lat * (nextEdgePt->lon - edgePt->lon)
					- edgePt->lon * (nextEdgePt->lat - edgePt->lat);
				double intersectY = (A * _C - _A * C) / (_A * B - A * _B);
				double intersectX = (-B * intersectY - C) / A;
				//�ж��Ƿ���prePt->succPt�ӳ��߷���
				double preToSuccX = succPt->lon - prePt->lon;
				double preToSuccY = succPt->lat - prePt->lat;
				double succToIntersectX = intersectX - succPt->lon;
				double succToIntersectY = intersectY - succPt->lat;
				if (preToSuccX * succToIntersectX > 0 && preToSuccY * succToIntersectY > 0)//�������ӳ��߷���
				{
					double tmpDist = succPt->distM(intersectY, intersectX);
					if (tmpDist < threshold && tmpDist < minDist)
					{
						minDist = tmpDist;
						candidateEdgeId = edge->id;
						candidateIntersectX = intersectX;
						candidateIntersectY = intersectY;
						onExtend = true;
					}
				}
				else if (preToSuccX * succToIntersectX < 0 && preToSuccY * succToIntersectY < 0)//������pre��succ֮��
				{
					double tmpDist = succPt->distM(intersectY, intersectX);
					if (tmpDist < threshold && tmpDist < minDist)
					{
						minDist = tmpDist;
						candidateEdgeId = edge->id;
						candidateIntersectX = intersectX;
						candidateIntersectY = intersectY;
						onExtend = false;
					}
				}
			}
			ptIter++;
			nextPtIter++;
		}
	}
	if (candidateEdgeId == -1) //��Χ��û������������·��
		return -1;
	else
	{
		//cout << "candidateEdgeId" << candidateEdgeId << endl;
		//printf("%.8lf,%.8lf", candidateIntersectY, candidateIntersectX);
		//system("pause");
		return roadNetwork.splitEdge(candidateEdgeId, candidateIntersectY, candidateIntersectX);
	}

}

Edge* addNewPolyLineIntoMap(PolylineGenerator& pg)
{
	//create figure
	//ofstream ofs("road.txt");
	//ofs << fixed << showpoint << setprecision(8);
	Figure* newFigure = new Figure();
	for (int i = 0; i < pg.polyline.size(); i++)
	{
		GeoPoint* pt = new GeoPoint(md.screenToGeo((int)pg.polyline[i].x, (int)pg.polyline[i].y));
		newFigure->push_back(pt);
	}
	//connection part
	GeoPoint* firstPt = newFigure->front();
	GeoPoint* lastPt = newFigure->back();
	double intersectionThres = 30; //Ѱ���������Χ����û��·�ڵ�,�о�ֱ������
	double splitRoadThres = 80; //�ӳ�����·�εĽ����������С�������ֵ,����˭������
	//connectivity of the first
	vector<Edge*> nearEdges;
	roadNetwork.getNearEdges(firstPt->lat, firstPt->lon, intersectionThres, nearEdges);
	int minIntersectDist = INFINITE;
	int startNodeId = -1;
	for (int i = 0; i < nearEdges.size(); i++)
	{
		double distToStartNode = firstPt->distM(roadNetwork.nodes[nearEdges[i]->startNodeId]);
		if (distToStartNode < intersectionThres && distToStartNode < minIntersectDist)
		{
			minIntersectDist = distToStartNode;
			startNodeId = nearEdges[i]->startNodeId;
		}
		double distToEndNode = firstPt->distM(roadNetwork.nodes[nearEdges[i]->endNodeId]);
		if (distToEndNode < intersectionThres && distToEndNode < minIntersectDist)
		{
			minIntersectDist = distToEndNode;
			startNodeId = nearEdges[i]->endNodeId;
		}
	}
	if (startNodeId != -1) //�ڷ�Χ���ҵ���intersection������
	{
		newFigure->push_front(roadNetwork.nodes[startNodeId]);
	}
	else
	{
		Figure::iterator firstSuccPtIter = newFigure->begin();
		firstSuccPtIter++;
		GeoPoint* firstSuccPt = *firstSuccPtIter; //newFigure��һ����ĺ�̵�
		bool onExtend;
		int newNodeId = extendAndSplitEdge(firstSuccPt, firstPt, splitRoadThres, onExtend);
		if (newNodeId != -1) //�ҵ�������Ҫ���·��
		{
			if (onExtend)
			{
				newFigure->push_front(roadNetwork.nodes[newNodeId]);
			}
			else
			{
				newFigure->front() = roadNetwork.nodes[newNodeId];
			}			
			startNodeId = newNodeId;
		}
		else //û������Ҫ���·��,����
		{
			startNodeId = roadNetwork.insertNode(newFigure->front()->lat, newFigure->front()->lon);
		}
	}

	//connectivity of the last
	nearEdges.clear();
	roadNetwork.getNearEdges(lastPt->lat, lastPt->lon, intersectionThres, nearEdges);
	minIntersectDist = INFINITE;
	int endNodeId = -1;
	for (int i = 0; i < nearEdges.size(); i++)
	{
		double distToStartNode = lastPt->distM(roadNetwork.nodes[nearEdges[i]->startNodeId]);
		if (distToStartNode < intersectionThres && distToStartNode < minIntersectDist)
		{
			minIntersectDist = distToStartNode;
			endNodeId = nearEdges[i]->startNodeId;
		}
		double distToEndNode = lastPt->distM(roadNetwork.nodes[nearEdges[i]->endNodeId]);
		if (distToEndNode < intersectionThres && distToEndNode < minIntersectDist)
		{
			minIntersectDist = distToEndNode;
			endNodeId = nearEdges[i]->endNodeId;
		}
	}
	if (endNodeId != -1) //�ڷ�Χ���ҵ���intersection������
	{
		newFigure->push_back(roadNetwork.nodes[endNodeId]);
	}
	else
	{
		Figure::iterator lastPrePtIter = newFigure->end();
		lastPrePtIter--; lastPrePtIter--;
		GeoPoint* lastPrePt = *lastPrePtIter; //newFigure���һ�����ǰ�̵�
		bool onExtend;
		int newNodeId = extendAndSplitEdge(lastPrePt, lastPt, splitRoadThres, onExtend);
		if (newNodeId != -1) //�ҵ�������Ҫ���·��
		{
			if (onExtend)
			{
				newFigure->push_back(roadNetwork.nodes[newNodeId]);
			}
			else
			{
				newFigure->back() = roadNetwork.nodes[newNodeId];
			}
			endNodeId = newNodeId;
		}
		else //û������Ҫ���·��,����
		{
			endNodeId = roadNetwork.insertNode(newFigure->back()->lat, newFigure->back()->lon);
		}
	}
	//����˫��·
	Figure* newFigureReverse = new Figure();
	for (Figure::iterator iter = newFigure->begin(); iter != newFigure->end(); iter++)
	{
		newFigureReverse->push_front(*iter);
	}
	int newEdgeid = roadNetwork.insertEdge(newFigure, startNodeId, endNodeId);
	int newEdgeidR = roadNetwork.insertEdge(newFigureReverse, endNodeId, startNodeId);
	return roadNetwork.edges[newEdgeid];
	//drawOneTraj(Color::Black, map.edges[newEdgeid]->figure);
	/*for (int row = 0; row < map.gridHeight; row++)
	{
	for (int col = 0; col < map.gridWidth; col++)
	{
	for each (Edge* edge in *map.grid[row][col])
	{
	if (edge->id == newEdgeid)
	{
	printf("[%d, %d]\n", row, col);
	}
	}
	}
	}
	cout << gridWidth << endl;
	for (int row = 0; row < map.gridHeight; row++)
	{
	for (int col = 0; col < map.gridWidth; col++)
	{
	for each (Edge* edge in *map.grid[row][col])
	{
	if (edge->id == newEdgeidR)
	{
	printf("[%d, %d]\n", row, col);
	}
	}
	}
	}*/

	/*for (Figure::iterator ptIter = newFigure->begin(); ptIter != newFigure->end(); ptIter++)
	{
	ofs << (*ptIter)->lat << " " << (*ptIter)->lon << endl;
	}
	ofs.close();
	exit(0);*/
	//map.drawMap(Color::Blue, md);
}

Edge* genPolyLine(Cluster& cluster)
{
	//////////////////////////////////////////////////////////////////////////
	///��cluster�еĹ켣������·,������·ָ��(��·��˫���,ֻ����һ��)
	///������·��,��cluster�й켣ɾ�� [TODO: �Ƿ�Ҫ�ŵ�reMM��������ɾ��ֵ����ȶ]
	//////////////////////////////////////////////////////////////////////////
	PolylineGenerator pg;
	list<Pt> pts;
	for each (IndexedTraj* iTraj in cluster)
	{
		for each (GeoPoint* gPt in *(iTraj->traj))
		{
			Pt tmpPt;
			Point gdiPt = md.geoToScreen(gPt->lat, gPt->lon);
			tmpPt.x = (double)gdiPt.X;
			tmpPt.y = (double)gdiPt.Y;
			pts.push_back(tmpPt);
		}
	}
	pg.genPolyline(pts);
	for (int i = 0; i < 9; i++)
	{
		pg.optimization_();
	}
	//drawPolyline(pg);
	Edge* newEdge = addNewPolyLineIntoMap(pg);
	//ɾ��cluster�еĹ켣�Լ����cluster
	//[TODO]��ȷ����ô���ò���
	for each(IndexedTraj* iTraj in cluster)
	{
		delete iTraj->traj;
		iTraj->traj = NULL;
		iTraj->cluster = NULL;
	}
	cluster.clear();
	gennedEgdes.push_back(newEdge->figure);
	return newEdge;
	//map.drawMap(Color::Blue, md);
	//reMap matching
	/*list<Edge*> result;
	for (Cluster::iterator trajIter = cluster.begin(); trajIter != cluster.end(); trajIter++)
	{
	IndexedTraj* iTraj = *trajIter;
	Traj* traj = iTraj->traj;
	result = MapMatching(*(iTraj->traj), 50.0); //MapMatching
	//��ÿһ����
	Traj::iterator ptIter = traj->begin();
	list<Edge*>::iterator edgeIter = result.begin();
	while (ptIter != traj->end())
	{
	if ((*edgeIter) != NULL) //ƥ��ɹ�
	{
	md.drawBigPoint(Color::Green, (*ptIter)->lat, (*ptIter)->lon);
	}
	else //ƥ��ʧ��
	{
	md.drawBigPoint(Color::Yellow, (*ptIter)->lat, (*ptIter)->lon);
	}
	ptIter++;
	edgeIter++;
	}
	}	*/
}
	//cluster
double dist_old(GeoPoint* pt, Traj* traj)
{
	//////////////////////////////////////////////////////////////////////////
	///���ص�pt���켣traj��ͶӰ���룬����ж��ͶӰ������С�ģ����ûͶӰ���ص��˵���̾���
	///����Ϊ��γ�Ⱦ��룬û�зŴ���׵�λ   
	///TODO:
	///		����汾û��Map.cpp�İ汾��
	//////////////////////////////////////////////////////////////////////////
	Traj::iterator iter = traj->begin();
	Traj::iterator nextIter = traj->begin();
	nextIter++;
	double minDist = 9999;
	while (nextIter != traj->end())
	{
		//��ͶӰ
		if (cosAngle(pt, (*iter), (*nextIter)) <= 0 && cosAngle(pt, (*nextIter), (*iter)) <= 0)
		{
			double A = ((*nextIter)->lat - (*iter)->lat);
			double B = -((*nextIter)->lon - (*iter)->lon);
			double C = (*iter)->lat * ((*nextIter)->lon - (*iter)->lon)
				- (*iter)->lon * ((*nextIter)->lat - (*iter)->lat);
			double dist = abs(A * pt->lon + B * pt->lat + C) / sqrt(A * A + B * B);
			if (minDist > dist)
				minDist = dist;
		}
		iter++;
		nextIter++;
	}
	if (minDist > 9000) //û��ͶӰ�򷵻���̶˵����
	{
		double headDist = sqrt((pt->lat - traj->front()->lat) * (pt->lat - traj->front()->lat)
			+ (pt->lon - traj->front()->lon) * (pt->lon - traj->front()->lon));
		double tailDist = sqrt((pt->lat - traj->back()->lat) * (pt->lat - traj->back()->lat)
			+ (pt->lon - traj->back()->lon) * (pt->lon - traj->back()->lon));
		return headDist < tailDist ? headDist : tailDist;
	}
	else
		return minDist;
}

double distM(GeoPoint* pt, Traj* traj)
{
	//////////////////////////////////////////////////////////////////////////
	///���ص�pt��traj�ľ���
	///���붨��Ϊ��min(�㵽��ͶӰ�ߵ�ͶӰ���룬�㵽������״���ŷ�Ͼ���)
	//////////////////////////////////////////////////////////////////////////
	double minDist = 9999;
	//�����˵����
	for (Traj::iterator iter = traj->begin(); iter != traj->end(); iter++)
	{
		double tmpDist = GeoPoint::distM(pt->lat, pt->lon, (*iter)->lat, (*iter)->lon);
		if (tmpDist < minDist)
			minDist = tmpDist;
	}
	//����ͶӰ����
	Traj::iterator iter = traj->begin();
	Traj::iterator nextIter = traj->begin();
	nextIter++;
	while (nextIter != traj->end())
	{
		//��ͶӰ
		if (cosAngle(pt, (*iter), (*nextIter)) <= 0 && cosAngle(pt, (*nextIter), (*iter)) <= 0)
		{
			double A = ((*nextIter)->lat - (*iter)->lat);
			double B = -((*nextIter)->lon - (*iter)->lon);
			double C = (*iter)->lat * ((*nextIter)->lon - (*iter)->lon)
				- (*iter)->lon * ((*nextIter)->lat - (*iter)->lat);
			double tmpDist = abs(A * pt->lon + B * pt->lat + C) / sqrt(A * A + B * B);
			tmpDist *= GeoPoint::geoScale;
			if (minDist > tmpDist)
				minDist = tmpDist;
		}
		iter++;
		nextIter++;
	}
	return minDist;
}

double hausdorffDist(Traj* traj1, Traj* traj2)
{
	//////////////////////////////////////////////////////////////////////////
	///���������켣��hausdoff����(һ���켣�ϵĵ㵽��һ���켣����Զ����),���ص�λΪ��
	//////////////////////////////////////////////////////////////////////////
	double maxDist = -1.0;
	//��һ�����ڶ���
	for (Traj::iterator iter = traj1->begin(); iter != traj1->end(); iter++)
	{
		double tmpDist = distM((*iter), traj2);
		if (tmpDist > maxDist)
			maxDist = tmpDist;
	}
	//�ڶ�������һ��
	for (Traj::iterator iter = traj2->begin(); iter != traj2->end(); iter++)
	{
		double tmpDist = distM((*iter), traj1);
		if (tmpDist > maxDist)
			maxDist = tmpDist;
	}
	return maxDist;
}

double hausdorffDist(Traj* traj1, Traj* traj2, double threshold)
{
	//////////////////////////////////////////////////////////////////////////
	///���������켣��hausdoff����(һ���켣�ϵĵ㵽��һ���켣����Զ����),���ص�λΪ��
	///�����������з����Ѿ�����threshold��,��ֱ�ӷ���
	//////////////////////////////////////////////////////////////////////////
	double maxDist = -1.0;
	double thresholdDeg = threshold;
	//��һ�����ڶ���
	for (Traj::iterator iter = traj1->begin(); iter != traj1->end(); iter++)
	{
		double tmpDist = distM((*iter), traj2);
		if (tmpDist > maxDist)
		{
			maxDist = tmpDist;
			if (maxDist > thresholdDeg)
				return maxDist;
		}
	}
	//�ڶ�������һ��
	for (Traj::iterator iter = traj2->begin(); iter != traj2->end(); iter++)
	{
		double tmpDist = distM((*iter), traj1);
		if (tmpDist > maxDist)
		{
			maxDist = tmpDist;
			if (maxDist > thresholdDeg)
				return maxDist;
		}
	}
	return maxDist;
}

double hausdorffDistLite(Traj* traj1, Traj* traj2)
{
	//////////////////////////////////////////////////////////////////////////
	///���������켣��hausdoff���뾫��汾��ֻ�������˵�     
	///	TODO:��ʱ����,���޸�
	//////////////////////////////////////////////////////////////////////////
	double maxDist = 0;
	Traj traj1Lite, traj2Lite;
	traj1Lite.push_back(traj1->front());
	traj1Lite.push_back(traj1->back());
	traj2Lite.push_back(traj2->front());
	traj2Lite.push_back(traj2->back());
	return hausdorffDist(&traj1Lite, &traj2Lite);
}

void getNearTrajs(Traj* traj, double thresholdM, list<IndexedTraj*>& dest)
{
	//////////////////////////////////////////////////////////////////////////
	///���ع켣traj��mbr����threshold�׷�Χ������������й켣(����ȷ,�Ⱦ�ȷֵ�Զ�),����dest
	///���Ա�֤���ǲ���dest�ڵĹ켣,���ǵ�iTraj����̾���һ������threshold
	//////////////////////////////////////////////////////////////////////////
	dest.clear();
	int gridSearchRange = int(thresholdM / (gridSizeDeg * GeoPoint::geoScale)) + 1;
	int minRow = INFINITE, maxRow = -1, minCol = INFINITE, maxCol = -1;
	for each (GeoPoint* pt in *traj)
	{
		int row = (pt->lat - minLat) / gridSizeDeg;
		int col = (pt->lon - minLon) / gridSizeDeg;
		if (row < minRow) minRow = row;
		if (row > maxRow) maxRow = row;
		if (col < minCol) minCol = col;
		if (col > maxCol) maxCol = col;
	}
	//�������MBR
	minRow -= gridSearchRange;
	if (minRow < 0) minRow = 0;
	maxRow += gridSearchRange;
	if (maxRow >= gridHeight) maxRow = gridHeight - 1;
	minCol -= gridSearchRange;
	if (minCol < 0) minCol = 0;
	maxCol += gridSearchRange;
	if (maxCol >= gridWidth) maxCol = gridWidth - 1;
	int visitFlag = int(((double)rand()) / RAND_MAX * 99999999);
	double minDist = INFINITE;
	Cluster* destCluster = NULL;
	for (int i = minRow; i <= maxRow; i++)
	{
		for (int j = minCol; j <= maxCol; j++)
		{
			for (list<IndexedTraj*>::iterator iter = grid[i][j].begin(); iter != grid[i][j].end(); iter++)
			{
				if ((*iter)->traj == NULL)
					continue;
				if ((*iter)->traj == traj) //��ֹ���Լ��ӽ�ȥ
					continue;
				//���ε�����δ���ʹ�
				if ((*iter)->flag != visitFlag)
				{
					(*iter)->flag = visitFlag; //���Ϊ�ѷ���
					dest.push_back((*iter));
				}
			}
		}
	}
}

void doCluster(IndexedTraj* iTraj)
{
	//////////////////////////////////////////////////////////////////////////
	///��iTraj���о���
	//////////////////////////////////////////////////////////////////////////
	if (iTraj->traj == NULL)
		return;
	list<IndexedTraj*> nearTrajs;
	getNearTrajs(iTraj->traj, clusterDist, nearTrajs);
	double minDist = INFINITE;
	Cluster* destCluster = NULL;
	for each (IndexedTraj* candidateITraj in nearTrajs)
	{
		if (candidateITraj->cluster != iTraj->cluster)
		{
			double dist = hausdorffDist(iTraj->traj, candidateITraj->traj, clusterDist);
			if (dist < clusterDist && dist < minDist)
			{
				minDist = dist;
				destCluster = candidateITraj->cluster;
				//goto jmp;
			}
		}
	}
	//jmp:
	if (destCluster != NULL)
	{
		Cluster* srcCluster = iTraj->cluster;
		//��srcCluster������й켣������clusterȫ����ΪdestCluter
		for (Cluster::iterator iter = srcCluster->begin(); iter != srcCluster->end(); iter++)
		{
			(*iter)->cluster = destCluster;
		}
		destCluster->splice(destCluster->end(), *srcCluster);
	}
}

void doCluster_v2(IndexedTraj* iTraj)
{
	//////////////////////////////////////////////////////////////////////////
	///��iTraj���о���
	///��������iTraj����С��clusterDistance��clusterȫ��merge����
	//////////////////////////////////////////////////////////////////////////
	if (iTraj->traj == NULL)
		return;
	list<IndexedTraj*> nearTrajs;
	getNearTrajs(iTraj->traj, clusterDist, nearTrajs);
	double minDist = INFINITE;
	list<Cluster*> mergeClusters;
	for each (IndexedTraj* candidateITraj in nearTrajs)
	{
		if (candidateITraj->cluster != iTraj->cluster)
		{
			double dist = hausdorffDist(iTraj->traj, candidateITraj->traj, clusterDist);
			if (dist < clusterDist)
			{
				mergeClusters.push_back(candidateITraj->cluster);
			}
		}
	}
	Cluster* destCluster = iTraj->cluster;
	for each (Cluster* srcCluster in mergeClusters)
	{
		if (srcCluster->size() == 0)
			continue;
		//��srcCluster������й켣������clusterȫ����ΪdestCluter
		for (Cluster::iterator iter = srcCluster->begin(); iter != srcCluster->end(); iter++)
		{
			(*iter)->cluster = destCluster;
		}
		destCluster->splice(destCluster->end(), *srcCluster);
	}
}

void reMM_and_Cluster(Edge* newEdge)
{
	//////////////////////////////////////////////////////////////////////////
	///1.����·newEdge��Χ�Ĺ켣����reMM
	///2.��reMM����֮ǰMM��������ı����Щ�켣����split & extend
	///3.��ԭ�켣��trajs�Ͷ�Ӧ��cluster��ɾ��,Ȼ���¹켣����
	///4.�Դ��������ӹ켣��һ����recluster
	//////////////////////////////////////////////////////////////////////////

	list<IndexedTraj*> nearTrajs;
	getNearTrajs(newEdge->figure, clusterDist, nearTrajs);
	//��ÿһ�������ĺ�ѡ�켣
	for each(IndexedTraj* candidateITraj in nearTrajs)
	{
		//drawOneTraj(Color::Aqua, candidateITraj->traj);
		
		//********rematch********
		//TODO: MM��ѡ����Χ�趨ֵ����ȶ
		list<Edge*> result = MapMatching(*(candidateITraj->traj),30.0);
		bool mmRoadChanged = false; //��־reMM����û�е㱾��û��ƥ���ϵ����ڱ�ƥ������
		//��ÿһ����
		Traj::iterator ptIter = candidateITraj->traj->begin();
		list<Edge*>::iterator edgeIter = result.begin();
		while (ptIter != candidateITraj->traj->end())
		{
			if ((*edgeIter) != NULL) //ƥ��ɹ�
			{
				md.drawBigPoint(Color::Green, (*ptIter)->lat, (*ptIter)->lon);
				if ((*edgeIter)->id != (*ptIter)->mmRoadId)
				{
					(*ptIter)->mmRoadId = (*edgeIter)->id;
					mmRoadChanged = true;
				}
			}
			else //ƥ��ʧ��
			{
				md.drawBigPoint(Color::Red, (*ptIter)->lat, (*ptIter)->lon);
				if ((*ptIter)->mmRoadId != -1)
				{
					(*ptIter)->mmRoadId = -1;
					mmRoadChanged = true;
				}
			}
			ptIter++;
			edgeIter++;
		}
		
		//split & extend
		list<Traj*> newTrajs;
		if (mmRoadChanged)
		{
			doExtendForOneMMTraj(candidateITraj->traj, newTrajs, 20, 30);
			drawOneTraj(Color::Aqua, candidateITraj->traj);
			drawTrajs(Color::Red, newTrajs, false, false);
			//********recluster*******
				//�Ȱ�ԭ���ϵĹ켣ɾ��
			candidateITraj->cluster->remove(candidateITraj);
			delete candidateITraj->traj;
			//delete candidateITraj;
			candidateITraj->traj = NULL;
			candidateITraj->cluster = NULL;
			//��ÿ�����ɵ�reMM�ӹ켣
			for each(Traj* traj in newTrajs)
			{
				//����trajs��clusters
				IndexedTraj* tmpITraj = new IndexedTraj(traj);
				Cluster* tempCluster = new Cluster();
				tempCluster->push_back(tmpITraj);
				tmpITraj->cluster = tempCluster;
				trajs.push_back(tmpITraj);
				clusters.push_back(tempCluster);
				doCluster_v2(tmpITraj);
			}
		}
	} // end of [for each(IndexedTraj* candidateITraj in nearTrajs)]
}

void reMM_and_Cluster_v2(Edge* newEdge)
{
	//////////////////////////////////////////////////////////////////////////
	///1.����·newEdge��Χ�Ĺ켣����reMM
	///2.��reMM����֮ǰMM��������ı����Щ�켣����split & extend
	///3.��ԭ�켣��trajs�Ͷ�Ӧ��cluster��ɾ��,Ȼ���¹켣����
	///4.�Դ��������ӹ켣��һ����recluster
	//////////////////////////////////////////////////////////////////////////

	list<IndexedTraj*> nearTrajs;
	getNearTrajs(newEdge->figure, clusterDist, nearTrajs);
	list<IndexedTraj*> _newTrajs;
	list<Cluster*> newClusters;
	//��ÿһ�������ĺ�ѡ�켣
	//reMM��extend�����newClusters
	for each(IndexedTraj* candidateITraj in nearTrajs)
	{
		//drawOneTraj(Color::Aqua, candidateITraj->traj);

		//********rematch********
		//TODO: MM��ѡ����Χ�趨ֵ����ȶ
		list<Edge*> result = MapMatching(*(candidateITraj->traj), 30.0);
		bool mmRoadChanged = false; //��־reMM����û�е㱾��û��ƥ���ϵ����ڱ�ƥ������
		//��ÿһ����
		Traj::iterator ptIter = candidateITraj->traj->begin();
		list<Edge*>::iterator edgeIter = result.begin();
		while (ptIter != candidateITraj->traj->end())
		{
			if ((*edgeIter) != NULL) //ƥ��ɹ�
			{
				md.drawBigPoint(Color::Green, (*ptIter)->lat, (*ptIter)->lon);
				if ((*edgeIter)->id != (*ptIter)->mmRoadId)
				{
					(*ptIter)->mmRoadId = (*edgeIter)->id;
					mmRoadChanged = true;
				}
			}
			else //ƥ��ʧ��
			{
				md.drawBigPoint(Color::Red, (*ptIter)->lat, (*ptIter)->lon);
				if ((*ptIter)->mmRoadId != -1)
				{
					(*ptIter)->mmRoadId = -1;
					mmRoadChanged = true;
				}
			}
			ptIter++;
			edgeIter++;
		}

		//split & extend
		list<Traj*> newTrajs;
		if (mmRoadChanged)
		{
			doExtendForOneMMTraj(candidateITraj->traj, newTrajs, 20, 30);
			//drawOneTraj(Color::Aqua, candidateITraj->traj);
			//drawTrajs(Color::Red, newTrajs, false, false);
			//********recluster*******
			//�Ȱ�ԭ���ϵĹ켣ɾ��
			candidateITraj->cluster->remove(candidateITraj);
			delete candidateITraj->traj;
			candidateITraj->traj = NULL;
			candidateITraj->cluster = NULL;
			//��candidateITraj���ɵ�ÿ��reMM�ӹ켣
			for each(Traj* traj in newTrajs)
			{
				//����newTrajs��newClusters
				IndexedTraj* tmpITraj = new IndexedTraj(traj);
				Cluster* tempCluster = new Cluster();
				tempCluster->push_back(tmpITraj);
				tmpITraj->cluster = tempCluster;
				_newTrajs.push_back(tmpITraj);
				newClusters.push_back(tempCluster);
			}
		}
	} // end of [for each(IndexedTraj* candidateITraj in nearTrajs)]
	//reCluster
	bool newRoadGenned = true;
	while (newRoadGenned) //��������û����·����������
	{
		newRoadGenned = false;
		//����newTrajs�е�����traj���о���
		for (list<IndexedTraj*>::iterator newTrajIter = _newTrajs.begin(); newTrajIter != _newTrajs.end(); newTrajIter++)
		{
			IndexedTraj* currentITraj = *newTrajIter;
			if (currentITraj == NULL || currentITraj->traj == NULL)
			{
				continue;
			}
			doCluster_v2(currentITraj);
			//���µ���·����
			if (currentITraj->cluster->size() > 6)
			{
				newRoadGenned = true;
				drawTrajs(Color::Green, *(currentITraj->cluster), true, false);
				cout << "����·����!" << endl;
				Edge* newSubEdge = genPolyLine(*(currentITraj->cluster));
				//ReMM
				list<IndexedTraj*> nearTrajs;
				getNearTrajs(newSubEdge->figure, clusterDist, nearTrajs);
				for each(IndexedTraj* candidateITraj in nearTrajs)
				{
					//drawOneTraj(Color::Aqua, candidateITraj->traj);

					//********rematch********
					//TODO: MM��ѡ����Χ�趨ֵ����ȶ
					list<Edge*> result = MapMatching(*(candidateITraj->traj), 30.0);
					bool mmRoadChanged = false; //��־reMM����û�е㱾��û��ƥ���ϵ����ڱ�ƥ������
					//��ÿһ����
					Traj::iterator ptIter = candidateITraj->traj->begin();
					list<Edge*>::iterator edgeIter = result.begin();
					while (ptIter != candidateITraj->traj->end())
					{
						if ((*edgeIter) != NULL) //ƥ��ɹ�
						{
							md.drawBigPoint(Color::Green, (*ptIter)->lat, (*ptIter)->lon);
							if ((*edgeIter)->id != (*ptIter)->mmRoadId)
							{
								(*ptIter)->mmRoadId = (*edgeIter)->id;
								mmRoadChanged = true;
							}
						}
						else //ƥ��ʧ��
						{
							md.drawBigPoint(Color::Red, (*ptIter)->lat, (*ptIter)->lon);
							if ((*ptIter)->mmRoadId != -1)
							{
								(*ptIter)->mmRoadId = -1;
								mmRoadChanged = true;
							}
						}
						ptIter++;
						edgeIter++;
					}

					//split & extend
					list<Traj*> newExtendTrajs;
					if (mmRoadChanged)
					{
						doExtendForOneMMTraj(candidateITraj->traj, newExtendTrajs, 20, 30);
						//drawOneTraj(Color::Aqua, candidateITraj->traj);
						//drawTrajs(Color::Red, newTrajs, false, false);
						//********recluster*******
						//�Ȱ�ԭ���ϵĹ켣ɾ��
						candidateITraj->cluster->remove(candidateITraj);
						delete candidateITraj->traj;
						candidateITraj->traj = NULL;
						candidateITraj->cluster = NULL;
						//��ÿ�����ɵ�reMM�ӹ켣
						for each(Traj* traj in newExtendTrajs)
						{
							//����trajs��clusters
							IndexedTraj* tmpITraj = new IndexedTraj(traj);
							Cluster* tempCluster = new Cluster();
							tempCluster->push_back(tmpITraj);
							tmpITraj->cluster = tempCluster;
							//trajs.push_back(tmpITraj);
							_newTrajs.push_back(tmpITraj);
							newClusters.push_back(tempCluster);
						}
					}
				}
			}
		}
	} // end while
	//��newTrajs��newClusters����trajs��clusters
	for each (IndexedTraj* iTraj in _newTrajs)
	{
		trajs.push_back(iTraj);
	}
	for each (Cluster* cluster in newClusters)
	{
		clusters.push_back(cluster);
	}
}

void doCluster(vector<IndexedTraj*>& trajs)
{
	//////////////////////////////////////////////////////////////////////////
	///�Թ켣���о���,�켣������Ϊvector<IndexedTraj*>���͵�trajs
	///[ע��] ��ʱ�Ȳ���
	//////////////////////////////////////////////////////////////////////////
	cout << ">> start clustering" << endl;
	//initialization
	clusters.clear();
	//��ʼ״̬ÿ���켣����һ��cluster
	for (int i = 0; i < trajs.size(); i++)
	{
		Cluster* tempCluster = new Cluster();
		tempCluster->push_back(trajs[i]);
		trajs[i]->cluster = tempCluster;
		clusters.push_back(tempCluster);
	}
	//��ÿ���켣
	int tenPercent = trajs.size() / 10; //������ʾ����
	for (int iteration = 0; iteration < 1; iteration++)
	{
		for (int k = 0; k < trajs.size(); k++)
		{
			if (k % tenPercent == 0 && k != 0)
			{
				cout << k / tenPercent << "0% done" << endl;
			}
			//range iteration
			//�켣��MBR
			//����ֱ���߶������ִ���
			/*int row1 = (trajs[k]->traj->front()->lat - minLat) / gridSizeDeg;
			int col1 = (trajs[k]->traj->front()->lon - minLon) / gridSizeDeg;
			int row2 = (trajs[k]->traj->back()->lat - minLat) / gridSizeDeg;
			int col2 = (trajs[k]->traj->back()->lon - minLon) / gridSizeDeg;
			int minRow = row1 < row2 ? row1 : row2;
			int maxRow = row1 < row2 ? row2 : row1;
			int minCol = col1 < col2 ? col1 : col2;
			int maxCol = col1 < col2 ? col2 : col1;*/
			//���ڻ����������ִ���
			int minRow = INFINITE, maxRow = -1, minCol = INFINITE, maxCol = -1;
			for each (GeoPoint* pt in (*trajs[k]->traj))
			{
				int row = (pt->lat - minLat) / gridSizeDeg;
				int col = (pt->lon - minLon) / gridSizeDeg;
				if (row < minRow) minRow = row;
				if (row > maxRow) maxRow = row;
				if (col < minCol) minCol = col;
				if (col > maxCol) maxCol = col;
			}			
			//�������MBR
			minRow -= gridSearchRange;
			if (minRow < 0) minRow = 0;
			maxRow += gridSearchRange;
			if (maxRow >= gridHeight) maxRow = gridHeight - 1;
			minCol -= gridSearchRange;
			if (minCol < 0) minCol = 0;
			maxCol += gridSearchRange;
			if (maxCol >= gridWidth) maxCol = gridWidth - 1;
			trajs[k]->flag = k; //���Լ����Ϊ�ѷ���
			double minDist = INFINITE;
			Cluster* destCluster = NULL;
			for (int i = minRow; i <= maxRow; i++)
			{
				for (int j = minCol; j <= maxCol; j++)
				{
					for (list<IndexedTraj*>::iterator iter = grid[i][j].begin(); iter != grid[i][j].end(); iter++)
					{
						//���ε�����δ���ʹ��������켣�ڲ�ͬ��cluster��
						if ((*iter)->flag != k && trajs[k]->cluster != (*iter)->cluster)
						{
							(*iter)->flag = k; //���Ϊ�ѷ���
							double dist = hausdorffDist(trajs[k]->traj, (*iter)->traj, clusterDist);
							if (dist < clusterDist && dist < minDist)
							{
								minDist = dist;
								destCluster = (*iter)->cluster;
								//goto jmp;
							}
						}
					}
				}
			}
		//jmp:
			if (destCluster != NULL)
			{
				Cluster* srcCluster = trajs[k]->cluster;
				//��srcCluster������й켣������clusterȫ����ΪdestCluter
				for (Cluster::iterator iter = srcCluster->begin(); iter != srcCluster->end(); iter++)
				{
					(*iter)->cluster = destCluster;
				}
				destCluster->splice(destCluster->end(), *srcCluster);
				if (destCluster->size() > supportThreshold)
				{
					drawTrajs(Color::Red, *destCluster, true, false);
					genPolyLine(*destCluster);
					return;
				}				
				//cout << "src cluter size = " << srcCluster->size() << endl;
				//cout << "dest cluster size = " << destCluster->size() << endl;
				//system("pause");
			}
		}
	}
	cout << ">> clustering finished" << endl;
}

	//core
void core()
{
	clusterDist = 20;
	cout << "doing core func..." << endl;
	//initialization
	//��ʼ״̬ÿ���켣����һ��cluster
	for (int i = 0; i < trajs.size(); i++)
	{
		Cluster* cluster = new Cluster();
		cluster->push_back(trajs[i]);
		trajs[i]->cluster = cluster;
		clusters.push_back(cluster);
	}
	
	int newEdgeCount = 0;

	int tenPercent = trajs.size() / 10;
	for (int iteration = 0; iteration < 20; iteration++)
	{
		for (int k = 0; k < trajs.size(); k++)
		{
			if (k % tenPercent == 0 && k != 0)
			{
				cout << k / tenPercent << "0% done" << endl;
			}
			/*list<IndexedTraj*> nearTrajs;
			getNearTrajs(trajs[k]->traj, clusterDist, nearTrajs);
			double minDist = 9999;
			Cluster* destCluster = NULL;
			//��ÿһ�������ĺ�ѡ�켣
			for each(IndexedTraj* candidateITraj in nearTrajs)
			{
			double dist = hausdorffDist(trajs[k]->traj, candidateITraj->traj, clusterDist);
			if (dist < clusterDist && dist < minDist)
			{
			minDist = dist;
			destCluster = candidateITraj->cluster;
			//goto jmp;
			}
			}
			//jmp:
			if (destCluster != NULL)
			{
			Cluster* srcCluster = trajs[k]->cluster;
			//��srcCluster������й켣������clusterȫ����ΪdestCluter
			for (Cluster::iterator iter = srcCluster->begin(); iter != srcCluster->end(); iter++)
			{
			(*iter)->cluster = destCluster;
			}
			destCluster->splice(destCluster->end(), *srcCluster);
			}*/
			//simple version
			if (trajs[k]->traj != NULL)
			{
				doCluster(trajs[k]);
				if (trajs[k]->cluster->size() > supportThreshold)
				{
					drawTrajs(Color::Red, *(trajs[k]->cluster), true, false);
					reMM_and_Cluster(genPolyLine(*(trajs[k]->cluster)));
					newEdgeCount++;
					//TODO: need test!
					if (newEdgeCount == 6)
					{
						roadNetwork.drawMap(Color::Blue, md);
						drawAllGennedEdges();
						drawClusteredTrajs();
						return;
					}
				}
			}
		} // end for (int k = 0; k < trajs.size(); k++)
	}
	roadNetwork.drawMap(Color::Blue, md);
	drawClusteredTrajs();
}

void core_v2()
{
	//////////////////////////////////////////////////////////////////////////
	///�Ľ��汾
	//////////////////////////////////////////////////////////////////////////
	clusterDist = 25;
	cout << "doing core2 func..." << endl;
	//initialization
	//��ʼ״̬ÿ���켣����һ��cluster
	for (int i = 0; i < trajs.size(); i++)
	{
		Cluster* cluster = new Cluster();
		cluster->push_back(trajs[i]);
		trajs[i]->cluster = cluster;
		clusters.push_back(cluster);
	}

	int newEdgeCount = 0;

	int tenPercent = trajs.size() / 10;
	for (int iteration = 0; iteration < 1; iteration++)
	{
		for (int k = 0; k < trajs.size(); k++)
		{
			if (k % tenPercent == 0 && k != 0)
			{
				cout << k / tenPercent << "0% done" << endl;
			}
			//simple version
			if (trajs[k]->traj != NULL)
			{
				doCluster_v2(trajs[k]);
				if (trajs[k]->cluster->size() > supportThreshold)
				{
					drawTrajs(Color::Red, *(trajs[k]->cluster), true, false);
					reMM_and_Cluster_v2(genPolyLine(*(trajs[k]->cluster)));
					newEdgeCount++;
					//TODO: need test!
					/*if (newEdgeCount == 10)
					{
						map.drawMap(Color::Blue, md);
						drawAllGennedEdges();
						//drawClusteredTrajs();
						return;
					}*/
				}
			}
		} // end for (int k = 0; k < trajs.size(); k++)
	}
	roadNetwork.drawMap(Color::Blue, md);
	drawAllGennedEdges();
	roadNetwork.drawMap(Color::Blue, md);
	//drawClusteredTrajs();
}

void drawTrajPts(vector<IndexedTraj*>& trajs)
{
	for each(IndexedTraj* iTraj in trajs)
	{
		Traj* traj = iTraj->traj;
		for (Traj::iterator ptIter = traj->begin(); ptIter != traj->end(); ptIter++)
		{
			if ((*ptIter)->mmRoadId != -1) //ƥ��ɹ�
			{
				md.drawPoint(Color::Green, (*ptIter)->lat, (*ptIter)->lon);
			}
			else //ƥ��ʧ��
			{
				md.drawPoint(Color::Red, (*ptIter)->lat, (*ptIter)->lon);
			}
		}
	}
}

void MM(list<Traj*>& unmatchedTrajs, bool doOutput = false, string outPutFilePath = "")
{
	//////////////////////////////////////////////////////////////////////////
	///<����>��unmatchedTrajs�е�ÿ���켣����MapMatching
	///doOutput:������ļ�����
	///outPutFileName: ����ļ�·��
	//////////////////////////////////////////////////////////////////////////
	int count = 0;
	ofstream fout;
	if (doOutput)
	{
		fout.open("wy_MMTrajs.txt");
		fout << fixed << showpoint << setprecision(8);
	}
	cout << ">> starting MapMatching" << endl
		<< unmatchedTrajs.size() << " trajs in total" << endl;
	//��ÿһ���켣
	for (list<Traj*>::iterator trajIter = unmatchedTrajs.begin(); trajIter != unmatchedTrajs.end(); trajIter++, count++)
	{
		list<Edge*> result = MapMatching(*(*trajIter), 50); //MapMatching
		if (count % 500 == 0)
			cout << ">> MM" << count << " finished" << endl;
		Traj* traj = (*trajIter);
		if (traj == NULL)
			continue;
		//��ÿһ����
		Traj::iterator ptIter = traj->begin();
		list<Edge*>::iterator edgeIter = result.begin();
		while (ptIter != traj->end())
		{
			if ((*edgeIter) != NULL) //ƥ��ɹ�
			{
				(*ptIter)->mmRoadId = (*edgeIter)->id;
				if (doOutput)
					fout << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << (*edgeIter)->id << endl;
			}
			else //ƥ��ʧ��
			{
				(*ptIter)->mmRoadId = -1;
				if (doOutput)
					fout << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << -1 << endl;
			}
			ptIter++;
			edgeIter++;
		}
		if (doOutput)
			fout << -1 << endl;
	}
	if (doOutput)
		fout.close();
}

void MM(vector<IndexedTraj*>& unmatchedITrajs, bool doOutput = false)
{
	int count = 0;
	ofstream fout;
	if (doOutput)
	{
		fout.open("wy_MMTrajs.txt");
		fout << fixed << showpoint << setprecision(8);
	}
	cout << ">> starting MapMatching" << endl
		<< unmatchedITrajs.size() << " trajs in total" << endl;
	//��ÿһ���켣
	for (vector<IndexedTraj*>::iterator trajIter = unmatchedITrajs.begin(); trajIter != unmatchedITrajs.end(); trajIter++, count++)
	{
		list<Edge*> result = MapMatching(*(*trajIter)->traj, 50); //MapMatching
		if (count % 500 == 0)
			cout << ">> MM" << count << " finished" << endl;
		Traj* traj = (*trajIter)->traj;
		if (traj == NULL)
			continue;
		//��ÿһ����
		Traj::iterator ptIter = traj->begin();
		list<Edge*>::iterator edgeIter = result.begin();
		while (ptIter != traj->end())
		{
			if ((*edgeIter) != NULL) //ƥ��ɹ�
			{
				(*ptIter)->mmRoadId = (*edgeIter)->id;
				if (doOutput)
					fout << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << (*edgeIter)->id << endl;
			}
			else //ƥ��ʧ��
			{
				(*ptIter)->mmRoadId = -1;
				if (doOutput)
					fout << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << -1 << endl;
			}
			ptIter++;
			edgeIter++;
		}
		if (doOutput)
			fout << -1 << endl;
	}
	if (doOutput)
		fout.close();
}

void testTCC()
{
	PolylineGenerator pg;
	list<Pt> pts;
	ifstream ifs("C:\\Users\\wuhao\\Desktop\\tcc.txt");
	double x, y;
	while (ifs)
	{
		ifs >> x >> y;
		Pt tmpPt(x, y);
		pts.push_back(tmpPt);
		if (ifs.fail())
		{
			break;
		}
	}
	pg.genPolyline(pts);
	for (int i = 0; i < 10; i++)
	{
		pg.optimization_();
	}

	//draw
	MapDrawer md;
	md.setResolution(10000, 10000);
	md.newBitmap();
	md.lockBits();
	//draw pts
	for each (Pt pt in pts)
	{
		md.drawBigPoint(Color::Black, int(pt.x), int(pt.y));
		md.drawBigPoint(Color::Black, int(pt.x) + 1, int(pt.y) + 1);
		md.drawBigPoint(Color::Black, int(pt.x) - 1, int(pt.y) + 1);
		md.drawBigPoint(Color::Black, int(pt.x) + 1, int(pt.y) - 1);
		md.drawBigPoint(Color::Black, int(pt.x) - 1, int(pt.y) - 1);
	}
	//draw polyline
	for (int i = 0; i < pg.polyline.size() - 1; i++)
	{
		md.drawLine(Color::Green, (int)pg.polyline[i].x, (int)pg.polyline[i].y, (int)pg.polyline[i + 1].x, (int)pg.polyline[i + 1].y);
	}
	for (int i = 0; i < pg.polyline.size(); i++)
	{
		md.drawBigPoint(Color::Blue, (int)pg.polyline[i].x, (int)pg.polyline[i].y);
	}
	md.unlockBits();
	md.saveBitmap("testTCC.png");
	exit(0);
}

void testTCCAll()
{
	list<Pt> pts;
	ifstream ifs("C:\\Users\\wuhao\\Desktop\\pts.txt");
	double x, y;
	MapDrawer md;
	md.setResolution(1000, 1000);
	md.newBitmap();
	md.lockBits();
	while (ifs)
	{
		ifs >> x;
		if (x < 0)
		{
			if (pts.size()>0)
			{
				Color color = randomColor();
				for each (Pt pt in pts)
				{
					md.drawBigPoint(color, int(pt.x), int(pt.y));
					/*md.drawBigPoint(color, int(pt.x) + 1, int(pt.y) + 1);
					md.drawBigPoint(color, int(pt.x) - 1, int(pt.y) + 1);
					md.drawBigPoint(color, int(pt.x) + 1, int(pt.y) - 1);
					md.drawBigPoint(color, int(pt.x) - 1, int(pt.y) - 1);*/
				}
				PolylineGenerator pg;
				pg.genPolyline(pts);
				for (int i = 0; i < 10; i++)
				{
					pg.optimization_();
				}
				for (int i = 0; i < pg.polyline.size() - 1; i++)
				{
					md.drawLine(color, (int)pg.polyline[i].x, (int)pg.polyline[i].y, (int)pg.polyline[i + 1].x, (int)pg.polyline[i + 1].y);
				}
				for (int i = 0; i < pg.polyline.size(); i++)
				{
					md.drawBigPoint(Color::Blue, (int)pg.polyline[i].x, (int)pg.polyline[i].y);
				}
				pts.clear();
			}
		}
		else
		{
			ifs >> y;
			Pt tmpPt(x/10, y/10);
			pts.push_back(tmpPt);
		}
		/*if (ifs.fail())
		{
			break;
		}*/
	}
	

	//draw

	
	//draw polyline
	
	md.unlockBits();
	md.saveBitmap("testTCC_2.png");
	exit(0);
}

void drawIntersection(list<Traj*> trajs)
{
	double steadySpeed = 0.001;
	//�Ȼ�MMƥ��ʧ�ܵĵ�
	for each (Traj* traj in trajs)
	{
		Traj::iterator ptIter = traj->begin();
		for (; ptIter != traj->end(); ptIter++)
		{
			GeoPoint* pt = (*ptIter);
			if (md.inArea(pt->lat, pt->lon) && pt->mmRoadId == -1)
			{
				md.drawBigPoint(Color::Red, pt->lat, pt->lon);
			}
		}
	}

	for each (Traj* traj in trajs)
	{
		Traj::iterator ptIter = traj->begin(); ptIter++;
		Traj::iterator preIter = traj->begin();

		double steadyTime = 0;
		double steadyTimeThreshold = 150;
		bool steadyStatus = false;
		for (; ptIter != traj->end(); ptIter++, preIter++)
		{
			GeoPoint* pt = (*ptIter), *prePt = (*preIter);
			if (md.inArea(pt->lat, pt->lon))
			{
				double dist = GeoPoint::distM(pt, prePt);
				if (dist < 5.0 && abs(pt->time - prePt->time) < steadyTimeThreshold && dist / abs(pt->time - prePt->time) < steadySpeed)
				{
					if (steadyStatus == false)
					{
						steadyStatus = true;
						steadyTime = abs(pt->time - prePt->time);
					}
					else
					{
						steadyTime += abs(pt->time - prePt->time);
					}
				}
				else
				{
					if (steadyStatus)
					{
						if (steadyTime < steadyTimeThreshold && steadyTime > 10)
						{
							md.drawBigPoint(Color::Black, prePt->lat, prePt->lon);
						}
						steadyStatus = false;
					}
				}
			}
		}
	}
}

/////////////////////////////////////////�ҵ��㷨///////////////////////////////////////////


void mFirstClust()
{
	//////////////////////////////////////////////////////////////////////////
	///��û��ƥ��Ĺ켣��۳�һ��������
	//////////////////////////////////////////////////////////////////////////
	
}


void myMain()
{

}

void main()
{

	int startTime = clock();
	srand((unsigned)time(NULL));

	testTCCAll();
/**********************************************************/
/*test code starts from here*/
	/*Map m;
	m.open("D:\\trajectory\\washington_data\\washington_map\\",10000);
	system("pause");
	MapDrawer md;
	md.setArea(m.minLat, m.maxLat, m.minLon, m.maxLon);
	md.setResolution(15000);
	md.newBitmap();
	md.lockBits();
	m.drawMap(Color::Blue, md);
	md.unlockBits();
	md.saveBitmap("washington.png");
	system("pause");
	exit(0);*/
	/*test code ends*/
/**********************************************************/

//=======================================initialization start========================================//
	string mapFilePath = "D:\\trajectory\\singapore_data\\singapore_map\\WA_EdgeGeometry.txt";
	string trajDir = "D:\\trajectory\\singapore_data\\20110102_03\\";	
	vector<string> trajFolders;
	string trajFileName;
	//trajFolders.push_back(trajDir + "20110110_11\\");
	//trajFolders.push_back(trajDir + "20110120_21\\");
	md.setArea(minLat, maxLat, minLon, maxLon);
	md.setResolution(size);
		
	/**********************************************************/
	/*test code starts from here*/
	/*createGridIndex();
	md.setArea(minLat, maxLat, minLon, maxLon);
	md.setResolution(size);
	md.newBitmap();
	md.lockBits();
	GeoPoint* pt = new GeoPoint(1.44570000, 103.77409000);

	vector<Edge*> nearEdges = map.getNearEdges(pt->lat, pt->lon, 50);
	printf("nearEdge count = %d", nearEdges.size());

	md.drawMap(Gdiplus::Color::Gray, mapFilePath);
	for (int i = 0; i < nearEdges.size(); i++)
	{
	drawOneTraj(Gdiplus::Color::Blue, nearEdges[i]->figure);
	}
	int songshen[] = { 7169, 7199, 7252, 7253, 7254, 7387, 7635, 7815, 8074, 8075, 8076, 8077, 8115, 8116, 8117, 8118,
	8336, 8337, 8411, 8467, 8530, 8542, 8552, 8562, 35116, 35145, 35198, 35199, 35200, 35333, 35581, 35762, 36034,
	36035, 36036, 36037, 36053, 36054, 36055, 36056, 36281, 36282, 36357, 36413, 36476, 36488, 36498, 36508 };
	for (int i = 0; i < 48; i++)
	{
	drawOneTraj(randomColor(), map.edges[songshen[i]]->figure);
	}
	drawGridLine(Gdiplus::Color::Green);
	md.drawBigPoint(Gdiplus::Color::Firebrick, pt->lat, pt->lon);
	system("pause");
	md.unlockBits();
	md.saveBitmap("test.png");
	system("pause");
	exit(0);*/
	/*test code ends*/
	/**********************************************************/
	
	/*zooming part start*/
	zoomed = true;
	if (zoomed)
	{
		//int zoomWide = 800;
		//int zoomHeight = 600;
		int zoomWide = 1600;
		int zoomHeight = 1300;
		gridWidth = int(double(zoomWide) / double(size) * double(gridWidth));
		size = 5000;
		//md.zoomIn(8900, 7150, zoomWide, zoomHeight, size);
		md.zoomIn(6500, 6800, zoomWide, zoomHeight, size);
		printf("zoomed: minlat:%lf,maxlat:%lf,minlon:%lf,maxlon:%lf\nresolution:%d*%d\n", md.minLat, md.maxLat, md.minLon, md.maxLon, md.r_width, md.r_height);
		minLat = md.minLat;
		maxLat = md.maxLat;
		minLon = md.minLon;
		maxLon = md.maxLon;		
	}
	/*zooming part end*/

	roadNetwork.setArea(md);
	roadNetwork.open("D:\\trajectory\\singapore_data\\singapore_map\\", (int)(500.0 * 1600.0/ 15000.0));
	printf("\n");
	trajDir += "4000\\";

/*��������������������������������������������������������������������������������initialization end��������������������������������������������������������������������������������*/

	/*һ����*/
	//�������Ե���doExtend��ʹ�õĵ�������limit����
	if (0)
	{
		limitSpeed = 50; //�������33m/s��prune��
		limitDist = 400; //�켣��֮�䳬��300m��prune��
		limitTime = 100; //�����������60���prune��
		double minTrajDist = 40;
		double extendDist = 30;
		trajFileName = "wy_MMTrajs.txt";
		readStdTrajs(trajDir + trajFileName, tempTrajs);
		list<Traj*> extendTrajs;
		doExtend(tempTrajs, extendTrajs, extendDist, true);
		//��
		md.newBitmap();
		md.lockBits();
		drawTrajs(Color::Red, extendTrajs, true, false);
		createGridIndex(createGridIndexForOneTraj);
		drawGridLine(Color::Green);
		md.drawMap(Color::Blue, mapFilePath);
		md.unlockBits();
		md.saveBitmap("map.png");
		system("pause");
		exit(0);		
	}

	/*����ʮ��·�ڵ�*/
	if (1)
	{
		trajFileName = "20110102_03.txt";
		readStdTrajs(trajDir + trajFileName, tempTrajs);
		cout << "tempTrajs size = " << tempTrajs.size() << endl;
		
		md.newBitmap();
		md.lockBits();
		md.drawMap(Color::Blue, mapFilePath);
		drawIntersection(tempTrajs);
		md.unlockBits();
		md.saveBitmap("intersection.png");
		system("pause");
		exit(0);
	}


	/*��SRC�켣ת��std�켣��ʽ*/
	if (0)
	{
		for (int i = 0; i < trajFolders.size(); i++)
			scanTrajFolder(trajDir, readSRCTrajs);
		cout << "raw trajs's size = " << rawTrajs.size() << endl;
		//doExtend(rawTrajs, tempTrajs, true);
		genStdTrajFile(rawTrajs, trajDir + "20110102_03.txt");
		system("pause");
		exit(0);
	}
	
	/*Ԥ����SRC�켣,����ָ�,��TrajFoloder����splitedTrajs.txt*/
	if (0)
	{
		splitSRCTrajFiles(trajDir);
		system("pause");
		exit(0);
	}	

	/*��SRC�Ĺ켣��ʽ����extendTraj*/	
	if (0)
	{
		for (int i = 0; i < trajFolders.size(); i++)
			scanTrajFolder(trajFolders[i], readSRCTrajs);
		cout << "raw trajs's size = " << rawTrajs.size() << endl;
		doExtend(rawTrajs, tempTrajs, true);
		system("pause");
		exit(0);
	}

	/*��std�켣��ʽ(wy_MMTraj.txt)����extendTraj*/
	if (0)
	{
		double extendDist = 30;
		trajFileName = "wy_MMTrajs.txt";
		readStdTrajs(trajDir + trajFileName, rawTrajs);
		cout << "MMed trajs's size = " << rawTrajs.size() << endl;
		doExtend(rawTrajs, tempTrajs, extendDist, true);
		system("pause");
		exit(0);
	}

/////////////////////////////////����켣�ļ�///////////////////////////////////////
	//trajFileName = "wy_extended_unmatched_trajs_smallarea.txt";
	//trajFileName = "splitedTrajs.txt";
	trajFileName = "wy_MMTrajs.txt";
	readStdTrajs(trajDir + trajFileName, trajs);
	cout << "extacted unmatched traj's size = " << trajs.size() << endl;
//////////////////////////////////////////////////////////////////////////////////


	/*MM using viterbi and output in Project Folder*/	
	if (0)
	{
		bool doOutput = true;
		MM(trajs, doOutput);
		system("pause");
		exit(0);
	}
	
	
	/**********************************************************/
	/*test code starts from here*/
	//��ʹ��viterbi MM��Ĺ켣��
	if (1)
	{
		md.newBitmap();
		md.lockBits();
		md.drawMap(Color::Blue, mapFilePath);
		cout << trajs.size() << endl;
		drawTrajPts(trajs);
		//createGridIndex(createGridIndexForOneTraj);
		//drawGridLine(Color::Green);
		md.unlockBits();
		md.saveBitmap("shzrjj.png");
		system("pause");
		exit(0);
	}	
	/*test code ends*/
	/**********************************************************/
	

	/**********************************************************/
	/*test code starts from here*/
	//��MMʧ�ܵĹ켣
	if (0)
	{
		md.newBitmap();
		md.lockBits();
		md.drawMap(Color::Blue, mapFilePath);
		drawTrajs(Color::Red, trajs, true, false);
		createGridIndex(createGridIndexForOneTraj);
		drawGridLine(Color::Green);
		md.unlockBits();
		md.saveBitmap("map.png");
		system("pause");
		exit(0);
	}	
	/*test code ends*/
	/**********************************************************/

	createGridIndex(createGridIndexForOneTraj);
	cout << endl;

	
	//readStdTrajs("C:\\Users\\wuhao\\Desktop\\folder\\split\\splitedTrajs.txt", tempTrajs);
	//rawMapMatching(tempTrajs, "C:\\Users\\wuhao\\Desktop\\folder\\split\\MMTrajs.txt");
	//system("pause");
	//exit(0);
	
	/**********************************************************/
	/*test code starts from here*/
	/*for (list<Traj*>::iterator trajIter = tempTrajs.begin(); trajIter != tempTrajs.end(); trajIter++)
	{
		Traj::iterator ptIter = (*trajIter)->begin(), nextPtIter = ptIter;
		nextPtIter++;
		while (1)
		{
			if (nextPtIter == (*trajIter)->end())
				break;
			if (//overDistLimit(*ptIter, *nextPtIter))
				GeoPoint::distM(*ptIter, *nextPtIter) > 1000)
			{
				cout << "over dist!";
				printf("pt(%.8lf,%.8lf) next(%.8lf,%.8lf)\n", (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
				printf("dist = %lf\n", GeoPoint::distM(*ptIter, *nextPtIter));
				system("pause");
			}
			ptIter++;
			nextPtIter++;
		}
	}*/
	/*test code ends*/
	/**********************************************************/
	

	//drawing part
	cout << ">> start drawing..." << endl;
	md.newBitmap();
	md.lockBits();
	drawGridLine(Gdiplus::Color::Green);
	//md.drawMap(Gdiplus::Color::Blue, mapFilePath);
	core_v2();
	//map.drawMap(Color::Blue, md);
	//drawClusteredTrajs();	
	md.unlockBits();
	
	
	
	//�����ļ���
	/*string pngName =
		"trajSize=" + StringOperator::intToString(rawTrajs.size()) +
		"_speed=" + StringOperator::doubleToString(limitSpeed) +
		"_time=" + StringOperator::doubleToString(limitTime) +
		"_dist=" + StringOperator::doubleToString(limitDist);
	if (zoomed)
	{
		pngName += " [zoom in]";
	}
	pngName += ".png";*/
	string pngName = "map.png";
	md.saveBitmap(pngName);
	cout << ">> drawing finished, output to " + pngName << endl;
	int endTime = clock();
	cout << "running time: " << (endTime - startTime) / 1000.0 << "s" << endl;
	system("pause");
}
