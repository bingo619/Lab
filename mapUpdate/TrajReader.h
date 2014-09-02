/* 
 * Last Updated at [2014/4/21 10:01] by wuhao
 */
#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>
#include "GeoPoint.h"
#include <iomanip>
using namespace std;
#define INF 999999999
#define INVALID_ROADID -1
typedef list<GeoPoint*> Traj;

class TrajReader
{
public:
	TrajReader();
	TrajReader(string filePath); //���캯��ʱ�ʹ򿪹켣�ļ� 
	void open(string filePath); //�򿪹켣�ļ����켣�ļ�·��ΪfilePath
	void readTrajs(vector<Traj*>& dest, int count = INF); //����count���켣����dest(dest�������)��Ĭ��Ϊȫ�����룬���ر��ļ�
	void readTrajs(list<Traj*>& dest, int count = INF); //����count���켣����dest(dest�������)��Ĭ��Ϊȫ�����롣���ر��ļ�
	static void outputTrajs(list<Traj*>& trajs, string filePath, int count = INF); //��trajs��count���켣���ձ�׼��ʽ�����filePath

private:
	//�趨������������Ĺ켣����
	double minLat;
	double maxLat;
	double minLon;
	double maxLon;
	ifstream trajIfs;
};
