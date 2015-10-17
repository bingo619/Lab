/* 
 * Last Updated at [2015/9/56 9:56] by wuhao
 * version 2.1.2
 * comments:����ȡ�켣����Ϊ-1�ĳ�Ϊ�ɶ����⸺�� @2015/9/29
 *			���Զ��ر��ļ��������Ա���� @2015/8/16  
 *			���Ӷ������GeoPoint���� @2015/2/13
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
	void readGeoPoints(list<GeoPoint*>& dest, Area* area = NULL, int count = INF); //������area�ڵ�count��GeoPoint(dest�������)��Ĭ��Ϊarea = NULL��ʾ����������ȫ������, ���ر��ļ�
	void close();
	
	static void outputTrajs(list<Traj*>& trajs, string filePath, int count = INF); //��trajs��count���켣���ձ�׼��ʽ�����filePath
	static void outputPts(list<GeoPoint*>& pts, string filePath, int count = INF); //��pts��count���㰴�ձ�׼��ʽ�����filePath

private:
	ifstream trajIfs;
};