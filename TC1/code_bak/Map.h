/* 
 * Last Updated at [2014/1/24 22:23] by wuhao
 */
#pragma once
#include "GeoPoint.h"
#include <list>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
using namespace std;
#define min(a,b)	(((a) < (b)) ? (a) : (b))
#define max(a,b)	(((a) > (b)) ? (a) : (b))
typedef list<GeoPoint*> Figure; //����һ��·�Σ�ÿ��GeoPoint*����·�ε㣬��β�ڵ㼴·�������˵�
typedef std::pair<double, double> simplePoint; //�ڲ����ͣ���Ķ�

struct  Edge 
{
	Figure* figure;  //·����Ϣ
	double lengthM;  //��¼·���ܳ�����λΪm
	int startNodeId; //·����ʼ��id
	int endNodeId;  //·����ֹ��id
	bool visited;  //�����ֶΣ��ⲿ������Ķ�
	int id;
};

struct AdjNode //�ڽӱ���
{
	int endPointId;
	int edgeId;
	AdjNode* next;
};

class Map
{
public:
	vector<Edge*> edges; //�������бߵļ��ϣ�����ߵ������˵�����һ�����ڷ�Χ����ΪNULL��������������ֶ��ж�NULL��
	vector<GeoPoint*> nodes; //�������е�ļ���,����㲻�ڷ�Χ����ΪNULL��������������ֶ��ж�NULL��
	vector<AdjNode*> adjList; //�ڽӱ�
	
	Map(string folderDir, int gridWidth);  //��gridWidth�е���������������

	vector<Edge*> getNearEdges(double lat, double lon, double threshold) const; //���ؾ���(lat, lon)���ϸ�С��threshold�׵�����Edge*
	double distM(double lat, double lon, Edge* edge) const; //����(lat,lon)�㵽edge�ľ��룬��λΪ��
	double distM(double lat, double lon, Edge* edge, double& prjDist) const;//ͬ�ϣ�ͬʱ��¼ͶӰ�㵽edge���ľ������prjDist����ͶӰ���Ϊ0
	int hasEdge(int startNodeId, int endNodeId) const; //�ж�startNodeId��endNodeId֮�����ޱ�,û�б߷���-1���б߷���edgeId
	void insertNode(double lat, double lon);
	void insertEdge(Edge* edge, int startNodeId, int endNodeId); //�ڵ�ǰͼ�в����
	void delEdge(int edgeId);
	
	

private:
	int gridWidth, gridHeight;
	double gridSizeDeg;
	double strictThreshold = 0.1;
	list<Edge*>* **grid;
	/*double minLat = 1.22;
	double maxLat = 1.5;
	double minLon = 103.620;
	double maxLon = 104.0;*/
	double minLat = 0.99999;
	double maxLat = 1.6265;
	double minLon = 103.548;
	double maxLon = 104.1155;


	int getRowId(double lat) const;
	int getColId(double lon) const;
	double distM_withThres(double lat, double lon, Edge* edge, double threshold) const; //����(lat,lon)�㵽edge�ľ����Ͻ�,��ǰԤ���Ż��汾	
	double calEdgeLength(Figure* figure) const;
	bool inArea(double lat, double lon) const;
	bool inArea(int nodeId) const;
	void createGridIndex();
	void createGridIndexForEdge(Edge *edge);
	void insertEdgeIntoGrid(Edge* edge,int row, int col);
	void insertEdge(int edgeId, int startNodeId, int endNodeId);
	
	void split(const string& src, const string& separator, vector<string>& dest);
	double cosAngle(GeoPoint* pt1, GeoPoint* pt2, GeoPoint* pt3) const;
	void test();
};

