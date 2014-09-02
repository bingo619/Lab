/* 
 * Last Updated at [2014/6/26 11:32] by wuhao
 */
#pragma once
#include "GeoPoint.h"
#include <list>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
#include <queue>
#include "MapDrawer.h"
#define eps 1e-10
#define INF  1e7 //���·������
#define MAXSPEED 50 //����ٶ�
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

//���·�������������ݽṹ
struct NODE_DIJKSTRA 
{
	int t; double dist;
	NODE_DIJKSTRA(int i, double dist)
	{
		this->t = i;
		this->dist = dist;
	}
	bool operator < (const NODE_DIJKSTRA &rhs) const 
	{
		return dist > rhs.dist;
	}
};


class Map
{
public:
	vector<Edge*> edges; //�������бߵļ��ϣ�����ߵ������˵�����һ�����ڷ�Χ����ΪNULL��������������ֶ��ж�NULL��
	vector<GeoPoint*> nodes; //�������е�ļ���,����㲻�ڷ�Χ����ΪNULL��������������ֶ��ж�NULL��
	vector<AdjNode*> adjList; //�ڽӱ�
	
	void setArea(Area* area);
	Map(); //Ĭ�Ϲ��캯��,��Ҫ�ֶ�����open()��������ʼ��
	Map(string folderDir, Area* area, int gridWidth = 0);  //��folderDir·���������ͼ�ļ�,����gridWidth�е���������������,gridWidth<=0ʱΪ����������
	void open(string folderDir, int gridWidth = 0);  //��folderDir·���������ͼ�ļ�,����gridWidth�е���������������,�������޲ι��캯��	


	/*�������ܺ���*/
	int hasEdge(int startNodeId, int endNodeId) const; //�ж�startNodeId��endNodeId֮�����ޱ�,û�б߷���-1���б߷���edgeId
	int insertNode(double lat, double lon); //����һ���½��,�����½��id
	int insertEdge(Figure* figure, int startNodeId, int endNodeId); //�ڵ�ǰͼ�в����,�����±�id
	int splitEdge(int edgeId, double lat, double lon); //��edge��(lat,lon)�㴦�ֿ�������,(lat,lon)��Ϊ�½�����,�����½���nodeId
	void getMinMaxLatLon(string nodeFilePath);
	void drawMap(Gdiplus::Color color, MapDrawer& md); //������ͼ
	double distM(double lat, double lon, Edge* edge) const; //����(lat,lon)�㵽edge�ľ��룬��λΪ��

	/*Map Matching��غ���*/
	vector<Edge*> getNearEdges(double lat, double lon, double threshold) const; //���ؾ���(lat, lon)���ϸ�С��threshold�׵�����Edge*,������ڴ�й¶
	void getNearEdges(double lat, double lon, double threshold, vector<Edge*>& dest); //�Ƽ��汾
	void getNearEdges(double lat, double lon, int k, vector<Edge*>& dest); //������(lat, lon)����������k��·�Σ�����dest
	double shortestPathLength(int ID1, int ID2, double dist1, double dist2, double deltaT);//�����·������������ɶ��˼��Ҳ��֪���������ҵĴ���
	double distM(double lat, double lon, Edge* edge, double& prjDist) const;//ͬ�ϣ�ͬʱ��¼ͶӰ�㵽edge���ľ������prjDist����ͶӰ���Ϊ0
	double distMFromTransplantFromSRC(double lat, double lon, Edge* edge, double& prjDist); //��ֲSRC�汾������(lat,lon)�㵽edge�ľ��룬��λΪ�ף�ͬʱ��¼ͶӰ�㵽edge���ľ������prjDist
	
	
	/*ɾ·���*/
	void delEdge(int edgeId, bool delBirectionEdges = true); //�ӵ�ͼ��ɾ��edgeId������ڶ�������Ϊtrue��ͬʱɾ������·
	void drawDeletedEdges(Gdiplus::Color color, MapDrawer& md); //����ɾ����·
	void deleteEdgesRandomly(int delNum, double minEdgeLengthM); //�ڵ�ͼ�����ɾ�����Ȳ�����minEdgeLengthM��delNum��Edge(ͬʱ��ɾ������·)�����������������main�г�ʼ��
	//��ǿ�汾�����ɾ�����Ȳ�����minEdgeLengthM��delNum��Edge��
	//ͬʱɾ����edge�����㵽�����С��aroundThresholdM��·�β�����aroundNumThreshold����˫��·��2�ƣ���
	//��������Χ����Щ·ȫ��ɾ�������������������main�г�ʼ��
	//doOutputΪtrueʱ���������ɾ����·�κ�������ļ���һ��һ��id��
	void deleteEdgesRandomlyEx(int delNum, double minEdgeLengthM, double aroundThresholdM, int aroundNumThreshold, bool doOutput = true);
	void deleteEdges(string path); //����deleteEdgesRandomlyEx���������ɾ����id�ŵ��ļ��������·����ɾ������֤���ô˺������״̬�����deleteEdgesRandomlyEx()���״̬һ��
	vector<Edge*> deletedEdges; //��¼��ɾ�����ı�

private:
	int gridWidth, gridHeight;
	double gridSizeDeg;
	double strictThreshold = 0;
	list<Edge*>* **grid;
	Area* area;
	//һЩ���õ�area�ο�ֵ
	//singapore half
	/*double minLat = 1.22;
	double maxLat = 1.5;
	double minLon = 103.620;
	double maxLon = 104.0;*/
	
	/*singapore full
	double minLat = 0.99999;
	double maxLat = 1.6265;
	double minLon = 103.548;
	double maxLon = 104.1155;*/

	//washington full
	//minLat:45.559102, maxLat : 49.108823, minLon : -124.722781, maxLon : -116.846465
	/*double minLat = 45.0;
	double maxLat = 49.5;
	double minLon = -125.0;
	double maxLon = -116.5;*/

	int getRowId(double lat) const;
	int getColId(double lon) const;
	double distM_withThres(double lat, double lon, Edge* edge, double threshold) const; //����(lat,lon)�㵽edge�ľ����Ͻ�,��ǰԤ���Ż��汾	
	double calEdgeLength(Figure* figure) const;
	bool inArea(double lat, double lon) const;
	void createGridIndex();
	void createGridIndexForEdge(Edge *edge);
	void createGridIndexForSegment(Edge *edge, GeoPoint* fromPT, GeoPoint* toPt);
	void insertEdgeIntoGrid(Edge* edge,int row, int col);
	void insertEdge(int edgeId, int startNodeId, int endNodeId);
	
	void split(const string& src, const string& separator, vector<string>& dest);
	void split(const string& src, const char& separator, vector<string>& dest);
	double cosAngle(GeoPoint* pt1, GeoPoint* pt2, GeoPoint* pt3) const;
	void test();
	void findNearEdges(Edge* edge, list<Edge*>& dest, double thresholdM); //���ؾ���edge���Ȳ�����thresholdM��·��
	double distM(Edge* edge1, Edge* edge2, double threshold);
	void getNearEdges(Edge* edge, double thresholdM, vector<Edge*>& dest);
};

