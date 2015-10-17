#include <iostream>
#include <fstream>
#include "GeoPoint.h"
#include "MapDrawer.h"
#include "Map.h"
#include "TrajReader.h"
#include "MapMatching.h"
#include "TrajDrawer.h"
#include "StringOperator.h"
using namespace std;
using namespace Gdiplus;

//string mapFolder = "D:\\trajectory\\singapore_data\\singapore_map\\old\\";
string mapFolder = "D:\\trajectory\\Porto_data\\map\\partial\\";
string inputFolder;
Map roadNetwork;
MapMatcher mm(&roadNetwork);
MapDrawer md;
//Area area(1.294788, 1.393593, 103.784667, 103.906266); //test
//Area area(1.094788, 1.593593, 103.484667, 104.206266); //singapore full
Area area(41.1393, 41.1607, -8.6325, -8.5962); //porto dense area
list<Traj*> trajs;
int currentId = 1;

//���·������
#define INF  1e7 
vector<double> dist;//��¼�����ĵ�ǰdist
vector<bool> flag;//��¼�Ƿ���S��
vector<int> preNode;//��¼���·����ǰһ���ڵ�
vector<int> preEdge;//��¼���·����ǰһ����

void initialization()
{
	double gridSizeM = 50.0;
	int gridWidth = (area.maxLon - area.minLon) * GeoPoint::geoScale / gridSizeM;
	roadNetwork.setArea(&area);
	roadNetwork.open(mapFolder, gridWidth);

	dist = vector<double>(roadNetwork.nodes.size());
	flag = vector<bool>(roadNetwork.nodes.size());
	preNode = vector<int>(roadNetwork.nodes.size());
	preEdge = vector<int>(roadNetwork.nodes.size());
}

void deleteList(list<Traj*>& victimList)
{
	for each (Traj* traj in victimList)
	{
		for each (GeoPoint* pt in *traj)
		{
			delete pt;
		}
		delete traj;

	}
	victimList.clear();
}

bool shortestPath(int fromNodeID, int toNodeID, vector<int>& resultEdgeIDs)
{
	//////////////////////////////////////////////////////////////////////////
	///���fromNodeId��toNodeId֮������·��,��edgeID���������resultEdgeIDs
	///����֮����ͨ����true,���ɴﷵ��false
	//////////////////////////////////////////////////////////////////////////
	//initialization
	if (resultEdgeIDs.size() != 0)
	{
		cout << "�쳣@shortestPath:" << endl;
		system("pause");
	}
	for (int i = 0; i < roadNetwork.nodes.size(); i++) 
	{
		dist[i] = INF;
		flag[i] = false; //δ����S
		preNode[i] = -1;
		preEdge[i] = -1;
	}
	dist[fromNodeID] = 0;
	priority_queue<NODE_DIJKSTRA> Q;
	NODE_DIJKSTRA tmp(fromNodeID, 0);
	Q.push(tmp);
	while (!Q.empty()) 
	{
		NODE_DIJKSTRA x = Q.top();
		Q.pop();
		int currentNodeId = x.t;
		if (flag[currentNodeId]) //���Ѽ���S,����
		{
			continue;
		}
		flag[currentNodeId] = true;
		if (currentNodeId == toNodeID)
		{
			break;
		}
		//relax
		for (AdjNode* i = roadNetwork.adjList[currentNodeId]->next; i != NULL; i = i->next)
		{
			if (dist[i->endPointId] > dist[currentNodeId] + roadNetwork.edges[i->edgeId]->lengthM) 
			{
				dist[i->endPointId] = dist[currentNodeId] + roadNetwork.edges[i->edgeId]->lengthM;
				NODE_DIJKSTRA tmp(i->endPointId, dist[i->endPointId]);
				Q.push(tmp);
				preNode[i->endPointId] = currentNodeId; //��¼ǰ��
				preEdge[i->endPointId] = i->edgeId; //��¼ǰ��
			}
		}
	}
	vector<int> reverseRoute;
	//��ԭ·��
	int currentNodeId = toNodeID;
	//���ɴ����
	if (abs(dist[toNodeID] - INF) < eps)
	{
		return false;
	}
	while (currentNodeId != fromNodeID)
	{
		reverseRoute.push_back(preEdge[currentNodeId]);
		currentNodeId = preNode[currentNodeId];
	}
	for (int i = reverseRoute.size() - 1; i >= 0; i--)
	{
		resultEdgeIDs.push_back(reverseRoute[i]);
	}
	return true;
}

void matchForOneTraj(ofstream& oldOFS, ofstream& newOFS, Traj* traj, MapMatcher& mm)
{
	//////////////////////////////////////////////////////////////////////////
	///Ϊtraj���е�ͼƥ��
	///��ͳ����ƥ���������oldOFS
	///�µİ�·��ƥ���������newOFS
	///1)�м��е�ƥ��ʧ������ǰ�����·����
	///2)��������켣ƥ��ʧ��������
	///3)���ֻ��һ����ƥ��ɹ����඼ʧ��������
	///4)ȫ�ֱ���currentId���ƹ켣id, oldOFS newOFS����Ĺ켣id����һ��
	//////////////////////////////////////////////////////////////////////////
	if (traj == NULL)
		return;
	if (traj->size() <= 2)
		return;
	list<Edge*> result;
	mm.MapMatching(*traj, result, 50.0); //MapMatching
	//////////////////////////////////////////////
	//���newOFS
	list<Edge*>::iterator edgeIter = result.begin();
	while (*edgeIter == NULL) //��ֹһ��ͷ����NULL
	{
		edgeIter++;
		if (edgeIter == result.end())
		{
			//cout << "����@matchForOneTraj: �����켣��ûƥ����" << endl;
			return;
		}
	}
	newOFS << "-" << StringOperator::intToString(currentId) << " ";
	newOFS << (*edgeIter)->id << " ";
	//TrajDrawer::drawOneTraj((*edgeIter)->figure, md, Color::Green);
	int currentFromID = (*edgeIter)->endNodeId;
	int currentToID;
	edgeIter++;
	while (edgeIter != result.end())
	{
		if ((*edgeIter) == NULL)//ƥ��ʧ��
		{
			edgeIter++;
			continue;
		}
		currentToID = (*edgeIter)->startNodeId;
		if ((*edgeIter)->endNodeId == currentFromID)//ǰ��������ͬһ��·��
		{
			edgeIter++;
			continue;
		}
		else if (currentFromID == currentToID) //ǰ������edge����
		{
			//do nothing
		}
		else if (roadNetwork.hasEdge(currentFromID, currentToID) != -1) //ǰ������edge����һ����
		{
			newOFS << roadNetwork.hasEdge(currentFromID, currentToID) << " ";
			//TrajDrawer::drawOneTraj(roadNetwork.edges[roadNetwork.hasEdge(currentFromID, currentToID)]->figure, md, Color::Purple);
		}
		else //ǰ������֮����ֱ�ӱ�����
		{
			vector<int> resultRoute;
			bool reachable = shortestPath(currentFromID, currentToID, resultRoute);
			if (reachable == false)
			{
				//cout << "����@matchForOneTraj: �켣����·�β��ɴ�" << endl;
				//md.drawBigPoint(Color::Orange, roadNetwork.nodes[currentFromID]->lat, roadNetwork.nodes[currentFromID]->lon);
				//md.drawBigPoint(Color::Purple, roadNetwork.nodes[currentToID]->lat, roadNetwork.nodes[currentToID]->lon);
				//printf("from:%d, to:%d\n", currentFromID, currentToID);
			}
			else
			{
				for (int i = 0; i < resultRoute.size(); i++)
				{
					newOFS << resultRoute[i] << " ";
					//TrajDrawer::drawOneTraj(roadNetwork.edges[resultRoute[i]]->figure, md, Color::Red);
				}
			}			
		}
		newOFS << (*edgeIter)->id << " ";
		//TrajDrawer::drawOneTraj((*edgeIter)->figure, md, Color::Green);// , true);
		currentFromID = (*edgeIter)->endNodeId;
		edgeIter++;
	}
	newOFS << endl;

	//////////////////////////////////////////////
	//���oldOFS
	//��ÿһ����
	Traj::iterator ptIter = traj->begin();
	edgeIter = result.begin();
	while (ptIter != traj->end())
	{
		if ((*edgeIter) != NULL) //ƥ��ɹ�
		{
			//(*ptIter)->mmRoadId = (*edgeIter)->id;
			//�����ļ���mmRoadId��ʵ������speed������(*ptIter)->mmRoadId�������speed
			oldOFS << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << (*ptIter)->mmRoadId << " " << (*edgeIter)->id << endl;
		}
		else //ƥ��ʧ��
		{
			//(*ptIter)->mmRoadId = -1;
			oldOFS << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << (*ptIter)->mmRoadId << " " << -1 << endl;
		}
		ptIter++;
		edgeIter++;
	}
	oldOFS << "-" << StringOperator::intToString(currentId) << endl;
	currentId++;
}

void matchForOneFile(string filePath, int fileId)
{
	//////////////////////////////////////////////////////////////////////////
	///ƥ��һ���ļ�
	//////////////////////////////////////////////////////////////////////////
	cout << "��ʼ����" << filePath << endl;
	ofstream newOFS(StringOperator::intToString(fileId) + "mm_edges.txt");
	ofstream oldOFS(StringOperator::intToString(fileId) + "mm.txt");
	newOFS << fixed << showpoint << setprecision(8);
	oldOFS << fixed << showpoint << setprecision(8);
	TrajReader reader(filePath);
	reader.readTrajs(trajs);
	
	//check
	
	/**********************************************************/
	/*test code starts from here*/
	for each (Traj* traj in trajs)
	{
		for each (GeoPoint* pt in *traj)
		{
			if (!area.inArea(pt->lat, pt->lon))
			{
				printf("not in area");
				pt->print();
				system("pause");
			}
		}
	}
	/*test code ends*/
	/**********************************************************/
	

	for each(Traj* traj in trajs)
	{
		matchForOneTraj(oldOFS, newOFS, traj, mm);
	}
	oldOFS.close();
	newOFS.close();

	//�ͷ��ڴ�
	deleteList(trajs);
	cout << filePath << "�������" << endl;
}

void forEXE(int argc, char* argv[])
{
	if (argc != 5)
	{
		MessageBox(NULL, TEXT("������������"), TEXT("Parameter Exception !"), MB_OK);
		exit(0);
	}
	mapFolder = argv[1];
	inputFolder = argv[2];
	int a = StringOperator::stringToInt(string(argv[3]));
	int b = StringOperator::stringToInt(string(argv[4]));

	initialization();

	for (int i = a; i <= b; i++)
	{
		matchForOneFile(inputFolder + StringOperator::intToString(i) + ".txt", i);
	}
	cout << "�����ļ��������" << endl;
}

void forDebug()
{
	initialization();
	string trajFilePath = "D:\\trajectory\\Porto_data\\TrajsInDA.txt";
	//TrajReader reader();
	//reader.readTrajs(trajs);// , 10000);


	md.setArea(&area);
	md.setResolution(15000);
	md.newBitmap();
	md.lockBits();
	roadNetwork.drawMap(Color::Gray, md);
	/*for each(Traj* traj in trajs)
	{
		TrajDrawer::drawOneTraj(traj, md, Color::Blue);
		//matchForOneTraj(oldOFS, newOFS, traj, mm);
	}*/
	matchForOneFile(trajFilePath, 1);
	md.unlockBits();
	md.saveBitmap("1.png");
}


int main(int argc, char* argv[])
{
	//forEXE(argc, argv);
	forDebug();
}