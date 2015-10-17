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

//最短路径所用
#define INF  1e7 
vector<double> dist;//记录到起点的当前dist
vector<bool> flag;//记录是否在S中
vector<int> preNode;//记录最短路径中前一个节点
vector<int> preEdge;//记录最短路径中前一条边

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
	///求从fromNodeId到toNodeId之间的最短路径,用edgeID按序输出到resultEdgeIDs
	///两点之间连通返回true,不可达返回false
	//////////////////////////////////////////////////////////////////////////
	//initialization
	if (resultEdgeIDs.size() != 0)
	{
		cout << "异常@shortestPath:" << endl;
		system("pause");
	}
	for (int i = 0; i < roadNetwork.nodes.size(); i++) 
	{
		dist[i] = INF;
		flag[i] = false; //未加入S
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
		if (flag[currentNodeId]) //若已加入S,跳过
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
				preNode[i->endPointId] = currentNodeId; //记录前驱
				preEdge[i->endPointId] = i->edgeId; //记录前驱
			}
		}
	}
	vector<int> reverseRoute;
	//还原路径
	int currentNodeId = toNodeID;
	//不可达情况
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
	///为traj进行地图匹配
	///传统按点匹配结果输出到oldOFS
	///新的按路段匹配结果输出到newOFS
	///1)中间有点匹配失败则用前后最短路代替
	///2)如果整条轨迹匹配失败则跳过
	///3)如果只有一个点匹配成功其余都失败则跳过
	///4)全局变量currentId控制轨迹id, oldOFS newOFS输出的轨迹id保持一致
	//////////////////////////////////////////////////////////////////////////
	if (traj == NULL)
		return;
	if (traj->size() <= 2)
		return;
	list<Edge*> result;
	mm.MapMatching(*traj, result, 50.0); //MapMatching
	//////////////////////////////////////////////
	//输出newOFS
	list<Edge*>::iterator edgeIter = result.begin();
	while (*edgeIter == NULL) //防止一开头就是NULL
	{
		edgeIter++;
		if (edgeIter == result.end())
		{
			//cout << "警告@matchForOneTraj: 整条轨迹都没匹配上" << endl;
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
		if ((*edgeIter) == NULL)//匹配失败
		{
			edgeIter++;
			continue;
		}
		currentToID = (*edgeIter)->startNodeId;
		if ((*edgeIter)->endNodeId == currentFromID)//前后两点在同一条路上
		{
			edgeIter++;
			continue;
		}
		else if (currentFromID == currentToID) //前后两个edge相邻
		{
			//do nothing
		}
		else if (roadNetwork.hasEdge(currentFromID, currentToID) != -1) //前后两个edge隔了一条边
		{
			newOFS << roadNetwork.hasEdge(currentFromID, currentToID) << " ";
			//TrajDrawer::drawOneTraj(roadNetwork.edges[roadNetwork.hasEdge(currentFromID, currentToID)]->figure, md, Color::Purple);
		}
		else //前后两点之间无直接边相连
		{
			vector<int> resultRoute;
			bool reachable = shortestPath(currentFromID, currentToID, resultRoute);
			if (reachable == false)
			{
				//cout << "警告@matchForOneTraj: 轨迹中有路段不可达" << endl;
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
	//输出oldOFS
	//对每一个点
	Traj::iterator ptIter = traj->begin();
	edgeIter = result.begin();
	while (ptIter != traj->end())
	{
		if ((*edgeIter) != NULL) //匹配成功
		{
			//(*ptIter)->mmRoadId = (*edgeIter)->id;
			//输入文件的mmRoadId域实际上是speed，所以(*ptIter)->mmRoadId输出的是speed
			oldOFS << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << (*ptIter)->mmRoadId << " " << (*edgeIter)->id << endl;
		}
		else //匹配失败
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
	///匹配一个文件
	//////////////////////////////////////////////////////////////////////////
	cout << "开始处理" << filePath << endl;
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

	//释放内存
	deleteList(trajs);
	cout << filePath << "处理完毕" << endl;
}

void forEXE(int argc, char* argv[])
{
	if (argc != 5)
	{
		MessageBox(NULL, TEXT("参数个数错误"), TEXT("Parameter Exception !"), MB_OK);
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
	cout << "所有文件处理完成" << endl;
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