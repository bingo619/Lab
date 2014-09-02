/* 
 * Last Updated at [2014/6/26 11:32] by wuhao
 */
#include "Map.h"

bool smallerInX(simplePoint& pt1, simplePoint& pt2);
bool smallerInDist(pair<Edge*, double>& c1, pair<Edge*, double>& c2);

//////////////////////////////////////////////////////////////////////////
///public part
//////////////////////////////////////////////////////////////////////////
Map::Map()
{
	area = new Area();
}

Map::Map(string folderDir, Area* area, int gridWidth /* = 0 */ )
{
	this->setArea(area);
	this->open(folderDir, gridWidth);
}

void Map::open(string folderDir, int gridWidth /* = 0 */ )
{
	/*�ļ�Ŀ¼�ṹΪ,�ļ���Ҫ��Ӧ
	* folderDir
	* |-nodeOSM.txt
	* |-edgeOSM.txt
	*/
	//////////////////////////////////////////////////////////////////////////
	///�ų����򣺵�edge�������˵㶼��area���򲻼��루node��edge��Ӧλ�÷�NULL��
	///node�����ڲ���area��ȫ������
	//////////////////////////////////////////////////////////////////////////
	
	this->gridWidth = gridWidth;
	int count = 0;
	//////////////////////////////////////////////////////////////////////////
	//��ȡnodeOSM.txt
	//��ʽ��nodeId \t lat \t lon \n
	//////////////////////////////////////////////////////////////////////////
	ifstream nodeIfs(folderDir + "nodeOSM.txt");
	if (!nodeIfs)
	{
		cout << "open " + folderDir + "nodeOSM.txt" + " error!\n";
		system("pause");
		exit(0);
	}
	while (nodeIfs)
	{
		double lat, lon;
		int nodeId;
		GeoPoint* pt;
		nodeIfs >> nodeId >> lat >> lon;
		if (nodeIfs.fail())
			break;
		if (inArea(lat, lon))
			pt = new GeoPoint(lat, lon);
		else
		{
			//pt = NULL; //[ATTENTION]node�������ⲻ�뱣���ô˾�(edge����ʱ�����bug�������д���)
			pt = new GeoPoint(lat, lon);
			count++;
		}
		nodes.push_back(pt);
	}
	printf("nodes count = %d\n", nodes.size());
	printf("nodes not in area count = %d\n", count);
	nodeIfs.close();

	//////////////////////////////////////////////////////////////////////////
	//��ȡedgeOSM.txt
	//��ʽ��edgeOSM.txt: edgeId \t startNodeId \t endNodeId \t figureNodeCount \t figure1Lat \t figure1Lon \t ... figureNLat \t figureNLon \n  
	//////////////////////////////////////////////////////////////////////////
	count = 0;
	std::ifstream edgeIfs(folderDir + "edgeOSM.txt");
	if (!edgeIfs)
	{
		cout << "open " + folderDir + "WedgeOSM.txt" + " error!\n";
		system("pause");
		exit(0);
	}
	//��ʼ���ڽӱ�
	for (int i = 0; i < nodes.size(); i++)
	{
		AdjNode* head = new AdjNode();
		head->endPointId = i;
		head->next = NULL;
		adjList.push_back(head);
	}
	while (edgeIfs)
	{
		int edgeId, startNodeId, endNodeId, figureCount;
		edgeIfs >> edgeId >> startNodeId >> endNodeId >> figureCount;
		
		Figure* figure = new Figure();
		for (int i = 0; i < figureCount; i++)
		{
			double lat, lon;
			edgeIfs >> lat >> lon;
			figure->push_back(new GeoPoint(lat, lon));
		}
		if (edgeIfs.fail())
			break;
		if (!inArea(nodes[startNodeId]->lat, nodes[startNodeId]->lon) && !inArea(nodes[endNodeId]->lat, nodes[endNodeId]->lon))
		{
			edges.push_back(NULL);
			count++;
			continue;
		}
		Edge* edge = new Edge();
		edge->id = edgeId;
		edge->visited = false;
		edge->figure = figure;
		edge->lengthM = calEdgeLength(figure);
		edges.push_back(edge);
		insertEdge(edgeId, startNodeId, endNodeId);
	}
	
	printf("edges count = %d\n", edges.size());
	printf("not in area edges count = %d\n", count);
	edgeIfs.close();
	printf(">> reading map finished\n");
	createGridIndex();
	printf(">> creating grid index finished\n");
}

void Map::setArea(Area* area)
{
	//[ATTENTION][MEMORY_LEAK]���ﲢû�н�ԭarea���ڴ�����գ����ܻ�����ڴ�й¶
	this->area = area;
}

vector<Edge*> Map::getNearEdges(double lat, double lon, double threshold) const
{
	//////////////////////////////////////////////////////////////////////////
	///����(lat, lon)��Χ����С��threshold�׵�����·��
	///[ע��]������ڴ�й¶
	//////////////////////////////////////////////////////////////////////////
	vector<Edge*> result;
	vector<Edge*> fail;
	int gridSearchRange = int(threshold / (gridSizeDeg * GeoPoint::geoScale)) + 1;
	int rowPt = getRowId(lat);
	int colPt = getColId(lon);
	int row1 = rowPt - gridSearchRange;
	int col1 = colPt - gridSearchRange;
	int row2 = rowPt + gridSearchRange;
	int col2 = colPt + gridSearchRange;
	if (row1 < 0) row1 = 0;
	if (row2 >= gridHeight) row2 = gridHeight - 1;
	if (col1 < 0) col1 = 0;
	if (col2 >= gridWidth) col2 = gridWidth - 1;
	//cout << "gridrange = " << gridSearchRange << endl;
	for (int row = row1; row <= row2; row++)
	{
		for (int col = col1; col <= col2; col++)
		{
			for (list<Edge*>::iterator iter = grid[row][col]->begin(); iter != grid[row][col]->end(); iter++)
			{
				//if (grid[row][col]->size() != 0)
				//	cout << "grid count = " << grid[row][col]->size() << endl;
				if (!((*iter)->visited))
				{
					(*iter)->visited = true;
					double dist = distM_withThres(lat, lon, (*iter), threshold);
					if (dist < threshold)
						result.push_back((*iter));
					else
						fail.push_back((*iter));
				}
			}
		}
	}
	for (int i = 0; i < result.size(); i++)
	{
		result[i]->visited = false;
	}
	for (int i = 0; i < fail.size(); i++)
	{
		fail[i]->visited = false;
	}
	return result;
}

void Map::getNearEdges(double lat, double lon, double threshold, vector<Edge*>& dest)
{
	//////////////////////////////////////////////////////////////////////////
	///����(lat, lon)��Χ����С��threshold�׵�����·��
	//////////////////////////////////////////////////////////////////////////
	dest.clear();
	vector<Edge*> fail;
	int gridSearchRange = int(threshold / (gridSizeDeg * GeoPoint::geoScale)) + 1;
	int rowPt = getRowId(lat);
	int colPt = getColId(lon);
	int row1 = rowPt - gridSearchRange;
	int col1 = colPt - gridSearchRange;
	int row2 = rowPt + gridSearchRange;
	int col2 = colPt + gridSearchRange;
	if (row1 < 0) row1 = 0;
	if (row2 >= gridHeight) row2 = gridHeight - 1;
	if (col1 < 0) col1 = 0;
	if (col2 >= gridWidth) col2 = gridWidth - 1;
	//cout << "gridrange = " << gridSearchRange << endl;
	for (int row = row1; row <= row2; row++)
	{
		for (int col = col1; col <= col2; col++)
		{
			for (list<Edge*>::iterator iter = grid[row][col]->begin(); iter != grid[row][col]->end(); iter++)
			{
				//if (grid[row][col]->size() != 0)
				//	cout << "grid count = " << grid[row][col]->size() << endl;
				if (!((*iter)->visited))
				{
					(*iter)->visited = true;
					double dist = distM_withThres(lat, lon, (*iter), threshold);
					if (dist < threshold)
						dest.push_back((*iter));
					else
						fail.push_back((*iter));
				}
			}
		}
	}
	for (int i = 0; i < dest.size(); i++)
	{
		dest[i]->visited = false;
	}
	for (int i = 0; i < fail.size(); i++)
	{
		fail[i]->visited = false;
	}
}

void Map::getNearEdges(double lat, double lon, int k, vector<Edge*>& dest)
{
	//////////////////////////////////////////////////////////////////////////
	///�ҳ���(lat, lon)���������k���ߣ����մӽ���Զ�ľ������dest��
	///���������� ���Բ�ѯ�����ڵĸ���Ϊ���ģ�gridSearchRangeΪ�����뾶����������
	///Ȼ����ÿ�������뾶����gridExtendStep�ķ���������ɢ������ֱ��candidates������Ԫ�ظ���
	///���ڵ���kֹͣ������
	//////////////////////////////////////////////////////////////////////////
	dest.clear();
	vector<pair<Edge*, double>> candidates;
	//��������
	int gridSearchRange = 3;
	int rowPt = getRowId(lat);
	int colPt = getColId(lon);
	int row1 = rowPt - gridSearchRange;
	int col1 = colPt - gridSearchRange;
	int row2 = rowPt + gridSearchRange;
	int col2 = colPt + gridSearchRange;
	if (row1 < 0) row1 = 0;
	if (row2 >= gridHeight) row2 = gridHeight - 1;
	if (col1 < 0) col1 = 0;
	if (col2 >= gridWidth) col2 = gridWidth - 1;
	for (int row = row1; row <= row2; row++)
	{
		for (int col = col1; col <= col2; col++)
		{
			for (list<Edge*>::iterator iter = grid[row][col]->begin(); iter != grid[row][col]->end(); iter++)
			{
				if (!((*iter)->visited))
				{
					(*iter)->visited = true;
					double dist = distM(lat, lon, (*iter));
					candidates.push_back(make_pair((*iter), dist));
				}
			}
		}
	}
	
	int gridExtendStep = 1; //��չ������ÿ��������1��	
	int newRow1, newRow2, newCol1, newCol2; //��¼�µ�������Χ
	while (candidates.size() < k)
	{
		newRow1 = row1 - gridExtendStep;
		newRow2 = row2 + gridExtendStep;
		newCol1 = col1 - gridExtendStep;
		newCol2 = col2 + gridExtendStep;
		if (newRow1 < 0) newRow1 = 0;
		if (newRow2 >= gridHeight) newRow2 = gridHeight - 1;
		if (newCol1 < 0) newCol1 = 0;
		if (newCol2 >= gridWidth) newCol2 = gridWidth - 1;
		for (int row = newRow1; row <= newCol2; row++)
		{
			for (int col = newCol1; col <= newCol2; col++)
			{
				if (row >= row1 && row <= row2 && col >= col1 && col <= col2) //�Ѿ��������ľͲ�������
					continue;
				for (list<Edge*>::iterator iter = grid[row][col]->begin(); iter != grid[row][col]->end(); iter++)
				{
					if (!((*iter)->visited))
					{
						(*iter)->visited = true;
						double dist = distM(lat, lon, (*iter));
						candidates.push_back(make_pair((*iter), dist));
					}
				}
				
			}
		}
		if (newRow1 == 0 && newRow2 == gridHeight - 1 && newCol1 == 0 && newCol2 == gridWidth - 1)//�������ȫͼ��û���㣬���ж�����
			break;
	}
	sort(candidates.begin(), candidates.end(), smallerInDist);
	for (int i = 0; i < k; i++)
	{
		dest.push_back(candidates[i].first);
	}
	
	//��ԭ���бߵ�visitFlag
	for (int i = 0; i < candidates.size(); i++)
	{
		candidates[i].first->visited = false;
	}
	return;

}

double Map::shortestPathLength(int ID1, int ID2, double dist1, double dist2, double deltaT)
{
	int maxNodeNum = nodes.size();
	vector<double> dist = vector<double>(maxNodeNum);
	vector<bool> flag = vector<bool>(maxNodeNum);
	for (int i = 0; i < maxNodeNum; i++) {
		dist[i] = INF;
		flag[i] = false;
	}
	dist[ID1] = 0;
	priority_queue<NODE_DIJKSTRA> Q;
	NODE_DIJKSTRA tmp(ID1, 0);
	Q.push(tmp);
	while (!Q.empty()) {
		NODE_DIJKSTRA x = Q.top();
		Q.pop();
		int u = x.t;
		if (x.dist > deltaT*MAXSPEED){
			return INF;
		}
		if (flag[u]) {
			continue;
		}
		flag[u] = true;
		if (u == ID2) {
			break;
		}
		for (AdjNode* i = adjList[u]->next; i != NULL; i = i->next) {
			if (dist[i->endPointId] > dist[u] + edges[i->edgeId]->lengthM) {
				dist[i->endPointId] = dist[u] + edges[i->edgeId]->lengthM;
				NODE_DIJKSTRA tmp(i->endPointId, dist[i->endPointId]);
				Q.push(tmp);
			}
		}
	}
	double resultLength = dist[ID2];
	return resultLength;

}

double Map::distM(double lat, double lon, Edge* edge) const
{
	//////////////////////////////////////////////////////////////////////////
	///���ص�(lat, lon)����edge�ľ�ȷ����
	///���붨��Ϊ��min(�㵽��ͶӰ�ߵ�ͶӰ���룬�㵽������״���ŷ�Ͼ���)
	//////////////////////////////////////////////////////////////////////////
	double minDist = INF;
	//�����˵����
	for (Figure::iterator iter = edge->figure->begin(); iter != edge->figure->end(); iter++)
	{
		double tmpDist = GeoPoint::distM(lat, lon, (*iter)->lat, (*iter)->lon);
		if (tmpDist < minDist)
			minDist = tmpDist;
	}
	//����ͶӰ����
	Figure::iterator iter = edge->figure->begin();
	Figure::iterator nextIter = edge->figure->begin();
	nextIter++;
	while (nextIter != edge->figure->end())
	{
		//��ͶӰ
		GeoPoint* pt = new GeoPoint(lat, lon);
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

double Map::distM(double lat, double lon, Edge* edge, double& prjDist) const
{
	//////////////////////////////////////////////////////////////////////////
	///���ص�(lat, lon)����edge�ľ�ȷ����
	///���붨��Ϊ��min(�㵽��ͶӰ�ߵ�ͶӰ���룬�㵽������״���ŷ�Ͼ���)
	///�����ͶӰ�Ļ���prjDist��¼ͶӰ�㵽�켣���ľ��룬û�еĻ�prjDistΪ0
	//////////////////////////////////////////////////////////////////////////
	Figure::iterator iter = edge->figure->begin();
	Figure::iterator nextIter = edge->figure->begin();
	nextIter++;
	prjDist = 0;
	double frontSegmentDist = 0;
	double tempTotalPrjDist = 0;
	double minDist = INF;
	//�����˵����
	while (nextIter != edge->figure->end())
	{
		double tmpDist = GeoPoint::distM(lat, lon, (*iter)->lat, (*iter)->lon);
		if (tmpDist < minDist)
		{
			minDist = tmpDist;
			tempTotalPrjDist = frontSegmentDist;
		}
		frontSegmentDist += GeoPoint::distM((*iter), (*nextIter));
		iter++;
		nextIter++;
	}
	//�����һ����
	double tmpDist = GeoPoint::distM(lat, lon, (*iter)->lat, (*iter)->lon);
	if (tmpDist < minDist)
	{
		minDist = tmpDist;
		tempTotalPrjDist = frontSegmentDist;
	}
	//����ͶӰ����
	frontSegmentDist = 0;
	iter = edge->figure->begin();
	nextIter = edge->figure->begin();
	nextIter++;
	while (nextIter != edge->figure->end())
	{
		//��ͶӰ
		GeoPoint* pt = new GeoPoint(lat, lon);
		if (cosAngle(pt, (*iter), (*nextIter)) <= 0 && cosAngle(pt, (*nextIter), (*iter)) <= 0)
		{
			double A = ((*nextIter)->lat - (*iter)->lat);
			double B = -((*nextIter)->lon - (*iter)->lon);
			double C = (*iter)->lat * ((*nextIter)->lon - (*iter)->lon)
				- (*iter)->lon * ((*nextIter)->lat - (*iter)->lat);
			double tmpDist = abs(A * pt->lon + B * pt->lat + C) / sqrt(A * A + B * B);
			tmpDist *= GeoPoint::geoScale;
			if (minDist > tmpDist)
			{
				minDist = tmpDist;
				double tmpPjDist = GeoPoint::distM(pt, (*iter));
				tmpPjDist *= -cosAngle(pt, (*iter), (*nextIter));
				tempTotalPrjDist = frontSegmentDist + tmpPjDist;
			}
		}
		frontSegmentDist += GeoPoint::distM((*iter), (*nextIter));
		iter++;
		nextIter++;
	}
	prjDist = tempTotalPrjDist;
	return minDist;
}

double Map::distMFromTransplantFromSRC(double lat, double lon, Edge* edge, double& prjDist)
{
	//////////////////////////////////////////////////////////////////////////
	///��ֲSRC�汾������(lat,lon)�㵽edge�ľ��룬��λΪ�ף�ͬʱ��¼ͶӰ�㵽edge���ľ������prjDist
	//////////////////////////////////////////////////////////////////////////
	double tmpSideLen = 0;
	double result = 1e80, tmp = 0;
	double x = -1, y = -1;
	for (Figure::iterator figIter = edge->figure->begin(); figIter != edge->figure->end(); figIter++){
		if (x != -1 && y != -1){
			double x2 = (*figIter)->lat;
			double y2 = (*figIter)->lon;
			double dist = GeoPoint::distM(x, y, lat, lon); //circle Distance(x, y, nodeX, nodeY);
			if (dist<result){
				result = dist;
				tmpSideLen = tmp;
			}
			double vecX1 = x2 - x;
			double vecY1 = y2 - y;
			double vecX2 = lat - x;
			double vecY2 = lon - y;
			double vecX3 = lat - x2;
			double vecY3 = lon - y2;
			if (vecX1*vecX2 + vecY1*vecY2>0 && -vecX1*vecX3 - vecY1*vecY3 > 0 && (vecX1 != 0 || vecY1 != 0)){
				double rate = ((lat - x2)*vecX1 + (lon - y2)*vecY1) / (-vecX1*vecX1 - vecY1*vecY1);
				double nearX = rate*x + (1 - rate)*x2, nearY = rate*y + (1 - rate)*y2;
				double dist = GeoPoint::distM(nearX, nearY, lat, lon);
				if (dist < result){
					result = dist;
					tmpSideLen = tmp + GeoPoint::distM(x, y, nearX, nearY);
				}
			}
			tmp += GeoPoint::distM(x, y, x2, y2);
		}
		x = (*figIter)->lat;
		y = (*figIter)->lon;
	}
	prjDist = tmpSideLen;
	return result;
}

int Map::hasEdge(int startNodeId, int endNodeId) const
{
	AdjNode* current = adjList[startNodeId]->next;
	while (current != NULL)
	{
		if (current->endPointId == endNodeId)
		{
			return current->edgeId;
		}
		else
			current = current->next;
	}
	return -1;
}

int Map::insertNode(double lat, double lon)
{
	//////////////////////////////////////////////////////////////////////////
	///����һ���½��(lat, lon),��ͬʱ���ڽӱ���Ҳ��Ӧ����һ���ڽӱ���,�����½���id
	//////////////////////////////////////////////////////////////////////////
	//if (!inArea(lat, lon))
	//	return -1;
	GeoPoint* pt = new GeoPoint(lat, lon);
	nodes.push_back(pt);
	AdjNode* adjNode = new AdjNode();
	adjNode->endPointId = adjList.size();
	adjNode->next = NULL;
	adjList.push_back(adjNode);
	return nodes.size() - 1;
}

int Map::insertEdge(Figure* figure, int startNodeId, int endNodeId)
{
	//////////////////////////////////////////////////////////////////////////
	///��figureΪ·�ι���һ���±߲����ͼ,��������������
	///[ע��]������û�н�����������ʱ����!
	//////////////////////////////////////////////////////////////////////////
	Edge* newEdge = new Edge();
	newEdge->figure = figure;
	newEdge->startNodeId = startNodeId;
	newEdge->endNodeId = endNodeId;
	newEdge->lengthM = calEdgeLength(figure);
	newEdge->id = edges.size();
	edges.push_back(newEdge);
	AdjNode* current = adjList[startNodeId];
	insertEdge(newEdge->id, startNodeId, endNodeId); //������ͨ��ϵ
	createGridIndexForEdge(newEdge); //������������
	return newEdge->id;
}

int Map::splitEdge(int edgeId, double lat, double lon)
{
	//////////////////////////////////////////////////////////////////////////
	///��edgeId��·��(lat, lon)���и������·,(lat, lon)��Ϊintersection
	///�и֤�ǰ�ȫ��,�޸����õ�
	//////////////////////////////////////////////////////////////////////////
	Edge* edge = edges[edgeId];
	Figure* figure = edge->figure;
	pair<int, int> result;
	bool bidirection = false;
	Edge* edgeR = NULL; //��¼˫���ʱ�ķ����Ӧ·��
	//�ҵ��и��
	Figure* subFigure1 = new Figure(); //��¼�и���ǰ���·��
	Figure* subFigure2 = new Figure(); //��¼�и��ĺ���·��
	Figure::iterator iter = figure->begin();
	Figure::iterator nextIter = figure->begin();
	nextIter++;
	int newNodeId;	
	while (nextIter != edge->figure->end())
	{
		GeoPoint* pt = (*iter);
		GeoPoint* nextPt = (*nextIter);
		subFigure1->push_back(pt);
		//��ͶӰ
		double A = (nextPt->lat - pt->lat);
		double B = -(nextPt->lon - pt->lon);
		double C = pt->lat * (nextPt->lon - pt->lon)
					- pt->lon * (nextPt->lat - pt->lat);
		if (abs(A * lon + B * lat + C) < eps)
		{
			newNodeId = insertNode(lat, lon);
			subFigure1->push_back(nodes[newNodeId]);
			subFigure2->push_back(nodes[newNodeId]);
			//������·�Ƿ�Ϊ˫��·
			//��������R�Ĵ���reverse
			AdjNode* currentAdjNode = adjList[edge->endNodeId]->next;
			bidirection = false;
			while (currentAdjNode != NULL && bidirection == false)
			{
				if (currentAdjNode->endPointId == edge->startNodeId)
				{
					edgeR = edges[currentAdjNode->edgeId];
					Figure::iterator iterR = edgeR->figure->begin();
					Figure::iterator nextIterR = edgeR->figure->begin();
					nextIterR++;
					while (nextIterR != edgeR->figure->end())
					{
						GeoPoint* ptR = (*iterR);
						GeoPoint* nextPtR = (*nextIterR);
						if (abs(ptR->lat - nextPt->lat) < eps && abs(ptR->lon - nextPt->lon) < eps
							&& abs(nextPtR->lat - pt->lat) < eps && abs(nextPtR->lon - pt->lon) < eps)
						{
							bidirection = true;
							break;
						}
						iterR++;
						nextIterR++;
					}
				}
				currentAdjNode = currentAdjNode->next;
			}
			iter++;
			break;
		}
		iter++;
		nextIter++;
	}
	if (nextIter == figure->end()) //�и�㲻��·���򱨴��˳�
	{
		cout << "error: split point is not on the edge" << endl;
		system("pause");
		exit(0);
	}
	//������ѹ��subFigure2
	while (iter != figure->end())
	{
		subFigure2->push_back(*iter);
		iter++;
	}
	//���±߼���,�޸����ӹ�ϵ
		//��subFigure2��Ϊ�±߼����ͼ
	Edge* edge2 = edges[insertEdge(subFigure2, newNodeId, edge->endNodeId)];
		//��subFigure1���ԭ����edge
	delete edge->figure;
	edge->figure = subFigure1;
	edge->lengthM = calEdgeLength(subFigure1);
	edge->endNodeId = newNodeId;
		//�޸�ǰ��ε���ͨ��ϵ
	AdjNode* current = adjList[edge->startNodeId]->next;
	while (current->edgeId != edge->id)
	{
		current = current->next;
	}
	current->endPointId = newNodeId;
	//����˫������
	if (bidirection)
	{
		Figure* subFigure1R = new Figure();
		Figure* subFigure2R = new Figure();
		for (Figure::iterator iter = subFigure1->begin(); iter != subFigure1->end(); iter++)
		{
			subFigure1R->push_front(*iter);
		}
		for (Figure::iterator iter = subFigure2->begin(); iter != subFigure2->end(); iter++)
		{
			subFigure2R->push_front(*iter);
		}
		//��subFigure2R���edgeR
		delete edgeR->figure;
		edgeR->figure = subFigure2R;
		edgeR->lengthM = calEdgeLength(subFigure2R);
		//���´���edge2R
		insertEdge(subFigure2R, edge2->endNodeId, edge2->startNodeId);
	}
	return newNodeId;
}

void Map::delEdge(int edgeId, bool delBirectionEdges /* = true */)
{
	//��ע�⡿���ܻᷢ���ڴ�й¶��ԭedgeû�б�del��
	//��ע��ע�⣡��TODO������û�а�·ɾ��������취ֻ�ǽ�ɾ����·��visited�ֶθĳ�true��ʱӦ�����¶���
	if (edges[edgeId] == NULL)
		return;	
	int startNodeId = edges[edgeId]->startNodeId;
	int endNodeId = edges[edgeId]->endNodeId;
	if (edges[edgeId])
		edges[edgeId]->visited = true; //��ֹ�������ҵ�
	edges[edgeId] = NULL;
	AdjNode* currentAdjNode = adjList[startNodeId];
	while (currentAdjNode->next != NULL)
	{
		if (currentAdjNode->next->endPointId == endNodeId)
		{
			AdjNode* delNode = currentAdjNode->next;
			currentAdjNode->next = currentAdjNode->next->next;
			delete delNode;
			break;
		}
		currentAdjNode = currentAdjNode->next;
	}
	if (delBirectionEdges) //ɾ�������
	{
		int reverseEdgeId = hasEdge(endNodeId, startNodeId);
		if (reverseEdgeId == -1) //û�з����
			return;		
		if (edges[reverseEdgeId])
			edges[reverseEdgeId]->visited = true; //��ֹ�������ҵ�
		edges[reverseEdgeId] = NULL;		
		AdjNode* currentAdjNode = adjList[endNodeId];
		while (currentAdjNode->next != NULL)
		{
			if (currentAdjNode->next->endPointId == startNodeId)
			{
				AdjNode* delNode = currentAdjNode->next;
				currentAdjNode->next = currentAdjNode->next->next;
				delete delNode;
				break;
			}
			currentAdjNode = currentAdjNode->next;
		}
	}
}

//Gdiplus::Color randomColor();

void Map::getMinMaxLatLon(string nodeFilePath)
{
	ifstream nodeIfs(nodeFilePath);
	if (!nodeIfs)
	{
		cout << "open " + nodeFilePath + " error!\n";
		system("pause");
		exit(0);
	}
	area->minLon = 999;
	area->maxLon = -999;
	area->minLat = 999;
	area->maxLat = -999;
	while (nodeIfs)
	{
		double lat, lon;
		int nodeId;
		nodeIfs >> nodeId >> lat >> lon;
		if (nodeIfs.fail())
			break;
		if (lon < area->minLon) area->minLon = lon;
		if (lon > area->maxLon) area->maxLon = lon;
		if (lat < area->minLat) area->minLat = lat;
		if (lat > area->maxLat) area->maxLat = lat;
	}
	printf("minLat:%lf, maxLat:%lf, minLon:%lf, maxLon:%lf\n", area->minLat, area->maxLat, area->minLon, area->maxLon);
	nodeIfs.close();
}

void Map::drawMap(Gdiplus::Color color, MapDrawer& md)
{
	for (int i = 0; i < edges.size(); i++)
	{
		if (edges[i] == NULL)
			continue;
		Figure::iterator ptIter = edges[i]->figure->begin(), nextPtIter = ptIter;                                                     
		nextPtIter++;
		//Gdiplus::Color rndColor = randomColor();
		while (1)
		{
			if (nextPtIter == edges[i]->figure->end())
				break;
			md.drawLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			md.drawBigPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
			md.drawBigPoint(Gdiplus::Color::Black, (*nextPtIter)->lat, (*nextPtIter)->lon);
			ptIter++;
			nextPtIter++;
		}
	}
}

void Map::drawDeletedEdges(Gdiplus::Color color, MapDrawer& md)
{
	cout << deletedEdges.size() << endl;
	system("pause");
	for (int i = 0; i < deletedEdges.size(); i++)
	{
		if (deletedEdges[i] == NULL || deletedEdges[i]->figure == NULL)
		{
			cout << "NULL";
			system("pause");
		}
		
		Figure::iterator ptIter = deletedEdges[i]->figure->begin(), nextPtIter = ptIter;
		nextPtIter++;
		while (1)
		{
			if (nextPtIter == deletedEdges[i]->figure->end())
				break;
			md.drawBoldLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			
			/**********************************************************/
			/*test code starts from here*/
			//if (deletedEdges[i]->visited == true)
			//{
			//	md.drawBoldLine(Gdiplus::Color::Aqua, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			//}
			
			/*test code ends*/
			/**********************************************************/
			
			
			//md.drawBigPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
			//md.drawBigPoint(Gdiplus::Color::Black, (*nextPtIter)->lat, (*nextPtIter)->lon);
			ptIter++;
			nextPtIter++;
		}
	}	
}

void Map::deleteEdgesRandomly(int delNum, double minEdgeLengthM)
{
	//////////////////////////////////////////////////////////////////////////
	///���ɾ��delNum��·��·�ĳ����賬��minEdgeLengthM����λΪm��
	///���������Ҫ�Լ���main������д
	//////////////////////////////////////////////////////////////////////////
	
	int victimId;
	int count = 0;
	while (1)
	{
		if (count == delNum)
			break;
		if (edges[victimId = int(((double)rand()) / RAND_MAX * (edges.size() - 1))] == NULL)
			continue;
		if (edges[victimId]->lengthM < minEdgeLengthM)
			continue;

		deletedEdges.push_back(edges[victimId]);
		cout << "delete " << victimId << endl;
		int reverseVictimId = hasEdge(edges[victimId]->endNodeId, edges[victimId]->startNodeId);
		if (reverseVictimId != -1)
			deletedEdges.push_back(edges[reverseVictimId]);
		delEdge(victimId);
		count++;
	}
	cout << ">> randomly deleted " << delNum << " edges" << endl;
}

void Map::deleteEdgesRandomlyEx(int delNum, double minEdgeLengthM, double aroundThresholdM, int aroundNumThreshold, bool doOutput /* = true */)
{
	int victimId;
	int count = 0;
	while (1)
	{
		if (count == delNum)
			break;
		if (edges[victimId = int(((double)rand()) / RAND_MAX * (edges.size() - 1))] == NULL)
			continue;
		if (edges[victimId]->lengthM < minEdgeLengthM)
			continue;
		vector<Edge*> nearEdges;
		getNearEdges(edges[victimId], aroundThresholdM, nearEdges);
		if (nearEdges.size() >= aroundNumThreshold)
			continue;
		deletedEdges.push_back(edges[victimId]);
		edges[victimId]->visited = true; //
		cout << "delete " << victimId << endl;
		int reverseVictimId = hasEdge(edges[victimId]->endNodeId, edges[victimId]->startNodeId);
		if (reverseVictimId != -1)
		{
			deletedEdges.push_back(edges[reverseVictimId]);
			edges[reverseVictimId]->visited = true; //
			cout << "delete reverse " << reverseVictimId << endl;
		}
		delEdge(victimId);
		//ɾ�����и�����·
		for each (Edge* nearEdge in nearEdges)
		{
			if (edges[nearEdge->id] == NULL)
				continue;
			deletedEdges.push_back(nearEdge);
			cout << "\tdelete " << nearEdge->id << endl;
			int reverseNearEdgeId = hasEdge(nearEdge->endNodeId, nearEdge->startNodeId);
			if (reverseNearEdgeId != -1)
			{
				deletedEdges.push_back(edges[reverseNearEdgeId]);
				cout << "\tdelete reverse " << reverseNearEdgeId << endl;
			}
			delEdge(nearEdge->id);			
		}
		count++;
	}
	cout << ">> randomly deleted " << delNum << " edges" << endl;
	if (doOutput)
	{
		ofstream ofs("deletedEdges.txt");
		if (!ofs)
		{
			cout << "open deletedEdges.txt error" << endl;
		}
		for (int i = 0; i < deletedEdges.size(); i++)
		{
			ofs << deletedEdges[i]->id << endl;
		}
		ofs.close();
	}	
}

void Map::deleteEdges(string path)
{
	ifstream ifs(path);
	if (!ifs)
	{
		cout << "open file " << path << " error!" << endl;
	}
	int edgeId;
	vector<int> deletedEdgesId;
	while (ifs)
	{
		ifs >> edgeId;
		if (ifs.fail())
			break;
		deletedEdgesId.push_back(edgeId);
	}
	for (int i = 0; i < deletedEdgesId.size(); i++)
	{
		int victimId = deletedEdgesId[i];
		deletedEdges.push_back(edges[victimId]);
	}
	for (int i = 0; i < deletedEdgesId.size(); i++)
	{
		 delEdge(deletedEdgesId[i]);
	}		
	ifs.close();
}

//////////////////////////////////////////////////////////////////////////
///private part
//////////////////////////////////////////////////////////////////////////
double Map::distM_withThres(double lat, double lon, Edge* edge, double threshold) const
{
	//////////////////////////////////////////////////////////////////////////
	///���ص�(lat, lon)����edge�ľ����Ͻ� ��ע�⡿�������ڼ��㾫ȷ���룡
	///���붨��Ϊ��min(�㵽��ͶӰ�ߵ�ͶӰ���룬�㵽������״���ŷ�Ͼ���)
	///��������Ͻ�ʱ�����Ѿ�����threshold(��λ��)��ֱ�ӷ���
	//////////////////////////////////////////////////////////////////////////
	double minDist = INF;
	if (edge == NULL)
	{
		cout << "edge = NULL";
		system("pause");
	}
	if (edge->figure == NULL)
	{
		cout << "edge->figure = NULL";
		system("pause");
	}
	//�����˵����
	for (Figure::iterator iter = edge->figure->begin(); iter != edge->figure->end(); iter++)
	{
		if (*iter == NULL)
		{
			cout << "*iter = NULL";
			system("pause");
		}
		/*if (!inArea(lat, lon))// || !inArea((*iter)->lat, (*iter)->lon))
		{
			cout << "not in area";
			printf("(lat,lon) = (%lf,%lf), iter = (%lf, %lf)\n", lat, lon, (*iter)->lat, (*iter)->lon);
			printf("minlat = %lf, maxlat = %lf\n", minLat, maxLat);
			printf("minlon = %lf, maxlon = %lf\n", minLon, maxLon);
			system("pause");
		}*/
		double tmpDist = GeoPoint::distM(lat, lon, (*iter)->lat, (*iter)->lon);
		if (tmpDist < threshold)
			return tmpDist;
		if (tmpDist < minDist)
			minDist = tmpDist;
	}
	//����ͶӰ����
	Figure::iterator iter = edge->figure->begin();
	Figure::iterator nextIter = edge->figure->begin();
	nextIter++;
	while (nextIter != edge->figure->end())
	{
		//��ͶӰ
		GeoPoint* pt = new GeoPoint(lat, lon);
		if (cosAngle(pt, (*iter), (*nextIter)) <= 0 && cosAngle(pt, (*nextIter), (*iter)) <= 0)
		{
			double A = ((*nextIter)->lat - (*iter)->lat);
			double B = -((*nextIter)->lon - (*iter)->lon);
			double C = (*iter)->lat * ((*nextIter)->lon - (*iter)->lon)
				- (*iter)->lon * ((*nextIter)->lat - (*iter)->lat);
			double tmpDist = abs(A * pt->lon + B * pt->lat + C) / sqrt(A * A + B * B);
			tmpDist *= GeoPoint::geoScale;
			if (tmpDist < threshold)
				return tmpDist;
			if (minDist > tmpDist)
				minDist = tmpDist;
		}
		iter++;
		nextIter++;
	}
	return minDist;
}

double Map::calEdgeLength(Figure* figure) const
{
	//////////////////////////////////////////////////////////////////////////
	///����·�εĳ��ȣ���λΪm
	//////////////////////////////////////////////////////////////////////////
	double lengthM = 0;
	Figure::iterator ptIter = figure->begin(), nextPtIter = ptIter;
	nextPtIter++;
	while (1)
	{
		if (nextPtIter == figure->end())
			break;
		lengthM += GeoPoint::distM((*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
		ptIter++;
		nextPtIter++;
	}
	return lengthM;
}

bool Map::inArea(double lat, double lon) const
{
	return (lat > area->minLat && lat < area->maxLat && lon > area->minLon && lon < area->maxLon);
}

void Map::test()
{
	int* flag = new int[nodes.size()];
	for (int i = 0; i < nodes.size(); i++)
	{
		flag[i] = -1;
	}
	for (int i = 0; i < adjList.size(); i++)
	{
		if (i % 1000 == 0)
		{
			cout << i << endl;
		}		
		AdjNode* current = adjList[i]->next;
		while (current != NULL)
		{
			int j = current->endPointId;
			/*int edgeIJId = current->edgeId;
			int edgeJIId = hasEdge(j, i);
			if (edgeJIId != -1)
			{
				Edge* edgeIJ = edges[edgeIJId];
				Edge* edgeJI = edges[edgeJIId];
				if (edgeIJ->size() != edgeJI->size())
				{
					cout << "XXXXXX" << endl;
				}
				Edge::iterator iter1 = edgeIJ->begin();
				Edge::iterator iter2 = edgeJI->end();
				iter2--;
				while (iter2 != edgeJI->begin())
				{
					if (abs((*iter1)->lat - (*iter2)->lat) > 1e-8 ||
						abs((*iter1)->lon - (*iter2)->lon) > 1e-8)
					{
						cout << "YYYYYY" << endl;
						printf("%lf,%lf,%lf,%lf\n", (*iter1)->lat, (*iter2)->lat, (*iter1)->lon, (*iter2)->lon);
						printf("edgeIJ = %d, edgeJI = %d", edgeIJId, edgeJIId);
						system("pause");
					}
					//printf("%lf,%lf,%lf,%lf\n", (*iter1)->lat, (*iter2)->lat, (*iter1)->lon, (*iter2)->lon);
					//system("pause");
					iter2--;
					iter1++;
				}
			}*/
			if (flag[j] == i)
			{
				cout << "ZZZZZZZ" << endl;
				printf("%d, %d\n", i, j);
				system("pause");
			}
			flag[j] = i;
			current = current->next;
		}
	}
}

void Map::createGridIndex()
{
	//////////////////////////////////////////////////////////////////////////
	///��ȫͼ������������
	//////////////////////////////////////////////////////////////////////////
	//initialization
	if (gridWidth <= 0)
		return;	
	gridHeight = int((area->maxLat - area->minLat) / (area->maxLon - area->minLon) * double(gridWidth)) + 1;
	gridSizeDeg = (area->maxLon - area->minLon) / double(gridWidth);
	grid = new list<Edge*>* *[gridHeight];
	for (int i = 0; i < gridHeight; i++)
		grid[i] = new list<Edge*>* [gridWidth];
	for (int i = 0; i < gridHeight; i++)
	{
		for (int j = 0; j < gridWidth; j++)
		{
			grid[i][j] = new list<Edge*>();
		}
	}
	printf("Map index gridWidth = %d, gridHeight = %d\n", gridWidth, gridHeight);
	cout << "gridSize = " << gridSizeDeg * GeoPoint::geoScale << "m" << endl;
	for (vector<Edge*>::iterator edgeIter = edges.begin(); edgeIter != edges.end(); edgeIter++)
	{
		createGridIndexForEdge((*edgeIter));
	}
}

void Map::insertEdgeIntoGrid(Edge* edge, int row, int col)
{
	//////////////////////////////////////////////////////////////////////////
	///��·��edge����grid[row][col]������������Ѿ�����������
	///�ĺ���һ���ڶ�ĳ��edge��������ʱ����,���Լ������grid�����һ��һ����edge
	//////////////////////////////////////////////////////////////////////////
	if (row >= gridHeight || row < 0 || col >= gridWidth || col < 0)
		return;
	if (grid[row][col]->size() > 0 && grid[row][col]->back() == edge)
		return;
	else
		grid[row][col]->push_back(edge);
}

void Map::createGridIndexForSegment(Edge *edge, GeoPoint* fromPT, GeoPoint* toPt)
{
	//////////////////////////////////////////////////////////////////////////
	///��edge·�е�fromPt->toPt�β����������������������񶼼�����ָ�룬����������ཻ���ȹ�С�򲻼�������
	///���2���˵㶼�ڵ�ǰ�������򲻼������������һ���˵��ڵ�ǰ�����⣬������������ڵ��Ǹ��˵����ڵ�����
	//////////////////////////////////////////////////////////////////////////
	if (edge == NULL)
		return;
	//������������������
	if (!inArea(fromPT->lat, fromPT->lon) && !inArea(toPt->lat, toPt->lon))
		return;
	//�Ը�������������ֻ������һ���˵����ڵ�����
	//TODO:��������������׼ȷ
	if (!inArea(fromPT->lat, fromPT->lon))
	{
		int row = (toPt->lat - area->minLat) / gridSizeDeg;
		int col = (toPt->lon - area->minLon) / gridSizeDeg;
		insertEdgeIntoGrid(edge, row, col);
		return;
	}
	if (!inArea(toPt->lat, toPt->lon))
	{
		int row = (fromPT->lat - area->minLat) / gridSizeDeg;
		int col = (fromPT->lon - area->minLon) / gridSizeDeg;
		insertEdgeIntoGrid(edge, row, col);
		return;
	}
	
	
	bool crossRow;
	GeoPoint* pt1 = fromPT;
	GeoPoint* pt2 = toPt;
	double x1 = pt1->lon - area->minLon;
	double y1 = pt1->lat - area->minLat;
	double x2 = pt2->lon - area->minLon;
	double y2 = pt2->lat - area->minLat;
	int row1 = y1 / gridSizeDeg;
	int row2 = y2 / gridSizeDeg;
	int col1 = x1 / gridSizeDeg;
	int col2 = x2 / gridSizeDeg;
	if (row1 >= gridHeight || row1 < 0 || col1 >= gridWidth || col1 < 0 ||
		row2 >= gridHeight || row2 < 0 || col2 >= gridWidth || col2 < 0)
	{
		cout << "************test**************" << endl;
		cout << "row1 = " << row1 << " col1 = " << col1 << endl;
		cout << "row2 = " << row2 << " col2 = " << col2 << endl;
		printf("pt1(%.8lf,%.8lf)\n", pt1->lat, pt1->lon);
		printf("pt2(%.8lf,%.8lf)\n", pt2->lat, pt2->lon);
		printf("min(%.8lf,%.8lf), max(%.8lf,%.8lf)\n", area->minLat, area->minLon, area->maxLat, area->maxLon);
		cout << "edgeId = " << edge->id << endl;
		cout << "from node " << edge->startNodeId << " to node " << edge->endNodeId << endl;
		cout << "inarea = " << inArea(pt2->lat, pt2->lon) << endl;
		cout << "maxRow = " << gridHeight - 1 << " maxCol = " << gridWidth - 1 << endl;
		system("pause");
		//TODO����һ��û����ϸ����ܲ�����ôд
		/*if (row1 >= gridHeight)	row1 = gridHeight;
		if (row2 >= gridHeight)	row2 = gridHeight;
		if (col1 >= gridWidth)	col1 = gridWidth;
		if (col2 >= gridWidth)	col2 = gridWidth;
		if (row1 < 0)	row1 = 0;
		if (row2 < 0)	row2 = 0;
		if (col1 < 0)	col1 = 0;
		if (col2 < 0)	col2 = 0;*/
	}
	double A = y2 - y1;
	double B = -(x2 - x1);
	double C = -B * y1 - A * x1;
	int i, j;
	//pt1,pt2����һ��cell��
	if (row1 == row2 && col1 == col2)
	{
		insertEdgeIntoGrid(edge, row1, col1);
		return;
	}
	//ֻ��Խ�������
	if (row1 == row2)
	{
		//ͷ
		double headDist = ((min(col1, col2) + 1) * gridSizeDeg - min(x1, x2)) / gridSizeDeg;
		if (headDist / gridSizeDeg > strictThreshold)
			insertEdgeIntoGrid(edge, row1, min(col1, col2));
		//�м�
		for (i = min(col1, col2) + 1; i < max(col1, col2); i++)
		{
			insertEdgeIntoGrid(edge, row1, i);
		}
		//β
		double tailDist = (max(x1, x2) - max(col1, col2) * gridSizeDeg) / gridSizeDeg;
		if (tailDist / gridSizeDeg > strictThreshold)
			insertEdgeIntoGrid(edge, row1, max(col1, col2));
		return;
	}
	//ֻ��Խ�������
	if (col1 == col2)
	{
		//ͷ
		double headDist = ((min(row1, row2) + 1) * gridSizeDeg - min(y1, y2)) / gridSizeDeg;
		if (headDist / gridSizeDeg > strictThreshold)
			insertEdgeIntoGrid(edge, min(row1, row2), col1);
		//�м�
		for (i = min(row1, row2) + 1; i < max(row1, row2); i++)
		{
			insertEdgeIntoGrid(edge, i, col1);
		}
		//β
		double tailDist = (max(y1, y2) - max(row1, row2) * gridSizeDeg) / gridSizeDeg;
		if (tailDist / gridSizeDeg > strictThreshold)
			insertEdgeIntoGrid(edge, max(row1, row2), col1);
		return;
	}
	//б��Խ
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
	double xL = leftPt->lon - area->minLon;
	double xR = rightPt->lon - area->minLon;
	double yL = leftPt->lat - area->minLat;
	double yR = rightPt->lat - area->minLat;
	
	//ͷ
	double headDist = sqrt((xL - pts[0].first)*(xL - pts[0].first) + (yL - pts[0].second)*(yL - pts[0].second));
	if (headDist / gridSizeDeg > strictThreshold)
	{
		insertEdgeIntoGrid(edge, (int)(yL / gridSizeDeg), (int)(xL / gridSizeDeg));
	}
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
			insertEdgeIntoGrid(edge, row, col);
		}
	}
	//β
	double tailDist = sqrt((xR - pts[n_pts - 1].first)*(xR - pts[n_pts - 1].first) + (yR - pts[n_pts - 1].second)*(yR - pts[n_pts - 1].second));
	if (tailDist / gridSizeDeg > strictThreshold)
	{
		insertEdgeIntoGrid(edge, (int)(yR / gridSizeDeg), (int)(xR / gridSizeDeg));
	}
	return;
}

void Map::createGridIndexForEdge(Edge *edge)
{
	if (edge == NULL)
		return;
	Figure::iterator ptIter = edge->figure->begin();
	Figure::iterator nextPtIter = edge->figure->begin(); nextPtIter++;
	while (nextPtIter != edge->figure->end())
	{
		createGridIndexForSegment(edge, *ptIter, *nextPtIter);
		ptIter++;
		nextPtIter++;
	}
}

void Map::insertEdge(int edgeId, int startNodeId, int endNodeId)
{
	//////////////////////////////////////////////////////////////////////////
	///���ڽӱ�adjList�в���һ���ߵ���ͨ��ϵ�����ι���ͼʱʹ�ã�˽�а汾���������ⲿ����
	///TODO: ���ܻ�������@Line1047��while
	//////////////////////////////////////////////////////////////////////////
	/*if (startNodeId == -1)
	{
		system("pause");
	}
	if (startNodeId >= adjList.size())
	{
		cout << "startNodeId: " << startNodeId;
		cout << " adjList: " << adjList.size() << endl;
	}*/
	AdjNode* current = adjList[startNodeId];
	
	while (current->next != NULL)
	{
		current = current->next;
	}
	AdjNode* tmpAdjNode = new AdjNode();
	tmpAdjNode->endPointId = endNodeId;
	tmpAdjNode->edgeId = edgeId;
	tmpAdjNode->next = NULL;
	current->next = tmpAdjNode;
	edges[edgeId]->startNodeId = startNodeId;
	edges[edgeId]->endNodeId = endNodeId;
}

int Map::getRowId(double lat) const
{
	return (int)((lat - area->minLat) / gridSizeDeg);
}

int Map::getColId(double lon) const
{
	return (int)((lon - area->minLon) / gridSizeDeg);
}

double Map::cosAngle(GeoPoint* pt1, GeoPoint* pt2, GeoPoint* pt3) const
{
	double v1x = pt2->lon - pt1->lon;
	double v1y = pt2->lat - pt1->lat;
	double v2x = pt3->lon - pt2->lon;
	double v2y = pt3->lat - pt2->lat;
	return (v1x * v2x + v1y * v2y) / sqrt((v1x * v1x + v1y * v1y)*(v2x * v2x + v2y * v2y));
}

void Map::split(const string& src, const string& separator, vector<string>& dest)
{
	std::string str = src;
	std::string substring;
	std::string::size_type start = 0, index;
	do
	{
		index = str.find_first_of(separator, start);
		if (index != std::string::npos)
		{
			substring = str.substr(start, index - start);
			dest.push_back(substring);
			start = str.find_first_not_of(separator, index);
			if (start == std::string::npos) return;
		}
	} while (index != std::string::npos);
	//the last token
	substring = str.substr(start);
	dest.push_back(substring);
}

void Map::split(const string& src, const char& separator, vector<string>& dest)
{
	int index = 0, start = 0;
	while (index != src.size())
	{
		if (src[index] == separator)
		{
			string substring = src.substr(start, index - start);
			dest.push_back(substring);
			while (src[index + 1] == separator)
			{
				dest.push_back("");
				index++;
			}
			index++;
			start = index;
		}
		else
			index++;
	}
	//the last token
	string substring = src.substr(start);
	dest.push_back(substring);
	
}

double Map::distM(Edge* edge1, Edge* edge2, double thresholdM)
{
	//////////////////////////////////////////////////////////////////////////
	///��������·�ε���̾���(һ��·���ϵĵ㵽��һ��·�ε���̾���),���ص�λΪ��
	///�����������з����Ѿ�С��threshold��,��ֱ�ӷ���
	///ע�⣺����hausdoff���룡
	//////////////////////////////////////////////////////////////////////////
	double minDist = 9999999;
	//��һ�����ڶ���
	for (Figure::iterator iter = edge1->figure->begin(); iter != edge1->figure->end(); iter++)
	{
		double tmpDist = distM((*iter)->lat, (*iter)->lon, edge2);
		if (tmpDist < minDist)
		{
			minDist = tmpDist;
			if (minDist < thresholdM)
				return minDist;
		}
	}
	//�ڶ�������һ��
	for (Figure::iterator iter = edge2->figure->begin(); iter != edge2->figure->end(); iter++)
	{
		double tmpDist = distM((*iter)->lat, (*iter)->lon, edge1);
		if (tmpDist < minDist)
		{
			minDist = tmpDist;
			if (minDist < thresholdM)
				return minDist;
		}
	}
	return minDist;
}

void Map::getNearEdges(Edge* edge, double thresholdM, vector<Edge*>& dest)
{
	//////////////////////////////////////////////////////////////////////////
	///�������о�ȷ����edgeС��thresholdM��·�Σ�����dest
	//////////////////////////////////////////////////////////////////////////
	dest.clear();
	int gridSearchRange = int(thresholdM / (gridSizeDeg * GeoPoint::geoScale)) + 1;
	int minRow = INFINITE, maxRow = -1, minCol = INFINITE, maxCol = -1;
	for each (GeoPoint* pt in *(edge->figure))
	{
		int row = (pt->lat - area->minLat) / gridSizeDeg;
		int col = (pt->lon - area->minLon) / gridSizeDeg;
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
	double minDist = INFINITE;
	vector<Edge*> failList;
	for (int i = minRow; i <= maxRow; i++)
	{
		for (int j = minCol; j <= maxCol; j++)
		{
			for (list<Edge*>::iterator iter = grid[i][j]->begin(); iter != grid[i][j]->end(); iter++)
			{
				if ((*iter) == NULL)
					continue;
				if ((*iter) == edge) //��ֹ���Լ��ӽ�ȥ
					continue;
				//���ε�����δ���ʹ�
				if ((*iter)->visited != true)
				{
					(*iter)->visited = true; //���Ϊ�ѷ���
					double dist = distM(edge, (*iter), thresholdM);
					if (dist < thresholdM)// && dist > eps)
					{
						dest.push_back((*iter));
					}			
					else
						failList.push_back((*iter));
				}
			}
		}
	}
	//visitFlag����
	for (int i = 0; i < dest.size(); i++)
	{
		dest[i]->visited = false;
	}
	for (int i = 0; i < failList.size(); i++)
	{
		failList[i]->visited = false;
	}	
}

bool smallerInX(simplePoint& pt1, simplePoint& pt2)
{
	//////////////////////////////////////////////////////////////////////////
	///��ֱ�������񽻵�ͳһ����x�������������
	//////////////////////////////////////////////////////////////////////////
	return pt1.first < pt2.first;
}

bool smallerInDist(pair<Edge*, double>& c1, pair<Edge*, double>& c2)
{
	//////////////////////////////////////////////////////////////////////////
	///void getNearEdges(double lat, double lon, int k, vector<Edge*>& dest)������ʹ�õ��ıȽϺ���
	//////////////////////////////////////////////////////////////////////////
	return c1.second < c2.second;
}