#include "MapMatching.h"


//��ͼƥ�����ò���
#define SIGMAZ 4.591689
#define N2_SIGMAZ2 -0.0237151062104234
#define SQR_2PI_SIGMAZ 0.0868835586212075
#define RANGEOFCANADIDATEEDGES 50.0 //��ѡ·��ѡȡ��Χ
#define TRAJSAMPLERATE 30 //�켣������
#define MINPROB 1e-150 //������ʵ�����
#define INF  1000000000 //���·������
#define MAXSPEED 50 //����ٶ�
typedef list<GeoPoint*> Traj;

//�ڲ�ͬ�����ʣ�1~30s/�㣩�¸���beta��ֵ�����ó�= =
const double BETA_ARR[31] = {
	0,
	0.490376731, 0.82918373, 1.24364564, 1.67079581, 2.00719298,
	2.42513007, 2.81248831, 3.15745473, 3.52645392, 4.09511775,
	4.67319795, 5.41088180, 6.47666590, 6.29010734, 7.80752112,
	8.09074504, 8.08550528, 9.09405065, 11.09090603, 11.87752824,
	12.55107715, 15.82820829, 17.69496773, 18.07655652, 19.63438911,
	25.40832185, 23.76001877, 28.43289797, 32.21683062, 34.56991141
};

//��ͼƥ���������ݽṹ
struct Score//����ĳ���켣���Ӧ��һ����ѡ·��
{
	Edge* edge;//��ѡ·�ε�ָ��
	long double score;//��ѡ·�������е��������
	int preColumnIndex;//��ѡ·�ε�ǰ��·�ε�������
	double distLeft;//�켣���ͶӰ�㵽��ѡ·�����ľ���
	Score(Edge* edge, long double score, int pre, double distLeft){
		this->edge = edge;
		this->score = score;
		this->preColumnIndex = pre;
		this->distLeft = distLeft;
	}
};

//���·�������������ݽṹ
struct NODE_DIJKSTRA {
	int t; double dist;
	NODE_DIJKSTRA(int i, double dist){
		this->t = i;
		this->dist = dist;
	}
	bool operator < (const NODE_DIJKSTRA &rhs) const {
		return dist > rhs.dist;
	}
};

//A·����㵽B·��������С·������
double ShortestPathLength(int ID1, int ID2, double dist1, double dist2, double deltaT){
	int size = map.nodes.size();
	//double *dist = new double[size];
	double dist[100000];
	//bool *flag = new bool[size];
	bool flag[100000];
	memset(flag, 0, sizeof(flag));
	for (int i = 0; i < size; i++) {
		dist[i] = INF;
	}
	dist[ID1] = 0;
	priority_queue<NODE_DIJKSTRA> Q;
	NODE_DIJKSTRA tmp(ID1, 0);
	Q.push(tmp);
	while (!Q.empty()) {
		NODE_DIJKSTRA x = Q.top();
		Q.pop();
		int u = x.t;
		if (x.dist + dist1 - dist2 > deltaT*MAXSPEED){
			return INF;
		}
		if (flag[u]) {
			continue;
		}
		flag[u] = true;
		if (u == ID2) {
			break;
		}
		for (AdjNode* i = map.adjList[u]->next; i != NULL; i = i->next) {
			if (dist[i->endPointId] > dist[u] + map.edges[i->edgeId]->lengthM) {
				dist[i->endPointId] = dist[u] + map.edges[i->edgeId]->lengthM;
				NODE_DIJKSTRA tmp(i->endPointId, dist[i->endPointId]);
				Q.push(tmp);
			}
		}
	}
	double resultLength = dist[ID2];
	//delete []dist;
	//delete []flag;
	return resultLength;
}

//������ʣ�ʹ�ù켣�㵽��ѡ·�εľ����ڸ�˹�ֲ��ϵĸ���
//����t��ΪWang Yin�㷨���裬��ʾǰ��켣����ʱ���
//����dist���켣�㵽��ѡ·�εľ���
double logEmissionProb(double t, double dist){
	return ((t + dist * dist + N2_SIGMAZ2))*log(exp(1)) + log(SQR_2PI_SIGMAZ);
}

//��������������������������scoreMatrix�и���������������ĺ�ѡ·�ε�����
int GetStartColumnIndex(vector<Score> &row){
	int resultIndex = -1;
	long double currentMaxProb = -1;
	for (int i = 0; i<row.size(); i++){
		if (row.at(i).score>currentMaxProb){
			currentMaxProb = row.at(i).score;
			resultIndex = i;
		}
	}
	return resultIndex;
}

list<Edge*> MapMatching(Traj &trajectory){
	list<Edge*> mapMatchingResult;
	long double BT = (long double)BETA_ARR[TRAJSAMPLERATE];//��ǰ�켣�����õ�betaֵ������ת�Ƹ���ʱʹ��
	vector<vector<Score>> scoreMatrix = vector<vector<Score>>();//���й켣��ĸ��ʾ���
	GeoPoint* formerTrajPoint = NULL;//��һ���켣�㣬����·������ʱ��Ҫ
	bool cutFlag = true;//û��ǰһ�켣���ǰһ�켣��û�д��ĺ�ѡ·��
	int currentTrajPointIndex = 0;//��ǰ�켣�������
	list<GeoPoint*>::iterator trajectoryIterator = trajectory.begin();
	double deltaT;
	for (; trajectoryIterator != trajectory.end(); trajectoryIterator++)
	{
		long double currentMaxProb = -1e10;//��ǰ������
		vector<Score> scores = vector<Score>();//��ǰ�켣���Score����
		//vector<Edge*> canadidateEdges = map.getNearEdges((*trajectoryIterator)->lat, (*trajectoryIterator)->lon, RANGEOFCANADIDATEEDGES);//���������ָ����Χ�ڵĺ�ѡ·�μ���

		long double *emissionProbs = new long double[canadidateEdges.size()];//������Щ��ѡ·�εķ������
		int currentCanadidateEdgeIndex = 0;//��ǰ��ѡ·�ε�����
		
		for each (Edge* canadidateEdge in canadidateEdges)
		{
			int preColumnIndex = -1;//���浱ǰ��ѡ·�ε�ǰ��·�ε�������
			double currentDistLeft = 0;//��ǰ�켣���ں�ѡ·���ϵ�ͶӰ���·�����ľ���
			double DistBetweenTrajPointAndEdge = map.distM((*trajectoryIterator)->lat, (*trajectoryIterator)->lon, canadidateEdge, currentDistLeft);
			//������Щ��ѡ·�εķ�����ʣ�Wang Yinд����
			if (cutFlag == false){
				deltaT = (*trajectoryIterator)->time - formerTrajPoint->time;
				emissionProbs[currentCanadidateEdgeIndex] = logEmissionProb(deltaT, DistBetweenTrajPointAndEdge);
			}
			else{
				//����д��
				emissionProbs[currentCanadidateEdgeIndex] = logEmissionProb(1, DistBetweenTrajPointAndEdge);
			}
			//����ǰ���ǵ�һ���켣�㣬�������Щ��ѡ·�ε�ת�Ƹ��ʣ����Ҳ����emissionProbs����
			if (cutFlag == false){
				int currentMaxProbTmp = -1e10;
				for (int i = 0; i < scoreMatrix.back().size(); i++)
				{
					double formerDistLeft = scoreMatrix[currentTrajPointIndex - 1][i].distLeft;//ǰһ���켣���ں�ѡ·���ϵ�ͶӰ���·�����ľ���
					double routeNetworkDistBetweenTwoEdges;//��·������ľ���
					double routeNetworkDistBetweenTwoTrajPoints;//���켣���Ӧ��ͶӰ����·������
					if (canadidateEdge == scoreMatrix[currentTrajPointIndex - 1][i].edge){
						routeNetworkDistBetweenTwoTrajPoints = currentDistLeft - scoreMatrix[currentTrajPointIndex - 1][i].distLeft;
					}
					else
					{
						routeNetworkDistBetweenTwoEdges = ShortestPathLength(scoreMatrix[currentTrajPointIndex - 1][i].edge->startNodeId, canadidateEdge->startNodeId, currentDistLeft, formerDistLeft, deltaT);
						routeNetworkDistBetweenTwoTrajPoints = routeNetworkDistBetweenTwoEdges + currentDistLeft - formerDistLeft;
					}
					//���켣����ֱ�Ӿ���
					double distBetweenTwoTrajPoints = GeoPoint::distM((*trajectoryIterator)->lat, (*trajectoryIterator)->lon, formerTrajPoint->lat, (*trajectoryIterator)->lon);
					//double transactionProb = exp(-fabs((long double)routeNetworkDistBetweenTwoTrajPoints - (long double)distBetweenTwoTrajPoints) / BT) / BT;
					//Wang Yin�㷨
					long double transactionProb = log(exp(-fabs((long double)routeNetworkDistBetweenTwoTrajPoints) / BT) / BT);
					/*GIS2012CUP���Ż����ڴ˴�����transactionProb�����޸�*/
					long double tmpTotalProbForTransaction = scoreMatrix[currentTrajPointIndex - 1][i].score + transactionProb;
					if (tmpTotalProbForTransaction > currentMaxProbTmp){
						currentMaxProbTmp = tmpTotalProbForTransaction;
						preColumnIndex = i;
					}
				}
				emissionProbs[currentCanadidateEdgeIndex] += currentMaxProbTmp;
			}
			//if (emissionProbs[currentCanadidateEdgeIndex] > MINPROB){
			//	scores.push_back(Score(canadidateEdge, emissionProbs[currentCanadidateEdgeIndex], preColumnIndex, distLeft));
			//}
			scores.push_back(Score(canadidateEdge, emissionProbs[currentCanadidateEdgeIndex], preColumnIndex, currentDistLeft));
			if (currentMaxProb < emissionProbs[currentCanadidateEdgeIndex]){
				currentMaxProb = emissionProbs[currentCanadidateEdgeIndex];
			}
			currentCanadidateEdgeIndex++;
		}
		delete[]emissionProbs;
		formerTrajPoint = (*trajectoryIterator);
		currentTrajPointIndex++;
		for (int i = 0; i < scores.size(); i++)
		{
			scores[i].score /= currentMaxProb;
		}
		//�Ѹù켣���Scores�������scoreMatrix��
		scoreMatrix.push_back(scores);
		//��scores����Ϊ�գ���˵��û��һ�����ĺ�ѡ·�Σ�cutFlag��Ϊtrue�������켣��Ϊ�¹켣����ƥ��
		if (scores.size() == 0){
			cutFlag = true;
			formerTrajPoint = NULL;
		}
		else
		{
			cutFlag = false;
		}
	}
	//�õ�ȫ��ƥ��·��
	int startColumnIndex = GetStartColumnIndex(scoreMatrix.back());//�õ����һ���켣�����scoreMatrix��Ӧ���е÷���ߵ�����������ȫ��ƥ��·�����յ�
	for (int i = scoreMatrix.size() - 1; i >= 0; i--){
		if (startColumnIndex != -1){
			mapMatchingResult.push_front(scoreMatrix[i][startColumnIndex].edge);
			startColumnIndex = scoreMatrix[i][startColumnIndex].preColumnIndex;
		}
		else
		{
			mapMatchingResult.push_front(NULL);
			if (i > 0){
				startColumnIndex = GetStartColumnIndex(scoreMatrix[i - 1]);
			}
		}
	}
	return mapMatchingResult;

	//�õ��񾭲�·����
	//list<list<GeoPoint*>> damnTrajPointsList = list<list<GeoPoint*>>();
	//bool startFlag = true;
	//list<GeoPoint*>::iterator trajIter = Traj.begin();
	//for each (Edge* matchedEdge in mapMatchingResult)
	//{
	//	if (matchedEdge == NULL){
	//		if (startFlag){
	//			startFlag = false;
	//			damnTrajPointsList.push_back(list<GeoPoint*>());
	//			if (trajIter != Traj.begin()){
	//				trajIter--;
	//				damnTrajPointsList.back().push_back(*trajIter);
	//				trajIter++;
	//			}	
	//		}
	//		damnTrajPointsList.back().push_back(*trajIter);
	//	}
	//	else{
	//		if (startFlag == false){
	//			damnTrajPointsList.back().push_back(*trajIter);
	//		}
	//		startFlag = true;
	//	}
	//	trajIter++;
	//}
	//return damnTrajPointsList;
}