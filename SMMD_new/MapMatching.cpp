/* 
 * Last Updated at [2014/9/1 12:33] by wuhao
 */
#include "MapMatching.h"

MapMatcher::MapMatcher(Map* roadNetwork)
{
	this->roadNetwork = roadNetwork;
}

void MapMatcher::setMap(Map* roadNetwork)
{
	this->roadNetwork = roadNetwork;
}

double MapMatcher::EmissionProb(double t, double dist)
{
	//////////////////////////////////////////////////////////////////////////
	///放射概率计算函数
	///放射概率：使用轨迹点到候选路段的距离在高斯分布上的概率
	///参数t：为Wang Yin算法而设，表示前后轨迹点间的时间差
	///参数dist：轨迹点到候选路段的距离
	//////////////////////////////////////////////////////////////////////////
	
	return t*sqrt(dist);
	//return t*sqrt(dist)*COEFFICIENT_FOR_EMISSIONPROB;
}

int MapMatcher::GetStartColumnIndex(vector<Score> &row)
{
	//////////////////////////////////////////////////////////////////////////
	///辅助函数：给定行索引，计算scoreMatrix中该行中整体概率最大的候选路段的索引
	//////////////////////////////////////////////////////////////////////////
	
	int resultIndex = -1;
	long double currentMaxProb = 1e10;
	for (size_t i = 0; i < row.size(); i++)
	{
		if (currentMaxProb > row.at(i).score)
		{
			currentMaxProb = row.at(i).score;
			resultIndex = i;
		}
	}
	return resultIndex;
}

void MapMatcher::MapMatching(list<GeoPoint*> &trajectory, list<Edge*>& resultEdges, double rangeOfCandidateEdges)
{
	resultEdges.clear();
	vector<vector<Score>> scoreMatrix = vector<vector<Score>>();//所有轨迹点的概率矩阵
	//需要在每次循环结束前更新的变量
	GeoPoint* formerTrajPoint = NULL;//上一个轨迹点，计算路网距离时需要
	bool cutFlag = true;//没有前一轨迹点或前一轨迹点没有达标的候选路段
	int currentTrajPointIndex = 0;//当前轨迹点的索引
	for (list<GeoPoint*>::iterator trajectoryIterator = trajectory.begin(); trajectoryIterator != trajectory.end(); trajectoryIterator++)//遍历每个轨迹点
	{
		double deltaT = 1;//当前序轨迹点存在时，delaT表示前后两轨迹点间的时间差
		if (formerTrajPoint != NULL){ deltaT = (*trajectoryIterator)->time - formerTrajPoint->time; }
		long double currentMaxProb = 1e10;//当前最大整体概率，初始值为1e10
		vector<Score> scores = vector<Score>();//当前轨迹点的Score集合
		vector<Edge*> canadidateEdges;//候选路段集合
		roadNetwork->getNearEdges((*trajectoryIterator)->lat, (*trajectoryIterator)->lon, rangeOfCandidateEdges, canadidateEdges);//获得所有在指定范围内的候选路段集合
		long double *emissionProbs = new long double[canadidateEdges.size()];//保存这些候选路段的放射概率
		int currentCanadidateEdgeIndex = 0;//当前候选路段的索引
		for each (Edge* canadidateEdge in canadidateEdges)
		{
			int preColumnIndex = -1;//保存当前候选路段的前序路段的列索引
			double currentDistLeft = 0;//当前轨迹点在候选路段上的投影点距路段起点的距离
			double DistBetweenTrajPointAndEdge = roadNetwork->distMFromTransplantFromSRC((*trajectoryIterator)->lat, (*trajectoryIterator)->lon, canadidateEdge, currentDistLeft);
			//计算这些候选路段的放射概率
			emissionProbs[currentCanadidateEdgeIndex] = deltaT*sqrt(DistBetweenTrajPointAndEdge);
			if (!cutFlag)
			{
				//如果当前采样点不是轨迹第一个点或匹配中断后的第一个点，则计算来自前序轨迹点匹配路段的转移概率
				long double currentMaxProbTmp = 1e10;//当前最大转移概率，初始值为1e10
				int formerCanadidateEdgeIndex = 0;
				for each(Score formerCanadidateEdge in scoreMatrix.back())
				{
					double formerDistLeft = formerCanadidateEdge.distLeft;//前一个轨迹点在候选路段上的投影点距路段起点的距离
					double formerDistToEnd = formerCanadidateEdge.edge->lengthM - formerDistLeft;//前一个轨迹点在候选路段上的投影点距路段终点的距离
					double routeNetworkDistBetweenTwoEdges;//两路段起点间的距离
					double routeNetworkDistBetweenTwoTrajPoints;//两轨迹点对应的投影点间的路网距离					
					if (canadidateEdge == formerCanadidateEdge.edge)
					{
						//如果前一匹配路段和当前候选路段是同一路段，则两者计算距离起点的差即为路网距离
						routeNetworkDistBetweenTwoTrajPoints = fabs(currentDistLeft - formerCanadidateEdge.distLeft);
					}
					else
					{
						pair<int, int> odPair = make_pair(formerCanadidateEdge.edge->endNodeId, canadidateEdge->startNodeId);
						//给定起点和终点最短路已经计算过，且不是INF
						if (shortestDistPair.find(odPair) != shortestDistPair.end() && shortestDistPair[odPair].first < INF)
						{
							//如果当前deltaT下的移动距离上限比最短距离要大，调用最短路函数得到的也是保存的距离值；反之得到的就是INF
							shortestDistPair[odPair].first <= MAXSPEED*deltaT ? routeNetworkDistBetweenTwoEdges = shortestDistPair[odPair].first : routeNetworkDistBetweenTwoEdges = INF;
						}
						else
						{
							if (shortestDistPair.find(odPair) != shortestDistPair.end() && deltaT <= shortestDistPair[odPair].second)
							{//保存的给定起点和终点最短路结果是INF，且当前deltaT比上次计算最短路时的移动时间要小，说明当前deltaT下得到的最短路距离仍是INF
								routeNetworkDistBetweenTwoEdges = INF;
							}
							else
							{
								//或者未保存给定起点和终点的最短路结果；或者当前deltaT比保存的deltaT要大，可能得到真正的最短路结果；总之就是要调用函数计算最短路
								list<Edge*> shortestPath = list<Edge*>();
								routeNetworkDistBetweenTwoEdges = roadNetwork->shortestPathLength(formerCanadidateEdge.edge->endNodeId, canadidateEdge->startNodeId, currentDistLeft, formerDistToEnd, deltaT);
								shortestDistPair[odPair] = make_pair(routeNetworkDistBetweenTwoEdges, deltaT);
							}
						}
						routeNetworkDistBetweenTwoTrajPoints = routeNetworkDistBetweenTwoEdges + currentDistLeft + formerDistToEnd;
					}
					//double distBetweenTwoTrajPoints = GeoPoint::distM((*trajectoryIterator)->lat, (*trajectoryIterator)->lon, formerTrajPoint->lat, formerTrajPoint->lon);//两轨迹点间的直接距离
					long double transactionProb = (long double)routeNetworkDistBetweenTwoTrajPoints*COEFFICIENT_FOR_TRANSATIONPROB;//转移概率
					long double tmpTotalProbForTransaction = formerCanadidateEdge.score + transactionProb;
					if (currentMaxProbTmp > tmpTotalProbForTransaction)
					{
						//保留当前转移概率和已知最小转移概率中较小者（由于取log的缘故）
						currentMaxProbTmp = tmpTotalProbForTransaction;
						preColumnIndex = formerCanadidateEdgeIndex;
					}
					formerCanadidateEdgeIndex++;
				}
				//此时，emissionProbs保存的是候选路段的放射概率，加上转移概率则变为候选路段的整体概率（由于取log的缘故）
				emissionProbs[currentCanadidateEdgeIndex] += currentMaxProbTmp;
			}
			/*若需要提升代码运行速度，则只把整体概率大于MINPROB的候选路段放入当前轨迹点的Score集合中；否则把所有候选路段放入Score集合中*/
			scores.push_back(Score(canadidateEdge, emissionProbs[currentCanadidateEdgeIndex], preColumnIndex, currentDistLeft));
			//得到当前最小整体概率（由于取log的缘故），以便归一化
			if (currentMaxProb > emissionProbs[currentCanadidateEdgeIndex])
			{
				currentMaxProb = emissionProbs[currentCanadidateEdgeIndex]; 
			}
			currentCanadidateEdgeIndex++;
		}
		delete[]emissionProbs;
		formerTrajPoint = *trajectoryIterator;
		currentTrajPointIndex++;
		//for (int i = 0; i < scores.size(); i++)	{ scores[i].score /= currentMaxProb; }//归一化		
		scoreMatrix.push_back(scores);//把该轨迹点的Scores数组放入scoreMatrix中
		if (scores.size() == 0)
		{//若scores数组为空，则说明没有一个达标的候选路段，cutFlag设为true，后续轨迹作为新轨迹进行匹配
			cutFlag = true;
			formerTrajPoint = NULL;
		}
		else
		{
			cutFlag = false;
		}
	}

	//得到全局匹配路径
	int startColumnIndex = GetStartColumnIndex(scoreMatrix.back());//得到最后一个轨迹点的在scoreMatrix对应行中得分最高的列索引，即全局匹配路径的终点
	for (int i = scoreMatrix.size() - 1; i >= 0; i--)
	{
		if (startColumnIndex != -1)
		{
			resultEdges.push_front(scoreMatrix[i][startColumnIndex].edge);
			startColumnIndex = scoreMatrix[i][startColumnIndex].preColumnIndex;
		}
		else
		{
			resultEdges.push_front(NULL);
			if (i > 0)
			{
				startColumnIndex = GetStartColumnIndex(scoreMatrix[i - 1]);
			}
		}
	}
	//调试代码：输出最终的概率矩阵：如果有某个轨迹点的所有候选路段的整体概率均为均为无穷小/无穷大，那可能就不正常，需要进一步检查该行概率的得到过程
	//for (int i = 0; i < scoreMatrix.size(); i++){
	//	logOutput << scoreMatrix.at(i).size() << "\t";
	//	for (int j = 0; j < scoreMatrix.at(i).size(); j++){
	//		logOutput << "[" << scoreMatrix.at(i).at(j).edge->id << "][" << scoreMatrix.at(i).at(j).preColumnIndex << "][" << scoreMatrix.at(i).at(j).score << "]\t";
	//	}
	//	logOutput << endl;
	//}
	//调试结束

	//return resultEdges;
	//return linkMatchedResult(mapMatchingResult);
}
