#include "DCMU.h"

//////////////////////////////////////////////////////////////////////////
///public part
//////////////////////////////////////////////////////////////////////////

void DCMU::run()
{
	//�������
	gridClustering();
	//����
 	for (int i = 0; i < ptIndexes.size(); i++)
 	{
 		denoiser.run(ptIndexes[i]);
 
// 		/**********************************************************/
// 		/*test code starts from here*/
// 		denoiser.drawPts(md);
// 		/*test code ends*/
// 		/**********************************************************/
// 
 	}
	
	//�������
	for (int i = 0; i < ptIndexes.size(); i++)
	{
		ptCluster.run(ptIndexes[i]);
		ptCluster.drawClusters(md);
	}

	//����·��
	RoadGenerator rg;
	rg.run(&allPtIndex, ptCluster.clusters);
}


void DCMU::run1()
{
	//�������
	int index = 54;
	gridClustering();
	denoiser.run(ptIndexes[index]);

	//����
	// 	for (int i = 0; i < ptIndexes.size(); i++)
	// 	{
	// 
	// 		/**********************************************************/
	// 		/*test code starts from here*/
	// 		denoiser.drawPts(md);
	// 		/*test code ends*/
	// 		/**********************************************************/
	// 
	// 	}

	//�������
	ptCluster.run(ptIndexes[index]);
	ptCluster.drawClusters(md);

	//����·��
	RoadGenerator rg;
	rg.run(ptIndexes[index], ptCluster.clusters);
}


//////////////////////////////////////////////////////////////////////////
///private part
//////////////////////////////////////////////////////////////////////////
void DCMU::gridClustering()
{
	//////////////////////////////////////////////////////////////////////////
	///����ֲھ��࣬ʵ���Ͼ��Ǹ�dfs
	///��ÿ�������еĵ������ptIndexes[i]
	///�����ͨ��֧��С����ԣ������
	//////////////////////////////////////////////////////////////////////////
	bool** dfsState = new bool*[allPtIndex.gridHeight];
	for (int i = 0; i < allPtIndex.gridHeight; i++)
		dfsState[i] = new bool[allPtIndex.gridWidth];
	for (int i = 0; i < allPtIndex.gridHeight; i++)
	{
		for (int j = 0; j < allPtIndex.gridWidth; j++)
		{
			dfsState[i][j] = false;
		}
	}
	int count = 0; //��ͨ��֧���

	for (int row = 0; row < allPtIndex.gridHeight; row++)
	{
		for (int col = 0; col < allPtIndex.gridWidth; col++)
		{
			if (dfsState[row][col])
				continue;
			else
			{
				vector<pair<int, int>> connectingCompnt; //��¼��ǰdfs����ͨ��֧
				dfs(row, col, dfsState, connectingCompnt);
				if (connectingCompnt.size() < 10) //��ͨ��֧��С������
					continue;
				else
				{
					//������ͨ��֧ȥ������ѵ����
					PointGridIndex* tempPtIndex = new PointGridIndex;
					list<GeoPoint*> tempPts;
					for (int i = 0; i < connectingCompnt.size(); i++)
					{						
						for each (GeoPoint* pt in *(allPtIndex.grid[connectingCompnt[i].first][connectingCompnt[i].second]))
						{
							tempPts.push_back(pt);
						}
					}
					tempPtIndex->createIndex(tempPts, &area, gridWidth);
					ptIndexes.push_back(tempPtIndex);
					count++;
				}
			}
		}
	}	
}

void DCMU::dfs(int row, int col, bool** dfsState, vector<pair<int, int>>& connectingCompnt)
{
	//////////////////////////////////////////////////////////////////////////
	///������[row][col]����dfs���������б�Ƿ���Ϊ�˺���ְ��dfsStateΪ����״̬��trueΪ�ѷ���
	///���Լ���������Ŵ���connectingCompnt�Լ�¼��ͨ��֧��Ϣ
	//////////////////////////////////////////////////////////////////////////
	if (row >= allPtIndex.gridHeight || row < 0 || col >= allPtIndex.gridWidth || col < 0)
		return;
	if (dfsState[row][col])
		return;
	else
	{
		dfsState[row][col] = true;
		if (allPtIndex.grid[row][col]->size() <= 3) //�����������С��һ���̶ȵĺ���
			return;
		connectingCompnt.push_back(make_pair(row, col));
		dfs(row + 1, col, dfsState, connectingCompnt);
		dfs(row - 1, col, dfsState, connectingCompnt);
		dfs(row, col + 1, dfsState, connectingCompnt);
		dfs(row, col - 1, dfsState, connectingCompnt);
	}
}