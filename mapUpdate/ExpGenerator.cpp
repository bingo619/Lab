/*
* Last Updated at [2014/9/24 11:07] by wuhao
*/
#include "ExpGenerator.h"

void ExpGenerator::genExpData()
{
	//////////////////////////////////////////////////////////////////////////
	///生成实验数据
	///1）两次地图匹配后的区域内所有的轨迹点
	///2）1中匹配失败的轨迹点
	//////////////////////////////////////////////////////////////////////////
	//输出所有经过两次匹配后的轨迹
	for (int i = 0; i < inputFileNames.size(); i++)
	{
		cout << ">> 处理文件 " << inputFileNames[i] << endl;
		genExpData(inputFolder + inputFileNames[i]);
	}
	//输出两次匹配后失败的轨迹
	extractUnmatchedTrajs();
}

void ExpGenerator::genExpData(string rawTrajFilePath)
{
	double mmThres = 25.0;
	readRawTrajs(rawTrajFilePath);
	doSplit();
	deleteList(rawTrajs); //释放rawTrajs内存
	//第一次使用原地图匹配
	cout << ">> 进行第一次地图匹配" << endl;
	doMM(&originalRoadNetwork, trajsInArea, mmThres);

	//将结果存入每个pt的mmRoadId2字段
	for each(Traj* traj in trajsInArea)
	{
		for each (GeoPoint* pt in *traj)
		{
			pt->mmRoadId2 = pt->mmRoadId;
		}
	}

	//第二次使用地图匹配
	cout << ">> 进行第二次地图匹配" << endl;
	doMM(&roadNetwork, trajsInArea, mmThres);

	outputNewTrajs(trajsInArea);
	deleteList(trajsInArea);
	//dumpTo(trajsInArea, doneTrajs);
}

void ExpGenerator::genSubSampledData(int interval, string folder, string fileName)
{
	//////////////////////////////////////////////////////////////////////////
	///对原采样轨迹数据,路径为folder+fileName，进行subsample,输出文件名为interval_fileName，在程序根目录下
	///[注意]只适用于新加坡数据，原采样间隔为30s
	///interval只能选60,90，120这三个值
	//////////////////////////////////////////////////////////////////////////
	char buffer[20];
	_itoa_s(interval, buffer, 10);
	string outPath = buffer;
	outPath += "_" + fileName;
	cout << outPath << endl;
	ifstream ifs(folder + fileName);
	ofstream ofs(outPath);
	if (!ifs)
	{
		cout << "open" << folder + fileName << "error" << endl;
		system("pause");
	}
	if (!ofs)
	{
		cout << "open " << outPath << " error!" << endl;
		system("pause");
	}
	ofs << fixed << showpoint << setprecision(8);
	int flag = 0;
	int step = interval / 30 + 1;
	if (interval != 60 && interval != 90 && interval != 120)
	{
		cout << "interval param error!" << endl;
		system("pause");
	}

	while (ifs)
	{
		double lat, lon;
		int time, mmRoadId;
		ifs >> time;
		if (ifs.fail())
			break;
		if (time == -1)
		{
			ofs << -1 << endl;
			flag = 0;
			continue;
		}
		else
		{
			ifs >> lat >> lon >> mmRoadId;
			if (flag == 0)
				ofs << time << " " << lat << " " << lon << " " << mmRoadId << endl;
			flag++;
			flag %= step;
		}
	}
	ofs.close();
	ifs.close();
}

void ExpGenerator::extractUnmatchedTrajs()
{
	//////////////////////////////////////////////////////////////////////////
	///将生成的实验数据提取出匹配失败的轨迹
	///注意：提取出来的轨迹会存在长度为1的情况
	//////////////////////////////////////////////////////////////////////////
	TrajReader tReader(outputFolder + newMMTrajsFileName);
	list<Traj*> trajs;
	tReader.readTrajs(trajs);//, 10000);
	ofstream ofs(outputFolder + newMMTrajsFileName_unmatched);
	ofs << fixed << showpoint << setprecision(8);
	bool lastOutputIsNegative1 = false;
	cout << trajs.size() << endl;
	for each(Traj* traj in trajs)
	{
		for each (GeoPoint* pt in *traj)
		{
			if (pt->mmRoadId == -1) //留下那些第一次匹配就失败的点
			{
				//system("pause");
				ofs << pt->time << " " << pt->lat << " " << pt->lon << " " << -1 << endl;
				lastOutputIsNegative1 = false;
			}
			else
			{
				if (!lastOutputIsNegative1)
				{
					ofs << -1 << endl;
					lastOutputIsNegative1 = true;
				}
			}
		}

		if (!lastOutputIsNegative1)
		{
			ofs << -1 << endl;
			lastOutputIsNegative1 = true;
		}
	}
	ofs.close();
}

void ExpGenerator::setArea(Area* area)
{
	this->area = area;
}

void ExpGenerator::readRawTrajs(string rawTrajFilePath)
{
	TrajReader tReader(rawTrajFilePath);
	tReader.readTrajs(rawTrajs);
	cout << "read " << rawTrajs.size() << " trajs" << endl;
}

void ExpGenerator::doSplit()
{
	//////////////////////////////////////////////////////////////////////////
	///1.将rawTrajs选出在area内的轨迹
	///2.轨迹点之间距离太长等将会被切断
	///3.轨迹长度为1的丢弃
	//////////////////////////////////////////////////////////////////////////
	GeoPoint prePt, currentPt;
	Traj* tmpTraj = NULL;
	for (list<Traj*>::iterator trajIter = rawTrajs.begin(); trajIter != rawTrajs.end(); trajIter++)
	{
		bool startFlag = true;
		Traj* currentTraj = (*trajIter);
		for (Traj::iterator ptIter = currentTraj->begin(); ptIter != currentTraj->end(); ptIter++)
		{
			if (startFlag)
			{
				if (area->inArea((*ptIter)->lat, (*ptIter)->lon))
				{
					tmpTraj = new Traj;
					GeoPoint* tmpPt = new GeoPoint((*ptIter)->lat, (*ptIter)->lon, (*ptIter)->time);
					tmpTraj->push_back(tmpPt);
					prePt.lat = (*ptIter)->lat;
					prePt.lon = (*ptIter)->lon;
					prePt.time = (*ptIter)->time;
					startFlag = false;
				}
				else
					continue;
			}
			else
			{
				currentPt.lat = (*ptIter)->lat;
				currentPt.lon = (*ptIter)->lon;
				currentPt.time = (*ptIter)->time;
				if (area->inArea((*ptIter)->lat, (*ptIter)->lon))
				{
					if (!overDistLimit(&prePt, &currentPt))
					{
						GeoPoint* tmpPt = new GeoPoint((*ptIter)->lat, (*ptIter)->lon, (*ptIter)->time);
						tmpTraj->push_back(tmpPt);
						prePt.lat = (*ptIter)->lat;
						prePt.lon = (*ptIter)->lon;
						prePt.time = (*ptIter)->time;
						startFlag = false;
					}
					else
					{
						if (tmpTraj->size() > 1)
						{
							trajsInArea.push_back(tmpTraj);
						}
						else
							delete tmpTraj;
						tmpTraj = new Traj;
						GeoPoint* tmpPt = new GeoPoint((*ptIter)->lat, (*ptIter)->lon, (*ptIter)->time);
						tmpTraj->push_back(tmpPt);
						prePt.lat = (*ptIter)->lat;
						prePt.lon = (*ptIter)->lon;
						prePt.time = (*ptIter)->time;
						startFlag = false;
					}
				}
				else
				{
					if (tmpTraj->size() > 1)
					{
						trajsInArea.push_back(tmpTraj);
					}
					else
						delete tmpTraj;
					tmpTraj = new Traj;
					startFlag = true;
				}
			}
		}
		if (startFlag == false)
		{
			if (tmpTraj->size() > 1)
			{
				trajsInArea.push_back(tmpTraj);
			}
			else
				delete tmpTraj;
			tmpTraj = new Traj;
		}
	}
}

void ExpGenerator::doMM(Map* roadNetwork, list<Traj*>& trajs, double thresM /* = 50.0 */)
{
	//////////////////////////////////////////////////////////////////////////
	///对trajs中的每条轨迹进行MapMatching
	///将结果存入Geopoint::mmRoadId字段中
	//////////////////////////////////////////////////////////////////////////
	int count = 0;
	ofstream fout;
	cout << ">> starting MapMatching" << endl
		<< trajs.size() << " trajs in total" << endl;
	MapMatcher mm(roadNetwork);
	//对每一条轨迹
	for (list<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++, count++)
	{
		list<Edge*> result;
		mm.MapMatching(*(*trajIter), result, thresM); //MapMatching

		if (count % 1000 == 0)
			cout << ">> MM" << count << " finished" << endl;
		Traj* traj = (*trajIter);
		if (traj == NULL)
			continue;
		//对每一个点
		Traj::iterator ptIter = traj->begin();
		list<Edge*>::iterator edgeIter = result.begin();
		while (ptIter != traj->end())
		{
			if ((*edgeIter) != NULL) //匹配成功
			{
				(*ptIter)->mmRoadId = (*edgeIter)->id;
			}
			else //匹配失败
			{
				(*ptIter)->mmRoadId = -1;
			}
			ptIter++;
			edgeIter++;
		}
	}
}

void ExpGenerator::outputNewTrajs(list<Traj*>& trajs)
{
	//////////////////////////////////////////////////////////////////////////
	///将两次匹配都为-1的点排除掉，输出到文件newMMTrajsFile中
	///文件写入方式为追加写入
	//////////////////////////////////////////////////////////////////////////
	newMMTrajsFile.open(outputFolder + newMMTrajsFileName, ios::app);
	newMMTrajsFile << fixed << showpoint << setprecision(8);
	bool lastOutputIsNegative1 = false;
	for each(Traj* traj in trajs)
	{
		for each (GeoPoint* pt in *traj)
		{
			if (pt->mmRoadId2 != -1) //排除掉那些第一次匹配就失败的点
			{
				newMMTrajsFile << pt->time << " " << pt->lat << " " << pt->lon << " " << pt->mmRoadId << endl;
				lastOutputIsNegative1 = false;
			}
			else
			{
				if (!lastOutputIsNegative1)
				{
					newMMTrajsFile << -1 << endl;
					lastOutputIsNegative1 = true;
				}
			}
		}

		if (!lastOutputIsNegative1)
		{
			newMMTrajsFile << -1 << endl;
			lastOutputIsNegative1 = true;
		}
	}
	newMMTrajsFile.close();
}

void ExpGenerator::dumpTo(list<Traj*>& source, list<Traj*>& dest)
{
	for (list<Traj*>::iterator trajIter = source.begin(); trajIter != source.end(); trajIter++)
	{
		dest.push_back(*trajIter);
	}
	source.clear();
}

void ExpGenerator::deleteList(list<Traj*>& victimList)
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

bool ExpGenerator::overDistLimit(GeoPoint* pt1, GeoPoint* pt2)
{
	//////////////////////////////////////////////////////////////////////////
	///时间间隔大于limitTime或者速度大于limitSpeed或者之间距离大于limitDist返回false
	//////////////////////////////////////////////////////////////////////////
	double  dist = GeoPoint::distM(*pt1, *pt2);
	double speed;
	if (pt2->time - pt1->time != 0)
	{
		speed = dist / abs(pt2->time - pt1->time);
	}
	else
		speed = 0;
	return dist > limitDist || abs(pt2->time - pt1->time) > limitTime || (speed > limitSpeed);
}

void ExpGenerator::deleteForGeo()
{
	roadNetwork.deleteEdgesRandomlyEx(300, 200, 50, 8);
}

void ExpGenerator::deleteType1()
{
	roadNetwork.deleteIntersectionType1(110, 300.0);
}

void ExpGenerator::deleteType2()
{
	roadNetwork.deleteIntersectionType2(30, 100.0);
}

void ExpGenerator::deleteType3()
{
	roadNetwork.deleteIntersectionType3(40, 150.0);
}