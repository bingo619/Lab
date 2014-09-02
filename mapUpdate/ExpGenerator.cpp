#include "ExpGenerator.h"

void ExpGenerator::genExpData()
{
	for (int i = 0; i < inputFileNames.size(); i++)
	{
		cout << ">> �����ļ� " << inputFileNames[i] << endl;
		genExpData(inputFolder + inputFileNames[i]);
	}
}

void ExpGenerator::genExpData(string rawTrajFilePath)
{
	double mmThres = 25.0;
	readRawTrajs(rawTrajFilePath);
	doSplit();
	deleteList(rawTrajs); //�ͷ�rawTrajs�ڴ�
	//��һ��ʹ��ԭ��ͼƥ��
	cout << ">> ���е�һ�ε�ͼƥ��" << endl;
	doMM(&originalRoadNetwork, trajsInArea, mmThres);
	
	//���������ÿ��pt��mmRoadId2�ֶ�
	for each(Traj* traj in trajsInArea)
	{
		for each (GeoPoint* pt in *traj)
		{
			pt->mmRoadId2 = pt->mmRoadId;
		}
	}

	//�ڶ���ʹ�õ�ͼƥ��
	cout << ">> ���еڶ��ε�ͼƥ��" << endl;
	doMM(&roadNetwork, trajsInArea, mmThres);

	outputNewTrajs(trajsInArea); 
	deleteList(trajsInArea);
	//dumpTo(trajsInArea, doneTrajs);
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
	///1.��rawTrajsѡ����area�ڵĹ켣
	///2.�켣��֮�����̫���Ƚ��ᱻ�ж�
	///3.�켣����Ϊ1�Ķ���
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
	///��trajs�е�ÿ���켣����MapMatching
	///���������Geopoint::mmRoadId�ֶ���
	//////////////////////////////////////////////////////////////////////////
	int count = 0;
	ofstream fout;
	cout << ">> starting MapMatching" << endl
		<< trajs.size() << " trajs in total" << endl;
	MapMatcher mm(roadNetwork);
	//��ÿһ���켣
	for (list<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++, count++)
	{
		list<Edge*> result;
		mm.MapMatching(*(*trajIter), result, thresM); //MapMatching

		if (count % 1000 == 0)
			cout << ">> MM" << count << " finished" << endl;
		Traj* traj = (*trajIter);
		if (traj == NULL)
			continue;
		//��ÿһ����
		Traj::iterator ptIter = traj->begin();
		list<Edge*>::iterator edgeIter = result.begin();
		while (ptIter != traj->end())
		{
			if ((*edgeIter) != NULL) //ƥ��ɹ�
			{
				(*ptIter)->mmRoadId = (*edgeIter)->id;
			}
			else //ƥ��ʧ��
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
	///������ƥ�䶼Ϊ-1�ĵ��ų�����������ļ�newMMTrajsFile��
	///�ļ�д�뷽ʽΪ׷��д��
	//////////////////////////////////////////////////////////////////////////
	newMMTrajsFile.open(outputFolder + newMMTrajsFileName, ios::app);
	newMMTrajsFile << fixed << showpoint << setprecision(8);
	bool lastOutputIsNegative1 = false;
	for each(Traj* traj in trajs)
	{
		for each (GeoPoint* pt in *traj)
		{
			if (pt->mmRoadId2 != -1) //�ų�����Щ��һ��ƥ���ʧ�ܵĵ�
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
	///ʱ��������limitTime�����ٶȴ���limitSpeed����֮��������limitDist����false
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