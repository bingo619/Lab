/*
* Last Updated at [2015/2/13 13:14] by wuhao
*/
#include "TrajReader.h"

//////////////////////////////////////////////////////////////////////////
///public part
//////////////////////////////////////////////////////////////////////////
TrajReader::TrajReader(string filePath)
{
	open(filePath);
}

void TrajReader::open(string filePath)
{
	trajIfs.open(filePath);
	if (!trajIfs)
	{
		cout << "open " + filePath + " error!\n";
		system("pause");
		exit(0);
	}
}

void TrajReader::readTrajs(vector<Traj*>& dest, int count /* = INF */)
{
	//////////////////////////////////////////////////////////////////////////
	///��ʽ(ÿһ��):time lat lon mmRoadId
	///�켣������-1 ����ռһ��
	///�켣����Ϊ1�Ļᶪ��
	//////////////////////////////////////////////////////////////////////////
	dest.clear();
	cout << ">> start reading trajs" << endl;
	bool isStart = true;
	int time, mmRoadId;
	double lat, lon;
	Traj* tmpTraj = NULL;
	int currentCount = 0;
	while (trajIfs)
	{
		if (currentCount == count)
		{
			break;
		}
		int time;
		trajIfs >> time;
		if (trajIfs.fail())
		{
			break;
		}
		if (time == -1)
		{
			isStart = true;
			if (tmpTraj != NULL && tmpTraj->size() > 1)
			{
				dest.push_back(tmpTraj);
				currentCount++;
			}
			continue;
		}
		else
		{
			trajIfs >> lat >> lon >> mmRoadId;
			GeoPoint* pt = new GeoPoint(lat, lon, time, mmRoadId);
			if (isStart)
			{
				tmpTraj = new Traj();
				tmpTraj->push_back(pt);
				isStart = false;
			}
			else
			{
				tmpTraj->push_back(pt);
			}
		}
	}
	cout << ">> reading trajs finished" << endl;
	cout << dest.size() << "trajs in all" << endl;
	trajIfs.close();
}

void TrajReader::readTrajs(list<Traj*>& dest, int count /* = INF */)
{
	//////////////////////////////////////////////////////////////////////////
	///��ʽ(ÿһ��):time lat lon mmRoadId
	///�켣������-1 ����ռһ��
	///�켣����Ϊ1�Ļᶪ��
	//////////////////////////////////////////////////////////////////////////
	dest.clear();
	cout << ">> start reading trajs" << endl;
	bool isStart = true;
	int time, mmRoadId;
	double lat, lon;
	Traj* tmpTraj = NULL;
	int currentCount = 0;
	while (trajIfs)
	{
		if (currentCount == count)
		{
			break;
		}
		if (currentCount % 10000 == 0 && currentCount > 0)
		{
			printf("read %d trajs\n", currentCount++);
		}
		int time;
		trajIfs >> time;
		if (trajIfs.fail())
		{
			break;
		}
		if (time == -1)
		{
			isStart = true;
			if (tmpTraj != NULL && tmpTraj->size() > 1)
			{
				dest.push_back(tmpTraj);
				currentCount++;
			}
			continue;
		}
		else
		{
			trajIfs >> lat >> lon >> mmRoadId;
			GeoPoint* pt = new GeoPoint(lat, lon, time, mmRoadId);
			if (isStart)
			{
				tmpTraj = new Traj();
				tmpTraj->push_back(pt);
				isStart = false;
			}
			else
			{
				tmpTraj->push_back(pt);
			}
		}
	}
	cout << ">> reading trajs finished" << endl;
	cout << dest.size() << " trajs in all" << endl;
	trajIfs.close();
}

void TrajReader::readGeoPoints(list<GeoPoint*>& dest, Area* area/* = NULL */, int count /* = INF */)
{
	//////////////////////////////////////////////////////////////////////////
	///��ʽ(ÿһ��):time lat lon mmRoadId
	///�켣������-1 ����ռһ�� //���в���
	//////////////////////////////////////////////////////////////////////////
	dest.clear();
	cout << ">> start reading GeoPoints" << endl;
	int time, mmRoadId;
	double lat, lon;
	int currentCount = 0;
	while (trajIfs)
	{
		if (currentCount == count)
		{
			break;
		}
		int time;
		trajIfs >> time;
		if (trajIfs.fail())
		{
			break;
		}
		if (time == -1)
			continue;
		else
		{
			trajIfs >> lat >> lon >> mmRoadId;
			if (area != NULL)
			{
				if (area->inArea(lat, lon))
				{
					GeoPoint* pt = new GeoPoint(lat, lon, time, mmRoadId);
					dest.push_back(pt);
					currentCount++;
				}
				else
					continue;
			}
			else
			{
				GeoPoint* pt = new GeoPoint(lat, lon, time, mmRoadId);
				dest.push_back(pt); //area = NULL, ����������ȫ������
				currentCount++;
			}
		}
		if (currentCount % 10000 == 0 && currentCount > 0)
		{
			printf("read %d GeoPoints\n", currentCount);
		}
	}
	cout << ">> reading GeoPoints finished" << endl;
	cout << dest.size() << " GeoPoints in all" << endl;
	trajIfs.close();
}

void TrajReader::outputTrajs(list<Traj*>& trajs, string filePath, int count /* = INF */)
{
	ofstream ofs(filePath);
	if (!ofs)
	{
		cout << "open " << filePath << " error!" << endl;
		system("pause");
	}
	ofs << fixed << showpoint << setprecision(8);
	int currentCount = 0;
	for (list<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		if (currentCount == count)
		{
			break;
		}
		for (Traj::iterator ptIter = (*trajIter)->begin(); ptIter != (*trajIter)->end(); ptIter++)
		{
			ofs << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << (*ptIter)->mmRoadId << endl;
		}
		ofs << -1 << endl;
		currentCount++;
	}
	cout << ">> " << currentCount << " trajs have been output to " << filePath << endl;
	ofs.close();
}

void TrajReader::outputPts(list<GeoPoint*>& pts, string filePath, int count /* = INF */)
{
	ofstream ofs(filePath);
	if (!ofs)
	{
		cout << "open " << filePath << " error!" << endl;
		system("pause");
	}
	ofs << fixed << showpoint << setprecision(8);
	int currentCount = 0;
	for (list<GeoPoint*>::iterator ptIter = pts.begin(); ptIter != pts.end(); ptIter++)
	{
		ofs << (*ptIter)->time << " " << (*ptIter)->lat << " " << (*ptIter)->lon << " " << (*ptIter)->mmRoadId << endl;
		currentCount++;
		if (currentCount == count)
			break;
	}
	cout << ">> " << currentCount << " GeoPoints have been output to " << filePath << endl;
	ofs.close();
}