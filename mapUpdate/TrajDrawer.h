/*
* Last Updated at [2014/4/3 11:34] by wuhao
*/
#pragma once
#include "MapDrawer.h"
#include <iostream>
#include "GeoPoint.h"
#include <list>
#include <vector>
using namespace std;

typedef list<GeoPoint*> Traj;
class TrajDrawer
{
public:
	//////////////////////////////////////////////////////////////////////////
	///��һ��Ĺ켣�Ĺ��߷���
	///boolLine: �켣�Ƿ��ô��߻���Ĭ��Ϊfalse
	///drawLines: �Ƿ񻭳��켣������Ĭ��Ϊtrue
	///bigPoint: �켣���Ƿ���ʮ�ֵ㣬Ĭ��Ϊtrue
	///drawTrajPt: �켣�Ƿ񻭳��켣�㣬Ĭ��Ϊtrue,�켣����ɫΪ��ɫ
	//////////////////////////////////////////////////////////////////////////
	static void drawOneTraj(Traj* traj, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawTrajs(vector<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawTrajs(list<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);

	//////////////////////////////////////////////////////////////////////////
	///ר�û�ƥ���Ĺ켣
	///Ĭ��ƥ��ɹ��ĵ�Ϊ�̵㣬ʧ�ܵĵ�Ϊ���
	//////////////////////////////////////////////////////////////////////////
	static void drawOneMMTraj(Traj* traj, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawMMTrajs(vector<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawMMTrajs(list<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);

};

void TrajDrawer::drawOneTraj(Traj* traj, MapDrawer& md, Gdiplus::Color color, bool boldLine /* = false */, bool drawLines /* = true */, bool bigPoint /* = true */, bool drawTrajPts /* = true */)
{
	if (traj == NULL)
		return;
	Traj::iterator ptIter = traj->begin(), nextPtIter = ptIter;
	if (drawTrajPts) //�����
	{
		if (bigPoint)
			md.drawBigPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
		else
			md.drawPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
	}
	nextPtIter++;
	while (1)
	{
		if (nextPtIter == traj->end())
			break;
		if (drawLines)
		{
			if (boldLine)
				md.drawBoldLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			else
				md.drawLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
		}
		if (drawTrajPts)
		{
			if (bigPoint)
				md.drawBigPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
			else
				md.drawPoint(Gdiplus::Color::Black, (*ptIter)->lat, (*ptIter)->lon);
		}

		ptIter++;
		nextPtIter++;
	}
	if (drawTrajPts)//���յ�
	{
		if (bigPoint)
			md.drawBigPoint(Gdiplus::Color::Black, traj->back()->lat, traj->back()->lon);
		else
			md.drawPoint(Gdiplus::Color::Black, traj->back()->lat, traj->back()->lon);
	}
}

void TrajDrawer::drawTrajs(vector<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine /* = false */, bool drawLines /* = true */, bool bigPoint /* = true */, bool drawTrajPts /* = true */)
{
	for (vector<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		drawOneTraj((*trajIter), md, color, boldLine, drawLines, drawTrajPts);
	}
}

void TrajDrawer::drawTrajs(list<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine /* = false */, bool drawLines /* = true */, bool bigPoint /* = true */, bool drawTrajPts /* = true */)
{
	for (list<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		drawOneTraj((*trajIter), md, color, boldLine, drawLines, drawTrajPts);
	}
}

void TrajDrawer::drawOneMMTraj(Traj* traj, MapDrawer& md, Gdiplus::Color color, bool boldLine /* = false */, bool drawLines /* = true */, bool bigPoint /* = true */, bool drawTrajPts /* = true */)
{
	if (traj == NULL)
		return;
	Traj::iterator ptIter = traj->begin(), nextPtIter = ptIter;
	if (drawTrajPts) //�����
	{
		Gdiplus::Color ptColor = (*ptIter)->mmRoadId == -1 ? Gdiplus::Color::Red : Gdiplus::Color::Green;
		if (bigPoint)
			md.drawBigPoint(ptColor, (*ptIter)->lat, (*ptIter)->lon);
		else
			md.drawPoint(ptColor, (*ptIter)->lat, (*ptIter)->lon);
	}
	nextPtIter++;
	while (1)
	{
		if (nextPtIter == traj->end())
			break;
		if (drawLines)
		{
			if (boldLine)
				md.drawBoldLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
			else
				md.drawLine(color, (*ptIter)->lat, (*ptIter)->lon, (*nextPtIter)->lat, (*nextPtIter)->lon);
		}
		if (drawTrajPts)
		{
			Gdiplus::Color ptColor = (*ptIter)->mmRoadId == -1 ? Gdiplus::Color::Red : Gdiplus::Color::Green;
			if (bigPoint)
				md.drawBigPoint(ptColor, (*ptIter)->lat, (*ptIter)->lon);
			else
				md.drawPoint(ptColor, (*ptIter)->lat, (*ptIter)->lon);
		}

		ptIter++;
		nextPtIter++;
	}
	if (drawTrajPts)//���յ�
	{
		Gdiplus::Color ptColor = (*ptIter)->mmRoadId == -1 ? Gdiplus::Color::Red : Gdiplus::Color::Green;
		if (bigPoint)
			md.drawBigPoint(ptColor, traj->back()->lat, traj->back()->lon);
		else
			md.drawPoint(ptColor, traj->back()->lat, traj->back()->lon);
	}
}

void TrajDrawer::drawMMTrajs(vector<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine /* = false */, bool drawLines /* = true */, bool bigPoint /* = true */, bool drawTrajPts /* = true */)
{
	for (vector<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		drawOneMMTraj((*trajIter), md, color, boldLine, drawLines, drawTrajPts);
	}
}

void TrajDrawer::drawMMTrajs(list<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine /* = false */, bool drawLines /* = true */, bool bigPoint /* = true */, bool drawTrajPts /* = true */)
{
	for (list<Traj*>::iterator trajIter = trajs.begin(); trajIter != trajs.end(); trajIter++)
	{
		drawOneMMTraj((*trajIter), md, color, boldLine, drawLines, drawTrajPts);
	}
}
