/* 
 * Last Updated at [2015/4/14 21:50] by wuhao
 */
#include "TrajDrawer.h"


void TrajDrawer::drawOneTraj(Traj* traj, MapDrawer& md, Gdiplus::Color color, bool boldLine /* = false */, bool drawLines /* = true */, bool bigPoint /* = true */, bool drawTrajPts /* = true */)
{
	if (traj == NULL)
		return;
	Traj::iterator ptIter = traj->begin(), nextPtIter = ptIter;
	if (drawTrajPts) //画起点
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
	if (drawTrajPts)//画终点
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
	if (drawTrajPts) //画起点
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
	if (drawTrajPts)//画终点
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
