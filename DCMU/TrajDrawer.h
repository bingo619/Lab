/* 
 * Last Updated at [2015/4/14 21:50] by wuhao
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
	///画一般的轨迹的工具方法
	///boolLine: 轨迹是否用粗线画，默认为false
	///drawLines: 是否画出轨迹线条，默认为true
	///bigPoint: 轨迹点是否是十字点，默认为true
	///drawTrajPt: 轨迹是否画出轨迹点，默认为true,轨迹点颜色为黑色
	//////////////////////////////////////////////////////////////////////////
	static void drawOneTraj(Traj* traj, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawTrajs(vector<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawTrajs(list<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);

	//////////////////////////////////////////////////////////////////////////
	///专用画匹配后的轨迹
	///默认匹配成功的点为绿点，失败的点为红点
	//////////////////////////////////////////////////////////////////////////
	static void drawOneMMTraj(Traj* traj, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawMMTrajs(vector<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);
	static void drawMMTrajs(list<Traj*>& trajs, MapDrawer& md, Gdiplus::Color color, bool boldLine = false, bool drawLines = true, bool bigPoint = true, bool drawTrajPts = true);

};