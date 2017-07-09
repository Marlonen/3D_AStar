/*************************************************
Copyright:HIK
Author:ZYVV
Date:2017-06-27
Description:A*算法源文件
**************************************************/
#include <math.h>
#include "astar.h"
#include <algorithm>

/*************************************************
Function:       // InitAstar
Description:    // A*算法数据初始化
Input:          // _maze    地图数据
                // coef_x   x方向权重
                // coef_y   y方向权重
Output:         // 无
Return:         // 无
Others:         // A*算法使用前需调用次函数来设置地图数据
*************************************************/
void Astar::InitAstar(std::vector<std::vector<MapPoint> > &_maze, float coef_x, float coef_y)
{
    maze=_maze;                 //设置地图数据
    Coef_X = coef_x;            //设置启发函数系数
    Coef_Y = coef_y;
    Size_X = maze.size() -1;    //设置地图大小
    Size_Y  = maze[0].size()-1;
}

/*************************************************
Function:       // ~Astar()
Description:    // 类析构函数
Input:          // 无
Output:         // 无
Return:         // 无
Others:         // 无
*************************************************/
Astar::~Astar()
{
    //释放内存
    map<PointKey,Point *>::iterator mit;
    for(mit = this->openList.begin();mit != this->openList.end();mit++)
    {
        delete (mit->second);
    }
    for(mit = this->closeList.begin();mit != this->closeList.end();mit++)
    {
        delete (mit->second);
    }
    list<Point *>::iterator it;
    for(it = this->TrashList.begin();it != this->TrashList.end();it++)
    {
        delete *it;
    }
}

/*************************************************
Function:       // calcG
Description:    // G值计算函数
Input:          // temp_start   当前点
                // point        下一个点
Output:         // 无
Return:         // G值
Others:         // 无
*************************************************/
float Astar::calcG(Point *temp_start,Point *point)
{
    int extraG=(abs(point->x-temp_start->x)+abs(point->y-temp_start->y));
    int parentG=point->parent==NULL?0:point->parent->G;
    return parentG+extraG;
}

/*************************************************
Function:       // calcH
Description:    // H值计算函数
Input:          // point   当前点
                // end     目标点
Output:         // 无
Return:         // H值
Others:         // 无
*************************************************/
float Astar::calcH(Point *point,Point *end)
{
    return (abs(end->x-point->x)*Coef_X + abs(end->y-point->y)*Coef_Y);
}

/*************************************************
Function:       // calcF
Description:    // F值计算函数
Input:          // point   当前点
Output:         // 无
Return:         // F值
Others:         // 无
*************************************************/
float Astar::calcF(Point *point)
{
    return point->G+point->H;
}

/*************************************************
Function:       // findPath
Description:    // 路径搜索函数
Input:          // startPoint   起点
                // endPoint     终点
                // DelayTime    时间延时
Output:         // 无
Return:         // Point指针
Others:         // 无
*************************************************/
Point *Astar::findPath(Point startPoint, Point endPoint,ZY_UINT32 DelayTime)
{
    //初始化起始点
    Point *StartPoint = new Point(startPoint.x,startPoint.y);
    //设置起始点时间延时
    StartPoint->G = DelayTime;
    //添加起始点到开放列表中
    openList.insert(map<PointKey,Point*>::value_type(PointKey(StartPoint->x,StartPoint->y),StartPoint));
    while(!openList.empty())
    {
        //从开启列表中获取F值最小点，并设置为当前点，然后将其从开启列表移动到关闭列表中
        map<PointKey,Point*>::iterator pit = openList.begin();
        map<PointKey,Point*>::iterator epit = openList.end();
        map<PointKey,Point*>::iterator pit_t = pit;
        pit++;
        for(;pit !=epit;pit++)
        {
            if((pit->second)->F<(pit_t->second)->F)
                pit_t = pit;
        }
        Point *curPoint= pit_t->second;
        openList.erase(pit_t);
        closeList.insert(map<PointKey,Point*>::value_type(PointKey(curPoint->x,curPoint->y),curPoint));
        //获取四周可达点
        vector<Point *> surroundPoints=getSurroundPoints(curPoint);
        vector<Point*>::iterator it;
        for(it = surroundPoints.begin();it != surroundPoints.end();it++)
        {
            Point* target = *it;
            map<PointKey,Point*>::iterator tit = openList.find(PointKey(target->x,target->y));
            if(tit == openList.end())           //若不在开放列表中，则添加到开放列表
            {
                target->parent=curPoint;

                target->G=calcG(curPoint,target);
                target->H=calcH(target,&endPoint);
                target->F=calcF(target);
                openList.insert(map<PointKey,Point*>::value_type(PointKey(target->x,target->y),target));
            }
            else                                //若已经存在，则重新计算F值
            {
                int tempG=calcG(curPoint,target);
                if(tempG<target->G)
                {
                    target->parent=curPoint;

                    target->G=tempG;
                    target->F=calcF(target);
                }
                TrashList.push_back(target);
            }
            map<PointKey,Point*>::iterator tit2 = openList.find(PointKey(endPoint.x,endPoint.y));
            if(tit2 != openList.end())         //若终点已在开放列表中，则结束函数，返回路径
                return tit2->second;
        }
    }
    return NULL;
}

/*************************************************
Function:       // GetPath
Description:    // 路径获取函数
Input:          // startPoint   起点
                // endPoint     终点
                // DelayTime    时间延时
Output:         // 无
Return:         // vector<PointItem> 路径
Others:         // 无
*************************************************/
std::vector<PointItem> Astar::GetPath(PointItem startPoint, PointItem endPoint, unsigned int DelayTime)
{
    Point *result=findPath(Point(startPoint.px,startPoint.py),Point(endPoint.px,endPoint.py),DelayTime);  //搜索路径
    std::vector<PointItem> path;
    while(result)
    {
        path.push_back(PointItem(result->x,result->y));
        result=result->parent;
    }
    //翻转路径点
    std::reverse(path.begin(),path.end());
    return path;
}

/*************************************************
Function:       // isCanreach
Description:    // 检测路径点是否可达
Input:          // startPoint   起点
                // target       目标点
Output:         // 无
Return:         // bool 可达返回true，不可达返回false
Others:         // 无
*************************************************/
bool Astar::isCanreach(const Point *point, const Point *target) const
{
    if(target->x<0||target->x>Size_X
        ||(target->y<0)||(target->y>Size_Y)
        ||((maze[target->x][target->y]).PointType == 1)
        ||(target->x==point->x)&&(target->y==point->y)
        ||(closeList.find(PointKey(target->x,target->y)) != closeList.end()))
        return false;
    else if((maze[target->x][target->y]).PointType == 2) //出入口的可达处理
    {
        //由于出入口允许重叠，因此仅需考虑相遇冲突情况
        const std::set<ZY_UINT32> &time_set1 = (maze[target->x][target->y]).OccupyTime_set;
        const std::set<ZY_UINT32> &time_set2 = (maze[point->x][point->y]).OccupyTime_set;
        if((time_set2.find(point->G+1) != time_set2.end())&&(time_set1.find(point->G) != time_set1.end()))
            return false;
        return true;
    }
    else
    {
        //其他过道需要避免重叠和相遇两种情况
        const std::set<ZY_UINT32> &time_set1 = (maze[target->x][target->y]).OccupyTime_set;
        if(time_set1.find(point->G+1) != time_set1.end())
            return false;
        const std::set<ZY_UINT32> &time_set2 = (maze[point->x][point->y]).OccupyTime_set;
        if((time_set2.find(point->G+1) != time_set2.end())&&(time_set1.find(point->G) != time_set1.end()))
            return false;
        return true;
    }
}
/*************************************************
Function:       // getSurroundPoints
Description:    // 检测路径点是否可达
Input:          // point指针
Output:         // 无
Return:         // vector<Point *> 周边可达点vector
Others:         // 无
*************************************************/
std::vector<Point *> Astar::getSurroundPoints(const Point *point) const
{
    std::vector<Point *> surroundPoints;
    for(int x=point->x-1;x<=point->x+1;x++)
        for(int y=point->y-1;y<=point->y+1;y++)
        {
            //只允许相邻点
            if(abs(point->x-x)+abs(point->y -y) == 1)
            {
                Point t_point = Point(x,y);
               if(isCanreach(point,&t_point))
                   surroundPoints.push_back(new Point(x,y));
            }

        }
    return surroundPoints;
}
