#include <iostream>
#include "astar.h"


int main()
{
    //初始化地图，用二维矩阵代表地图，200表示障碍物，0表示可通 其他表示时间
    vector<vector<int>> maze={
        {200,200,200,200,200,200,200,200,200,200,200,200},
        {200,  0,  2,200,200,  0,200,  0,  0,  0,  0,200},
        {200,  0,  1,200,200,  0,  0,  0,  0,  0,  0,200},
        {200,  0,  0,  0,  0,  0,200,  0,  0,200,200,200},
        {200,200,200,  0,  0,  0,  0,  0,200,200,  0,200},
        {200,200,  0,200,  0,  0,  0,  0,  0,  0,  0,200},
        {200,  0,200,  0,  0,  0,  0,200,  0,  0,  0,200},
        {200,200,200,200,200,200,200,200,200,200,200,200}
    };

    ZY_UINT32 MapSize_X = maze.size();
    ZY_UINT32 MapSize_Y = maze[0].size();

    Astar astar;
    vector<vector<MapPoint>> maze1(MapSize_X);
    for(int i=0;i<MapSize_X;i++) {
         maze1[i].resize(MapSize_Y);
     }

    //初始化地图
    for(int i =0; i< MapSize_X;i++)
        for(int j=0; j< MapSize_Y;j++)
        {
            MapPoint map_point;
            if(maze[i][j] == 200)
                map_point.PointType = 1;
            else
                map_point.PointType = 0;
            maze1[i][j] = map_point;
        }

    //设置出入口和时间占用点
    (maze1[1][2]).OccupyTime_set.insert(2);
    (maze1[2][2]).OccupyTime_set.insert(1);
    (maze1[4][10]).PointType =2;
    astar.InitAstar(maze1,1,1);


    //A*算法找寻路径
    vector<PointItem> path=astar.GetPath(PointItem(1,1),PointItem(4,10),0);
    //打印
    for(auto &p:path)
        cout<<'('<<p.px<<','<<p.py<<')'<<endl;

    system("pause");
    return 0;
}

