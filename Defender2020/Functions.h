//目前函数
//射门角度选择函数  2.0  
//传球角度选择函数  1.0


//返回是否有射门角度。若存在射门角度，将最优角度传给参数kickAngle
bool getKickAngle(float& kickAngle){
	//注释掉的代码为输出检验代码
	
	const float kickTol=60;//上次最优位置与本次最优位置之间的可容忍区间长度差距
	const float lenOk=400;//稳稳进球的区间大小
	const float diameterBall=100;//球的半径
	
	//Line类型由base（线上一点），direction（方向向量）来决定
	Geometry::Line lineGoal(Vector2f(theFieldDimensions.xPosOpponentGroundline,0),Vector2f(0,1));//球门线
	Geometry::Line lineFromBallToObstacle;//从球到障碍物边缘的线
	Vector2f posBall=Transformation::robotToField(theRobotPose,theBallModel.estimate.position);
	lineFromBallToObstacle.base=posBall;
	bool ifIntersect;
	Vector2f intersection;//两线交点
	Vector2f endObstacle[2];//障碍物的左右边缘，0是左，1是右
	std::vector<std::pair<float,float> > intervalCovered;//存放障碍物遮挡区间
	float oneCovered[2];//一个遮挡区间

	for(const auto& obstacle:theObstacleModel.obstacles){
		endObstacle[0]=Transformation::robotToField(theRobotPose,obstacle.left);
		endObstacle[1]=Transformation::robotToField(theRobotPose,obstacle.right);
		
		if(endObstacle[0][0]>posBall[0] || endObstacle[1][0]>posBall[0]){//保证障碍物不完全在球后面
			for(int i=0;i<=1;++i){
				if(endObstacle[i][0]>posBall[0]){
					lineFromBallToObstacle.direction=endObstacle[i]-posBall;
					
					ifIntersect=Geometry::getIntersectionOfLines(lineFromBallToObstacle,lineGoal,intersection);
					if(intersection[1]>theFieldDimensions.yPosLeftGoal)
						oneCovered[i]=theFieldDimensions.yPosLeftGoal;
					else if(intersection[1]<theFieldDimensions.yPosRightGoal)
						oneCovered[i]=theFieldDimensions.yPosRightGoal;
					else
						oneCovered[i]=intersection[1];
				}
				else{//如果障碍物的一个边缘在球的后面，整个一侧都会被遮挡
					if(endObstacle[i][1]>posBall[1])
						oneCovered[i]=theFieldDimensions.yPosLeftGoal;
					else
						oneCovered[i]=theFieldDimensions.yPosRightGoal;
				}
			}
						
			if(oneCovered[0]>oneCovered[1])
				intervalCovered.push_back(std::pair<float,float>(oneCovered[0],oneCovered[1]));
		}
	}
	
	std::vector<std::pair<float,float> > intervalAvailable;//存放可射门区间
	std::pair<float,float> oneAvailable;//一个可射门区间
	if(!intervalCovered.empty()){
		//合并遮挡区间
		std::sort(intervalCovered.begin(),intervalCovered.end(),std::greater<std::pair<float,float> >());
		std::vector<std::pair<float,float> > intervalCoveredMerged;//存放合并后的遮挡区间
		std::pair<float,float> oneCoveredMerged=intervalCovered[0];//一个合并后的遮挡区间
		for(const auto& i:intervalCovered){
			if(i.first>=oneCoveredMerged.second){
				if(i.second<oneCoveredMerged.second)
					oneCoveredMerged.second=i.second;
			}		
			else{
				intervalCoveredMerged.push_back(oneCoveredMerged);
				oneCoveredMerged=i;
			}
		}
		intervalCoveredMerged.push_back(oneCoveredMerged);
		
		//去除合并后的遮挡区间，剩下可射门区间
		oneAvailable.first=theFieldDimensions.yPosLeftGoal;
		for(const auto& i:intervalCoveredMerged){
			oneAvailable.second=i.first;
			if(oneAvailable.first-oneAvailable.second>diameterBall)
				intervalAvailable.push_back(oneAvailable);
			oneAvailable.first=i.second;
		}
		oneAvailable.second=theFieldDimensions.yPosRightGoal;
		if(oneAvailable.first-oneAvailable.second>diameterBall){
				intervalAvailable.push_back(oneAvailable);
		}		
	}
	else{
		//没有遮挡整个门都可射
		oneAvailable.first=theFieldDimensions.yPosLeftGoal;
		oneAvailable.second=theFieldDimensions.yPosRightGoal;
		intervalAvailable.push_back(oneAvailable);
	}
	
	if(!intervalAvailable.empty()){
		static float kickMaxPosOld=0;//上次最优射门的门线位置
		
		//寻找最优射门位置
		float maxLen=0,maxPos;//最长可射门区间的长度、中点
		float lenForOld=0;
		for(const auto& i:intervalAvailable){
			if(i.first-i.second>maxLen){
				maxLen=i.first-i.second;
				maxPos=(i.first+i.second)/2;
			}
			if(kickMaxPosOld<i.first && kickMaxPosOld>i.second)
				lenForOld=(i.first-kickMaxPosOld<kickMaxPosOld-i.second)?(i.first-kickMaxPosOld):(kickMaxPosOld-i.second);
		}
		
		//检验原射门位置是否可以接受
		if(lenForOld>lenOk/2  || (lenForOld>diameterBall/2 && std::fabs(lenForOld-maxLen/2)<=kickTol))
			maxPos=kickMaxPosOld;
		else
			kickMaxPosOld=maxPos;
		
		//转化为相对角度
		Vector2f pointAtGoal(theFieldDimensions.xPosOpponentGroundline,maxPos);
		pointAtGoal=Transformation::fieldToRobot(theRobotPose,pointAtGoal);
		kickAngle=pointAtGoal.angle();
	}
	
	/*
	static int countPrint=0;//累计输出次数
	std::ofstream out;
	if(countPrint%10==0)
		out.open("/home/dingjiayu/data.txt",std::ios::app);
	else
		out.open("/home/dingjiayu/dataNotUsed.txt");
	++countPrint;
	out<<"point"<<std::endl<<'r'<<std::endl<<posBall<<std::endl;
	out<<"end"<<std::endl;
	out.close();
	*/
	
	return !intervalAvailable.empty();
}

//线段
class segment{
	public:
	Vector2f end[2];
	segment(Vector2f a,Vector2f b){
		end[0]=a;
		end[1]=b;
	}
};
//veca与vecb的行列式
float det(Vector2f veca, Vector2f vecb){
	return veca[0]*vecb[1]-veca[1]*vecb[0];
}
float min(float numa,float numb){
	return numa<numb?numa:numb;
}
float max(float numa,float numb){
	return numa>numb?numa:numb;
}
//两线段是否相交
bool ifTwoSegmentIntersect(segment sega,segment segb){
	//快速排斥
	if(min(sega.end[0][0],sega.end[1][0])>max(segb.end[0][0],segb.end[1][0]) ||
	   min(segb.end[0][0],segb.end[1][0])>max(sega.end[0][0],sega.end[1][0]) ||
	   min(sega.end[0][1],sega.end[1][1])>max(segb.end[0][1],segb.end[1][1]) ||
	   min(segb.end[0][1],segb.end[1][1])>max(sega.end[0][1],sega.end[1][1])
	)
		return false;
	//跨立检验
	Vector2f a0b0=segb.end[0]-sega.end[0],a0b1=segb.end[1]-sega.end[0],a0a1=sega.end[1]-sega.end[0];
	if(det(a0b0,a0a1)*det(a0b1,a0a1)>1e6)
		return false;
	Vector2f b0a0=sega.end[0]-segb.end[0],b0a1=sega.end[1]-segb.end[0],b0b1=segb.end[1]-segb.end[0];
	if(det(b0a0,b0b1)*det(b0a1,b0b1)>1e6)
		return false;
	return true;
}
//pointa与pointb距离的平方
float disSquare(Vector2f pointa,Vector2f pointb){
	return (pointa[0]-pointb[0])*(pointa[0]-pointb[0])+(pointa[1]-pointb[1])*(pointa[1]-pointb[1]);
}
//点与线段的最短距离
float minDistanceBetweenPointAndSegment(Vector2f point,segment seg){
	//设线段端点为a，b，另一点为c
	Vector2f ab=seg.end[1]-seg.end[0],ac=point-seg.end[0];
	float lenseg=disSquare(seg.end[0],seg.end[1]),innerproduct=ab[0]*ac[0]+ab[1]*ac[1];//内积
	if(innerproduct<0)//垂足在a的外侧
		return std::sqrt(disSquare(point,seg.end[0]));
	else if(innerproduct>lenseg)//垂足在b的外侧
		return std::sqrt(disSquare(point,seg.end[1]));
	else{//垂足在ab上
		return std::sqrt(disSquare(point,seg.end[0])-innerproduct*innerproduct/lenseg);
	}
}
//两线段之间最短距离
float minDistanceBetweenTwoSegment(segment sega,segment segb){
	if(ifTwoSegmentIntersect(sega,segb)){
		return 0;
	}
	else{
		//四端点离另一线段的最短距离
		float dis[4],min;
		dis[0]=minDistanceBetweenPointAndSegment(sega.end[0],segb);
		dis[1]=minDistanceBetweenPointAndSegment(sega.end[1],segb);
		dis[2]=minDistanceBetweenPointAndSegment(segb.end[0],sega);
		dis[3]=minDistanceBetweenPointAndSegment(segb.end[1],sega);
		
		//四个距离中的最小值
		min=dis[0];
		for(int i=1;i<=3;++i){
			if(dis[i]<min){
				min=dis[i];
			}
		}
		return min;
	}
}

bool ifKickAvailable(Vector2f targetPoint){
	const float ballRadius=50;//球的半径
	
	segment segment2(theTeamBallModel.position,targetPoint);
    for(const auto& obstacle : theTeamPlayersModel.obstacles){
      if((obstacle.center-theRobotPose.translation).norm()>1e-6){//除去自己
		  segment segment1(obstacle.left,obstacle.right);
		  if(minDistanceBetweenTwoSegment(segment1,segment2)<ballRadius){
			return false;
		  }
	  }
	}
	return true;
}
//返回是否有传球角度。若存在传球角度，将最优角度传给参数passAngle
bool getPassAngle(float& passAngle)
{ 
	/*
	static int countPrint=0;//累计输出次数
	std::ofstream out;
	if(countPrint%100==0)
		out.open("/home/dingjiayu/data.txt",std::ios::app);
	else
		out.open("/home/dingjiayu/dataNotUsed.txt");
	++countPrint;
	*/
	
  const float rol = 2000.f; //一脚踢出的距离
  
  const float Pi = (float)3.14159265;
  const float Inf=1e6;
  const Vector2f globalBall = theTeamBallModel.position;
  const int number=360;
  
  //以正对对方球门方向为0deg
  //候选点
  std::vector<float> candidateOne;
  const float delta = 2*Pi/number;
  for(float i = 0; i<2*Pi;i+=delta)
    {   
        float lenTeam = Inf; //据我方最近球员距离
        float lenOppo = Inf; //据观察到的敌方最近球员距离
        float distanceTeam = 0.f;
        float distanceOppo = 0.f;
        
        //禁止后传
        if(i >= 135*Pi/180 && i <=225*Pi/180)
            continue;
			
        Vector2f pointToTheField = globalBall+Vector2f(rol*cos(i), rol*sin(i));
		
		if(ifKickAvailable(pointToTheField)){
			for(const auto& obstacle : theTeamPlayersModel.obstacles)
			{
				switch(obstacle.type){
					case Obstacle::teammate:
						distanceTeam=(obstacle.center-pointToTheField).norm();
						if(distanceTeam<lenTeam)
							lenTeam=distanceTeam;
						break;
					case Obstacle::opponent:
						float distanceOppo=(obstacle.center-pointToTheField).norm();
						if(distanceOppo<lenOppo)
							lenOppo=distanceOppo;
						break;
				}
			}
			
			        //加入候选者
			if(lenTeam < lenOppo || (lenTeam==Inf && lenOppo==Inf)){
				candidateOne.push_back(i);
				//printSegment(globalBall,pointToTheField,"g",out);
			}
		}
    }
      
  if(!candidateOne.empty()){
	  //最终考核
	  float lenDoor = Inf; //据球门距离
	  float distanceDoor = 0.f;
	  float chosenOne;
	  Vector2f doorToTheField = Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f);
	  for(const auto& i:candidateOne)
	  { 
		Vector2f pointToTheField = globalBall+Vector2f(rol*cos(i), rol*sin(i));
		distanceDoor = (pointToTheField-doorToTheField).norm();
		if(distanceDoor < lenDoor){
			lenDoor = distanceDoor;
			chosenOne = i;   
		}
	  }
	  Vector2f chosenPointToTheField =globalBall+Vector2f(rol*cos(chosenOne), rol*sin(chosenOne));
	  Vector2f chosenPointSpot = Transformation::fieldToRobot(theRobotPose,chosenPointToTheField);
	  passAngle=chosenPointSpot.angle();
	  
	  //printSegment(globalBall,chosenPointToTheField,"r",out);
  }
  
  /*
  for(const auto& obstacle : theTeamPlayersModel.obstacles){
	  printSegment(obstacle.left,obstacle.right,"b",out);
  }
  	out<<"end"<<std::endl;
	out.close();
	*/
  return !candidateOne.empty();
}




option(Functions)
{
  initial_state(start)
  {
    transition
    {
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      Stand();
    }
  }

}
