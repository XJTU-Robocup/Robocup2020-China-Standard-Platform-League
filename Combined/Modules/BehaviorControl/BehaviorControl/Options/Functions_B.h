bool getShootTarget2(Vector2f& shootTarget){
	const float step=2*theFieldDimensions.yPosLeftGoal/400;
	const float shootLen=4000.f;

	Vector2f pointGoal(theFieldDimensions.xPosOpponentGroundline+ballRadius,0);
	segment segFromBallToGoal(theTeamBallModel.position,pointGoal);
	std::vector<std::pair<float,Vector2f> > candidates;
	std::pair<float,Vector2f> temp;
	for(float i=theFieldDimensions.yPosRightGoal;i<=theFieldDimensions.yPosLeftGoal;i+=step){
		pointGoal[1]=i;
		if((theTeamBallModel.position-pointGoal).norm()<shootLen){
			segFromBallToGoal.end[1]=pointGoal;
			temp.first=minDisBetweenSegmentAndObstacle(segFromBallToGoal);
			temp.second=pointGoal;
		if(temp.first < shootLen)
			candidates.push_back(temp);
		}
	}

	if(!candidates.empty())
		shootTarget=maxElement(candidates);

	return !candidates.empty();
}

int getKickTarget2(Vector2f& kickTarget)
{ 
	Vector2f shootTarget;
	if(getShootTarget2(shootTarget)){
		kickTarget=shootTarget;
		return 2;
	}
	
	const Vector2f globalBall2 = theTeamBallModel.position;
	const int number=360;
	const float borderOffset=100;
	
	//以正对对方球门方向为0deg
	std::vector<std::pair<float,Vector2f> > passCandidate;//候选点
	std::pair<float,Vector2f> temp;
	const float delta = 2*Pi/number;
	for(float i = 0; i<2*Pi;i+=delta)
	{   
			
		Vector2f pointToTheField = globalBall2+Vector2f(kickLen*cos(i), kickLen*sin(i));
		
		//禁止后传
		if(i >= 135*Pi/180 && i <=225*Pi/180)
			continue;
		
		if(ifKickAvoidObstacle(pointToTheField)){
				float lenTeam = Inf; //据我方最近球员距离
				float lenOppo = Inf; //据观察到的敌方最近球员距离
				float distanceTeam = 0.f;
				float distanceOppo = 0.f;
				
				if(std::fabs(pointToTheField[0])>=theFieldDimensions.xPosOpponentGroundline-borderOffset || 
				   std::fabs(pointToTheField[1])>=theFieldDimensions.yPosLeftSideline-borderOffset
				)
					continue;
		
				for(const auto& obstacle : theTeamPlayersModel.obstacles)
				{
					switch(obstacle.type){
					case Obstacle::teammate:
						distanceTeam=(obstacle.center-pointToTheField).norm();
						if(distanceTeam<lenTeam)
							lenTeam=distanceTeam;
						break;
					case Obstacle::opponent:
						distanceOppo=(obstacle.center-pointToTheField).norm();
						if(distanceOppo<lenOppo)
							lenOppo=distanceOppo;
						break;
					}
				}
			
					//加入候选者
				if(lenTeam < lenOppo || (lenTeam==Inf && lenOppo==Inf)){
					temp.first=posEvalConsiderOpponent(pointToTheField)+posEvalConsiderShoot(pointToTheField);
					temp.second=pointToTheField;
					passCandidate.push_back(temp);
				}
		}
	}

	if(!passCandidate.empty()){
		kickTarget=maxElement(passCandidate);
		return 1;
	}
	else{
		return 0;
	}
}

Vector2f globalBall2(){
	return Transformation::robotToField(theRobotPose,theBallModel.estimate.position);
}

Vector2f globalRobot2(){
	return theRobotPose.translation;
}

int modeDecision2()
{   //找球模式
	if(globalRobot2().x()>1800.f || globalRobot2().x()<-1800.f
	  || globalRobot2().y()>100.f || globalRobot2().y()<-3000.f)
		return 3;
	if(globalBall2().x()>1800.f || globalBall2().x()<-1800.f
	  || globalBall2().y()>100.f || globalBall2().y()<-3000.f)
		return 3;
	if(theLibCodeRelease.timeSinceBallWasSeen > 4000.f)
		return 0;
	if(globalBall2().x()<=1800.f && globalBall2().x()>=-1800.f
	  && globalBall2().y()<=100.f && globalBall2().y()>=-3000.f
	  && theBallModel.estimate.position.norm()>450.f)
		return 2;
	if(globalBall2().x()<=1800.f && globalBall2().x()>=-1800.f
	  && globalBall2().y()<=100.f && globalBall2().y()>=-3000.f
	  && theBallModel.estimate.position.norm()<=450.f)
		return 1;    
	  /*
	  bool teamGotBall = false;//判断队伍是否拿球
	  //计算队伍和敌人离球的最近距离
	  float lenTeam = Inf;
	  float lenOppo = Inf;
	  size_t i;
	  for( i=0;i<o.size();i++)
		{
		   if(o[i].type == Obstacle::teammate && o[i].center.norm()<lenTeam)
				lenTeam = o[i].center.norm();
		   if(o[i].type == Obstacle::opponent && o[i].center.norm()<lenOppo)
				lenOppo = o[i].center.norm();
		}
	  if(lenTeam < lenOppo && lenOppo-lenTeam >200.f && lenTeam < 500.f)
		teamGotBall = true;//队友持球
	  */
	  //如果球在防区时，进入防御模式：截球
	  //如果球不在防区内
	  /*
	  //计算球离防区边界四点最近距离
	  float lenField = inf;
	  float len1 = Vector2f(globalBall2.x()-3000.f, globalBall2.y()).norm();
	  float len2 = Vector2f(globalBall2.x()-3000.f, globalBall2.y()+1500.f).norm();
	  float len3 = Vector2f(globalBall2.x()+3000.f, globalBall2.y()).norm();
	  float len4 = Vector2f(globalBall2.x()+3000.f, globalBall2.y()+1500.f).norm();
	  if(len1 < lenField)
		  lenField = len1;
	  if(len2 < lenField)
		  lenField = len2;
	  if(len3 < lenField)
		  lenField = len3;
	  if(len4 < lenField)
		  lenField = len4;
	  */
	  //如果球不在防区，回到场地中央
	  /*if(lenTeam < lenOppo && lenTeam < 500.f)
		return 3;
	  /如果球不在防区，回到场地中央
	  if(lenTeam < lenOppo && (lenTeam > 500.f || lenField < 1000.f))
		return 3;
	  //如果球不在防区，回到场地中央
	  if(lenTeam >= lenOppo)
		return 3;
	 */
}

int modeDecision22()
{   //找球模式
	if(globalRobot2().x()>1800.f || globalRobot2().x()<-1800.f
	  || globalRobot2().y()<100.f || globalRobot2().y()>3000.f)
		return 3;
	if(globalBall2().x()>1800.f || globalBall2().x()<-1800.f
	  || globalBall2().y()<100.f || globalBall2().y()>3000.f)
		return 3;
	if(theLibCodeRelease.timeSinceBallWasSeen > 4000.f)
		return 0;
	if(globalBall2().x()<=1800.f && globalBall2().x()>=-1800.f
	  && globalBall2().y()>=100.f && globalBall2().y()<=3000.f
	  && theBallModel.estimate.position.norm()>450.f)
		return 2;
	if(globalBall2().x()<=1800.f && globalBall2().x()>=-1800.f
	  && globalBall2().y()>=100.f && globalBall2().y()<=3000.f
	  && theBallModel.estimate.position.norm()<=450.f)
		return 1;    
	  /*
	  bool teamGotBall = false;//判断队伍是否拿球
	  //计算队伍和敌人离球的最近距离
	  float lenTeam = Inf;
	  float lenOppo = Inf;
	  size_t i;
	  for( i=0;i<o.size();i++)
		{
		   if(o[i].type == Obstacle::teammate && o[i].center.norm()<lenTeam)
				lenTeam = o[i].center.norm();
		   if(o[i].type == Obstacle::opponent && o[i].center.norm()<lenOppo)
				lenOppo = o[i].center.norm();
		}
	  if(lenTeam < lenOppo && lenOppo-lenTeam >200.f && lenTeam < 500.f)
		teamGotBall = true;//队友持球
	  */
	  //如果球在防区时，进入防御模式：截球
	  //如果球不在防区内
	  /*
	  //计算球离防区边界四点最近距离
	  float lenField = inf;
	  float len1 = Vector2f(globalBall2.x()-3000.f, globalBall2.y()).norm();
	  float len2 = Vector2f(globalBall2.x()-3000.f, globalBall2.y()+1500.f).norm();
	  float len3 = Vector2f(globalBall2.x()+3000.f, globalBall2.y()).norm();
	  float len4 = Vector2f(globalBall2.x()+3000.f, globalBall2.y()+1500.f).norm();
	  if(len1 < lenField)
		  lenField = len1;
	  if(len2 < lenField)
		  lenField = len2;
	  if(len3 < lenField)
		  lenField = len3;
	  if(len4 < lenField)
		  lenField = len4;
	  */
	  //如果球不在防区，回到场地中央
	  /*if(lenTeam < lenOppo && lenTeam < 500.f)
		return 3;
	  /如果球不在防区，回到场地中央
	  if(lenTeam < lenOppo && (lenTeam > 500.f || lenField < 1000.f))
		return 3;
	  //如果球不在防区，回到场地中央
	  if(lenTeam >= lenOppo)
		return 3;
	 */
}
bool getKickAngle(Vector2f& shootTarget){
	const float step=2*(theFieldDimensions.yPosLeftGoal-100)/400;
	const float shootLen=5000.f;
	
	Vector2f pointGoal(theFieldDimensions.xPosOpponentGroundline+ballRadius,0);
	segment segFromBallToGoal(theBallModel.estimate.position,pointGoal);
	std::vector<std::pair<float,Vector2f> > candidates;
	std::pair<float,Vector2f> temp;
	for(float i=theFieldDimensions.yPosRightGoal+100;i<=theFieldDimensions.yPosLeftGoal-100;i+=step){
		pointGoal[1]=i;
			segFromBallToGoal.end[1]=pointGoal;
			temp.first=minDisBetweenSegmentAndObstacle(segFromBallToGoal);
			temp.second=pointGoal;
			candidates.push_back(temp);
		}
	
	if(!candidates.empty())
		shootTarget=maxElement(candidates);
	
	return !candidates.empty();
}
