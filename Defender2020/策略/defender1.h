//parameter　↓↓↓`9/26 15:52` @author: Daiyilong
//basic parameters of the field:
float passAngle=100;
const Vector2f frontLeft = Vector2f(4500.f, 3000.f);
const Vector2f frontRight = Vector2f(4500.f, -3000.f);
const Vector2f midMid = Vector2f(0.f, 0.f);
const Vector2f midRight = Vector2f(0.f, -3000.f);
const Vector2f midLeft = Vector2f(0.f, 3000.f);
const Vector2f backLeft = Vector2f(-4500.f, 3000.f);
const Vector2f backMid = Vector2f(-4500.f, 0.f);
const Vector2f backRight = Vector2f(-4500.f, -3000.f);
const Vector2f globalPatrolLeft = Vector2f(-3400.f, 500.f);
const Vector2f globalPatrolRight = Vector2f(-3400.f, -500.f);
const Vector2f quickShotPoint = Vector2f(4500.f, 0.f);
//basic parameters of ball and robots:
Vector2f rBall;	//i.e. relative ball
Vector2f gBall;	 //i.e. global ball
Vector2f selfLocation;
//basic parameters of defending strategies:
Vector2f patrolLeft;
Vector2f patrolRight;
Vector2f patrolPoint;
//basic areas of the field: (--> judge which strategy to take)
struct Area{
	

	Vector2f leftFrontPoint;
	Vector2f rightFrontPoint;
	Vector2f leftBackPoint;
	Vector2f rightBackPoint;
	
};

Area walkToBallArea = {Vector2f(-1800.f, 3000.f), Vector2f(-1800.f, -3000.f), backLeft, backRight};
Area defendOpponentArea = {Vector2f(-2500.f, 2000.f), Vector2f(-2500.f, -2000.f), Vector2f(-4500.f, 2000.f), Vector2f(-4500.f, -2000.f)};
Area defendBallArea = {midLeft, midRight, backLeft, backRight};
Area globalBallSafeArea = {frontLeft, frontRight, midLeft, midRight};
Area rightHalfCourt = {midMid, midRight, backMid, backRight};
Area leftHalfCourt = {midLeft, midMid, backLeft, backMid};
Area penaltyBox = {Vector2f(-3900.f, 1100.f), Vector2f(-3900.f, -1100.f), Vector2f(-4500.f, 1100.f), Vector2f(-4500.f, -1100.f)};
Area keeperArea = {Vector2f(-3400.f, 1500.f), Vector2f(-3400.f, -1500.f), Vector2f(-4500.f, 1500.f), Vector2f(-4500.f, -1500.f)};
//advanced parameters of players on the field:
std::vector<Obstacle> obstacle;
//std::vector<Teammate> teammate = theTeamData.teammates;
/*functions ↓↓ */
Vector2f toRobot(Vector2f a){
	Vector2f b;
	return b = Transformation::fieldToRobot(theRobotPose, a);
	//transform the current relation of coordinate into the other kind
}
Vector2f toField(Vector2f a){
	Vector2f b;
	return b = Transformation::robotToField(theRobotPose, a);
	//transform the current relation of coordinate into the other kind
}

//Judge whether there is Oppo seen 
bool ifAnyOppSeen(void)
{
	bool flag = false;
	for(int i=0;i<obstacle.size();i++)
	{
		if(obstacle[i].isOpponent())
		{
			flag = true;
			break;
		}
	}
	return flag;
}
//2020 07 25 DTW
//如果看不到对方机器人返回敌人在无限远处
Vector2f getNearestOpp(void){
	
	size_t pin = 0;
	size_t i = 0; 
	//2020 07 25 DTW
	//If there is no Opp then return A place which is INF far from  you
	Vector2f globalObs(1000000,1000000);
	for( i=0; i < obstacle.size(); i++)
	{	
		float  minDis = 10000;
			
		if(obstacle[i].isOpponent())
		{	
			if(std::abs(obstacle[i].center.norm()) <= minDis)
			{
				minDis = std::abs(obstacle[i].center.norm());
				pin = i;
				globalObs = Transformation::robotToField(theRobotPose, obstacle[pin].center);
			}
							
		}
	}
	return globalObs;
	//get the global location of the nearest opponent robot away from myself
}
Vector2f getNearestOppInArea(Area a){
		
	size_t pin = 0;
	size_t i = 0; 
	//2020 07 25 DTW
	//If there is no Opp then return A place which is INF far from  you
	Vector2f globalObs(1000000,1000000);
	for( i=0; i < obstacle.size(); i++)
	{	
		float  minDis = 10000;
		if(obstacle[i].type == Obstacle::opponent && judgePosition(toField(obstacle[i].center), a))
		{	
			if(std::abs(obstacle[i].center.norm()) <= minDis)
			{
				minDis = std::abs(obstacle[i].center.norm());
				pin = i;
				globalObs = Transformation::robotToField(theRobotPose, obstacle[pin].center);
			}
							
		}
	}
	return globalObs;
	//get the global location of the nearest opponent robot away from myself
}
Vector2f getNearestOppR(void){
	
	size_t pin = 0;
	size_t i = 0; 
	//2020 07 25 DTW
	//If there is no Opp then return A place which is INF far from  you
	Vector2f globalObs(1000000,1000000);
	for( i=0; i < obstacle.size(); i++)
	{	
		float  minDis = 10000;
			
		if(obstacle[i].isOpponent())
		{	
			if(std::abs(obstacle[i].center.norm()) <= minDis)
			{
				minDis = std::abs(obstacle[i].center.norm());
				pin = i;
				globalObs = obstacle[pin].center;
			}
							
		}
	}
	return globalObs;
	//get the global location of the nearest opponent robot away from myself
}
bool onMyLeftOrNot(void){
	
	if(getNearestOppR().y() <= 0)
		return true;
	else
		return false; 
}
//↓↓↓还要改！！！
/*
Vector2f countDefendPoint(void){
	float x1 = gBall.x();
	float y1 = gBall.y();
	float x2 = goalkeeperlocation.x();
	float y2 = goalkeeperlocation.y();
	float r = robotR;
	float y = ((y2 -r) - y1))/(x2 - x1))*(-4500 - x1) + y1;
	float dp = 0;
	if(y >=-750 && y <=750)
	{
		dp =  (y-750)/2;
	}
	else if(y > 750)
	{
		dp = 0;
	}
	else
	{
		dp = -375;
	}
	Vector2f defendP = (-3900, dp);
	return defendP;
}
↑↑↑还要改！！！ */

//following functions contain particular parameters, change them if needed !!
Vector2f getDefendPoint(Vector2f a, Vector2f b){ //location/coordinate, update needed, plz take it with a grain of salt.
	
	Vector2f def;
	float x0 = a.x();
	float y0 = a.y();
	float x1 = b.x();
	float y1 = b.y();
	if(y0 != 0)
	{
		def.x() =((x0+4500)*x1/y0+y1-y0*4500/(x0+4500))
					/(y0/(x0+4500)+(x0+4500)/y0);
					
		def.y() = (y0/(x0+4500)*(def.x()+4500)) - 50;
	}
	else 
	{
		def.x() = -3800.f;
		def.y() = -1000.f;
	}
	
	return def;
	//calculate the defend point (global coordinate)
}
float getRescueAngle(Vector2f b, Vector2f s){//return a suitable rescue angle, s.t. b&s --> global coordinate.  
	
	Vector2f rescuePoint;
	if(b.y() > s.y())
	{
		rescuePoint.x() = b.x() + 1000.f;
		rescuePoint.y() = 3000.f;
	}
	else
	{
		rescuePoint.x() = b.x() + 1000.f;
		rescuePoint.y() = -3000.f;
	}
	return(Transformation::fieldToRobot(theRobotPose, rescuePoint).angle());
		
}
float getQuickShotAngle(void){
	
	return(Transformation::fieldToRobot(theRobotPose, quickShotPoint).angle());
	
}
float checkSideKickAngle(Vector2f b, Vector2f s){//s.t. global ball coordinate.
	
	float xb = b.x();
	float yb = b.y();
	float xs = s.x();
	float ys = s.y();
	if(ys = yb)
		return true;
	else
	{
		float x = (xb - xs);
		float y = std::abs(yb - ys);
		if((x/y) > 1.8)
			return true;
		else
			return false;
	}
	
}
bool judgePosition(Vector2f a, Area b){ //area
		
	//a--> the estimated vector, b--> the field area
	if(a.x() <= b.leftFrontPoint.x() && a.x() >= b.rightBackPoint.x()
			&& a.y() <= b.leftFrontPoint.y() && a.y() >= b.rightBackPoint.y())
		return true;
	else
		return false;
	//whether the target is in the target area
}
bool ifBallLoseSight(void){// time limitation
	
	if(theLibCodeRelease.timeSinceBallWasSeen >4000)
		return true;
	else 
		return false;
	//judge if the ball is seen or in sight.
}
bool ifBallCloseEnough(void){ //distance
	
	if(rBall.norm() < 500.f) //change this parameter if you need 
		return true;
	else
		return false;
	//judge if the ball is close enough to make further decision.
}
bool ifAnyOppInArea(Area a){　//area
	
	bool jud = false;
	size_t i = 0; 
	Vector2f globalObs;
	for( i=0; i < obstacle.size(); i++)
	{	
		if(obstacle[i].type == Obstacle::opponent)
		{	
			globalObs = toField(obstacle[i].center);
			if(judgePosition(globalObs, a))
			{
				jud = true;
				return jud;
			}
		}
	}
	return jud;
	//judge if there are any opponent in the particular area.
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

float getPassAngleD(){
	
}

option(defender1)
{
	obstacle = theObstacleModel.obstacles; 
	rBall = theBallModel.estimate.position;	//i.e. relative ball
	gBall = Transformation::robotToField(theRobotPose, rBall);	 //i.e. global ball
	selfLocation = theRobotPose.translation;
	patrolLeft = Transformation::fieldToRobot(theRobotPose, globalPatrolLeft);
	patrolRight = Transformation::fieldToRobot(theRobotPose,globalPatrolRight);
	initial_state(start)
 	{
	  
		transition
		{
			if(state_time>1000)
				goto searchForBall;
		}
	
		action
		{
			HeadControlMode(HeadControl::lookForward);
			Stand();
		}
		
	}
	
	state(restart)
	{
		
		transition
		{
			if(state_time > 4500.f)
				goto searchForBall;
		}
		
		action
		{
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(0_deg, 0.f,0));
		}
		
	}

	state(searchForBall)
	{
		
		transition
		{
			if(theLibCodeRelease.timeSinceBallWasSeen<300)
				goto patrolToHind;
	
			if(theLibCodeRelease.timeSinceBallWasSeen<300 
				&& judgePosition(gBall, keeperArea) && ifAnyOppInArea(defendOpponentArea))
				goto defendOpponent;
		}
		
		action
		{
			Stand();
			LookRound();
			if(state_time > 3000)
			{
				HeadControlMode(HeadControl::lookForward);
				WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
			}
		}
		
	}

	state(patrolToHind)
	{
		
		transition
		{
			if(ifBallLoseSight())
				goto searchForBall;
			
			if(judgePosition(gBall,	defendBallArea) && !judgePosition(gBall, keeperArea))
				goto defendBall;
			
			if(judgePosition(gBall, globalBallSafeArea) && ifAnyOppInArea(defendOpponentArea))
				goto defendOpponent;
				
			if(judgePosition(gBall, keeperArea) && ifAnyOppInArea(defendOpponentArea))
				goto defendOpponent;
		} 
		
		action
		{
			//OUTPUT_TEXT(ifAnyOppInArea(defendOpponentArea));
			if(gBall.y() > 0)
				patrolPoint = patrolLeft;
			else
				patrolPoint = patrolRight;
			
			HeadControlMode(HeadControl::lookForward);
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(rBall.angle(), 0.f, 0.f));
			if(patrolPoint.norm() >= 200.f)
			{
				//OUTPUT_TEXT(patrolPoint.norm());
				//HeadControlMode(HeadControl::lookForward);
				Pose2f target;
				target.rotation = std::abs(patrolPoint.angle());
				target.translation = Transformation::robotToField(theRobotPose, patrolPoint);
				Vector2f rtarget = patrolPoint;
				WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f),Pose2f(0.f,rtarget));
				//theMotionRequest = thePathPlanner.plan(target,Pose2f(0.8f,0.8f,0.8f),false);
			}
			else //看向球之后站立
			{
				//OUTPUT_TEXT("yes");
				LookAtBall();
				if(std::abs(rBall.angle()) >= 5_deg)
				{
					WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(rBall.angle(), 0.f, 0.f));
				}
				else
				{
					Stand();
				}
				
			}
		
		}
		
	}
	
	state(defendBall)
	{
		transition
		{
			if(ifBallLoseSight())
				goto searchForBall;
				
			if(judgePosition(gBall, globalBallSafeArea) && ifAnyOppInArea(defendOpponentArea))
				goto defendOpponent;
			
			if(judgePosition(gBall, globalBallSafeArea) && !ifAnyOppInArea(defendOpponentArea))
				goto patrolToHind;
			
			if(std::abs(toRobot(getDefendPoint(gBall, selfLocation)).norm()) < 150.f && judgePosition(gBall, walkToBallArea))
				goto walkToBall;	
				
			if(std::abs(rBall.norm()) < 600.f && std::abs(getNearestOppR().norm()) > 700.f)
				goto prepareToPass;
				
			if(std::abs(rBall.norm()) < 600.f && std::abs(getNearestOppR().norm()) <= 700.f)
				goto prepareToRescue;
				
			if(judgePosition(gBall,walkToBallArea) && (gBall.x() < selfLocation.x()))
				goto walkToBall;
		}
		
		action
		{	
			if(toRobot(getDefendPoint(gBall, selfLocation)).norm()>=100)
			{
				LookAtBall();
				Vector2f rDefendBall;
				rDefendBall = toRobot(getDefendPoint(gBall, selfLocation));
				WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(rDefendBall.angle(), rDefendBall.x(), rDefendBall.y()));
			}
			else
			{
				LookAtBall();
				if(std::abs(rBall.angle()) >= 5_deg)
					WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(rBall.angle(), 0.f, 0.f));
				else
					Stand();
			}
			
		}
	}
	
	state(defendOpponent)
	{
		
		transition
		{
			if(theLibCodeRelease.timeSinceBallWasSeen >15000)
				goto searchForBall;
			
			if(judgePosition(gBall,	defendBallArea) && !judgePosition(gBall, keeperArea))
				goto defendBall;

			if(judgePosition(gBall, globalBallSafeArea) && !ifAnyOppInArea(defendOpponentArea))
				goto patrolToHind;
		}
		
		action
		{
			HeadControlMode(HeadControl::lookForward);
			Vector2f dop = getNearestOppInArea(defendOpponentArea);
			dop = Vector2f(dop.x()-80.f, dop.y());
			Vector2f rDop;
			rDop = toRobot(dop);
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(rDop.angle(), rDop.x(), rDop.y()));
		}
		
	}
	
	state(walkToBall)
	{
		/*
		OUTPUT_TEXT("disBall");
		OUTPUT_TEXT(std::abs(rBall.norm()));
		
		OUTPUT_TEXT("nearDisOppo");
		OUTPUT_TEXT(std::abs(getNearestOpp().norm()));
		 * */
		transition
		{
			if(ifBallLoseSight())
				goto searchForBall;
				
			if(judgePosition(gBall, globalBallSafeArea) && ifAnyOppInArea(defendOpponentArea))
				goto defendOpponent;
			
			if(judgePosition(gBall, globalBallSafeArea) && !ifAnyOppInArea(defendOpponentArea))
				goto patrolToHind;
				
			/*
			//2020 07 25 DTW
			//先判断右没有对手 若没有 则直接考虑传球
			if(!ifAnyOppSeen())
				goto prepareToPass;
			*/
			if(judgePosition(gBall, keeperArea) && !ifAnyOppInArea(defendOpponentArea))
				goto patrolToHind;
				
			if(judgePosition(gBall, keeperArea) && ifAnyOppInArea(defendOpponentArea))
				goto defendOpponent;
				
			if(std::abs(rBall.norm()) < 600.f && std::abs(getNearestOppR().norm()) > 700.f)
				goto prepareToPass;
				
			if(std::abs(rBall.norm()) < 600.f && std::abs(getNearestOppR().norm()) <= 700.f)
				goto prepareToRescue;
		}
		
		action
		{	
			HeadControlMode(HeadControl::lookForward);
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(rBall.angle(), 0.f, 0.f));
		
		　	Pose2f target;
			target.rotation=rBall.angle();
			target.translation=gBall;
			theMotionRequest=thePathPlanner.plan(target,Pose2f(0.5f,0.5f,0.5f),false);
			
			HeadControlMode(HeadControl::lookForward);
			
		}
		
	}
	
	state(prepareToPass)
	{
		
		transition
		{
			if(ifBallLoseSight())
				goto searchForBall;
				
			if(judgePosition(gBall, globalBallSafeArea) && ifAnyOppInArea(defendOpponentArea))
				goto defendOpponent;
			
			if(judgePosition(gBall, globalBallSafeArea) && !ifAnyOppInArea(defendOpponentArea))
				goto patrolToHind;
				
			if(std::abs(rBall.norm()) > 600.f)
				goto walkToBall;
				
			if(std::abs(getNearestOppR().norm()) <= 700.f && std::abs(rBall.norm()) <= 600.f)
				goto prepareToRescue;
			//
			if(getPassAngle(passAngle) && std::abs(getNearestOppR().norm()) > 700.f && std::abs(rBall.norm()) <= 600.f)
			{
				if(std::abs(passAngle) < 10_deg && std::abs(rBall.y()) < 100.f)
					goto alignToPass;
			}
			else if(1)//没有角度直接解围
			{
			
				if(std::abs(rBall.norm()) < 600.f && std::abs(getNearestOppR().norm()) < 700.f
					&& selfLocation.x() >= gBall.x())
					goto alignToRescueKick;
					
				if(std::abs(rBall.norm()) < 600.f && std::abs(getNearestOppR().norm()) < 700.f
					&& selfLocation.x() < gBall.x() && onMyLeftOrNot())
					goto alignSideKickLeft;
					
				if(std::abs(rBall.norm()) < 600.f && std::abs(getNearestOppR().norm()) < 700.f
					&& selfLocation.x() < gBall.x() && !onMyLeftOrNot())
					goto alignSideKickRight;
					
				else
					goto quickShot;
			}
				
		}
		
		action
		{
			HeadControlMode(HeadControl::lookForward);
			LookAtBall();
			if(getPassAngle(passAngle))
				WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(passAngle, rBall.x() - 300.f, rBall.y()));
		}
		
	}
	
	//待解决的问题： 要判断机器人在球后面才解围
	state(prepareToRescue)
	{
		
		transition
		{
			if(ifBallLoseSight())
				goto searchForBall;
				
			if(judgePosition(gBall, globalBallSafeArea) && ifAnyOppInArea(defendOpponentArea))
				goto defendOpponent;
			
			if(judgePosition(gBall, globalBallSafeArea) && !ifAnyOppInArea(defendOpponentArea))
				goto patrolToHind;
			//2020 07 25 DTW
			//防止循环
			if(std::abs(rBall.norm()) <= 600.f && getPassAngle(passAngle) 
					&& std::abs(toRobot(getNearestOpp()).norm()) > 700.f)
				goto prepareToPass;
				
			if(std::abs(rBall.norm()) > 600.f)
				goto walkToBall;
				
			if(std::abs(rBall.norm()) < 600.f && std::abs(getNearestOppR().norm()) < 700.f
				&& selfLocation.x() >= gBall.x())
				goto alignToRescueKick;
			
			if(std::abs(rBall.norm()) < 600.f && std::abs(getNearestOppR().norm()) < 700.f
				&& selfLocation.x() < gBall.x() && onMyLeftOrNot())
				goto alignSideKickLeft;
				
			if(std::abs(rBall.norm()) < 600.f && std::abs(getNearestOppR().norm()) < 700.f
				&& selfLocation.x() < gBall.x() && !onMyLeftOrNot())
				goto alignSideKickRight;	
				
				
		}
		
		action
		{
			HeadControlMode(HeadControl::lookForward);
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(rBall.angle(), 0.f, 0.f));
			theHeadControlMode = HeadControl::lookForward;
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(rBall.angle(), rBall.x() - 165.f, rBall.y()));
		}
		
	}
	
	state(alignToPass)
	{
		
		transition
		{
			if(theLibCodeRelease.timeSinceBallWasSeen > 3000)
				goto patrolToHind;
				
			if(getPassAngle(passAngle))
			{
				if(theLibCodeRelease.between(rBall.y(), 35.f, 55.f)
					&& theLibCodeRelease.between(rBall.x(), 150.f, 175.f)
					/*&& std::abs(passAngle) < 2_deg*/)
					goto pass;
			}
			else if(1)//没有角度直接解围
			{
				if(std::abs(rBall.norm()) < 600.f && std::abs(getNearestOppR().norm()) < 700.f
					&& selfLocation.x() >= gBall.x())
					goto alignToRescueKick;
					
				if(std::abs(rBall.norm()) < 600.f && std::abs(getNearestOppR().norm()) < 700.f
					&& selfLocation.x() < gBall.x() && onMyLeftOrNot())
					goto alignSideKickLeft;
					
				if(std::abs(rBall.norm()) < 600.f && std::abs(getNearestOppR().norm()) < 700.f
					&& selfLocation.x() < gBall.x() && !onMyLeftOrNot())
					goto alignSideKickRight;
					
				else
					goto quickShot;
			}
			
			if(std::abs(rBall.norm()) > 600.f)
				goto prepareToPass;
			
			
		}
		
		action
		{
			
			HeadControlMode(HeadControl::lookForward);
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(rBall.angle(), 0.f, 0.f));
			//HeadControlMode(HeadControl::lookDown);
			if(getPassAngle(passAngle))
				WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(passAngle, rBall.x() - 165.f, rBall.y() - 42.f));
			//OUTPUT_TEXT(rBall.x());
			//OUTPUT_TEXT(rBall.y());
		
		}
		
	}

	state(alignToRescueKick)
	{

		transition
		{
			if(theLibCodeRelease.timeSinceBallWasSeen > 3000)
				goto patrolToHind;
				
			if(theLibCodeRelease.between(rBall.y(), 20.f, 60.f)
					&& theLibCodeRelease.between(rBall.x(), 140.f, 180.f) && selfLocation.x() -50.f <= gBall.x())
				goto rescueKick;
			
			if(std::abs(rBall.norm()) > 600.f)
				goto prepareToRescue;
		}

		action
		{
			HeadControlMode(HeadControl::lookForward);
			//WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(rBall.angle(), 0.f, 0.f));
			
			//theHeadControlMode = HeadControl::lookForward;
			if(selfLocation.y() > gBall.y())
				WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(getRescueAngle(gBall, selfLocation), rBall.x() - 165.f, rBall.y() - 42.f));
			else
				WalkToTarget(Pose2f(-0.5f, 0.5f, 0.5f), Pose2f(getRescueAngle(gBall, selfLocation), rBall.x() - 165.f, rBall.y() - 42.f));
		}

	}
	
	state(rescueKick)
	{
		
		transition
		{
			if(state_time > 3000 || (state_time > 10 && action_done))
				goto restart;
		}
		
		action
		{
			HeadControlMode(HeadControl::lookForward);
			//SpecialAction(SpecialActionRequest::kickfoot);
			InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left),
						Pose2f(0, rBall.x() - 100.f, rBall.y() - 80.f));
		}
		
	}
	
	state(pass)
	{
		
		transition
		{
			if(state_time > 4000 || (state_time > 10 && action_done))
				goto restart;
			//OUTPUT_TEXT(action_done);
		}
		
		action
		{
			HeadControlMode(HeadControl::lookForward);
			SpecialAction(SpecialActionRequest::kickfoot);
		}
		
  }
	
	state(alignSideKickLeft)
	{
		
		transition
		{
			
			if(theLibCodeRelease.timeSinceBallWasSeen > 3000.f)
				goto searchForBall;
				
			if(theLibCodeRelease.between(rBall.y(), -30.f, 0.f)
				&& theLibCodeRelease.between(rBall.x(), 160.f, 190.f)
				&& checkSideKickAngle(gBall, selfLocation))
				goto sideKickLeft;
				
			if(!onMyLeftOrNot())
				goto alignSideKickRight;
				
			if(std::abs(rBall.norm()) > 600.f)
				goto prepareToRescue;
				
			if(getNearestOpp().x() < (selfLocation.x()-50))
				goto quickShot;
		}
	
		action
		{
			
			theHeadControlMode = HeadControl::lookForward;
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(rBall.angle(), rBall.x() - 180.f, rBall.y() + 30.f));
			
		}
		
	}
	
	state(sideKickLeft)
	{
		transition
		{
			if(theLibCodeRelease.timeSinceBallWasSeen > 3000.f)
				goto searchForBall;
			
			if(state_time > 4000 || (state_time > 10 && action_done))
				goto restart;
     
		}
   
		action
		{
			//OUTPUT_TEXT(onMyLeftOrNot());
			HeadControlMode(HeadControl::lookForward);
			InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::left),
						Pose2f(rBall.angle(), rBall.x()-100.f, rBall.y() - 80.f));
		}
 
	}
  
	state(alignSideKickRight)
	{
		
		transition
		{
			if(theLibCodeRelease.timeSinceBallWasSeen > 3000.f)
				goto searchForBall;
				
			if(theLibCodeRelease.between(rBall.y(), 0.f, 30.f)
				&& theLibCodeRelease.between(rBall.x(), 160.f, 190.f)
				&& checkSideKickAngle(gBall, selfLocation))
				goto sideKickRight;
			
			if(onMyLeftOrNot())
				goto alignSideKickLeft;
				
			if(std::abs(rBall.norm()) > 600.f)
				goto prepareToRescue;
				
			if(getNearestOpp().x() < (selfLocation.x()-50))
				goto quickShot;
		}
		
		action
		{
			//OUTPUT_TEXT(getNearestOppR().norm());
			theHeadControlMode = HeadControl::lookForward;
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(rBall.angle(), rBall.x() - 180.f, rBall.y() - 30.f));
			
		}
		
	}
	
	state(sideKickRight)
	{
		  
		transition
		{
			
			if(theLibCodeRelease.timeSinceBallWasSeen > 3000.f)
				goto searchForBall;
			
			if(state_time > 4000 || (state_time > 10 && action_done))
				goto restart;
				
		}
		
		action
		{
			//OUTPUT_TEXT(getNearestOppR().norm());
			HeadControlMode(HeadControl::lookForward);
			InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::right),
	　　　　　 	Pose2f(getRescueAngle(gBall, selfLocation), rBall.x()-10.f , rBall.y() + 80.f));
	
		}
		
	}

	state(quickShot)
	{
		transition
		{
			
			if(theLibCodeRelease.timeSinceBallWasSeen > 3000)
				goto patrolToHind;
				
			if(judgePosition(gBall, globalBallSafeArea) && !ifAnyOppInArea(defendOpponentArea))
				goto patrolToHind;
				
			if(std::abs(getQuickShotAngle()) < 10_deg && std::abs(rBall.y()) < 100.f)
				goto alignToShot;
			
			if(std::abs(rBall.norm()) > 600.f)
				goto prepareToRescue;
		}
		action
		{
			//OUTPUT_TEXT(getNearestOppR().norm());
			HeadControlMode(HeadControl::lookForward);
			LookAtBall();
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(getQuickShotAngle(), rBall.x() - 400.f, rBall.y()));
		}
	}
	
	state(alignToShot)
	{
		transition
		{
			if(theLibCodeRelease.timeSinceBallWasSeen > 3000)
				goto patrolToHind;
				
			if(theLibCodeRelease.between(rBall.y(), 35.f, 55.f)
					&& theLibCodeRelease.between(rBall.x(), 150.f, 175.f)
					/*&& std::abs(passAngle) < 2_deg*/)
				goto pass;
			
			if(std::abs(rBall.norm()) > 600.f)
				goto prepareToRescue;
		}
		action
		{

			//HeadControlMode(HeadControl::lookForward);
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(rBall.angle(), 0.f, 0.f));
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(getQuickShotAngle(), rBall.x() - 165.f, rBall.y() - 42.f));
		}
	}

}
