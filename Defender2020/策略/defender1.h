//parameter　↓↓↓`21:44
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
//basic parameters of ball and robots:
Vector2f rBall;	//i.e. relative ball
Vector2f gBall;	 //i.e. global ball
Vector2f selfLocation;
//basic parameters of defending strategies:
const Vector2f patrolHind = Transformation::fieldToRobot(theRobotPose,Vector2f(-3400.f,500.f));
const Vector2f patrolFront = Transformation::fieldToRobot(theRobotPose,Vector2f(-3400.f,-500.f));
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
Vector2f getDefendPoint(Vector2f a, Vector2f b){ //location/coordinate
	
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
	size_t pin = 0;
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



option(defender1)
{
	obstacle = theObstacleModel.obstacles; 
	rBall = theBallModel.estimate.position;	//i.e. relative ball
	gBall = Transformation::robotToField(theRobotPose, rBall);	 //i.e. global ball
	selfLocation = theRobotPose.translation;
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

	state(searchForBall)
	{
		
		transition
		{
			if(theLibCodeRelease.timeSinceBallWasSeen<300)
				goto patrolToHind;	
		}
		
		action
		{
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
			
			if(judgePosition(gBall,	defendBallArea))
				goto defendBall;
			
			if(judgePosition(gBall, globalBallSafeArea) && ifAnyOppInArea(defendOpponentArea))
				goto defendOpponent;
		}
		
		action
		{
			
			
			if(patrolHind.norm() >= 200.f)
			{
				//HeadControlMode(HeadControl::lookForward);
				Pose2f target;
				target.rotation = std::abs(patrolHind.angle());
				target.translation = Vector2f(-3800.f,-600.f);
				Vector2f rtarget = Transformation::fieldToRobot(theRobotPose,target.translation);
				WalkToTarget(Pose2f(0.8f,0.8f,0.8f),Pose2f(0.f,rtarget));
				//theMotionRequest = thePathPlanner.plan(target,Pose2f(0.8f,0.8f,0.8f),false);
			}
			else //看向球之后站立
			{
				LookAtBall();
				if(std::abs(rBall.angle()) >= 5_deg)
				{
					WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(rBall.angle(), 0.f, 0.f));
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
			
			if(std::abs(toRobot(getDefendPoint(gBall, selfLocation)).norm()) < 50.f && judgePosition(gBall, walkToBallArea))
				goto walkToBall;	
				
			if(std::abs(rBall.norm()) < 600.f)
				goto prepareToPass;
			if(judgePosition(gBall,walkToBallArea) && (gBall.x() < selfLocation.x()))
				goto walkToBall;
		}
		
		action
		{	
			if(toRobot(getDefendPoint(gBall, selfLocation)).norm()>=100)
			{
				LookAtBall();
				Pose2f target;				//如果需要避障
				target.rotation = toRobot(getDefendPoint(gBall, selfLocation)).angle();
				target.translation = getDefendPoint(gBall, selfLocation);
				theMotionRequest = thePathPlanner.plan(target,Pose2f(0.8f,0.8f,0.8f),false);
			}
			else
			{
				LookAtBall();
				if(std::abs(rBall.angle()) >= 5_deg)
					WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(rBall.angle(), 0.f, 0.f));
				else
					Stand();
			}
		
			
		}
	}
	
	state(defendOpponent)
	{
		
		transition
		{
			if(ifBallLoseSight())
				goto searchForBall;
			
			if(judgePosition(gBall,	defendBallArea))
				goto defendBall;

			if(judgePosition(gBall, globalBallSafeArea) && !ifAnyOppInArea(defendOpponentArea))
				goto patrolToHind;
		}
		
		action
		{
			Vector2f dop = getNearestOppInArea(defendOpponentArea);
			dop = Vector2f(dop.x()-100.f, dop.y());
			Pose2f target;				
			target.rotation = toRobot(dop).angle();
			target.translation = dop;
			theMotionRequest = thePathPlanner.plan(target,Pose2f(0.8f,0.8f,0.8f),false);
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
				
			if(std::abs(rBall.norm()) < 600.f && std::abs(toRobot(getNearestOpp()).norm()) > 600.f)
				goto prepareToPass;
				
			if(std::abs(toRobot(getNearestOpp()).norm()) <= 600.f && std::abs(rBall.norm()) < 600.f)
				goto prepareToRescue;
		}
		
		action
		{
			HeadControlMode(HeadControl::lookForward);
			WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(rBall.angle(), 0.f, 0.f));
		
		　	Pose2f target;
			target.rotation=rBall.angle();
			target.translation=gBall;
			theMotionRequest=thePathPlanner.plan(target,Pose2f(0.8f,0.8f,0.8f),false);
			
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
				
			if(std::abs(toRobot(getNearestOpp()).norm()) <= 600.f && std::abs(rBall.norm()) <= 600.f)
				goto prepareToRescue;
			//
			if(getPassAngle(passAngle))
			{
				if(std::abs(passAngle) < 10_deg && std::abs(rBall.y()) < 100.f)
					goto alignToPass;
			}
			else if(1)//没有角度直接解围
			{
				goto prepareToRescueNoPass;
			}
				
		}
		
		action
		{
			HeadControlMode(HeadControl::lookForward);
			LookAtBall();
			if(getPassAngle(passAngle))
				WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(passAngle, rBall.x() - 400.f, rBall.y()));
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
			if(std::abs(rBall.norm()) <= 600.f && getPassAngle(passAngle) && std::abs(toRobot(getNearestOpp()).norm()) > 600.f)
				goto prepareToPass;
				
			if(std::abs(rBall.norm()) > 600.f)
				goto walkToBall;
				
			if(std::abs(rBall.norm()) < 600.f && std::abs(toRobot(getNearestOpp()).norm()) < 600.f)
				goto alignToRescue;
		}
		
		action
		{
			HeadControlMode(HeadControl::lookForward);
			WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(rBall.angle(), 0.f, 0.f));
			theHeadControlMode = HeadControl::lookForward;
			WalkToTarget(Pose2f(40.f, 40.f, 40.f), Pose2f(rBall.angle(), rBall.x() - 165.f, rBall.y()));
		}
		
	}
	
	//防止循环
	state(prepareToRescueNoPass)
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
				
			if(std::abs(rBall.norm()) > 600.f)
				goto walkToBall;
				
			if(theLibCodeRelease.between(rBall.y(), -30.f, 20.f)
				&& theLibCodeRelease.between(rBall.x(), 140.f, 175.f))
				goto rescueKick;
		}
		
		action
		{
			HeadControlMode(HeadControl::lookForward);
			WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(rBall.angle(), 0.f, 0.f));
			theHeadControlMode = HeadControl::lookForward;
			WalkToTarget(Pose2f(40.f, 40.f, 40.f), Pose2f(rBall.angle(), rBall.x() - 165.f, rBall.y()));
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
			
			if(std::abs(rBall.norm()) > 600.f)
				goto prepareToPass;
			
		}
		
		action
		{
			
			HeadControlMode(HeadControl::lookForward);
			WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(rBall.angle(), 0.f, 0.f));
			theHeadControlMode = HeadControl::lookForward;
			if(getPassAngle(passAngle))
				WalkToTarget(Pose2f(20.f, 20.f, 20.f), Pose2f(passAngle, rBall.x() - 165.f, rBall.y() - 42.f));
		
		}
		
	}

	state(alignToRescue)
	{

		transition
		{
			if(theLibCodeRelease.timeSinceBallWasSeen > 3000)
				goto patrolToHind;
				
			if(theLibCodeRelease.between(rBall.y(), 35.f, 55.f)
				&& theLibCodeRelease.between(rBall.x(), 140.f, 175.f))
				goto rescueKick;
			
			if(std::abs(rBall.norm()) > 600.f)
				goto prepareToRescue;
		}

		action
		{
			HeadControlMode(HeadControl::lookForward);
			WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(rBall.angle(), 0.f, 0.f));
			theHeadControlMode = HeadControl::lookForward;
			WalkToTarget(Pose2f(20.f, 20.f, 20.f), Pose2f(rBall.angle(), rBall.x() - 165.f, rBall.y() - 42.f));
		}

	}
	
	state(rescueKick)
	{
		
		transition
		{
			if(state_time > 3000 || (state_time > 10 && action_done))
				goto patrolToHind;
		}
		
		action
		{
			HeadControlMode(HeadControl::lookForward);
			SpecialAction(SpecialActionRequest::kickfoot);
			//InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left),
			//			Pose2f(0, rBall.x() - 160.f, rBall.y() - 55.f));
		}
		
	}
	
	state(pass)
	{
		
		transition
		{
			if(state_time > 3000 || (state_time > 10 && action_done))
				goto patrolToHind;
		}
		
		action
		{
			HeadControlMode(HeadControl::lookForward);
			SpecialAction(SpecialActionRequest::kickfoot);
		}
		
  }

}
