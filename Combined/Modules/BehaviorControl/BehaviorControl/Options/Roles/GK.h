/*bool choosePosition(Vector2f guardposition)
{
	
}*/
option(GK)
{
	//std::vector<Obstacle> obs = theObstacleModel.obstacles;
	const Vector2f ball=theBallModel.estimate.position;	
	const Vector2f ball_absolute=Transformation::robotToField(theRobotPose,ball);
	const Vector2f ballv=theBallModel.estimate.velocity;
 	const Vector2f globalballv=theBallModel.estimate.velocity;
	const Vector2f myballv=theBallModel.estimate.velocity;
	const Vector2f myball=theBallModel.estimate.position;

	float robotx=theRobotPose.translation.x();
	float roboty=theRobotPose.translation.y();
	float v2=-200.f;
	float v1=-100.f;
	float v3=-50.f;
	float dangerline=-2500;
	float safeline=0;
	float kickline=theFieldDimensions.xPosOwnPenaltyArea+500;
	float guardliney=1800;
	
	initial_state(start)
  {
    transition
    {
      if(state_time>10000)
		  goto search;
	  if(theLibCodeRelease.timeSinceBallWasSeen < 300 )
	  {
		if(globalballv.x()<=v2)
			goto warningstate;
		if(globalballv.x()>v2)
			goto readystate;
		if(ball_absolute.x()>safeline)
			goto safenow;
		else
			goto readystate;
	  }
    }
    action
    {
      LookRoundFast();
	  //Stand();
    }
  }
  
  state(search)
  {
	  transition
	  {
		if(theLibCodeRelease.timeSinceBallWasSeen < 300 )
		{
			if(globalballv.x()<=v2)
				goto warningstate;
			if(ball_absolute.x()>safeline)
				goto safenow;
			else
				goto readystate;
		}
		if(state_time>5000)
			goto ownGoal1;
	  }
	  action
	  {
		  LookRoundFast();//待修改
	  }
  }
  
  state(safenow)
  {
	  transition
	  {
		if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut )
			goto search;
		if(globalballv.x()<=v2)
			goto warningstate;
		if(ball_absolute.x()<safeline && globalballv.x()<v3)
			goto readystate;
			
	  }
	  action
    {
		
		LookAtBall();
		WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(theBallModel.estimate.position.angle(), theFieldDimensions.xPosOwnGroundline+200-robotx,0-roboty));
		//theBallModel.estimate.position.angle()
    }
  }
  
  state(readystate)
  {
	  transition
	  {
		  if(globalballv.x()<=v2)//优先拦截快速球
			goto sitdown;
		  if((ball_absolute.x()<kickline&&std::abs(ball_absolute.y())<guardliney) || ball_absolute.x()<robotx)//禁区里，踢它！！
			goto kickball;
		  if((globalballv.x()>v2 && globalballv.x()<v3) || ball_absolute.x()<=dangerline)//
			goto warningstate;
		  if(ball_absolute.x()>safeline)//安全状态回退保障视野
			goto safenow;
		  if(theLibCodeRelease.timeSinceBallWasSeen > 3000)///丢球了！！！小心乌龙！！！
			goto search;
			
	  }
	  action
	  {
		  LookAtBall();
		  float yy=ball_absolute.y()*1100/4500-roboty;
		  WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(theLibCodeRelease.angleToGoal, theFieldDimensions.xPosOwnGroundline+200-robotx,yy));
		  //attention!!!!选位防守函数待加入
		  //WalkToTarget
	  }
  }
  
  state(warningstate)
  {
	  transition
	  {
		if(globalballv.x()<=v1 && ball_absolute.x()<=dangerline )
			goto sitdown;
		if(ball_absolute.x()>dangerline)
			goto readystate;
		if(ball_absolute.x()<kickline || ball_absolute.x()<robotx)//禁区里，踢它！！
			goto kickball;
		if(theLibCodeRelease.timeSinceBallWasSeen > 2000 || ball_absolute.x()<robotx)///丢球了！！！小心乌龙！！！
			goto ownGoal1;
		if(ball_absolute.x()>safeline)
			goto safenow;
		
	  }
	  action
	  {
		  LookAtBall();
		  float yy=ball_absolute.y()*1100/4500-roboty;
		  WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(theLibCodeRelease.angleToGoal, theFieldDimensions.xPosOwnGroundline+200-robotx,yy));
		  //attention!!!!选位防守函数待加入
		  //WalkToTarget
	  }
  }
  
  state(walkToSearch)
  {
	  transition
	  {
		if(theLibCodeRelease.timeSinceBallWasSeen<300 || std::abs(robotx-theFieldDimensions.xPosOwnPenaltyArea)<50.f)
			goto ownGoal1;
		if(ball_absolute.x()>safeline)
			goto safenow;
	  }
	  action
	  {
		  theHeadControlMode = HeadControl::lookForward;
		  WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f),Pose2f(theLibCodeRelease.angleToGoal,theFieldDimensions.xPosOwnPenaltyArea-robotx,0.f));
	  }
  }
  
  state(ownGoal1)
  {
	  transition
	  {
		if(theLibCodeRelease.timeSinceBallWasSeen<300)
		{
			if(ball_absolute.x()<=robotx||ball_absolute.x()<kickline)
				goto kickball;
			else
				goto readystate;
		}
		if(state_time>14500)
			goto walkToSearch;
			
	  }
	  action
	  {
		  LookAllAround();
	  }
  }
  
  state(sitdown)
  {
	  transition
	  {
		  if(state_time>3000)
			goto start;
	  }
	  action
	  {
		  //OUTPUT_TEXT(globalballv.x());
		  //OUTPUT_TEXT(ballv.x());
		  LookAtBall();
		  Stand();
		  //SpecialAction(SpecialActionRequest::sitDown);
	  }
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  state(kickball)
  {
	  transition
	  {
		if(ball_absolute.x()>safeline)
			goto safenow;
		if(ball_absolute.x()>kickline)
			goto warningstate;
		if(std::abs(ball_absolute.y())> guardliney)
			goto keepGuard;
		if(theLibCodeRelease.timeSinceBallWasSeen > 2000)
			goto walkToSearch;
		if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
			goto walkToBall;
	  }
	  action
	  {
		LookAtBall();
		WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
	  }
  }
  ////////////////////////////////////////////////////////////////////////////
  state(keepGuard)
  {
	  transition
	  {
		if(ball_absolute.x()>safeline)
			goto safenow;
		if(ball_absolute.x()>kickline)
			goto warningstate;
		if(std::abs(ball_absolute.y())< guardliney)
			goto kickball;
		if(theLibCodeRelease.timeSinceBallWasSeen > 2000)
			goto walkToSearch;
		if(ball_absolute.y()<0)
		{
			if(std::abs(theFieldDimensions.xPosOwnPenaltyArea-robotx) < 100.f && std::abs(-900.f-roboty) < 100.f)
				goto keepGuard2;
		}
		if(ball_absolute.y()>0)
		{
			if(std::abs(theFieldDimensions.xPosOwnPenaltyArea-robotx) < 100.f && std::abs(900.f-roboty) < 100.f)
				goto keepGuard2;
		}
	  }
	  action
	  {
		LookAtBall();
		if(ball_absolute.y()<0)
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(theLibCodeRelease.angleToGoal-30_deg, theFieldDimensions.xPosOwnPenaltyArea-robotx, -900.f-roboty));
		if(ball_absolute.y()>0)
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(theLibCodeRelease.angleToGoal+30_deg, theFieldDimensions.xPosOwnPenaltyArea-robotx, 900.f-roboty));
	  }
  }
  
  state(keepGuard2)
  {
	  transition
	  {
		if(ball_absolute.x()>safeline)
			goto safenow;
		if(ball_absolute.x()>kickline)
			goto warningstate;
		if(std::abs(ball_absolute.y())< guardliney)
			goto kickball;
		if(theLibCodeRelease.timeSinceBallWasSeen > 2000)
			goto walkToSearch;
		if(ball_absolute.y()<0)
		{
			if(std::abs(theFieldDimensions.xPosOwnPenaltyArea-400.f-robotx) < 50.f && std::abs(ball.angle())<10_deg)
				goto keepGuard3;
		}
		if(ball_absolute.y()>0)
		{
			if(std::abs(theFieldDimensions.xPosOwnPenaltyArea-400.f-robotx) < 50.f && std::abs(ball.angle())<10_deg)
				goto keepGuard3;
		}
	  }
	  action
	  {
		LookAtBall();
		if(ball_absolute.y()<0)
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(ball.angle(), theFieldDimensions.xPosOwnPenaltyArea-400.f-robotx, -900.f-roboty));
		if(ball_absolute.y()>0)
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(ball.angle(), theFieldDimensions.xPosOwnPenaltyArea-400.f-robotx, 900.f-roboty));
	  }
  }
  state(keepGuard3)
  {
	  transition
	  {
		if(ball_absolute.x()>safeline)
			goto safenow;
		if(ball_absolute.x()>kickline)
			goto warningstate;
		if(std::abs(ball_absolute.y())< guardliney)
			goto kickball;
		if(theLibCodeRelease.timeSinceBallWasSeen > 2000)
			goto walkToSearch;
	  }
	  action
	  {
		LookAtBall();
		Stand();
	  }
  }
  
  state(walkToBall)
  {
    transition
    {
	  if(ball_absolute.x()>kickline)
		goto warningstate;
	  if(std::abs(ball_absolute.y())> guardliney)
		goto keepGuard;
	  if(theLibCodeRelease.timeSinceBallWasSeen > 2000)
        goto walkToSearch;
      if(myball.norm() < 500.f)
        goto alignToGoal;
	  
    }
    action
    {
      LookAtBall();
      WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(theBallModel.estimate.position.angle(),theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
    }
  }
  
  
  state(alignToGoal)
  {
	  transition
	  {
		if(ball_absolute.x()>kickline)
			goto warningstate;
		if(std::abs(ball_absolute.y())> guardliney)
			goto keepGuard;
		if(theLibCodeRelease.timeSinceBallWasSeen > 2000)
			goto walkToSearch;
		if(ball_absolute.y()>=0)
		{
			if(std::abs(theLibCodeRelease.angleToGoal+50_deg) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
				goto alignBehindBall;
		}
		if(ball_absolute.y()<0)
		{
			if(std::abs(theLibCodeRelease.angleToGoal-50_deg) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
				goto alignBehindBall;
		}
		
	  }
	  action
	  {
		LookAtBall();
		if(ball_absolute.y()>=0)
			WalkToTarget(Pose2f(-0.5f, 0.5f, 0.5f), Pose2f(theLibCodeRelease.angleToGoal+50_deg, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
		if(ball_absolute.y()<0)
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(theLibCodeRelease.angleToGoal-50_deg, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
		//等待加入踢球角度选择函数
	  }
  }
  state(alignBehindBall)
  {
    transition
    {
		if(theLibCodeRelease.timeSinceBallWasSeen > 2000)
			goto walkToSearch;
		if(ball_absolute.x()>kickline)
			goto warningstate;
		if(std::abs(ball_absolute.y())> guardliney)
			goto keepGuard;
		if(ball_absolute.y()>=0)
		{
			if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 35.f, 55.f)
				&& theLibCodeRelease.between(theBallModel.estimate.position.x(), 155.f, 175.f)
				&& std::abs(theLibCodeRelease.angleToGoal+50_deg) < 5_deg)
			goto kick;
		}
		if(ball_absolute.y()<0)
		{
			if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 35.f, 55.f)
				&& theLibCodeRelease.between(theBallModel.estimate.position.x(), 155.f, 175.f)
				&& std::abs(theLibCodeRelease.angleToGoal-50_deg) < 5_deg)
			goto kick;
		}
    }
    action
    {
		LookAtBall();
		if(ball_absolute.y()>=0)
			WalkToTarget(Pose2f(-0.5f, 0.5f, 0.5f), Pose2f(theLibCodeRelease.angleToGoal+50_deg, theBallModel.estimate.position.x() - 165.f, theBallModel.estimate.position.y() - 42.f));
		if(ball_absolute.y()<0)
			WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(theLibCodeRelease.angleToGoal-50_deg, theBallModel.estimate.position.x() - 165.f, theBallModel.estimate.position.y() - 42.f));
		
		//WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));
    }
  }
  
  state(kick)
  {
	transition
	{
		if(state_time > 3000 || (state_time > 10 && action_done))
			goto start;
    }
    action
    {
		LookAtBall();
		InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 165.f, theBallModel.estimate.position.y() - 42.f));
    }
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
}