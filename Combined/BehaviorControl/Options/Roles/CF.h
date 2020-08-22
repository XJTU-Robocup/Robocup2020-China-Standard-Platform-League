bool ifBallInArea_CF(){
	if(theLibCodeRelease.between(fieldBall().x(),1800,4500))
		return true;
	else
		return false;

};
Vector2f getPositionTarget_CF(){
	return Vector2f(1800,0);
}
bool ifSideKick_CF(){
	Vector2f posObstacle;
	for(const auto& obstacle : theObstacleModel.obstacles){
		posObstacle=obstacle.center;
		if(theLibCodeRelease.between(posObstacle.x(),0,500) && theLibCodeRelease.between(posObstacle.y(),-500,500))
			return true;
	}
	return false;
}

option(CF)
{
	initial_state(BeiDou)
	{
		transition{
		    if(state_time>2000)
				goto start;
		}
		action{
			LookRound();
		}
	}
	state(start)
	{
		transition{
			if(theLibCodeRelease.timeSinceBallWasSeen < 3000){
				if(ifBallInArea_CF()){
					goto turnToBall;
				}
				else{
					goto position;
				}
			}
			if(theLibCodeRelease.timeSinceBallWasSeen > 3000)
				goto search;
		}
		action{
			LookForward();
		}
	}
	state(turnToBall)
	{
		transition
		{
		  if(theLibCodeRelease.timeSinceBallWasSeen > 3000)
			goto search;
		  if(std::abs(relativeBall().angle()) < 5_deg)
			goto walkToBall;
		}
		action
		{
		  LookAtBall();
		  WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(relativeBall().angle(), 0.f, 0.f));
		}
	}
	state(walkToBall)
	{
		transition
		{
			if(theLibCodeRelease.timeSinceBallWasSeen > 3000)
				goto search;
			if(relativeBall().norm() < 500.f){
				goto alignToGoal;
			}
			
			if(!ifBallInArea_CF()){
				goto position;
			}
		}
		action
		{ 
		  LookForward();
		  theMotionRequest=thePathPlanner.plan(fieldBall(),Pose2f(0.5f,0.5f,0.5f),true);
		}
	}
    state(alignToGoal)
	{
		transition
		{
			if(ifSideKick_CF()){
				goto alignSideKickLeft;
			}
			
		  if(theLibCodeRelease.timeSinceBallWasSeen > 7000)
			goto search;
		  if(std::abs(target2Angle(kickTarget)) < 10_deg && std::abs(relativeBall().y()) < 100.f)
			goto alignBehindBall;
		
		  if(!ifBallInArea_CF()){
				goto position;
		  }
		  if(relativeBall().norm() > 500.f){
			    goto turnToBall;
		  }
		}
		action
		{
		  getFastShootTarget_CF(kickTarget);
		  LookForward();
		  WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(target2Angle(kickTarget), relativeBall().x() - 400.f, relativeBall().y()));
		}
	}
	state(alignBehindBall){
	  transition{
		if(theLibCodeRelease.timeSinceBallWasSeen > 3000)
			goto search;
		if(theLibCodeRelease.between(relativeBall().y(), 35.f, 55.f)
           && theLibCodeRelease.between(relativeBall().x(), 150.f, 180.f)
         && std::abs(target2Angle(kickTarget)) <= 2_deg)
	     {
			 if(Transformation::fieldToRobot(theRobotPose,kickTarget).norm()<2000)
			{
				 goto shortkick;
			 }
			 else{
				 goto longkick;
			 }
				
		 }
			
	    if(!ifBallInArea_CF()){
			goto position;
	    }
		if(relativeBall().norm() > 500.f){
			goto turnToBall;
		}
	  }
	  action{
		  LookAtBall();
		  //OUTPUT_TEXT(kickTarget);
		  //OUTPUT_TEXT(std::abs(target2Angle(kickTarget)));
		  //OUTPUT_TEXT(relativeBall());
		  WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(target2Angle(kickTarget), relativeBall().x() - 165.f, relativeBall().y() - 42.f)); 
	  }
	}
	state(shortkick)
	{
		transition
		{
		  if(state_time > 3000 || (state_time > 10 && action_done))
			goto start;
		}
		action
		{
		  //printPoint(kickTarget,"kick");
		  LookAtBall();
		  InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 165.f, theBallModel.estimate.position.y() - 42.f));
		}
	}
	state(longkick)
	{
		transition
		{
		  if(state_time > 3000 || (state_time > 10 && action_done))
			goto start;
		}
		action
		{
		  //printPoint(kickTarget,"kick");
		  LookAtBall();
		  SpecialAction(SpecialActionRequest::kickfoot);
		}
	}
	
	state(alignSideKickLeft)
    {
		transition
		{
		  if(theLibCodeRelease.timeSinceBallWasSeen > 3000.f)
			goto search;
		  if(theLibCodeRelease.between(relativeBall().y(), -30.f, 0.f)
			 && theLibCodeRelease.between(relativeBall().x(), 160.f, 190.f))
			goto sideKickLeft;
			
			if(!ifBallInArea_CF()){
				goto position;
			}
			if(relativeBall().norm() > 500.f){
				goto turnToBall;
			}
		}
		action
		{
		  LookAtBall();
		  WalkToTarget(Pose2f(0.5, 0.5, 0.5), Pose2f(relativeBall().angle(), relativeBall().x() - 180.f, relativeBall().y() + 30.f));
		}
    }
  
  state(sideKickLeft)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > 3000.f)
        goto search;
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;
    }
    action
    {
      LookAtBall();
      InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::left), Pose2f(relativeBall().angle(), relativeBall().x()-100.f , relativeBall().y() - 80.f));
    }
  }
  
  state(alignSideKickRight)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > 3000.f)
        goto search;
      if(theLibCodeRelease.between(relativeBall().y(), 0.f, 30.f)
         && theLibCodeRelease.between(relativeBall().x(), 160.f, 190.f))
        goto sideKickRight;
		
		if(!ifBallInArea_CF()){
			goto position;
		}
		if(relativeBall().norm() > 500.f){
			goto turnToBall;
		}
    }
    action
    {
      LookAtBall();
      WalkToTarget(Pose2f(20.f, 20.f, 20.f), Pose2f(relativeBall().angle(),relativeBall().x() - 180.f, relativeBall().y() - 30.f));
    }
  }

  state(sideKickRight)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > 3000.f)
        goto search;
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;
    }
    action
    {
      LookAtBall();
      InWalkKick(WalkKickVariant(WalkKicks::sidewardsInner, Legs::right), Pose2f(relativeBall().angle(), relativeBall().x()-10.f , relativeBall().y() + 80.f));
    }
  }
	
	state(position){
		transition{
		  if(theLibCodeRelease.timeSinceBallWasSeen < 300 && ifBallInArea_CF()){
			goto turnToBall;
		  }
		  if(Transformation::fieldToRobot(theRobotPose,getPositionTarget_CF()).norm()<500.f){
			goto positionSearch;
		  }
		}
		action{
			LookForward();
			theMotionRequest=thePathPlanner.plan(getPositionTarget_CF(),Pose2f(0.5f,0.5f,0.5f),true);
		}
	}
	state(positionSearch){
		transition{
		  if(theLibCodeRelease.timeSinceBallWasSeen < 300 && ifBallInArea_CF()){
			goto turnToBall;
		  }
		  if(theLibCodeRelease.timeSinceBallWasSeen > 3000 && state_time>10000){
			  goto searchByPatrol;
		  }
		}
		action{
			if(theLibCodeRelease.timeSinceBallWasSeen < 1000 && std::abs(relativeBall().angle()) < 5_deg){
			  Stand();
			}
			else{
				LookForBall();
			}
		}
	}
	
	state(search)
	{
		transition
		{
		  if(theLibCodeRelease.timeSinceBallWasSeen < 300)
			goto start;
		  if(state_time>10000)
			goto searchByPatrol;
		}
		action
		{
		  LookForBall();
		}
	}
	state(searchByPatrol)
	{
		transition
		{
		  if(theLibCodeRelease.timeSinceBallWasSeen < 300)
			goto start;
		  if((Transformation::fieldToRobot(theRobotPose,pointPatrol[numPatrolNow]).norm()<500.f))
			numPatrolNow=(numPatrolNow+1)%numPointPatrol;
			goto search;
		}
		action
		{
		  LookRound();
		  theMotionRequest=thePathPlanner.plan(pointPatrol[numPatrolNow],Pose2f(0.6f,0.6f,0.6f),true);
		}
	}
}