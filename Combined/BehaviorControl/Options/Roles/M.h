/** A test striker option without common decision */
//std::vector<Obstacle> o = theObstacleModel.obstacles;  
//std::vector<Teammate> t = theTeamData.teammates;
option(M)
{
  initial_state(start)
  {
    transition
    {
       if(state_time > 3000)
           goto stateDecision;
    }
    action
    {
       LookRound();
    }
  }
  
  state(restart)
  {
    transition
    {
       
       if(state_time > 5000)
           goto stateDecision;
    }
    action
    {
        LookRound();
        Stand();
    }
  }
  
  state(stateDecision)
  {
    transition
    {
      if(globalRobot().x()>1800.f || globalRobot().x()<-1800.f
      || globalRobot().y()>100.f || globalRobot().y()<-3000.f)
          goto walkToTarget;
      if(globalBall().x()>1800.f || globalBall().x()<-1800.f
      || globalBall().y()>100.f || globalBall().y()<-3000.f)
          goto walkToTarget;
      if(theLibCodeRelease.timeSinceBallWasSeen > 4000.f)
          goto searchByHead;
      if(globalBall().x()<=1800.f && globalBall().x()>=-1800.f
      && globalBall().y()<=100.f && globalBall().y()>=-3000.f
      && theBallModel.estimate.position.norm()>450.f)
          goto walkToBall;
      if(globalBall().x()<=1800.f && globalBall().x()>=-1800.f
      && globalBall().y()<=100.f && globalBall().y()>=-3000.f
      && theBallModel.estimate.position.norm()<=450.f)
          goto alignToGoal;
    }
    action
    {
      LookForward();
      Stand();
    }
  }
  
  state(walkToBall)
  {
    transition
    {
      if(modeDecision2()!=2)
        goto stateDecision;
      if(theRobotPose,theBallModel.estimate.position.norm() < 500.f)
        goto alignToGoal;
    }
    action
    {
      Pose2f target;
      target.rotation=theBallModel.estimate.position.angle();
      target.translation=Transformation::robotToField(theRobotPose,theBallModel.estimate.position);
      theMotionRequest=thePathPlanner.plan(target,Pose2f(0.8f,0.8f,0.8f),false);
    }
  }
  
  state(walkToTarget)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
          goto walkToBall;
      Vector2f shootTarget = Transformation::fieldToRobot(theRobotPose, Vector2f(0.f, -1500.f));
      if(theLibCodeRelease.between(shootTarget.x(), -40.f, 40.f)
      && theLibCodeRelease.between(shootTarget.y(), -40.f, 40.f))
          goto waitForTarget;
    }
    action
    {
      LookRound();
      Pose2f target;
      target.rotation=theBallModel.estimate.position.angle();
      target.translation=Vector2f(0.f, -1500.f);
      theMotionRequest=thePathPlanner.plan(target,Pose2f(0.8f,0.8f,0.8f),false);
    }
  }
  
  state(waitForTarget)
  {
      transition
      {
        if(theLibCodeRelease.timeSinceBallWasSeen > 4000.f)
          goto searchByHead;
        if(modeDecision2()!=3)
          goto stateDecision;
      }
      action
      {
        HeadControlMode(HeadControl::lookForward);
        WalkToTarget(Pose2f(0.8f, 0.8f, 0.8f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
      }
  }
  
  state(alignToGoal)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > 4000.f)
        goto stateDecision;
      if(std::abs(theLibCodeRelease.angleToGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
        goto alignBehindBall;        
      if(theRobotPose,theBallModel.estimate.position.norm()>1000.f)
        goto stateDecision;
      if(globalBall().x()>1800.f || globalBall().x()<-1800.f
      || globalBall().y()>100.f || globalBall().y()<-3000.f)
        goto stateDecision;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      WalkToTarget(Pose2f(0.8f, 0.8f, 0.8f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
    }
  }

  state(alignBehindBall)
  {
    transition
    {
      if(modeDecision2()!=1)
        goto stateDecision;
      Vector2f target = Vector2f(0.f, 0.f);
      if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
         && std::abs(theLibCodeRelease.angleToGoal) < 2_deg
         && (getShootTarget2(target)))
        goto walkToShootPoint;
      /*if(theLibCodeRelease.between(posBall().y(), 20.f, 50.f)
         && theLibCodeRelease.between(posBall().x(), 140.f, 170.f)
         && std::abs(theLibCodeRelease.angleToGoal) < 2_deg
         && (passDecision()))
        goto walkToPassPoint;
      if(theLibCodeRelease.between(posBall().y(), 20.f, 50.f)
         && theLibCodeRelease.between(posBall().x(), 140.f, 170.f)
         && std::abs(theLibCodeRelease.angleToGoal) < 2_deg
         && (getKickTarget(target)==0))
        goto kick;*/
      goto walkToKickPoint;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(0.8f, 0.8f, 0.8f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 155.f, theBallModel.estimate.position.y()-42.f));
    }
  }
  
  state(walkToShootPoint)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > 4000.f)
        goto stateDecision;
      if(theLibCodeRelease.between(theBallModel.estimate.position.y(), -160.f, 160.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), -160.f, 160.f))
        goto shoot;
    }
    action
    {
      Vector2f target = Vector2f(0.f, 0.f);
      getShootTarget2(target);
      Vector2f kickTarget = Transformation::fieldToRobot(theRobotPose, target);
      WalkToTarget(Pose2f(0.8f, 0.8f, 0.8f), Pose2f(kickTarget.angle(), theBallModel.estimate.position.x() - 155.f, theBallModel.estimate.position.y() - 42.f));
    }
  }
  
  /*state(walkToPassPoint)
  {
    transition
    {
      if(modeDecision2()!=1)
        goto stateDecision;
      if(theLibCodeRelease.between(theBallModel.estimate.position.y(), -160.f, 160.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), -160.f, 160.f))
        goto pass;
    }
    action
    {
      Vector2f target = Vector2f(4500.f, 0.f);
      Vector2f kickTarget = Transformation::fieldToRobot(theRobotPose, target);
      WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(kickTarget.angle(), theBallModel.estimate.position.x() - 155.f, theBallModel.estimate.position.y() - 42.f));
    }
  }*/
  
  state(walkToKickPoint)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > 4000.f)
        goto stateDecision;
      if(theLibCodeRelease.between(theBallModel.estimate.position.y(), -160.f, 160.f)
         && theLibCodeRelease.between(theBallModel.estimate.position.x(), -160.f, 160.f))
        goto kick;
    }
    action
    {
      Vector2f target = Vector2f(4500.f, 0.f);
      getKickAngle2(target);
	  OUTPUT_TEXT(target);
      Vector2f kickTarget = Transformation::fieldToRobot(theRobotPose, target);
      WalkToTarget(Pose2f(0.8f, 0.8f, 0.8f), Pose2f(kickTarget.angle(), theBallModel.estimate.position.x() - 155.f, theBallModel.estimate.position.y() - 42.f));
    }
  }
  
  state(shoot) 
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto restart;
    }
    action
    {    
      HeadControlMode(HeadControl::lookForward);
      SpecialAction(SpecialActionRequest::kickfoot);
      //InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(shootTarget.angle(), theBallModel.estimate.position.x() - 145.f, theBallModel.estimate.position.y() - 40.f));
    }
  }
  
  /*state(pass)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto stateDecision;
    }
    action
    {
      Vector2f target = Vector2f(0.f, 0.f);
      getKickTarget(target);
      Vector2f shootTarget = Vector2f(target.x()-posBall().x(), target.y()-posBall().y());
      HeadControlMode(HeadControl::lookForward);
      InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(target.angle(), posBall().x() - 160.f, posBall().y() - 55.f));
    }
  }*/
  
  state(kick) 
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto restart;
    }
    action
    {
      HeadControlMode(HeadControl::lookForward);
      SpecialAction(SpecialActionRequest::kickfoot);
      //InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(shootTarget.angle(), theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y()-55.f));
    }
  }
  
  
  
  state(searchByHead)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
        goto waitForTarget;
	  if(state_time>5000)
		goto searchByRotate;
    }
    action
    {
      LookRound();
      Stand();
    }
  }
  
  state(searchByRotate){
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
        goto stateDecision;
	  if(state_time>10000){
		  numPatrolNow=(numPatrolNow+1)%numPointPatrol;
		  goto searchByPatrol;
	  }
    }
    action
    {
	  HeadControlMode(HeadControl::lookForward);
      WalkAtRelativeSpeed(Pose2f(0.5f, 0.f, 0.f));
    }
  }
  
  state(searchByPatrol){    
    transition
    {
      if(modeDecision2()!=0)
          goto stateDecision;
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
        goto stateDecision;
	  if((Transformation::fieldToRobot(theRobotPose,pointPatrol[numPatrolNow]).norm()<200.f))
		goto searchByHead;
    }
    action
    {
	  LookRound();
      theMotionRequest=thePathPlanner.plan(pointPatrol[numPatrolNow],Pose2f(0.3f,0.3f,0.3f),true);
    }
  }
}