/** behavior for the ready state */
option(ReadyState)
{
	Vector2f kickOffPos;
	
	int pNumber=Global::getSettings().playerNumber;
	if(pNumber==1)
	{
		//My Keeper();
		kickOffPos=Transformation::fieldToRobot(theRobotPose,Vector2f(-4200.f,0.f));
	}
	else if(pNumber==3)
	{
		//MyStriker();
		kickOffPos=Transformation::fieldToRobot(theRobotPose,Vector2f(-950.f,0.f));
	}
	else if(pNumber==5)
	{
		//MyDefender();
		kickOffPos=Transformation::fieldToRobot(theRobotPose,Vector2f(-3200.f,-800.f));
	}
	else if(pNumber==2)
	{
		//MyDefender2();
		kickOffPos=Transformation::fieldToRobot(theRobotPose,Vector2f(-3200.f,800.f));
	}
	else if(pNumber==4)
	{
		//Supporter
		kickOffPos=Transformation::fieldToRobot(theRobotPose,Vector2f(-1550.f,-200.f));
	}
  initial_state(stand)
  {
	transition
	{
		if(action_done||state_time>3000)
		{
			goto turnToKickOffPos;
		}
	}
    action
    {
      LookForward();
	  Stand();
	  //WalkToTarget(Pose2f(50.f,50.f,50.f),Pose2f(libCodeRelease.angleToGoal,Transformation::fieldToRobot(theRobotPose,Vector2f(-theFieldDimensions.centerCircleRadius-200.f,0.f))));
    }
  }
  
  state(turnToKickOffPos)
  {
	  transition
	  {
		  if(kickOffPos.angle()<5_deg)
		  {
			  goto walkToKickOffPos;
		  }
	  }
	  action
	  {
		LookForward();
		  WalkToTarget(Pose2f(10.f,10.f,10.f),Pose2f(kickOffPos.angle(),0.f,0.f));
	  }
  }
  
  state(walkToKickOffPos)
  {
	  transition
	  {
		  if(kickOffPos.norm()<100.f)
		  {
			  goto turnToBall;
		  }
	  }
	  action
	  {
		LookForward();
			WalkToTarget(Pose2f(10.f,10.f,10.f),Pose2f(kickOffPos.angle(),kickOffPos));
	  }
  }
  
  state(turnToBall)
  {
	  transition
	  {
		  if(kickOffPos.norm()<20.f&&std::abs(Transformation::fieldToRobot(theRobotPose,Vector2f(0.f,0.f)).angle())<3_deg)
		  {
			  goto finish;
		  }
	  }
	  action
	  {
		LookForward();
		  WalkToTarget(Pose2f(10.f,10.f,10.f),Pose2f(Transformation::fieldToRobot(theRobotPose,Vector2f(0.f,0.f)).angle(),kickOffPos));
	  }
  }
  
  state(finish)
  {
	  action
	  {
		 // LookRound(1.f);
         LookForward();
		  Stand();
	  }
  }
}
