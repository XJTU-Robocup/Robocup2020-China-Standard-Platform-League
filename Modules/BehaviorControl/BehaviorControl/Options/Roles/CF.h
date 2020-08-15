bool ifBallInArea(int idArea){
	switch (idArea){
		case 5:
			if(theLibCodeRelease.between(fieldBall().x(),1800,4500))
				return true;
			else
				return false;
			break;
		default:
			return false;
	}
};
Vector2f getPositionTarget(int idArea){
	Vector2f positionTarget(1800,0);
	switch(idArea){
		case 5:
			positionTarget[1]=0;
			return positionTarget;
			break;
	}
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
			if(theLibCodeRelease.timeSinceBallWasSeen < 300){
				if(ifBallInArea(5)){
					goto turnToBall;
				}
				else{
					goto position;
				}
			}
			if(theLibCodeRelease.timeSinceBallWasSeen > 3000)
				goto searchByHead;
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
			goto searchByHead;
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
				goto searchByHead;
			if(relativeBall().norm() < 500.f){
				goto alignToGoal;
			}
			if(!ifBallInArea(5)){
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
		  if(theLibCodeRelease.timeSinceBallWasSeen > 7000)
			goto searchByHead;
		  if(std::abs(target2Angle(kickTarget)) < 10_deg && std::abs(relativeBall().y()) < 100.f)
			goto alignBehindBall;
		
		  if(!ifBallInArea(5)){
				goto position;
		  }
		  if(relativeBall().norm() > 500.f){
			    goto turnToBall;
		  }
		}
		action
		{
		  getShootTarget(kickTarget);
		  LookForward();
		  WalkToTarget(Pose2f(0.8f, 0.8f, 0.8f), Pose2f(target2Angle(kickTarget), relativeBall().x() - 400.f, relativeBall().y()));
		}
	}
	state(alignBehindBall){
	  transition{
		if(theLibCodeRelease.timeSinceBallWasSeen > 3000)
			goto searchByHead;
		if(theLibCodeRelease.between(relativeBall().y(), 35.f, 55.f)
           && theLibCodeRelease.between(relativeBall().x(), 150.f, 180.f)
         && std::abs(target2Angle(kickTarget)) <= 2_deg)
			goto kick;
	    if(!ifBallInArea(5)){
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
	state(kick)
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
		
	state(position){
		transition{
		  if(theLibCodeRelease.timeSinceBallWasSeen < 300 && ifBallInArea(5)){
			goto turnToBall;
		  }
		  if(Transformation::fieldToRobot(theRobotPose,getPositionTarget(5)).norm()<500.f){
			goto positionSearch;
		  }
		}
		action{
			LookForward();
			theMotionRequest=thePathPlanner.plan(getPositionTarget(5),Pose2f(0.5f,0.5f,0.5f),true);
		}
	}
	state(positionSearch){
		transition{
		  if(theLibCodeRelease.timeSinceBallWasSeen < 300 && ifBallInArea(5)){
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
			  LookForward();
			  WalkAtRelativeSpeed(Pose2f(0.5f, 0.f, 0.f));
			}
		}
	}
	
	state(searchByHead)
	{
		transition
		{
		  if(theLibCodeRelease.timeSinceBallWasSeen < 300)
			goto start;
		  if(state_time>5000)
			goto searchByRotate;
		}
		action
		{
		  LookRound();
		  Stand();
		}
	}
	state(searchByRotate)
	{
		transition
		{
		  if(theLibCodeRelease.timeSinceBallWasSeen < 300)
			goto start;
		  if(state_time>10000){
			  numPatrolNow=(numPatrolNow+1)%numPointPatrol;
			  goto searchByPatrol;
		  }
		}
		action
		{
		  LookForward();
		  WalkAtRelativeSpeed(Pose2f(0.6f, 0.f, 0.f));
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
			goto searchByHead;
		}
		action
		{
		  LookRound();
		  theMotionRequest=thePathPlanner.plan(pointPatrol[numPatrolNow],Pose2f(0.6f,0.6f,0.6f),true);
		}
	}
}