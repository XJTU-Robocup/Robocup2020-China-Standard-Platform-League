option(Try){
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
				goto turnToBall;
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
			if(relativeBall().
			norm() < 600.f){
				goto alignToGoal;
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
		
		  if(relativeBall().norm() > 700.f){
			    goto turnToBall;
		  }
		}
		action
		{
		  kickTarget=Vector2f(4500,0);
		  LookForward();
		  WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(target2Angle(kickTarget), relativeBall().x() - 400.f, relativeBall().y()));
		}
	}
	state(alignBehindBall){
	  transition{
		if(theLibCodeRelease.timeSinceBallWasSeen > 3000)
			goto searchByHead;
		if(theLibCodeRelease.between(relativeBall().y(), 35.f, 55.f)
           && theLibCodeRelease.between(relativeBall().x(), 150.f, 180.f)
         && std::abs(target2Angle(kickTarget)) <= 2_deg)
	     {
			goto kick;
		 }
			
		if(relativeBall().norm() > 700.f){
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
		}
		action
		{
		  LookForward();
		  WalkAtRelativeSpeed(Pose2f(0.5f, 0.f, 0.f));
		}
	}
}