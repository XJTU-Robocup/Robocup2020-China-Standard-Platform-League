option (MoveTheBall)
{

  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)
        goto walkToTarget;
    }
    action
    {
	 Stand();
     LookForward();
    }
  }
 state(walkToPoint)
 { 
   transition
   {
	 if(theLibCodeRelease.timeSinceBallWasSeen > 3000)
        goto searchTheBall;
     if(theLibCodeRelease.between(relativeBall().x(), -200.f, 200.f)
         && theLibCodeRelease.between(relativeBall().y(), -200.f, 200.f)
         && std::abs(_angleDribble(orientation)) < 10_deg 
		 )
        goto walkToTarget;
   }
   action
   {
	 LookForward();
     Pose2f direction;
     direction.rotation = _angleDribble(orientation);
	 direction.translation = relativeBall();
     WalkToTarget(Pose2f(60.f,60.f,60.f),direction);
   }
 }
 
 state(walkToTarget)
 {
   transition
   {
     if(theLibCodeRelease.timeSinceBallWasSeen > 3000)
         goto searchTheBall;
     if(!theLibCodeRelease.between(relativeBall().x(), -200.f, 200.f)
         || !theLibCodeRelease.between(relativeBall().y(), -200.f, 200.f))
         goto walkToPoint;
   }
   action
   { 
	 LookForward();
     WalkToTarget(Pose2f(60.f, 60.f, 60.f), relativeBall()); 
   }
 }
 
state(searchTheBall)
 {
   transition
   {
     if(theLibCodeRelease.timeSinceBallWasSeen <100)
         goto walkToPoint;
   }
   action
   { 
	 Stand();
     LookRound();
   }
 }
}