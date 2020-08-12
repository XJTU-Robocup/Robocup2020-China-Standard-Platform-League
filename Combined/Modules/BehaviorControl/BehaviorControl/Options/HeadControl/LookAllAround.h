option(LookAllAround, (float)(0.38f) tilt, (float)(1.0f) pan)
{
     //const float m=theRobotPose.translation.x();
    //const float n=theRobotPose.translation.y();
  initial_state(lookAllRound)
  {

    transition
    {
		if(state_time>8000)
			goto stage1;
        //if(n<=0)
         //   goto lookLeft;
    }
    action
    {
      WalkToTarget(Pose2f(10.f, 10.f, 10.f), Pose2f(theLibCodeRelease.angleToGoal+180_deg, 0.f, 0.f));
    }
  }
  state(stage1)
  {

    transition
    {
		if(state_time>7500)
			goto stage2;
    }
    action
    {
      WalkToTarget(Pose2f(10.f, 10.f, 10.f), Pose2f(180_deg, 0.f, 0.f));
    }
  }
  state(stage2)
  {

    transition
    {
    }
    action
    {
      Stand();
    }
  }
}