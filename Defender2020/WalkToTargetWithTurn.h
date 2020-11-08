/*
 * Dependencies: LookAtBall(2020), LookForBall(2020), LookForward, Stand, WalkToTarget
 * Important pre-arguments(args from robots itself): theBallModel(Ball position) fieldCognition(Knowing where it is in field)
 * Notes: The Module was written to solve that defender1(2020) walk to defendePoint slowly because of facing to other places
 * 
 * @param absPos: Absolute position in the field to be input.
 * @param WalkingSpeed: The speed(Pose2f(0.8f,0.8f,0.8f) by default) when walking to the target after facing it. THE SPEED CAN NOT BE ZERO IN ALL ARGUMENTS!!!
 * @param TurningSpeed: The speed(Pose2f(0.8f,0.8f,0.8f) by default) when turning to the target. THE SPEED CAN NOT BE ZERO IN ALL ARGUMENTS!!!
 * 
 * @Xavier This function are based on defendBall NOT DEFENDOPPONET, so dont use it for defendOpponet.
 * Coded by StevenKKXS
 */
option(WalkToTargetWithTurn, (Vector2f) absPos, (const Pose2f&)(Pose2f(0.8f,0.8f,0.8f)) WalkingSpeed, (const Pose2f&)(Pose2f(0.8f,0.8f,0.8f)) TurningSpeed)
{
	//errors accepted
	const float eps = 200;
	const float angleEps = 12_deg;
	const Vector2f relPos = Transformation::fieldToRobot(theRobotPose,absPos);
	
	initial_state(start)
	{
		transition
		{
			if((absPos - theRobotPose.translation).norm() < eps)
				goto requestCompleted;
			else
				goto turning;
		}
		action
		{
			LookForward();
			Stand();
		}
	}
	
	state(walking)
	{
		transition
		{
			if(std::abs(relPos.angle()) >= angleEps)
				goto turning;
			if((absPos - theRobotPose.translation).norm() < eps)
				goto requestCompleted;
		}
		action
		{
			WalkToTarget(WalkingSpeed,relPos);
			LookAtBall();
		}
	}
	
	state(turning)
	{
		
		transition
		{
			if(std::abs(relPos.angle()) < angleEps)
				goto walking;
		}
		action
		{
			WalkToTarget(TurningSpeed,Pose2f(relPos.angle(),0.f,0.f));
			LookAtBall();
		}
	}
	
	state(requestCompleted)
	{
		transition
		{
			//if the location is wrong at first and now the location is right
			if((absPos - theRobotPose.translation).norm() >= eps)
				goto start;
		}
		action
		{
			if(theLibCodeRelease.timeSinceBallWasSeen < 3000 && std::abs(theBallModel.estimate.position.angle()) >= 10_deg)
			{
				//theBallModel.estimate.position is not EMPTY anymore
				//Try to face to Ball when in the position
				WalkToTarget(TurningSpeed,Pose2f(theBallModel.estimate.position.angle(),0.f,0.f));
				LookAtBall();
			}
			else if(theLibCodeRelease.timeSinceBallWasSeen >= 3000)
			{
				LookForBall();
			}
			else
			{
				Stand();
				LookAtBall();
			}
		}
	}
	
}
