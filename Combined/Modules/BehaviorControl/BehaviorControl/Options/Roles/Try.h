option(Try){
	initial_state(start){
		transition
		{
			
		}
		action
		{
		  LookForward();
		  WalkAtRelativeSpeed(Pose2f(0.6f, 0.f, 0.f));
		}
	}
}