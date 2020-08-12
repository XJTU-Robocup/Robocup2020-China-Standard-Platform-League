option(PlayingState)
{
  initial_state(demo)
  {
    action
    {		
		switch(theRobotInfo.number){
			case 1:
				M();
				//MoveTheBall();
				//Striker();
				//Try();
				break;
			case 2:
				CF();
				break;
			case 3:
				GK();
				break;
			case 4:
				CB();
				break;
			default:
				Striker();
		}
		
    }
  }
}
