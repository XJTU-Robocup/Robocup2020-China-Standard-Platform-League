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
				LM();
				break;
			case 3:
				CF();
				break;
			case 4:
				GK();
				break;
			case 5:
				CB();
				break;
			default:
				Striker();
		}
		
    }
  }
}
