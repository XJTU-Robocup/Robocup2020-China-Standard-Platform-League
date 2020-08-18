option(PlayingState)
{
  initial_state(demo)
  {
    action
    {	
		
		switch(theRobotInfo.number){
			case 1:
				M();//右中场
				break;
			case 2:
				LM();//左中场
				break;
			case 3:
				CF();//中锋
				break;
			case 4:
				GK();//门将
				break;
			case 5:
				CB();//后卫
				break;
		}
		
		/*
		CF();
		*/

    }
  }
}
