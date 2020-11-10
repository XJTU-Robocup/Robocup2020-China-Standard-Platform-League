option(LookRoundFast, (float)(0.3f) tilt, (float)(1.0f) pan)
{
     //const float m=theRobotPose.translation.x();
    //const float n=theRobotPose.translation.y();
  initial_state(lookRight)
  {

    transition
    {
      if(action_done)    
          goto lookLeft;
        //if(n<=0)
         //   goto lookLeft;
    }
    action
    {
      SetHeadPanTilt(-pan, tilt, 40_deg);
    }
  }
  
    state(lookLeft)
  {
    transition
    {
      if(action_done)    
          goto backToMiddle;
      //  if(n>0)
        //  goto lookRight;
    }
    action
    {
      SetHeadPanTilt(pan, tilt, 40_deg);
    }
  }
  state(backToMiddle)
  {
	  transition
    {
      if(action_done)    
          goto lookRight;
      //  if(n>0)
        //  goto lookRight;
    }
    action
    {
      SetHeadPanTilt(0.f, tilt, 45_deg);
    }
  }
  
}