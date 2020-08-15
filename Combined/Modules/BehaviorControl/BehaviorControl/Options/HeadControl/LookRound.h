option(LookRound, (float)(0.38f) tilt, (float)(1.0f) pan)
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
      SetHeadPanTilt(-pan, tilt, 45_deg);
    }
  }
  
    state(lookLeft)
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
      SetHeadPanTilt(pan, tilt, 45_deg);
    }
  }
  
}