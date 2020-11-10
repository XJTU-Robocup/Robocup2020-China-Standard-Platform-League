/** This option lets the robot look for the ball. */
option(LookForBall_GK, (float)(0.38f) tilt, (float)(0.5f) pan)
{
  initial_state(lookRight)
  {
    transition
    {
       if(action_done)    
          goto lookLeft;
    }
    action
    {
      Stand();
      SetHeadPanTilt(-pan, tilt, 30_deg);
      
    }
  }
  
   state(lookLeft)
  {
    transition
    {
        
      if(action_done){
          HeadControlMode(HeadControl::lookForward);
          goto moveBody;
      }    
          
    }
    action
    {
      SetHeadPanTilt(pan, tilt, 30_deg);
    }
  }/** This option lets the robot look for the ball. */

  
  state(moveBody){
      transition{
              
          if(state_time > 2500) 
              goto lookRight;
      }
      action{
          
          HeadControlMode(HeadControl::lookForward);
          WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
          
      }
  }

}