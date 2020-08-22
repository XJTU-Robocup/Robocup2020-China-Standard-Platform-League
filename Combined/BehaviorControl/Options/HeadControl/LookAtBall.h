option(LookAtBall)
{
  initial_state(lookAtBall)
  {
    action
    {
		  /** Simply sets the necessary angles */
		Vector2f ball = theBallModel.estimate.position;
			//球当前的相对位置  
		//const double alfa=atan(g/f);
		double alfa=ball.angle();
			//球在相对坐标系中与机器人x方向的夹角（弧度值）
		//const float h =alfa*1.5/(3.14/2);
      SetHeadPanTilt((float) alfa , (float) 0.38f , 150_deg);
    }
  }
}
//=tilt 前后，+向下 ; pan 左右平动 +向左
//=？*1.5/(3.14/2)