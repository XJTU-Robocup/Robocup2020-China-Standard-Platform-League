bool getFastShootTarget_GK(Vector2f& shootTarget){
    
    Vector2f pointGoal(-3900.f,0.f);
    const float disCircle = 2500;
    std::vector<std::pair<float,Vector2f> > candidates;
	std::pair<float,Vector2f> temp;
    segment segFromBallToGoal(fieldBall(),pointGoal);
    
    if(theRobotPose.translation.x() > fieldBall().x()){
        
        for(float theta = 60*pi/100; theta <= 120*pi/100 ; theta += 1*pi/100){
            pointGoal[0] = -3900.f + disCircle * sin(theta);   
            pointGoal[1] = 0.f + disCircle * cos(theta);  
            segFromBallToGoal.end[1]=pointGoal;
            if(minDisBetweenSegmentAndObstacle(segFromBallToGoal) > 2.5*ballRadius){
                Vector2f v1,v2;
                v1 = fieldBall()-pointGoal;
                v2 = theRobotPose.translation-fieldBall();
                temp.first = abs(1-(v1[0]*v2[0]+v1[1]*v2[1])/(v1.norm()*v2.norm()));
                temp.second = pointGoal;
                candidates.push_back(temp);
            }
        }       
    }
    else{

    for(float theta = 30*pi/100; theta <= 150*pi/100 ; theta += 1*pi/100){
        pointGoal[0] = -3900.f + disCircle * sin(theta);   
        pointGoal[1] = 0.f + disCircle * cos(theta);  
        segFromBallToGoal.end[1]=pointGoal;
        if(minDisBetweenSegmentAndObstacle(segFromBallToGoal) > 2.5*ballRadius){
            Vector2f v1,v2;
            v1 = fieldBall()-pointGoal;
            v2 = theRobotPose.translation-fieldBall();
            temp.first = abs(1-(v1[0]*v2[0]+v1[1]*v2[1])/(v1.norm()*v2.norm()));
            temp.second = pointGoal;
            candidates.push_back(temp);
        }
    }
    
    
    }
    if(!candidates.empty())
    shootTarget=minElement(candidates);
    
	if(candidates.empty()){
        if(theRobotPose.translation.x() > fieldBall().x()){
        
        for(float theta = 60*pi/100; theta <= 120*pi/100 ; theta += 1*pi/100){
            pointGoal[0] = -3900.f + disCircle * sin(theta);   
            pointGoal[1] = 0.f + disCircle * cos(theta);  
            segFromBallToGoal.end[1]=pointGoal;
            if(minDisBetweenSegmentAndObstacle(segFromBallToGoal) > 1.25*ballRadius){
                Vector2f v1,v2;
                v1 = fieldBall()-pointGoal;
                v2 = theRobotPose.translation-fieldBall();
                temp.first = abs(1-(v1[0]*v2[0]+v1[1]*v2[1])/(v1.norm()*v2.norm()));
                temp.second = pointGoal;
                candidates.push_back(temp);
            }
        }       
    }
    else{

    for(float theta = 30*pi/100; theta <= 150*pi/100 ; theta += 1*pi/100){
        pointGoal[0] = -3900.f + disCircle * sin(theta);   
        pointGoal[1] = 0.f + disCircle * cos(theta);  
        segFromBallToGoal.end[1]=pointGoal;
        if(minDisBetweenSegmentAndObstacle(segFromBallToGoal) > 1.25*ballRadius){
            Vector2f v1,v2;
            v1 = fieldBall()-pointGoal;
            v2 = theRobotPose.translation-fieldBall();
            temp.first = abs(1-(v1[0]*v2[0]+v1[1]*v2[1])/(v1.norm()*v2.norm()));
            temp.second = pointGoal;
            candidates.push_back(temp);
        }
    }
    
    
    }
    }
	return !candidates.empty();

}
