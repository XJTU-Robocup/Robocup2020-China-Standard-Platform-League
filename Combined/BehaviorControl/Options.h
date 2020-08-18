/** All option files that belong to the current behavior have to be included by this file. */

#include "Options/Soccer.h"

#include "Options/GameControl/HandleGameState.h"
#include "Options/GameControl/HandlePenaltyState.h"
#include "Options/GameControl/PlayingState.h"
#include "Options/GameControl/ReadyState.h"

#include "Options/HeadControl/LookForward.h"

#include "Options/Output/Activity.h"
#include "Options/Output/Annotation.h"
#include "Options/Output/HeadControlMode.h"
#include "Options/Output/HeadMotionRequest/SetHeadPanTilt.h"
#include "Options/Output/HeadMotionRequest/SetHeadTarget.h"
#include "Options/Output/HeadMotionRequest/SetHeadTargetOnGround.h"
#include "Options/Output/MotionRequest/InWalkKick.h"
#include "Options/Output/MotionRequest/SpecialAction.h"
#include "Options/Output/MotionRequest/Stand.h"
#include "Options/Output/MotionRequest/WalkAtAbsoluteSpeed.h"
#include "Options/Output/MotionRequest/WalkAtRelativeSpeed.h"
#include "Options/Output/MotionRequest/WalkToTarget.h"
#include "Options/Output/MotionRequest/GetUpEngine.h"
#include "Options/Output/PlaySound.h"

#include "Options/Roles/Striker.h"
#include "Options/Roles/MoveTheBall.h"
#include "Options/Roles/CF.h"
#include "Options/Roles/CB.h"
#include "Options/Roles/GK.h"
#include "Options/Roles/Try.h"
#include "Options/Roles/M.h"
#include "Options/Roles/LM.h"

#include "Options/Skills/GetUp.h"

#include "Options/Tools/ButtonPressedAndReleased.h"
#include "Options/Tools/USBCheck.h"

#include "Options/HeadControl/LookRound.h"
#include "Options/HeadControl/LookAtBall.h"
#include "Options/HeadControl/LookRoundFast.h"
#include "Options/HeadControl/LookAllAround.h"
#include "Options/HeadControl/LookForBall.h"

#include "Options/Functions.h"
#include "Options/Functions_B.h"