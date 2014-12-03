unit RoCConsts;

{$mode objfpc}{$H+}

interface

const
  MaxRobots=4;
  RobotSpace=0.3;

  WheelToWheelDist=0.2318;
  WheelDiameter: double=0.076;

  FieldWidth=8;
  FieldLength=10;
  FieldOutsideSpace=1;

  type
  TPlay = (
    playHalt, playNormal, playSearch,playFormation
    );

const
  CPlayString: array[low(TPlay)..High(TPlay)] of string =
    (
    'Halt', 'Normal Play', 'Search Ball','Formation'
    );

var
  AxialDistance: double=0.2318;  // distancia entre as rodas
  SpeedMax: double=0.7;

  //---------------------------------------
  //Soccer Map - Change it with other maps
  //---------------------------------------
  GoalWidth: double;
  // this is actually Area Radius...
  AreaWidth: double;
  // calculated constants
  OurGoalX: double;
  OurAreaX: double;
  TheirGoalX: double;
  MaxFieldX: double;
  MaxFieldY: double;

implementation

end.

