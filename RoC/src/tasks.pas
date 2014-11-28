unit Tasks;

{$mode objfpc}{$H+}

interface

uses Math, Controls, Main, Graphics, RoCConsts, Roles, Astar, Field, SysUtils;

type
  task_func=procedure(num:integer);

type
  TTask=(taskIdle, taskGoToGoodPosition,
         taskDoFormation,
         taskDoFormationFollower
         );

const
  CTaskString: array[low(TTask)..High(TTask)] of string =
        ('taskIdle','taskGoToGoodPosition',
         'taskDoFormation',
         'taskDoFormationFollower'
         );


procedure taskIdleRules(num: integer);
procedure taskGoToGoodPositionRules(num: integer);
procedure taskDoFormationRules(num: integer);
procedure taskDoFormationFollowerRules(num: integer);

const
  CTaskFuncs: array[Low(TTask)..High(TTask)] of task_func =
         (@taskIdleRules, @taskGoToGoodPositionRules,
         @taskDoFormationRules,
         @taskDoFormationFollowerRules
         );

type
  TTaskPars=record
    x1, y1: double;
    xold,yold: double;
    teta, speed,speed_on_target: double;
    sumError:double;
    speedV,speedVn:double;
    avoid: TAvoidSet;
    taskIter:integer;
    avoid_is_set: boolean;
  end;

var
  TaskPars: array[0..MaxRobots-1] of TTaskPars;
  speedontgt: double;


function TaskString(t: TTask): string;

implementation

uses Tatic, Actions, Utils, Param;//, MPC;



function TaskString(t: TTask): string;
begin
  result:=CTaskString[t];
end;

//----------------------------------------------------------------------
//  Task Rules

procedure taskIdleRules(num: integer);
begin
  RobotInfo[num].action:=acStop;
  ActionPars[num].w:=0;
  ActionPars[num].chipkick_dist:=0.0;
end;

Procedure taskGoToGoodPositionRules(num: integer);
var
 Otherteta:double;
begin
  RobotInfo[num].action:=acGoToXYTeta;
  //RobotInfo[num].action:=acControlTrajectory;   (MUDAR DEPOIS)
  with ActionPars[num] do begin
      x := taskPars[num].x1;
      y := taskPars[num].y1;
      teta := taskPars[num].teta;

      speed := SpeedMax;
      speed_on_target:=0.1;
      chipkick_dist:=0.0;

      if abs(x)>FieldDims.FieldDepth/2 then begin
         x:=sign(x)*FieldDims.FieldDepth/2;
      end;

      if abs(y)>FieldDims.FieldWidth/2 then begin
         y:=sign(y)*FieldDims.FieldWidth/2;
      end;

      taskPars[num].avoid_is_set:=true;
      avoid:=[avoidRobot,avoidObstacles];
  end;
end;

//------------------------------------------------------------------------
// Do Formation Rules
//------------------------------------------------------------------------

procedure taskDoFormationRules(num: integer);
var d: double;
    //obs: array[0..MaxRobots+MaxObstacles] of TRObstacle;  //Acke provisório o mais 70
    nobs, l: integer;
begin
    //---------------------------------------------------
     //              Obstacle List
     //---------------------------------------------------

    {  nobs := 0;

      if avoidObstacles in ActionPars[num].avoid then begin
        for l:=0 to Obstacles.count-1 do begin
          obs[nobs].x:=Obstacles.Centers[l].x;
          obs[nobs].y:=Obstacles.Centers[l].y;
          obs[nobs].r:=Obstacles.Centers[l].r + (RobotSpace/2);
          inc(nobs);
        end;
      end;

     //---------------------------------------------------
     //           End of Obstacle List
     //---------------------------------------------------

    d:=sqrt(power((RobotState[myNumber].x - BallState.x),2)+power((RobotState[myNumber].y - BallState.y),2));

    if (FormMPC.ObstacleInSegment(RobotState[num].x,RobotState[num].y,RobotState[num].x,RobotState[num].y+3,obs,nobs)=true)and(FormMPC.ObstacleInSegment(RobotState[num].x,RobotState[num].y,BallState.x,BallState.y,obs,nobs)=true) then begin
        flagForm:=true;
    end;

    with ActionPars[num] do begin

        if ((flagForm=True))and(d>strtofloatdef(FormMPC.EditP1_badi.Text,0)+0.8) and
           (FormMPC.ObstacleInSegment(RobotState[num].x,RobotState[num].y,BallState.x,BallState.y,obs,nobs)) then begin
                RobotInfo[num].action:=acGoToXYTeta;
                speed:=1;
                speed_on_target:=0.1;
                x:=BallState.x;
                y:=BallState.y;
                teta:=ATan2(BallState.y-RobotState[num].y,BallState.x-RobotState[num].x);
                taskPars[num].avoid_is_set:=true;
                avoid:=[avoidObstacles];
                if (FormMPC.ObstacleInSegment(RobotState[num].x,RobotState[num].y,BallState.x,BallState.y,obs,nobs)=False) then begin
                    flagForm:=false;
                end;
        end else begin
           RobotInfo[num].action:=acDoFormation;
           taskPars[num].avoid_is_set:=true;
           speed := TaskPars[num].speed;
           speed_on_target := staticSpeedOnTarget;
           avoid:=[avoidObstacles];

        end;
    end;  }
end;

procedure taskDoFormationFollowerRules(num: integer);
var meh: double;
begin

   { RobotInfo[num].action:=acDoFormationFollower;

    taskPars[num].avoid_is_set:=true;

    with ActionPars[num] do begin
        speed := TaskPars[num].speed;
        speed_on_target := staticSpeedOnTarget;
        avoid:=[avoidObstacles];
    end;  }
end;


end.

