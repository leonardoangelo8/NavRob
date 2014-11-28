unit Tatic;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, RoCConsts, Roles, Main, Actions,
  Math, Tasks, Graphics, Field;

  type
  TRoundObstacle=record
    x,y,r: double;
    used: boolean;
  end;

type
  TRobotInfo=record
    last_role,role: TRole;
    roleTime: integer;
    last_task,task: TTask;
    taskTime: integer;
    last_action,action: TAction;
    actionTime: integer;
    action_complete: boolean;

    TimeToComplete: integer;  // estimated
  end;

type
  TRobotCalcData=record
    trajectory_length: double;
  end;

  procedure DoRules;
  procedure DoRobotRules(num: integer);


  var
    iterChangeRoles:integer=0;
    RobotInfo: array [0..MaxRobots-1] of TRobotInfo;
    RobotCalcData: array[0..MaxRobots-1] of TRobotCalcData;

  implementation

  uses Robots, Astar,Utils, Param;

procedure DoRobotRules(num: integer);
  var old_task: TTask;
      old_action: TAction;
      i: integer;
  begin
    with RobotInfo[num] do begin

      old_task:=task;

      // particular role rules
      RoleDefs[role].func(num);

      if old_task<>task then begin
        action_complete:=false;
        taskTime:=ControlTimeStamp;
        last_task:=old_task;
        ZeroMemory(@ActionPars[num],sizeof(ActionPars[num]));
      end;

      old_action:=action;

      // particular task rules
      CTaskFuncs[task](num);

      if old_action<>action then begin
        actionTime:=ControlTimeStamp;
        action_complete:=false;
        last_action:=old_action;
      end;

      // perform actions
      CActionFunc[action](num);
    end;
  end;

  // ----------------------------------------------------------------------
  //     Role decision
  // ----------------------------------------------------------------------


function EvaluateRobotRole(num: integer; role: TRole): double;
  var dcost,turncost,tcost,cpr: double;
      backRInfo: TRobotInfo;
      backTPars: TTaskPars;
      backAPars: TActionPars;
      backRState: TRobotState;
  begin
    result := 1e6;

    // backup current state
    backRInfo:=RobotInfo[num];
    backTPars:=TaskPars[num];
    backAPars:=ActionPars[num];
    backRState:=RobotState[num];

    // give the Robot the role to test
    RobotInfo[num].role:=role;

    // act as if the robot had that role
    DoRobotRules(num);

    // distance cost
    dcost:=RobotCalcData[num].trajectory_length;
    // "cost per radian": one radian costs as much as 5 cm
    cpr:=0.05;  // parameter
    turncost:=Abs(DiffAngle(RobotState[num].teta,ActionPars[num].teta))*cpr;
    // total cost is the cost of going + turning
    tcost:=dcost+turncost;
    result:=tcost;

    // restore the original state
    RobotInfo[num]:=backRInfo;
    TaskPars[num]:=backTPars;
    ActionPars[num]:=backAPars;
    RobotState[num]:=backRState;

    // give a bonus to keep the current roles
    if RobotInfo[num].role=role then begin
      result:=result-0.12;
    end;
  end;

  procedure DoRules;
  var newRoles,RolePri: array[0..MaxRobots-1] of TRole;
      RobotAvailable: array[0..MaxRobots-1] of boolean;
      i,RobotAvailableCount: integer;
  begin
    // clear avoid settings to draw alternative trajectories
    for i:=0 to MaxRobots-1 do begin
      avoided[i]:=false;
    end;

    // set available robots
    RobotAvailableCount := 0;
    for i:=0 to MaxRobots-1 do begin
      if not RobotStatus[i].active then begin
        RobotAvailable[i]:=false;
        newRoles[i]:=RobotInfo[i].role;
      end else begin
        RobotAvailable[i]:=true;
        Inc(RobotAvailableCount);
      end;
    end;


    // calculate role priority for current game situation
    //CalcRolePriority(RolePri);

    // the Keeper have the same task
    if RobotAvailableCount>=1 then begin
      RobotInfo[0].role:=RolePri[4];
    end;

  end;

procedure DoTactic;
  var rob: integer;
      old_role: array[0..MaxRobots-1] of TRole;
  begin


    // keep old state to see state changes
    for rob:=0 to MaxRobots-1 do begin
      old_role[rob]:=RobotInfo[rob].role;
    end;

    // select the right robots for each role
    DoRules;

    for rob:=0 to MaxRobots-1 do begin
      with RobotInfo[rob] do begin
        if old_role[rob]<>role then begin
          roleTime:=ControlTimeStamp;
          last_role:=old_role[rob];
          ZeroMemory(@TaskPars[rob], sizeof(TaskPars[0]));
        end;
      end;
    end;

end;



  initialization
  begin
    RobotInfo[0].role:=roleIdle;
    RobotInfo[1].role:=roleIdle;
    RobotInfo[2].role:=roleIdle;
    RobotInfo[3].role:=roleIdle;
  end;

end.

