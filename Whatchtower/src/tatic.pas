unit Tatic;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, RoCConsts, Roles, Main, Math, Graphics, Field;

  type
  TRoundObstacle=record
    x,y,r: double;
    used: boolean;
  end;

type
  TRobotInfo=record
    last_role,role: TRole;
    roleTime: integer;
    taskTime: integer;
    actionTime: integer;
    action_complete: boolean;

    TimeToComplete: integer;  // estimated
  end;

type
  TRobotCalcData=record
    trajectory_length: double;
  end;

function WhoIs(role: TRole): integer;
procedure DoRules;
procedure DoTactic;
procedure DoRobotRules(num: integer);


var
  oldPlay: TPlay;
  iterChangeRoles:integer=0;
  RobotInfo: array [0..MaxRobots-1] of TRobotInfo;
  RobotCalcData: array[0..MaxRobots-1] of TRobotCalcData;
  Play, last_Play: TPlay;
  PlayChangeTimeStamp: LongWord;

implementation

uses Robots, Utils, Param;


function WhoIs(role: TRole): integer;
var i: integer;
begin
   for i:=0 to MaxRobots-1 do begin
    if (robotInfo[i].role=role)then begin
      result:=i;
      exit;
    end;
  end;
 result:=-1;
end;

procedure DoRobotRules(num: integer);
var i: integer;
begin
  with RobotInfo[num] do begin
    RoleDefs[role].func(num);
  end;
end;

procedure CalcRolePriority(out pri_role: array of TRole);
var i:integer;
    DefLineDef: double;
begin

  // default roles

   case Play of
    playHalt: begin
      for i:=0 to 3 do pri_role[i]:=roleIdle;
    end;

    playNormal: begin
      // use default roles
      //
      //
    end;

    playFormation: begin
      //Por condição pra mudar o role
      pri_role[0]:=roleDoFormation;
      pri_role[1]:=roleDoFormation;
      pri_role[2]:=roleDoFormation;
      pri_role[3]:=roleDoFormation;
    end;

  end;
end;

procedure DoRules;
var newRoles,RolePri: array[0..MaxRobots-1] of TRole;
    RobotAvailable: array[0..MaxRobots-1] of boolean;
    i,j,best,act_role, cur_robot, RobotAvailableCount: integer;
    val,best_value, cur_cost: double;
    id:integer;
begin

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

  FMain.EditRobotAvailableCount.Text:=IntToStr(RobotAvailableCount);

  // calculate role priority for current game situation
  CalcRolePriority(RolePri);

end;

procedure DoTactic;
var rob: integer;
    old_role: array[0..MaxRobots-1] of TRole;
begin

  for rob:=0 to MaxRobots-1 do begin
    old_role[rob]:=RobotInfo[rob].role;
  end;

  // select the right robots for each role
  DoRules;
end;



initialization
  begin
    RobotInfo[0].role:=roleIdle;
    RobotInfo[1].role:=roleIdle;
    RobotInfo[2].role:=roleIdle;
    RobotInfo[3].role:=roleIdle;
  end;

end.

