unit Roles;

{$mode objfpc}{$H+}

interface

uses RoCConsts, Field;

type
  TRoleFunc=procedure(num: integer);

type
  TRole=(roleIdle,
         roleGoSearch,
         roleGoSearchFollower,
         roleDoFormation
         );

type
  TRoleDef = record
    name: string;
    func: TRoleFunc;
  end;


var
 deltadist,deltateta:double;


procedure RoleIdleRules(num: integer);
procedure RoleDoFormationRules(num: integer);
procedure RoleGoSearchRules(num: integer);
procedure RoleGoSearchFollowerRules(num: integer);

const
  RoleDefs: array[low(TRole) .. High(TRole)] of TRoleDef = (
    ( name:'roleIdle';                func: @RoleIdleRules ),
    ( name:'roleGoSearchFollower';           func: @roleGoSearchFollowerRules ),
    ( name:'roleGoSearch';           func: @roleGoSearchRules ),
    ( name:'roleDoFormation'; func: @RoleDoFormationRules)
  );

implementation

uses Main,Actions, Tatic, Math, Astar, Utils, Tasks, Param, MPC;

//----------------------------------------------------------------------
//  Role Rules
//----------------------------------------------------------------------

procedure RoleIdleRules(num: integer);
begin
  //RobotInfo[num].task:=taskIdle;
end;

procedure RoleDoFormationRules(num: integer);
begin
  {
  with TaskPars[num] do begin
      speed := staticSpeed;

  end;

  with RobotInfo[num] do begin
    if FormationSettings.active then
       task:=taskDoFormation
    else
       task := taskIdle
  end; }

end;

procedure RoleGoSearchFollowerRules(num: integer);
begin

  {with TaskPars[num] do begin
      speed := staticSpeed;

  end;

  with RobotInfo[num] do begin
    if FormationSettings.active then
       task:=taskDoFormationFollower
    else
       task := taskIdle
  end;

        }
end;

procedure RoleGoSearchRules(num: integer);
var X1Loc,Y1Loc,X2Loc,Y2Loc,X3Loc,Y3Loc,X4Loc,Y4Loc,val:double;
begin
       X1Loc:=FieldDims.FieldDepth/4;
       Y1Loc:=-FieldDims.FieldWidth/4;
       X2Loc:=-FieldDims.FieldDepth/4;
       Y2Loc:=-FieldDims.FieldWidth/4;
       X3Loc:=-FieldDims.FieldDepth/4;
       Y3Loc:=FieldDims.FieldWidth/4;
       X4Loc:=FieldDims.FieldDepth/4;
       Y4Loc:=FieldDims.FieldWidth/4;

       TaskPars[num].speed:=1;
       val:=0.2;
       with RobotInfo[num], TaskPars[num] do begin
         with TaskPars[num] do begin
           if (((xold<>X1loc)and(yold<>Y1loc))and((xold<>X2loc)and(yold<>Y2loc))and((xold<>X3loc)and(yold<>Y3loc)))or
              ((xold=X4loc)and(yold=Y4loc)) then begin
             x1 := X1Loc;
             y1 := Y1Loc;
             teta:=270*pi/180;
             if (((RobotState[num].x)<x1+val)and((RobotState[num].x)>x1-val)and
                 ((RobotState[num].y)<y1+val)and((RobotState[num].y)>y1-val)) then begin
               xold:=X1loc;
               yold:=Y1loc;
             end;
           end else if ((xold=X1loc)and(yold=Y1loc)) then begin
             x1 := X2loc;
             y1 := Y2loc;
             teta:=180*pi/180;
             if (((RobotState[num].x)<x1+val)and((RobotState[num].x)>x1-val)and
                 ((RobotState[num].y)<y1+val)and((RobotState[num].y)>y1-val)) then begin
               xold:=X2loc;
               yold:=Y2loc;
             end;
           end else if ((xold=X2loc)and(yold=Y2loc)) then begin
             x1 := X3loc;
             y1 := Y3loc;
             teta:=90*pi/180;
             if (((RobotState[num].x)<x1+val)and((RobotState[num].x)>x1-val)and
                 ((RobotState[num].y)<y1+val)and((RobotState[num].y)>y1-val)) then begin
               xold:=X3loc;
               yold:=Y3loc;
             end;
           end else if ((xold=X3loc)and(yold=Y3loc)) then begin
             x1 := X4loc;
             y1 := Y4loc;
             teta:=0;
             if (((RobotState[num].x)<x1+val)and((RobotState[num].x)>x1-val)and
                 ((RobotState[num].y)<y1+val)and((RobotState[num].y)>y1-val)) then begin
               xold:=X4loc;
               yold:=Y4loc;
             end;
           end;
         end;

         deltateta:=1;
         task:=taskGoToGoodPosition;
         {if FormationSettings.active then
             task:=taskGoToGoodPosition
         else
             task := taskIdle}
       end;

end;


end.
