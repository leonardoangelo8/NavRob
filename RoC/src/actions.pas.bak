unit Actions;

{$mode objfpc}{$H+}

interface

uses Main,Math,RoCConsts,Astar,SysUtils, Model;//, MPC;

type
  TAction=(acStop,acGoToXYTeta,acControlTrajectory,acDoFormation,acDoFormationFollower);

const
  CActionString: array[low(TAction)..High(TAction)] of string =
        ('acStop','acGoToXYTeta','acControlTrajectory','acDoFormation','acDoFormationFollower');

const
  GoToXYTetaPrecision=0.025;
  minNextPointDistance=0.05;

procedure ActionStop(num: integer);
procedure VToVxy(teta,v: double; var Vx,Vy: double);
procedure VxyToV(teta,vx,vy: double; var V: double);
procedure ActionDoFormation(num:integer);
procedure ActionDoFormationFollower(num:integer);
procedure ActionControlTrajectory(num: integer);

// Procedures related to Controller
procedure ProportionalSatTurtleBot(var v1,v2: double; vmax: double);
procedure SetTacticCommandsInRobotRefNH(var com: TTacticCommand; v,w: double);

//TEMPORARIO. QUANDO TERMINAR CONTROLADOR RETIRAR ESSA FUNÇÃO
procedure ActionGoToXYTeta(num: integer);
procedure GotoXYTheta2(v: double; var state: TRobotState; var com: TTacticCommand; var traj: TTrajectory);


type
  TActionFunc = procedure(num: integer);

const
  CActionFunc: array[low(TAction)..High(TAction)] of TActionFunc =
      ( @ActionStop, @ActionGoToXYTeta, @ActionControlTrajectory, @ActionDoFormation, @ActionDoFormationFollower);

//****************************************************
//
// type TActionsPars
//
//----------------------------------------------------
// Parameters for the action that's currently being
// executed
//****************************************************
type
  TActionPars=record
    x,y,w,teta: double;

    sx,sy: double;    // start position for acFollowVector

    anyway: boolean;
    speed: double;
    speed_on_target: double;
    target_precision: double;
    controllerIndex : integer;
    desiredV,desiredW:double;
    chipkick_dist: double;
    avoid: TAvoidSet;
  end;

var
  //Array of TActionPars, action parameters for each robot
  ActionPars: array [0..MaxRobots-1] of TActionPars;

  //Trajectories of the current robot
  traj: TTrajectory;
  staticTraj: TTrajectory;

  //Controller Index
  controllerIndex : integer;

  gotoxytheta_flag: Boolean;


function ActionString(a: TAction): string;
function norm_ang(ang: double): double;

implementation

uses Tatic, Utils, Robots, Roles, Tasks;

function ActionString(a: TAction): string;
begin
  result:=CActionString[a];
end;

//-----------------------------------------------------------------------------
//  Controller
//-----------------------------------------------------------------------------

//-------------------------------------------------------------------------
// VxyToV()
//
// Convert speeds in Vxy to V
//-------------------------------------------------------------------------

procedure VxyToV(teta,vx,vy: double; var V: double);
var ct,st: double;
begin

  ct:=cos(teta);
  st:=sin(teta);
  v:=(vx+vy)/(ct+st);

end;

procedure VToVxy(teta, V: double; var vx,vy: double);
var ct,st: double;
begin

  vx:=v*cos(teta);
  vy:=v*sin(teta);


end;

function SatVal(v, vmax: double): double;
begin
  if v > vmax then v := vmax;
  if v < -vmax then v := -vmax;
  result := v;
end;

//-------------------------------------------------------------------------
// ProportionalSatTurtleBot
//
// Keeps vector directions if norms have to be scaled
//-------------------------------------------------------------------------
procedure ProportionalSatTurtleBot(var v1,v2: double; vmax: double);
var maxv,minv: double;
    scale,scalemax,scalemin: double;
begin
  maxv:=Max(v1,v2);
  minv:=Min(v1,v2);

  if maxv>vmax then scalemax:=maxv/vmax else scalemax:=1.0;
  if minv<-vmax then scalemin:=minv/(-vmax) else scalemin:=1.0;

  scale:=Max(scalemin,scalemax);

  v1:=v1/scale;
  v2:=v2/scale;

end;

//-------------------------------------------------------------------------
// SetTacticCommandsInRobotRefNH
//
// Defines the calculated v and w in the TTacticCommand object of the
// current Robot. This is the object that's read to send the speeds to
// the motors
//-------------------------------------------------------------------------

procedure SetTacticCommandsInRobotRefNH(var com: TTacticCommand; v,w: double);
begin
  com.v:=v;
  com.w:=w;
  with com do begin
    v1 := v + 0.2318*w;
    v2 := v - 0.2318*w;

    ProportionalSatTurtleBot(v1, v2, SpeedMax);
  end;
end;

//-------------------------------------------------------------------------
// Actions!!!
//
// ActionStop
//
// Stop Robot movement
//-------------------------------------------------------------------------

procedure ActionStop(num: integer);
begin
  with TacticCommands[num] do begin
    v1:=0;
    v2:=0;

    v:=0;
    w:=0;
  end;
  RobotCalcData[num].trajectory_length := 0;
end;

//-------------------------------------------------------------------------
// ActionControlTrajectory
//
// Follows a given trajectory (Dynamic Traj)
//-------------------------------------------------------------------------
procedure ActionControlTrajectory(num: integer);
var rx,ry,d: double;
    i: integer;
    v,w: double;
begin
    {  rx:=ActionPars[num].x;
      ry:=ActionPars[num].y;

      if (abs(rx)>FieldLength/2-0.75) and (abs(ry)<1) then begin
         rx:=FieldLength/2-0.8;
      end;

      //Get Robot trajectory (A*)
      RobotBestPath(num,rx,ry,traj,ActionPars[num].avoid);

      //Set target teta for each point as the final teta and the rotation power (w)
      for i:=0 to traj.count - 1 do begin
          traj.pts[i].teta := ActionPars[num].teta;
          traj.pts[i].teta_power := 1;
      end;

      //Run controller
      FormMPC.MPCcontrollerFollowTraj(RobotState[num],traj,ActionPars[num].speed,v,w);
      RobotCalcData[num].trajectory_length := traj.distance;

      FormMPC.showDebug2;

      //Set speed references
      SetTacticCommandsInRobotRef(TacticCommands[num],v,w);

      //Update RobotState State
      RobotState[num].v := V;
      RobotState[num].w := w;        }

end;


//-------------------------------------------------------------------------
// ActionDoFormation
//
// keeps formation
//-------------------------------------------------------------------------
procedure ActionDoFormation(num: integer);
var  v,w,distancia: double;
     i: integer;
begin

      {
        //Run controller
        FormMPC.MPCcontroller(RobotState[num],staticTraj,ActionPars[num].avoid,ActionPars[num].speed,v,w);

        FormMPC.showDebug;

        //Set speed references
        SetTacticCommandsInRobotRef(TacticCommands[num],v,w);

        //Update RobotState State
        RobotState[num].v := v;
        RobotState[num].w := w;       }
end;

//-------------------------------------------------------------------------
// ActionDoFormationFollower
//
// keeps formation when searching the target
//-------------------------------------------------------------------------
procedure ActionDoFormationFollower(num: integer);
var  v,w,distancia: double;
begin
   {
        //Run controller
        FormMPC.MPCcontrollerFollower(RobotState[num],staticTraj,ActionPars[num].avoid,ActionPars[num].speed,v,w);

        FormMPC.showDebug;

        //Set speed references
        SetTacticCommandsInRobotRef(TacticCommands[num],v,w);

        //Update RobotState State
        RobotState[num].v := v;
        RobotState[num].w := w;  }
end;

//-------------------------------------------------------------------------
// TrajectoryController
//
// Calculates the correct v and w to follow the trajectory calculated
// before. Limits speeds for deceleration when close to the target to arrive
// to it at the desired speed.
//-------------------------------------------------------------------------
procedure ActionGoToXYTeta(num: integer);
var rx,ry,d: double;
    i: integer;
begin
  rx:=ActionPars[num].x;
  ry:=ActionPars[num].y;

  //manuel
  if (abs(rx)>FieldLength/2-0.75) and (abs(ry)<1) then begin
    rx:=FieldLength/2-0.8;
    //ry:=RobotState[num].y;
  end;

  //Get Robot trajectory (A*)
  RobotBestPath(num, rx,ry,traj,ActionPars[num].avoid);

  //Set target teta for each point as the final teta and the rotation power (w)
  for i:=0 to traj.count - 1 do begin
    traj.pts[i].teta := ActionPars[num].teta;
    traj.pts[i].teta_power := 1;
  end;

  //Call Controller
  GotoXYTheta2(ActionPars[num].speed, RobotState[num], TacticCommands[num], traj);
  RobotCalcData[num].trajectory_length := traj.distance;

end;

procedure GotoXYTheta2(v: double; var state: TRobotState; var com: TTacticCommand; var traj: TTrajectory);
const
  teta_error_min=Pi/4;
  dist_error_max=0.001;
  kt=1;
  kd=1;
  A_trav=40;
var
  dist_error, teta_error, v_trav, dist_trav: double;
  tx, ty, segteta: double;
  idx: integer;
begin

  idx := Min(2, traj.count-1); //target is the 2nd trajectory point from current localization
  segteta := DiffAngle(traj.pts[idx].teta, state.teta);
  tx := traj.pts[idx].x;
  ty := traj.pts[idx].y;

  dist_error:= sqrt((tx-state.x)*(tx-state.x)+(ty-state.y)*(ty-state.y));
  teta_error:= norm_ang(arctan2(ty-state.y,tx-state.x)-state.teta);

  dist_trav:=(v*v)/A_trav-0.5*(v*v)/A_trav;

  if (abs(teta_error)>teta_error_min) and (abs(dist_error)>dist_error_max) then begin//alinha
    SetTacticCommandsInRobotRefNH(com, 0, kt*teta_error);
  end
  else begin
    gotoxytheta_flag:=true;
    if (dist_error>dist_trav) and (abs(dist_error)>dist_error_max) then begin //avança
      SetTacticCommandsInRobotRefNH(com, v, kt*teta_error);
    end
    else begin
      if abs(dist_error)>dist_error_max then begin //trava
        v_trav:=kd*dist_error;
        if v_trav>v then begin
          v_trav:=v;
        end;
        SetTacticCommandsInRobotRefNH(com, v_trav, kt*teta_error);
      end
      else begin//roda
        SetTacticCommandsInRobotRefNH(com, 0, kt*(norm_ang(traj.pts[idx].teta-state.teta)));
      end;
    end;
  end;
end;

function norm_ang(ang: double): double;
var
  temp: double;
begin
  temp:= ang/(2.0*Pi);

  if abs(temp)>=1.0 then
  begin
    ang:=ang-trunc(temp)*(2.0*Pi);
  end;

  if (ang>Pi) and (ang<=2.0*Pi) then
  begin
   ang:=ang-2.0*Pi;
  end;

  if (ang<=-Pi) and (ang>=-2.0*Pi) then
  begin
     ang:=ang+2.0*Pi;
  end;

  result:=ang;
end;


end.

