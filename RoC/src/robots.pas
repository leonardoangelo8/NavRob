unit Robots;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Field, RoCConsts,
  Graphics, Controls, Forms, Dialogs, Tatic,
  Math, Types, dynmatrix, Main, Tasks, Roles, Astar;


type
  TRobotStatus = record
    active: boolean;
    default_role: TRole;
    start_role: integer;
    manual_control: boolean;
    v1: integer;
    v2: integer;
  end;


var
  RobotStatus: array[0..MaxRobots-1] of TRobotStatus;

procedure PropagateXYTeta(var RS: TRobotstate);
procedure PropagateXYTetaOthers(var RS: TRobotstate);
procedure RobotStateDoublesToMatrix(var Robot: TRobotState);
procedure RobotStateMatrixToDoubles(var Robot: TRobotState);

procedure SaturateRobotXY(var RS: TRobotstate);

implementation

uses Utils, Param, Actions;


procedure RobotStateMatrixToDoubles(var Robot: TRobotState);
begin
  with Robot do begin
    x:=Xk.getv(0,0);
    y:=Xk.getv(1,0);
    teta:=Xk.getv(2,0);

    cov_x:=Pk.getv(0,0);
    cov_y:=Pk.getv(1,1);
    cov_xy:=0.5*(Pk.getv(0,1)+Pk.getv(1,0));
    cov_teta:=Pk.getv(2,2);
  end;
end;

procedure RobotStateDoublesToMatrix(var Robot: TRobotState);
begin
  with Robot do begin
    Xk.setv(0,0,x);
    Xk.setv(1,0,y);
    Xk.setv(2,0,teta);

    Pk.setv(0,0,cov_x);
    Pk.setv(1,1,cov_y);
    Pk.setv(0,1,cov_xy);
    Pk.setv(1,0,cov_xy);
    Pk.setv(2,2,cov_teta);
  end;
end;

procedure SaturateRobotXY(var RS: TRobotstate);
begin
  with RS do begin
    if x>FieldDims.BoundaryDepth/2 then x:=FieldDims.BoundaryDepth/2;
    if x<-FieldDims.BoundaryDepth/2 then x:=-FieldDims.BoundaryDepth/2;
    if y>FieldDims.BoundaryWidth/2 then y:=FieldDims.BoundaryWidth/2;
    if y<-FieldDims.BoundaryWidth/2 then y:=-FieldDims.BoundaryWidth/2;
  end;
  RobotStateDoublesToMatrix(RS);
end;

procedure PropagateXYTeta(var RS: TRobotstate);
var i: integer;
begin
  with RS do begin

    v := (View.Odos[0].speedw[0]+View.Odos[0].speedw[1])/2;
    w := (View.Odos[0].speedw[0]-View.Odos[0].speedw[1])/0.2318;

    VxyToV(teta, v, vx, vy);
  end;
end;

procedure PropagateXYTetaOthers(var RS: TRobotstate);
var cov_xy_noise,cov_teta_noise: double;
begin
  cov_xy_noise:=0.01;
  cov_teta_noise:=0.001;
  // TODO
  with RS do begin
    x:=x+0.04*v*cos(teta);
    y:=y+0.04*v*sin(teta);
    teta:=teta+0.04*w*pi/180;
    cov_x:=cov_x+cov_xy_noise;
    cov_y:=cov_y+cov_xy_noise;
    cov_teta:=cov_teta+cov_teta_noise;
  end;
end;


end.

