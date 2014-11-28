//------------------------------------------------------------------------------
//
// model.pas
//
//------------------------------------------------------------------------------
//
// Procedures for simulating robot behaviour (updates v, vn, and w references to
// actual speeds, according to robot model)
//
//------------------------------------------------------------------------------

unit model;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils,dynmatrix, dynmatrixutils,Utils,Main,LCLProc,math, Param, RoCConsts;

type



//------------------------------------------------------
// TMotorState
// Model parameters for simulation
//------------------------------------------------------
TMotorState = record
  v : double                //motor speed, in m/s
end;


//------------------------------------------------------
// TSimRobot
// Robot for simulation
//------------------------------------------------------
TSimRobot = record
  RobotState : TRobotState;
  MotorState : Array[0 .. 2] of TMotorState;
end;

//------------------------------------------------------
// TSimBall
// Robot for simulation
//------------------------------------------------------
{TSimBall = record
  BallState : TBallState;
  sigmaS: TDMatrix;
  detsigma: double;
end;
}

//------------------------------------------------------
// TModelParams
// Model parameters for simulation
//------------------------------------------------------
TModelParams = record
  d:double;                    //distance from wheels to center
  maxWheelSpeed : double;      //max wheel speed for saturation control
  motorTao : double            //[motor + controller + robot] system time constant
end;

//load robot parameters
procedure loadParameters(vmax,tao : double);
//reset model state
procedure resetModel();
//simulate Robot (vref,vnref,vwref -> v,vn,w)
procedure simRobotComplete(var Robot : TSimRobot; v_ref,w_ref : double; var v,w : double);
//simulate motors (motor ref speeds -> motor actual speeds)
procedure simMotors(var Robot : TSimRobot; var v1,v2 : double);
//inverse kinematic (v,w->v1,v2)
procedure IK(v,w: double; var v1,v2: double);
//direct kinematic (v1,v2->v,w)
procedure DK(v1, v2: double; var v, w: double);
//updates robot position
procedure simRobotMovement(var Robot : TSimRobot; var v,w : double);
//updates covariance matrix
//function SimCovariance(Robot : TSimRobot; Ball: TSimBall; val,k1,k2: double):TDMatrix;
//updates covariance matrix
//function SimCovariance2(MyRobot : TSimRobot; Robot : TSimRobot; Ball: TSimBall; val,k1,k2: double):TDMatrix;
//Sum the covariance matrixes
//function SimCovarianceSUM(sgt1, sgt2: TDMatrix):TDMatrix;
//updates ball position
//procedure simBallMovement(var Ball : TSimBall);

var
    //Model Parameters
    ModelParams : TModelParams;

    //SimRobot structure
    SimRobot : TSimRobot;

    //SimBall structure
    //SimBall : TSimBall;

    //global variables for debugging
    a_g,b_g,v1ref_g,v2ref_g,v3ref_g,v1real_g,v2real_g,v3real_g : double;

const

     simTimeStep = 0.01;


implementation

Uses MPC;

//----------------------------------------------------------------------------
//
// loadParameters()
//
//----------------------------------------------------------------------------
// define model parameters
//
//----------------------------------------------------------------------------
procedure loadParameters(vmax,tao : double);
begin

     ModelParams.d := WheelToWheelDist;  //distance from wheel to center
     ModelParams.maxWheelSpeed := vmax;   //max motor speed (m/s)
     ModelParams.motorTao := tao;         //[motor + controller + robot] system time constant

end;

//----------------------------------------------------------------------------
//
// resetModel()
//
//----------------------------------------------------------------------------
// reset model state
//
//----------------------------------------------------------------------------
procedure resetModel();
begin

    simRobot.RobotState.x := 0;
    simRobot.RobotState.y := 0;
    simRobot.RobotState.teta := 0;
    simRobot.RobotState.v:=0;
    simRobot.RobotState.w:=0;

    SimRobot.MotorState[0].v := 0;
    SimRobot.MotorState[1].v := 0;

    {SimBall.BallState.x := 0;
    SimBall.BallState.y := 0;
    SimBall.BallState.vy := 0;
    SimBall.BallState.vx := 0;
    SimBall.BallState.quality := 0;  }

end;


//----------------------------------------------------------------------------
//
// IK() Robo Diferencial
//
//----------------------------------------------------------------------------
// inverse kinematic (v,w->v1,v2)
//
//----------------------------------------------------------------------------
procedure IK(v, w: double; var v1, v2: double);
begin

    v1 := v + 0.2999*w;
    v2 := v - 0.2999*w;

end;

//----------------------------------------------------------------------------
//
// DK() Robo Diferencial
//
//----------------------------------------------------------------------------
// Direct Kinematics(v1,v2->v,w)
//
//----------------------------------------------------------------------------
procedure DK(v1, v2: double; var v, w: double);
begin
    v := (v1+v2)/2;
    w := (v1-v2)/0.2318;
end;


//----------------------------------------------------------------------------
//
// SimRobotMovement()
//
//----------------------------------------------------------------------------
// Updates the simulated robot's position taking into account the current
// position, and input values. Uses 10ms intervals (run 4 times for 40 ms)
//----------------------------------------------------------------------------
procedure simRobotMovement(var Robot : TSimRobot; var v,w : double);
var
   cteta,steta : double;
   v1,v2 : double;
begin
          cteta := cos(Robot.RobotState.teta);
          steta := sin(Robot.RobotState.teta);

          if Robot.RobotState.teta > pi then
             Robot.RobotState.teta := Robot.RobotState.teta - 2*pi;

          Robot.RobotState.teta := Robot.RobotState.teta + simTimeStep*w;
          Robot.RobotState.x := Robot.RobotState.x + simTimeStep*(v*cteta);
          Robot.RobotState.y := Robot.RobotState.y + simTimeStep*(v*steta);

end;

//----------------------------------------------------------------------------
//
// SimCovariance()
//
//----------------------------------------------------------------------------
// Updates the simulated ball's covariance taking into account the current
// positions of robot and ball. Uses 10ms intervals (run 4 times for 40 ms)
//----------------------------------------------------------------------------
{function SimCovariance(Robot : TSimRobot; Ball: TSimBall; val,k1,k2: double):TDMatrix;
var sgt,sgt2: TDMatrix;
    v,d,sigX,sigY,tetao,sigphi: double;
begin
  sgt.SetSize(2,2);
  sgt:=Mzeros(2,2);

  d:=(sqrt(power((Robot.RobotState.x-Ball.BallState.x),2)+power((Robot.RobotState.y-Ball.BallState.y),2)));

  sigX:=k1*(power(d,2));
  sigY:=k2*d;

  sgt.setv(0,0,sigX);
  sgt.setv(0,1,0);
  sgt.setv(1,0,0);
  sgt.setv(1,1,sigY);

  result:=sgt;
end;

function SimCovariance2(MyRobot : TSimRobot; Robot : TSimRobot; Ball: TSimBall; val,k1,k2: double):TDMatrix;
var sgt,sgt2: TDMatrix;
    dist,alfa,sigX,sigY,v,a,aa,b,bb,c,cc,d,temp: double;
begin
  sgt.SetSize(2,2);
  sgt:=Mzeros(2,2);

  alfa:=atan2((MyRobot.RobotState.y - Ball.BallState.y),(MyRobot.RobotState.x - Ball.BallState.x))-atan2((Robot.RobotState.y - Ball.BallState.y),(Robot.RobotState.x - Ball.BallState.x));

  dist:=(sqrt(power((Robot.RobotState.x-Ball.BallState.x),2)+power((Robot.RobotState.y-Ball.BallState.y),2)));

  sigX:=k1*(power(dist,2));
  sigY:=k2*dist;

  a:=(cos(alfa)*sigX*cos(alfa))+(sin(alfa)*sigY*sin(alfa));
  b:=(cos(alfa)*sigX*sin(alfa))-(sin(alfa)*sigY*cos(alfa));
  c:=(sin(alfa)*sigX*cos(alfa))-(cos(alfa)*sigY*sin(alfa));
  d:=(sin(alfa)*sigX*sin(alfa))+(cos(alfa)*sigY*cos(alfa));

  sgt.setv(0,0,a);
  sgt.setv(0,1,b);
  sgt.setv(1,0,c);
  sgt.setv(1,1,d);

  result:=sgt;
end;

function SimCovarianceSUM(sgt1, sgt2: TDMatrix):TDMatrix;
var sgt,sgttemp,cac: TDMatrix;
    v,a,b,c,d,r,s,t,u: double;
begin
  sgt.SetSize(2,2);
  sgt:=Mzeros(2,2);
  sgttemp.SetSize(2,2);
  sgttemp:=Mzeros(2,2);
  cac.SetSize(2,2);
  cac:=Mzeros(2,2);

  sgttemp.setv(0,0,(sgt1.getv(0,0)+sgt2.getv(0,0)));    //a
  sgttemp.setv(0,1,(sgt1.getv(0,1)+sgt2.getv(0,1)));    //b
  sgttemp.setv(1,0,(sgt1.getv(1,0)+sgt2.getv(1,0)));    //c
  sgttemp.setv(1,1,(sgt1.getv(1,1)+sgt2.getv(1,1)));    //d

  v:=1/((sgttemp.getv(0,0)*sgttemp.getv(1,1))-(sgttemp.getv(0,1)*sgttemp.getv(1,0)));

  //inverts the matrix
  r:=(v*sgttemp.getv(1,1));                //r
  s:=-(v*sgttemp.getv(0,1));               //s
  t:=-(v*sgttemp.getv(1,0));               //t
  u:=(v*sgttemp.getv(0,0));                //u

  a:=(sgt1.getv(0,0));    //a
  b:=(sgt1.getv(0,1));    //b
  c:=(sgt1.getv(1,0));    //c
  d:=(sgt1.getv(1,1));    //d

  cac.setv(0,0,((((a*r)+(b*t))*a)+(((a*s)+(b*u))*c) ));
  cac.setv(0,1,((((a*r)+(b*t))*b)+(((a*s)+(b*u))*d) ));
  cac.setv(1,0,((((c*r)+(d*t))*a)+(((c*s)+(d*u))*c) ));
  cac.setv(1,1,((((c*r)+(d*t))*b)+(((c*s)+(d*u))*d) ));

  sgt.setv(0,0,(a-(cac.getv(0,0))));
  sgt.setv(0,1,(b-(cac.getv(0,1))));
  sgt.setv(1,0,(c-(cac.getv(1,0))));
  sgt.setv(1,1,(d-(cac.getv(1,1))));

  result:=sgt;
end; }


//----------------------------------------------------------------------------
//
// SimRobotComplete()
//
//----------------------------------------------------------------------------
// Simulate Robot (vref,vnref,wref -> v,vn,w), 10ms period
//
//----------------------------------------------------------------------------
procedure simRobotComplete(var Robot : TSimRobot; v_ref,w_ref : double; var v,w : double);
var
   v1,v2 : double;
begin

   //v,w -> v1,v2
   IK(v_ref,w_ref,v1,v2);

   //simulate motors
   //TODO: simMotors(Robot,v1,v2);

   //v1,v2 -> v,w
   DK(v1,v2,v,w);

end;

//----------------------------------------------------------------------------
//
// SimBallMovement()
//
//----------------------------------------------------------------------------
// Simulate Robot (vref,vnref,wref -> v,vn,w), 10ms period
//
//----------------------------------------------------------------------------
{procedure simBallMovement(var Ball: TSimBall);
begin

     Ball.BallState.x := Ball.BallState.x + simTimeStep*Ball.BallState.vx;
     Ball.BallState.y := Ball.BallState.y + simTimeStep*Ball.BallState.vy;

     Ball.BallState.vx:=Ball.BallState.vx*BallFrictionCoef;
     Ball.BallState.vy:=Ball.BallState.vy*BallFrictionCoef;

end;  }

//----------------------------------------------------------------------------
//
// SimMotors()
//
//----------------------------------------------------------------------------
// Simulate Motors (v1ref, v2ref -> v1,v2), 10ms period
//
//----------------------------------------------------------------------------
procedure simMotors(var Robot : TSimRobot; var v1,v2 : double);
var
   i: integer;
   a,b : double;
   u : array[0..1] of double;
begin

     //input vector (motor speed references)
     u[0] := v1;
     u[1] := v2;

     a := exp((-1/ModelParams.motorTao)*simTimeStep);
     b := 1 - a;

     //DEBUG
     v1ref_g:=u[0];
     v2ref_g:=u[1];

     a_g := a;
     b_g := b;

     //LTI step response simulation for motors
    for i:= 0 to 1  do begin
        Robot.MotorState[i].v := a*Robot.MotorState[i].v + b*u[i];
    end;

    //update reference
    v1 := Robot.MotorState[0].v;
    v2 := Robot.MotorState[1].v;

    //DEBUG
    v1real_g := v1;
    v2real_g := v2;

end;

end.

