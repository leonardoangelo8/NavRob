unit MPC;

{$mode objfpc}{$H+}
{$define MDEC}

interface

uses
  Classes, SysUtils, FileUtil, LResources, Forms, Controls, Graphics, Dialogs,
  StdCtrls, IniPropStorage, ComCtrls, ExtCtrls, Grids, Main, dynmatrix,
  Astar, Utils, model, Math, Roles, RoCConsts;

//----------------------------------------------------------------------------
//
// Structures for holding formation data (the cost functions and definitions)
//
//----------------------------------------------------------------------------
// NOTE: Shouldn't be defined here at all, only here due to circular
//       reference problems with other units (Bug from Dec)
//----------------------------------------------------------------------------

//------------------------------------------------------
// TROstacles
// holds parameters for obstacles avoidance
//------------------------------------------------------
  type
  TRObstacle=record
    x,y,r: double;
    used: boolean;
  end;

//Function pointer for cost functions
//type
  //TCostFunc=function(var Robot : TSimRobot; var Ball : TSimBall; var obs: array of TRObstacle; num_obs: integer; RobotState : array of TRobotState;  var U : TDMatrix) : double;

//Formation definition
type
  TFormationDef = record
    name: string;
    //func: TCostFunc;
  end;

//Formation data
type
    TFormationState = record
      BallSpeedDir_x, BallSpeedDir_y : double;
      RobotBallPos_x,RobotBallPos_y : double;
      RobotRobotPos_x,RobotRobotPos_y : double;
      RobotBallDist : double;
      RobotBallAngle : double;
      RobotRobotAngle : double;
      RobotBallDotProduct : double;
      RobotLeaderDotProduct : double;
      RobotFormationDistance : array[0..5] of double;
      RobotObstacleDistance : array[0..20] of double;
end;

//Formation list
type
  TFormation=(generalFormation);

//Formation settings (holds formation name, weights, and parameters)
type
    TFormationSettings = record
      formation : TFormation;
      active: boolean;
      pvalue: double;
      dvalue: double;
      k1: double;
      k2: double;
      k3: double;
      k4: double;

      //weights for cost function (change formation)
      weights : array[0 .. 10] of double;
      //parameters of the formation (change for formation)
      parameters : array[0 .. 10]of double;
      //formation mates
      formationMates : array[0..4] of integer;
end;

//Cost function
//function GeneralCostFunction(var Robot : TSimRobot; var Ball : TSimBall;  var obs: array of TRObstacle; num_obs: integer; RobotState : array of TRobotState; var U : TDMatrix) : double;
//function GeneralCostFunctionFollower(var Robot : TSimRobot;  var obs: array of TRObstacle; num_obs: integer; RobotState : array of TRobotState; var U : TDMatrix) : double;
//function TrajCostFunction(var Robot : TSimRobot; RobotState : array of TRobotState; var U : TDMatrix; refTraj : TTrajectory) : double;


//Cost functions array
{const
  FormationDefs: array[low(TFormation) .. High(TFormation)] of TFormationDef = (
    ( name: 'generalFormation';       func: @GeneralCostFunction)
  );
 }
//--------------------------------------------------------------------
// Rest of the stuff
//--------------------------------------------------------------------

type

//------------------------------------------------------
// TSimParameters
// holds parameters for MPC controller simulation part
//------------------------------------------------------
  TSimParameters=record
    N1,N2,Nu : integer;               //control and prediction horizons
    lambda : array[0 .. 2] of double; //weights for the cost function calculation
  end;


//------------------------------------------------------
// TOptParameters
// Optimizer parameters
//------------------------------------------------------
  TOptParameters=record
    MaxIterations : double;          //Max iterations of the minimization algorithm
    delta : double;                  //step for variable change for calculation of grad(J)
    Jstop : double;                  //Stop criteria for cost
    alpha : double;                  //Step size for steepest gradient

    etaPlus : double;                //RPROP coef
    etaMinus: double;                //RPROP coef
    stepMax: double;                 //RPROP max step
    stepMin: double;                 //RPROP min step

  end;

//------------------------------------------------------
// TOptData
// Optimizer data structures
//------------------------------------------------------
  TOptData=record
   Jsteps : TDMatrix;  //intermediate matrix for antigradient calculation

   Jgradient : TDMatrix;
   Jgradient_prev: TDMatrix;
   Jstep_prev: TDMatrix;

   beta,t1,t2,t3,a1,a2,a3,b1,b2,b3 : float;  //Step size for conjugate gradient

   iterationCount : integer; //Current iteration number
  end;

  { TFormMPC }

  TFormMPC = class(TForm)
    ButtonSetBallAtDistanceInline: TButton;
    CheckBoxEnable: TCheckBox;
    CheckBoxPropSat: TCheckBox;
    CheckBoxSimDynamics: TCheckBox;
    CheckBoxSimDynamicsM: TCheckBox;
    CheckBoxSimDynamicsR: TCheckBox;
    CheckBoxTauLine: TCheckBox;
    ComboBoxMate1: TComboBox;
    ComboBoxMate2: TComboBox;
    ComboBoxMate3: TComboBox;
    ComboBoxMate4: TComboBox;
    difCB: TCheckBox;
    edk1: TEdit;
    edk2: TEdit;
    EditJStop: TEdit;
    EditL1: TLabeledEdit;
    EditL2: TLabeledEdit;
    EditL3: TLabeledEdit;
    EditN1: TLabeledEdit;
    EditPValue: TEdit;
    EditDelta: TEdit;
    EditEtaMinus: TEdit;
    EditEtaPlus: TEdit;
    EditMaxIt: TEdit;
    EditN2: TLabeledEdit;
    EditNu: TLabeledEdit;
    EditP1_badi: TEdit;
    EditDValue: TEdit;
    EditSDAlpha: TEdit;
    EditTao: TEdit;
    EditTauLineB: TEdit;
    EditTauLineK: TEdit;
    EditVMax: TEdit;
    EditW1_badi: TEdit;
    EditW2_badi: TEdit;
    EditW3_badi: TEdit;
    EditW4_badi: TEdit;
    EditW5_badi: TEdit;
    EditW7_badi: TEdit;
    EditW6_badi: TEdit;
    GroupBox1: TGroupBox;
    GroupBox2: TGroupBox;
    GroupBox3: TGroupBox;
    GroupBoxDebug: TGroupBox;
    GroupBoxModelParams: TGroupBox;
    GroupBoxOptimizerPars: TGroupBox;
    FormStorage: TIniPropStorage;
    GroupBoxOtherPars: TGroupBox;
    Label1: TLabel;
    Label10: TLabel;
    Label11: TLabel;
    Label12: TLabel;
    Label13: TLabel;
    Label14: TLabel;
    Label15: TLabel;
    Label16: TLabel;
    Label17: TLabel;
    Label18: TLabel;
    Label19: TLabel;
    Label2: TLabel;
    Label20: TLabel;
    Label21: TLabel;
    Label22: TLabel;
    Label23: TLabel;
    Label24: TLabel;
    Label25: TLabel;
    Label3: TLabel;
    Label4: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    Label8: TLabel;
    Label9: TLabel;
    MemoDebug: TMemo;
    //procedure ButtonSetBallAtDistanceInlineClick(Sender: TObject);
    //procedure CheckBoxEnableChange(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure addDebug(varName: string; varValue : double);
    procedure FormShow(Sender: TObject);
  private
    { private declarations }
  public
    { public declarations }

    //load configuration from form
    procedure loadConfig();
    {function ObstacleInSegment(st_x,st_y,tg_x,tg_y: double; var obs: array of TRObstacle; num_obs: integer): boolean;
    //Show ball state on debug memo box
    procedure showDebug();
    procedure showDebug2();

    //main controller loop
    procedure MPCcontroller(ThisRobotState : TRobotState; var traj : TTrajectory; avoid: TAvoidSet; vRefModule: double; var v,vn,w : double);
    //main controller loop for the Follower during search
    procedure MPCcontrollerFollower(ThisRobotState : TRobotState; var traj : TTrajectory; avoid: TAvoidSet; vRefModule: double; var v,vn,w : double);
    //main controller to Follow Trajectory
    procedure MPCcontrollerFollowTraj(ThisRobotState : TRobotState; var traj : TTrajectory; vRefModule: double; var v,vn,w : double);

    //generate deltaU matrix
    procedure calcUSteps(var U, deltaU : TDMatrix);
    //calculate gradient of J from Jsteps vector
    procedure calcGradient(var Jsteps, Jgradient, Jgradient_prev : TDMatrix);
    //calculate a steepest descent step
    procedure calcSteepestDescentStep(var Jgradient, Uref : TDmatrix);
    //calculate a conjugate gradient step
    procedure calcConjugateCradientStep(var Jsteps, Jgradient, Jgradient_prev, Uref : TDmatrix);
    //calculate an RPROP step
    procedure calcRPROPStep(var OptData : TOptData; var Uref : TDmatrix);
    //get reference trajectory for controller
    procedure calcRefTraj(var traj,trajPred: TTrajectory; V : double);
    //keeps vector norms if one saturates
    procedure ProportionalSat(var v1,v2,v3: double; vmax: double);
    //prevents motor saturation
    procedure scaleForSaturation(var U : TDmatrix);  }

  end;

var
  FormMPC: TFormMPC;
  ProgReset: Boolean;

  AlgTimeMax,algTimeG, AlgTimeMin, AlgTimeSum : longword;
  AlgItCount : integer;
  var1, var2, var3: double;



  //global variables for debugging
  JbestG,JcurrentG,v1G,v2G,v3G:double;
  U_refG,UbestG: TDMatrix;

  //Controller structures
  SimParameters : TSimParameters;
  OptParameters : TOptParameters;
  OptData : TOptData;

  //for obstacles???
  Obst: array[0..MaxRobots+MaxObstacles] of TRObstacle;
  final: integer = MaxRobots+MaxObstacles;

  //Formation Settings
  FormationSettings : TFormationSettings = (formation:generalFormation; active:false);
  FormationState : TFormationState;

  //State of all robots in formation for simulation
  SimFormationRobots : array[0 .. 5] of TSimRobot;

implementation

uses
    Robots,Tatic;

{ TFormMPC }

//---------------------------------------------------------------------
//
// Initialization Code   - MPC PROCEDURES
//
//
// Auxiliary procedures, mainly form stuff
//----------------------------------------------------------------------------

//add text to debug
procedure TFormMPC.addDebug(varName:string;varValue:double);
begin
       MemoDebug.Append(format('%s %.8f',[varName, varValue]));
end;

//labeled editbox to float
function EditToFloatDef(edit: TLabeledEdit; default: double): double;
begin
try
    result:=strtofloatdef(edit.text,0);
  except
    result:=default;
    edit.text:=Format('%.8g',[default]);
  end;
end;

//editbox to float
function EditToFloatDef(edit: TEdit; default: double): double;
begin
  try
    result:=strtofloatdef(edit.text,0);
  except
    result:=default;
    edit.text:=Format('%.8g',[default]);
  end;
end;

//---------------------------------------------------------------------
//
// Initialization Code   - FORMATION CONTROL PROCEDURES
//
// session props, initializations, etc
//---------------------------------------------------------------------
procedure TFormMPC.FormCreate(Sender: TObject);
var  k: integer;
     SessionPropsList: TStringList;
     SessionPropsFileName: string;
begin

  //Add some items to boxes
  for k:= 0 to 5 do begin
      if k <> myNumber then begin
         ComboBoxMate1.Items.Add(IntToStr(k+1));
         ComboBoxMate2.Items.Add(IntToStr(k+1));
         ComboBoxMate3.Items.Add(IntToStr(k+1));
         ComboBoxMate4.Items.Add(IntToStr(k+1));
      end;
  end;

  //SessionProps code
  SessionPropsFileName := ExtractFilePath(Application.ExeName)+FMain.DataDir+'/SessionPropsMPC.txt';
  if FileExists(SessionPropsFileName) then begin
    SessionPropsList := TStringList.Create;
    try
      SessionPropsList.LoadFromFile(SessionPropsFileName);
      SessionPropsList.Delimiter:=';';
      SessionProperties := SessionPropsList.DelimitedText;
    finally
      SessionPropsList.Free;
    end;
  end;

  FormStorage.IniFileName:=extractfilepath(application.exename)+FMain.DataDir+'/MPC.ini';
  FormStorage.Restore;
  ProgReset := true;


  //Load Configuration
  loadConfig();
  //Load model parameters
  loadParameters(EditToFloatDef(EditVMax,2),EditToFloatDef(EditTao,0.3));
  //Reset Model
  resetModel();

  //Initialize algTime variables
  AlgTimeMax:=0;
  AlgTimeMin:=40;
  AlgTimeSum := 0;
  AlgItCount := 0;

end;

procedure TFormMPC.FormShow(Sender: TObject);
var k : integer;
begin
  //Add some items to boxes
  ComboBoxMate1.Items.Clear;
  ComboBoxMate2.Items.Clear;
  ComboBoxMate3.Items.Clear;
  ComboBoxMate4.Items.Clear;
  for k:= 0 to 5 do begin
      if k <> myNumber then begin
         ComboBoxMate1.Items.Add(IntToStr(k+1));
         ComboBoxMate2.Items.Add(IntToStr(k+1));
         ComboBoxMate3.Items.Add(IntToStr(k+1));
         ComboBoxMate4.Items.Add(IntToStr(k+1));
      end;
  end;
end;

//----------------------------------------------------------------------------
//
// loadConfig()
//
//---------------------------------------------------------------------------
// Loads configuration from form
//----------------------------------------------------------------------------
procedure TFormMPC.loadConfig();
var
   i : integer;
begin

     //Set model simulation parameters
     SimParameters.N1:=round(EditToFloatDef(EditN1,1));
     SimParameters.N2:=round(EditToFloatDef(EditN2,10));
     SimParameters.Nu:=round(EditToFloatDef(EditNu,1));
     SimParameters.lambda[0] := EditToFloatDef(EditL1,1);
     SimParameters.lambda[1] := EditToFloatDef(EditL2,1);
     SimParameters.lambda[2] := EditToFloatDef(EditL3,1);

     //Set optimizer parameters
     OptParameters.delta:= EditToFloatDef(EditDelta,0.1);;
     OptParameters.MaxIterations:=round(EditToFloatDef(EditMaxIt,20));
     OptParameters.Jstop:= EditToFloatDef(EditJStop,0.1);

     OptParameters.alpha := EditToFloatDef(EditSDAlpha,0.1);          //steepest descent step size
     OptParameters.alpha := EditToFloatDef(EditSDAlpha,0.1);

     OptParameters.etaPlus:= EditToFloatDef(EditEtaPlus,1.2);         //RPROP coef
     OptParameters.etaMinus:=EditToFloatDef(EditEtaMinus,0.5);        //RPROP coef
     OptParameters.stepMax:=50;
     OptParameters.stepMin:=0.1;


     //Initialize optimizer data structures
     OptData.Jsteps.SetSize(6*SimParameters.Nu,1);

     OptData.Jgradient.SetSize(3*SimParameters.Nu,1);
     OptData.Jgradient_prev.SetSize(3*SimParameters.Nu,1);
     OptData.Jstep_prev.SetSize(3*SimParameters.Nu,1);

     //RPROP initial data
     for i:= 0 to SimParameters.Nu-1 do begin
         OptData.Jgradient.setv(3*i+0,0,0);
         OptData.Jgradient.setv(3*i+1,0,0);
         OptData.Jgradient.setv(3*i+2,0,0);
         OptData.Jstep_prev.setv(3*i+0,0,0.1);
         OptData.Jstep_prev.setv(3*i+1,0,0.1);
         OptData.Jstep_prev.setv(3*i+2,0,0.1);
     end;

     OptData.iterationCount:= 0;
end;

{procedure TFormMPC.ButtonSetBallAtDistanceInlineClick(Sender: TObject);
begin
  loadConfig();
  flagForm := false;
  FormationSettings.formation:=TFormation(0);

  FormationSettings.formationMates[0]:=StrToInt(ComboBoxMate1.Text)-1;
  FormationSettings.formationMates[1]:=StrToInt(ComboBoxMate2.Text)-1;
  FormationSettings.formationMates[2]:=StrToInt(ComboBoxMate3.Text)-1;
  FormationSettings.formationMates[3]:=StrToInt(ComboBoxMate4.Text)-1;

  //weights
  FormationSettings.weights[0]:=EditToFloatDef(EditW1_badi,0);
  FormationSettings.weights[1]:=EditToFloatDef(EditW2_badi,0);
  FormationSettings.weights[2]:=EditToFloatDef(EditW3_badi,0);
  FormationSettings.weights[3]:=EditToFloatDef(EditW4_badi,0);
  FormationSettings.weights[4]:=EditToFloatDef(EditW6_badi,0);
  FormationSettings.weights[5]:=EditToFloatDef(EditW5_badi,0);
  FormationSettings.weights[6]:=EditToFloatDef(EditW7_badi,0);
  //parameters
  FormationSettings.parameters[0]:=EditToFloatDef(EditP1_badi,0);
  FormationSettings.pvalue:=EditToFloatDef(EditPValue,2);
  FormationSettings.dvalue:=EditToFloatDef(EditDValue,2);
  //parameters for Observation model
  FormationSettings.k1:=EditToFloatDef(edk1,2);
  FormationSettings.k2:=EditToFloatDef(edk2,2);

end;

procedure TFormMPC.CheckBoxEnableChange(Sender: TObject);
begin
  FormationSettings.active:=CheckBoxEnable.Checked;
end;

//----------------------------------------------------------------------
//
// OTHER PROCEDURES
//
//----------------------------------------------------------------------


//-----------------------------------------------
// Display ball state on debug memobox
//-----------------------------------------------

procedure TFormMPC.showDebug();
begin
     MemoDebug.Clear;
     MemoDebug.Append(format('vBall(dir): %.4f %.4f',[FormationState.BallSpeedDir_x, FormationState.BallSpeedDir_y]));
     MemoDebug.Append(format('pBallRobot(dir): %.3f %.3f',[FormationState.RobotBallPos_x, FormationState.RobotBallPos_y]));
     MemoDebug.Append(Format('dBallRobot: %.2f',[FormationState.RobotBallDist]));
     MemoDebug.Append(Format('angBallRobot: %.2f',[FormationState.RobotBallAngle]));
     MemoDebug.Append(Format('dotProduct: %.2f',[FormationState.RobotBallDotProduct]));
     MemoDebug.Append(Format('mates: %d %d %d %d',[FormationSettings.formationMates[0],
                                                   FormationSettings.formationMates[1],
                                                   FormationSettings.formationMates[2],
                                                   FormationSettings.formationMates[3]]));
     MemoDebug.Append(Format('dRobotObstacle: %.2f %.2f',[FormationState.RobotFormationDistance[4],
                                                          FormationState.RobotFormationDistance[5]
                                                          ]));

     MemoDebug.Append(Format('Matriz: %.2f %.2f',[sigver.getv(0,0),sigver.getv(0,1)]));
     MemoDebug.Append(Format('Matriz: %.2f %.2f',[sigver.getv(1,0),sigver.getv(1,1)]));
     MemoDebug.Append(format(' > It %d ----------------------',[OptData.iterationCount]));
     addDebug(' J:',JcurrentG);
     addDebug(' Jbest:',JbestG);
     MemoDebug.Append('--------------------------');
     MemoDebug.Append(format(' Ref U_ref: %.2f %.2f %.2f',[U_refG.getv(0,0),U_refG.getv(1,0),U_refG.getv(2,0)]));
     MemoDebug.Append(format(' Ref U_best: %.2f %.2f %.2f',[UbestG.getv(0,0),UbestG.getv(1,0),UbestG.getv(2,0)]));
     MemoDebug.Append('');

end;

procedure TFormMPC.showDebug2();
begin
     MemoDebug.Clear;
     MemoDebug.Append(Format('Beta: %.2f',[OptData.beta]));

     MemoDebug.Append(Format('a1: %.2f',[OptData.a1]));
     MemoDebug.Append(Format('a2: %.2f',[OptData.a2]));
     MemoDebug.Append(Format('a3: %.2f',[OptData.a3]));
     MemoDebug.Append(Format('b1: %.2f',[OptData.b1]));
     MemoDebug.Append(Format('b2: %.2f',[OptData.b2]));
     MemoDebug.Append(Format('b3: %.2f',[OptData.b3]));

     addDebug(' J:',JcurrentG);
     addDebug(' Jbest:',JbestG);
     //MemoDebug.Append('--------------------------');
     //MemoDebug.Append(format(' Ref U_ref: %.2f %.2f %.2f',[U_refG.getv(0,0),U_refG.getv(1,0),U_refG.getv(2,0)]));
     //MemoDebug.Append(format(' Ref U_best: %.2f %.2f %.2f',[UbestG.getv(0,0),UbestG.getv(1,0),UbestG.getv(2,0)]));
     //MemoDebug.Append('');

end;

//----------------------------------------------------------------------
//
// Cost function definitions  for formations
//
//----------------------------------------------------------------------

function CalcIntersectionPoint(x1,y1,x2,y2: double; var vx,vy: double ; obs: TRObstacle): boolean;
var p,q,ma,mb,a,b,c,k,delta: double;
begin
  p:=x2-x1;
  q:=y2-y1;

  ma:=x1-obs.x;
  mb:=y1-obs.y;

  a:=p*p+q*q;
  b:=2*ma*p+2*mb*q;
  c:=ma*ma+mb*mb-obs.r*obs.r;

  delta:=b*b-4*a*c;

  if delta<0 then begin
    result:=false;
    exit;
  end;

  delta:=sqrt(delta)/(2*a);

  k:=-b/(2*a);
  if (k-delta>1) or (k+delta<0) then begin
    result:=false;
    exit;
  end;
  k:=k-delta;

  vx:=x1+k*p;
  vy:=y1+k*q;

  result:=true;
end;

// checks for obstacles in a segment
function TFormMPC.ObstacleInSegment(st_x,st_y,tg_x,tg_y: double; var obs: array of TRObstacle; num_obs: integer): boolean;
var ix,iy: double;
    i: integer;
begin
  result := true;
  for i:=0 to Num_obs-1 do begin
    // if we are inside the obstacle, then we definitely intersect
    if (dist(st_x-obs[i].x,st_y-obs[i].y)<obs[i].r) then exit;
    if CalcIntersectionPoint(st_x,st_y,tg_x,tg_y,ix,iy,obs[i]) then exit;
  end;
  result := false;
end;

//----------------------------------------------------------------------------
//
// MPCcontroller()
//
//----------------------------------------------------------------------------
// Main loop for predictive controller
//
//----------------------------------------------------------------------------
procedure TFormMPC.MPCcontroller(ThisRobotState: TRobotState; var traj: TTrajectory; avoid: TAvoidSet; vRefModule: double; var v,vn,w : double);
var obs: array[0..MaxRobots+MaxObstacles] of TRObstacle;  //Acke provisório o mais 70
    nobs: integer;
    Uaux,Uref,Usteps,Ubest: TDMatrix;
    Jprev,Jcurrent,Jbest,J,dmin,d,tetaaux : double;
    aux,i,k, m, g,l : integer;
    algTime : longword;
    refTraj : TTrajectory;
    v1,v2,v3 : double;
    initMotorSpeeds : array [0..2] of double;
    tau,r1: double;
    currentV : double;
begin

     //---------------------------------------------------
     //              Obstacle List
     //---------------------------------------------------

      nobs := 0;

      if avoidObstacles in avoid then begin
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


     //EXPERIMENTAL
     currentV:=sqrt(power(ThisRobotState.v,2)+power(ThisRobotState.vn,2));

     //Calculate optimal tau
     if CheckBoxTauLine.Checked then begin
        EditTao.Enabled:=false;
        tau := EditToFloatDef(EditTauLineK,1)*currentV + EditToFloatDef(EditTauLineB,0);
        //limit minimal tau
        if tau < 0.4 then
           tau := 0.4;
        EditTao.Text:=format('%.2f',[tau]);
     end else begin
         EditTao.Enabled:=true;
     end;

     //initial time
     algTime := GetTickCount;
     AlgItCount:=AlgItCount+1;

     //Load optimizer and simulation parameters
     loadConfig();
     loadParameters(EditToFloatDef(EditVMax,2),EditToFloatDef(EditTao,1));
     resetModel();

     //U vectors and matrixes
     Uaux.SetSize(3,SimParameters.Nu);
     Uref.SetSize(3,SimParameters.Nu);
     Usteps.SetSize(3,SimParameters.Nu*6);
     Ubest.setSize(3,SimParameters.Nu);

     //Set initial reference input as current robot speed reference
     for i:= 0 to (SimParameters.Nu-1) do begin
       Uref.setv(0,i,ThisRobotState.sv);    //v
       if FormMPC.difCB.Checked=true then begin
           Uref.setv(1,i,0);   //vn
       end else begin
           Uref.setv(1,i,ThisRobotState.svn);   //vn
       end;
       Uref.setv(2,i,ThisRobotState.sw);    //w
     end;

     //Get current motor speeds (estimated from speed references)
     if FormMPC.difCB.Checked=true then begin
         IKd(ThisRobotState.v,ThisRobotState.w,v1,v2);
     end else begin
         IK(ThisRobotState.v,ThisRobotState.vn,ThisRobotState.w,v1,v2,v3);
     end;
     //Current state of SimRobot is actual robot
     //(keep motor speeds, as they are updated at the end of the mpc loop)
     simRobot.RobotState.x := ThisRobotState.x;
     simRobot.RobotState.y := ThisRobotState.y;
     simRobot.RobotState.teta := ThisRobotState.teta;
     simRobot.RobotState.v:=ThisRobotState.v;
     if FormMPC.difCB.Checked=true then begin
        simRobot.RobotState.vn:=0;
     end else begin
        simRobot.RobotState.vn:=ThisRobotState.vn;
     end;
     simRobot.RobotState.w:=ThisRobotState.w;
     simRobot.MotorState[0].v := v1;
     simRobot.MotorState[1].v := v2;
     if FormMPC.difCB.Checked=true then begin
        simRobot.MotorState[2].v := 0;
     end else begin
        simRobot.MotorState[2].v := v3;
     end;

     //Current state of simulated ball
     SimBall.BallState.x := BallState.x;
     SimBall.BallState.y := BallState.y;
     SimBall.BallState.vx := BallState.vx;
     SimBall.BallState.vy := BallState.vy;
     SimBall.sigmaS:=sigmaT;

     //Current state of remaining bots in formation
     for k:= 0 to 5 do begin
         SimFormationRobots[k].RobotState.x := RobotState[k].x;
         SimFormationRobots[k].RobotState.y := RobotState[k].y;
         SimFormationRobots[k].RobotState.vx := RobotState[k].vx;
         SimFormationRobots[k].RobotState.vy := RobotState[k].vy;
         SimFormationRobots[k].RobotState.v := RobotState[k].v;
         SimFormationRobots[k].RobotState.vn := RobotState[k].vn;
         SimFormationRobots[k].RobotState.w := RobotState[k].w;
     end;

     //save initial motor speeds
     initMotorSpeeds[0] := SimRobot.MotorState[0].v;
     initMotorSpeeds[1] := SimRobot.MotorState[1].v;
     initMotorSpeeds[2] := SimRobot.MotorState[2].v;

     //if speed is variable get speed reference from trajectory points
     if traj.varSpeed then begin
        vRefModule:= traj.pts[traj.index].vRef;
     end;

     //Calculate reference trajectory for this step
     calcRefTraj(traj,refTraj,vRefModule);

     //Limit wheel speed references in case of motor saturation (update references)
     scaleForSaturation(Uref);

     //Base cost values
     Jcurrent := FormationDefs[FormationSettings.formation].func(SimRobot,SimBall,obs,nobs,ThisRobotState,Uref);

     Jprev := Jcurrent + 1;
     Jbest := Jcurrent;

     //---------------------------------------------------
     //              Optimization loop
     //---------------------------------------------------
     while (OptData.iterationCount < 5) or (OptData.iterationCount < OptParameters.MaxIterations) and (Jcurrent > OptParameters.Jstop) do begin

           //get Usteps matrix
           calcUSteps(Uref,Usteps);

           //Calculate Jsteps vector (do one simulation for each input set)
           for k:= 0 to (SimParameters.Nu-1) do begin
             for i:= 0 to 5 do begin

                 //get current U for simulation from Usteps
                 for m := 0 to (SimParameters.Nu-1) do begin
                     if m=k then begin
                       Uaux.setv(0,m,Usteps.getv(0,i+6*k));
                       Uaux.setv(1,m,Usteps.getv(1,i+6*k));
                       Uaux.setv(2,m,Usteps.getv(2,i+6*k));
                     end else begin
                       Uaux.setv(0,m,Uref.getv(0,0));
                       Uaux.setv(1,m,Uref.getv(1,0));
                       Uaux.setv(2,m,Uref.getv(2,0))
                     end;
                 end;

                 //Reset robot initial state for each simulation
                 simRobot.RobotState.x := ThisRobotState.x;
                 simRobot.RobotState.y := ThisRobotState.y;
                 simRobot.RobotState.teta := ThisRobotState.teta;
                 simRobot.RobotState.v:=ThisRobotState.v;
                 simRobot.RobotState.vn:=ThisRobotState.vn;
                 simRobot.RobotState.w:=ThisRobotState.w;
                 SimRobot.MotorState[0].v := initMotorSpeeds[0];
                 SimRobot.MotorState[1].v := initMotorSpeeds[1];
                 SimRobot.MotorState[2].v := initMotorSpeeds[2];

                 //Reset ball state
                 SimBall.BallState.x := BallState.x;
                 SimBall.BallState.y := BallState.y;
                 SimBall.BallState.vx := BallState.vx;
                 SimBall.BallState.vy := BallState.vy;
                 SimBall.sigmaS:=sigmaT;

                 //Limit wheel speed references in case of motor saturation (update references)
                 scaleForSaturation(Uaux);

                 //Do simulation with current Uaux and add to Jsteps vector
                 //Switches between trajectory controller and formation controller
                 J := FormationDefs[FormationSettings.formation].func(SimRobot,SimBall,obs,nobs,ThisRobotState,Uaux);

                 //Add J to Jsteps
                 OptData.Jsteps.setv(i,0,J);
             end;

           end;

           //Compute gradient of J from Jsteps
           calcGradient(OptData.Jsteps, OptData.Jgradient, OptData.Jgradient_prev);

           //Minimization algorithm
           calcRPROPStep(OptData,Uref);

           //previous costs and inputs
           Jprev := Jcurrent;

           //Reset robot initial position
           simRobot.RobotState.x := ThisRobotState.x;
           simRobot.RobotState.y := ThisRobotState.y;
           simRobot.RobotState.teta := ThisRobotState.teta;
           simRobot.RobotState.v:=ThisRobotState.v;
           simRobot.RobotState.vn:=ThisRobotState.vn;
           simRobot.RobotState.w:=ThisRobotState.w;
           SimRobot.MotorState[0].v := initMotorSpeeds[0];
           SimRobot.MotorState[1].v := initMotorSpeeds[1];
           SimRobot.MotorState[2].v := initMotorSpeeds[2];

           //Reset ball state
           SimBall.BallState.x := BallState.x;
           SimBall.BallState.y := BallState.y;
           SimBall.BallState.vx := BallState.vx;
           SimBall.BallState.vy := BallState.vy;
           SimBall.sigmaS:=sigmaT;

           //Limit wheel speed references in case of motor saturation (update references)
           scaleForSaturation(Uref);

           //Calculate new current cost (do simulation)
           Jcurrent := FormationDefs[FormationSettings.formation].func(SimRobot,SimBall,obs,nobs,ThisRobotState,Uref);


           //Update JBest
           if Jcurrent < Jbest then begin
              Jbest := Jcurrent;
              Ubest.setV(0,0,Uref.getv(0,0));
              if FormMPC.difCB.Checked then begin
                 Ubest.setV(1,0,0);
              end else begin
                 Ubest.setV(1,0,Uref.getv(1,0));
              end;
              Ubest.setV(2,0,Uref.getv(2,0));
           end;

           OptData.iterationCount += 1;

     end;


     //get algorithm execution time
     algTime := GetTickCount-algTime;

     //min and max values for execution time
     if algtime > AlgTimeMax then
        algTimeMax := AlgTime;
     if algTime < AlgTimeMin then
        algTimeMin := algTime;

     //sum of algtime values for mean calculation
     AlgTimeSum := AlgTimeSum + algTime;

     V := Ubest.getv(0,0);
     Vn := Ubest.getv(1,0);
     w := Ubest.getv(2,0);

     siga:=sigver.getv(0,0);
     sigb:=sigver.getv(0,1);
     sigc:=sigver.getv(1,0);
     sigd:=sigver.getv(1,1);

     //global variables for debugging
     algTimeG:=algTime;
     JcurrentG:=Jcurrent;
     JbestG:=Jbest;
     U_refG:=Uref;
     UbestG:=Ubest;
end;

//----------------------------------------------------------------------------
//
// MPCcontrollerFollower()
//
//----------------------------------------------------------------------------
// Main loop for predictive controller used by the Followers on Search
//
//----------------------------------------------------------------------------
procedure TFormMPC.MPCcontrollerFollower(ThisRobotState: TRobotState; var traj: TTrajectory; avoid: TAvoidSet; vRefModule: double; var v,vn,w : double);
var obs: array[0..MaxRobots+MaxOponents+7+MaxObstacles] of TRObstacle;  //Acke provisório o mais 70
    nobs: integer;
    Uaux,Uref,Usteps,Ubest: TDMatrix;
    Jprev,Jcurrent,Jbest,J: double;
    i,k,m,g,l : integer;
    algTime : longword;
    refTraj : TTrajectory;
    v1,v2,v3 : double;
    initMotorSpeeds : array [0..2] of double;
    tau: double;
    currentV : double;
begin

     //---------------------------------------------------
     //              Obstacle List
     //---------------------------------------------------

      nobs := 0;

      if avoidObstacles in avoid then begin
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

     //EXPERIMENTAL
     currentV:=sqrt(power(ThisRobotState.v,2)+power(ThisRobotState.vn,2));

     //Calculate optimal tau
     if CheckBoxTauLine.Checked then begin
        EditTao.Enabled:=false;
        tau := EditToFloatDef(EditTauLineK,1)*currentV + EditToFloatDef(EditTauLineB,0);
        //limit minimal tau
        if tau < 0.4 then
           tau := 0.4;
        EditTao.Text:=format('%.2f',[tau]);
     end else begin
         EditTao.Enabled:=true;
     end;

     //initial time
     algTime := GetTickCount;
     AlgItCount:=AlgItCount+1;

     //Load optimizer and simulation parameters
     loadConfig();
     loadParameters(EditToFloatDef(EditVMax,2),EditToFloatDef(EditTao,1));
     resetModel();

     //U vectors and matrixes
     Uaux.SetSize(3,SimParameters.Nu);
     Uref.SetSize(3,SimParameters.Nu);
     Usteps.SetSize(3,SimParameters.Nu*6);
     Ubest.setSize(3,SimParameters.Nu);

     //Set initial reference input as current robot speed reference
     for i:= 0 to (SimParameters.Nu-1) do begin
       Uref.setv(0,i,ThisRobotState.sv);    //v
       if FormMPC.difCB.Checked=true then begin
           Uref.setv(1,i,0);   //vn
       end else begin
           Uref.setv(1,i,ThisRobotState.svn);   //vn
       end;
       Uref.setv(2,i,ThisRobotState.sw);    //w
     end;

     //Get current motor speeds (estimated from speed references)
     if FormMPC.difCB.Checked=true then begin
         IKd(ThisRobotState.v,ThisRobotState.w,v1,v2);
     end else begin
         IK(ThisRobotState.v,ThisRobotState.vn,ThisRobotState.w,v1,v2,v3);
     end;
     //Current state of SimRobot is actual robot
     //(keep motor speeds, as they are updated at the end of the mpc loop)
     simRobot.RobotState.x := ThisRobotState.x;
     simRobot.RobotState.y := ThisRobotState.y;
     simRobot.RobotState.teta := ThisRobotState.teta;
     simRobot.RobotState.v:=ThisRobotState.v;
     if FormMPC.difCB.Checked=true then begin
        simRobot.RobotState.vn:=0;
     end else begin
        simRobot.RobotState.vn:=ThisRobotState.vn;
     end;
     simRobot.RobotState.w:=ThisRobotState.w;
     simRobot.MotorState[0].v := v1;
     simRobot.MotorState[1].v := v2;
     if FormMPC.difCB.Checked=true then begin
        simRobot.MotorState[2].v := 0;
     end else begin
        simRobot.MotorState[2].v := v3;
     end;

     //Current state of remaining bots in formation
     for k:= 0 to 5 do begin
         SimFormationRobots[k].RobotState.x := RobotState[k].x;
         SimFormationRobots[k].RobotState.y := RobotState[k].y;
         SimFormationRobots[k].RobotState.vx := RobotState[k].vx;
         SimFormationRobots[k].RobotState.vy := RobotState[k].vy;
         SimFormationRobots[k].RobotState.v := RobotState[k].v;
         SimFormationRobots[k].RobotState.vn := RobotState[k].vn;
         SimFormationRobots[k].RobotState.w := RobotState[k].w;
     end;

     //save initial motor speeds
     initMotorSpeeds[0] := SimRobot.MotorState[0].v;
     initMotorSpeeds[1] := SimRobot.MotorState[1].v;
     initMotorSpeeds[2] := SimRobot.MotorState[2].v;

     //Limit wheel speed references in case of motor saturation (update references)
     scaleForSaturation(Uref);

     //Base cost values
     Jcurrent := GeneralCostFunctionFollower(SimRobot,obs,nobs,ThisRobotState,Uref);

     Jprev := Jcurrent + 1;
     Jbest := Jcurrent;

     //---------------------------------------------------
     //              Optimization loop
     //---------------------------------------------------
     while (OptData.iterationCount < 5) or (OptData.iterationCount < OptParameters.MaxIterations) and (Jcurrent > OptParameters.Jstop) do begin

           //get Usteps matrix
           calcUSteps(Uref,Usteps);

           //Calculate Jsteps vector (do one simulation for each input set)
           for k:= 0 to (SimParameters.Nu-1) do begin
             for i:= 0 to 5 do begin

                 //get current U for simulation from Usteps
                 for m := 0 to (SimParameters.Nu-1) do begin
                     if m=k then begin
                       Uaux.setv(0,m,Usteps.getv(0,i+6*k));
                       Uaux.setv(1,m,Usteps.getv(1,i+6*k));
                       Uaux.setv(2,m,Usteps.getv(2,i+6*k));
                     end else begin
                       Uaux.setv(0,m,Uref.getv(0,0));
                       Uaux.setv(1,m,Uref.getv(1,0));
                       Uaux.setv(2,m,Uref.getv(2,0))
                     end;
                 end;

                 //Reset robot initial state for each simulation
                 simRobot.RobotState.x := ThisRobotState.x;
                 simRobot.RobotState.y := ThisRobotState.y;
                 simRobot.RobotState.teta := ThisRobotState.teta;
                 simRobot.RobotState.v:=ThisRobotState.v;
                 simRobot.RobotState.vn:=ThisRobotState.vn;
                 simRobot.RobotState.w:=ThisRobotState.w;
                 SimRobot.MotorState[0].v := initMotorSpeeds[0];
                 SimRobot.MotorState[1].v := initMotorSpeeds[1];
                 SimRobot.MotorState[2].v := initMotorSpeeds[2];

                 //Limit wheel speed references in case of motor saturation (update references)
                 scaleForSaturation(Uaux);

                 //Do simulation with current Uaux and add to Jsteps vector
                 //Switches between trajectory controller and formation controller
                 J := GeneralCostFunctionFollower(SimRobot,obs,nobs,ThisRobotState,Uaux);

                 //Add J to Jsteps
                 OptData.Jsteps.setv(i,0,J);
             end;

           end;

           //Compute gradient of J from Jsteps
           calcGradient(OptData.Jsteps, OptData.Jgradient, OptData.Jgradient_prev);

           //Minimization algorithmo
           //TESTAR DEPOIS COM GRADIENTE CONJUGADO
           calcRPROPStep(OptData,Uref);

           //previous costs and inputs
           Jprev := Jcurrent;

           //Reset robot initial position
           simRobot.RobotState.x := ThisRobotState.x;
           simRobot.RobotState.y := ThisRobotState.y;
           simRobot.RobotState.teta := ThisRobotState.teta;
           simRobot.RobotState.v:=ThisRobotState.v;
           simRobot.RobotState.vn:=ThisRobotState.vn;
           simRobot.RobotState.w:=ThisRobotState.w;
           SimRobot.MotorState[0].v := initMotorSpeeds[0];
           SimRobot.MotorState[1].v := initMotorSpeeds[1];
           SimRobot.MotorState[2].v := initMotorSpeeds[2];

           //Limit wheel speed references in case of motor saturation (update references)
           scaleForSaturation(Uref);

           //Calculate new current cost (do simulation)
           Jcurrent := GeneralCostFunctionFollower(SimRobot,obs,nobs,ThisRobotState,Uref);

           //Update JBest
           if Jcurrent < Jbest then begin
              Jbest := Jcurrent;
              Ubest.setV(0,0,Uref.getv(0,0));
              if FormMPC.difCB.Checked then begin
                 Ubest.setV(1,0,0);
              end else begin
                 Ubest.setV(1,0,Uref.getv(1,0));
              end;
              Ubest.setV(2,0,Uref.getv(2,0));
           end;

           OptData.iterationCount += 1;

     end;


     //get algorithm execution time
     algTime := GetTickCount-algTime;

     //min and max values for execution time
     if algtime > AlgTimeMax then
        algTimeMax := AlgTime;
     if algTime < AlgTimeMin then
        algTimeMin := algTime;

     //sum of algtime values for mean calculation
     AlgTimeSum := AlgTimeSum + algTime;

     //Set best output
     V := Ubest.getv(0,0);
     Vn := Ubest.getv(1,0);
     w := Ubest.getv(2,0);

     //global variables for debugging
     algTimeG:=algTime;
     JcurrentG:=Jcurrent;
     JbestG:=Jbest;
     U_refG:=Uref;
     UbestG:=Ubest;
end;


//----------------------------------------------------------------------------
//
// MPCcontrollerFollowTraj()
//
//----------------------------------------------------------------------------
// Main loop for predictive controller
//
//----------------------------------------------------------------------------
procedure TFormMPC.MPCcontrollerFollowTraj(ThisRobotState : TRobotState; var traj : TTrajectory; vRefModule: double; var v,vn,w : double);
var Uaux,Uref,Usteps,Ubest: TDMatrix;
    Jprev,Jcurrent,Jbest,J: double;
    i,k,m,g : integer;
    algTime : longword;
    refTraj : TTrajectory;
    v1,v2,v3 : double;
    initMotorSpeeds : array [0..2] of double;
    tau: double;
    currentV : double;
begin

     //EXPERIMENTAL
     currentV:=sqrt(power(ThisRobotState.v,2)+power(ThisRobotState.vn,2));

     //Calculate optimal tau
     if CheckBoxTauLine.Checked then begin
        EditTao.Enabled:=false;
        tau := EditToFloatDef(EditTauLineK,1)*currentV + EditToFloatDef(EditTauLineB,0);
        //limit minimal tau
        if tau < 0.4 then
           tau := 0.4;
        EditTao.Text:=format('%.2f',[tau]);
     end else begin
         EditTao.Enabled:=true;
     end;

     //Load optimizer and simulation parameters
     loadConfig();
     loadParameters(EditToFloatDef(EditVMax,2),EditToFloatDef(EditTao,1));
     resetModel();

     //U vectors and matrixes
     Uaux.SetSize(3,SimParameters.Nu);
     Uref.SetSize(3,SimParameters.Nu);
     Usteps.SetSize(3,SimParameters.Nu*6);
     Ubest.setSize(3,SimParameters.Nu);

     //Set initial reference input as current robot speed reference
     for i:= 0 to (SimParameters.Nu-1) do begin
       Uref.setv(0,i,ThisRobotState.v);    //v
       Uref.setv(1,i,ThisRobotState.vn);   //vn
       Uref.setv(2,i,ThisRobotState.w);    //w
     end;

     //Get current motor speeds (estimated from speed references)
     IK(ThisRobotState.v,ThisRobotState.vn,ThisRobotState.w,v1,v2,v3);

     //Current state of SimRobot is actual robot
     //(keep motor speeds, as they are updated at the end of the mpc loop)
     simRobot.RobotState.x := ThisRobotState.x;
     simRobot.RobotState.y := ThisRobotState.y;
     simRobot.RobotState.teta := ThisRobotState.teta;
     simRobot.RobotState.v:=ThisRobotState.v;
     simRobot.RobotState.vn:=ThisRobotState.vn;
     simRobot.RobotState.w:=ThisRobotState.w;
     simRobot.MotorState[0].v := v1;
     simRobot.MotorState[1].v := v2;
     simRobot.MotorState[2].v := v3;

     //Current state of simulated ball
     SimBall.BallState.x := BallState.x;
     SimBall.BallState.y := BallState.y;
     SimBall.BallState.vx := BallState.vx;
     SimBall.BallState.vy := BallState.vy;

     //Current state of remaining bots in formation
     for k:= 0 to 5 do begin
         SimFormationRobots[k].RobotState.x := RobotState[k].x;
         SimFormationRobots[k].RobotState.y := RobotState[k].y;
         SimFormationRobots[k].RobotState.vx := RobotState[k].vx;
         SimFormationRobots[k].RobotState.vy := RobotState[k].vy;
         SimFormationRobots[k].RobotState.v := RobotState[k].v;
         SimFormationRobots[k].RobotState.vn := RobotState[k].vn;
         SimFormationRobots[k].RobotState.w := RobotState[k].w;
     end;

     //save initial motor speeds
     initMotorSpeeds[0] := SimRobot.MotorState[0].v;
     initMotorSpeeds[1] := SimRobot.MotorState[1].v;
     initMotorSpeeds[2] := SimRobot.MotorState[2].v;

     //if speed is variable get speed reference from trajectory points
     if traj.varSpeed then begin
        vRefModule:= traj.pts[traj.index].vRef;
     end;

     //Calculate reference trajectory for this step
     calcRefTraj(traj,refTraj,vRefModule);

     //Limit wheel speed references in case of motor saturation (update references)
     scaleForSaturation(Uref);

     //Base cost values
     Jcurrent := TrajCostFunction(SimRobot,ThisRobotState,Uref,refTraj);


     Jprev := Jcurrent + 1;
     Jbest := Jcurrent;

     //---------------------------------------------------
     //              Optimization loop
     //---------------------------------------------------
     while (OptData.iterationCount < 5) or (OptData.iterationCount < OptParameters.MaxIterations) and (Jcurrent > OptParameters.Jstop) do begin

           //get Usteps matrix
           calcUSteps(Uref,Usteps);

           //Calculate Jsteps vector (do one simulation for each input set)
           for k:= 0 to (SimParameters.Nu-1) do begin
             for i:= 0 to 5 do begin

                 //get current U for simulation from Usteps
                 for m := 0 to (SimParameters.Nu-1) do begin
                     if m=k then begin
                       Uaux.setv(0,m,Usteps.getv(0,i+6*k));
                       Uaux.setv(1,m,Usteps.getv(1,i+6*k));
                       Uaux.setv(2,m,Usteps.getv(2,i+6*k));
                     end else begin
                       Uaux.setv(0,m,Uref.getv(0,0));
                       Uaux.setv(1,m,Uref.getv(1,0));
                       Uaux.setv(2,m,Uref.getv(2,0))
                     end;
                 end;


                 //Reset robot initial state for each simulation
                 simRobot.RobotState.x := ThisRobotState.x;
                 simRobot.RobotState.y := ThisRobotState.y;
                 simRobot.RobotState.teta := ThisRobotState.teta;
                 simRobot.RobotState.v:=ThisRobotState.v;
                 simRobot.RobotState.vn:=ThisRobotState.vn;
                 simRobot.RobotState.w:=ThisRobotState.w;
                 SimRobot.MotorState[0].v := initMotorSpeeds[0];
                 SimRobot.MotorState[1].v := initMotorSpeeds[1];
                 SimRobot.MotorState[2].v := initMotorSpeeds[2];

                 //Reset ball state
                 SimBall.BallState.x := BallState.x;
                 SimBall.BallState.y := BallState.y;
                 SimBall.BallState.vx := BallState.vx;
                 SimBall.BallState.vy := BallState.vy;

                 //Limit wheel speed references in case of motor saturation (update references)
                 scaleForSaturation(Uaux);

                 //Do simulation with current Uaux and add to Jsteps vector
                 //Switches between trajectory controller and formation controller
                  J := TrajCostFunction(SimRobot,ThisRobotState,Uaux,refTraj);
                 //Add J to Jsteps
                 OptData.Jsteps.setv(i,0,J);
             end;

           end;

           //Compute gradient of J from Jsteps
           calcGradient(OptData.Jsteps, OptData.Jgradient, OptData.Jgradient_prev);

           //Minimization algorithm
           calcConjugateCradientStep(OptData.Jsteps,OptData.Jgradient,OptData.Jgradient_prev,Uref);


           //previous costs and inputs
           Jprev := Jcurrent;

           //Reset robot initial position
           simRobot.RobotState.x := ThisRobotState.x;
           simRobot.RobotState.y := ThisRobotState.y;
           simRobot.RobotState.teta := ThisRobotState.teta;
           simRobot.RobotState.v:=ThisRobotState.v;
           simRobot.RobotState.vn:=ThisRobotState.vn;
           simRobot.RobotState.w:=ThisRobotState.w;
           SimRobot.MotorState[0].v := initMotorSpeeds[0];
           SimRobot.MotorState[1].v := initMotorSpeeds[1];
           SimRobot.MotorState[2].v := initMotorSpeeds[2];

           //Reset ball state
           SimBall.BallState.x := BallState.x;
           SimBall.BallState.y := BallState.y;
           SimBall.BallState.vx := BallState.vx;
           SimBall.BallState.vy := BallState.vy;

          //Limit wheel speed references in case of motor saturation (update references)
          scaleForSaturation(Uref);

           //Calculate new current cost (do simulation)
           Jcurrent := TrajCostFunction(SimRobot,ThisRobotState,Uref,refTraj);

           //Update JBest
           if Jcurrent < Jbest then begin
              Jbest := Jcurrent;
              Ubest.setV(0,0,Uref.getv(0,0));
              Ubest.setV(1,0,Uref.getv(1,0));
              Ubest.setV(2,0,Uref.getv(2,0));
           end;

           OptData.iterationCount += 1;

     end;

         //Set best output
         V := Ubest.getv(0,0);
         Vn := Ubest.getv(1,0);
         w := Ubest.getv(2,0);


         //global variables for debugging
         JcurrentG:=Jcurrent;
         JbestG:=Jbest;
         //U_refG:=Uref;
         //UbestG:=Ubest;
end;

//----------------------------------------------------------------------------
//
// calcUSteps()
//
//----------------------------------------------------------------------------
// Generates the Ustep matrix, whose columns are used inputs to the simulator
// to calculate the Js vector
//----------------------------------------------------------------------------
procedure TFormMPC.calcUSteps(var U, deltaU : TDMatrix);
var
   i : integer;
begin

    for i:= 0 to (SimParameters.Nu-1) do begin
      //col 1 (v+d,vn,w)
      deltaU.setv(0,0+6*i,U.getv(0,i)+OptParameters.delta);
      deltaU.setv(1,0+6*i,U.getv(1,i));
      deltaU.setv(2,0+6*i,U.getv(2,i));
      //col 2 (v-d,vn,w)
      deltaU.setv(0,1+6*i,U.getv(0,i)-OptParameters.delta);
      deltaU.setv(1,1+6*i,U.getv(1,i));
      deltaU.setv(2,1+6*i,U.getv(2,i));
      //col 3 (v,vn+EditDelta,w)
      deltaU.setv(0,2+6*i,U.getv(0,i));
      deltaU.setv(1,2+6*i,U.getv(1,i)+OptParameters.delta);
      deltaU.setv(2,2+6*i,U.getv(2,i));
      //col 4 ...
      deltaU.setv(0,3+6*i,U.getv(0,i));
      deltaU.setv(1,3+6*i,U.getv(1,i)-OptParameters.delta);
      deltaU.setv(2,3+6*i,U.getv(2,i));
      //col 5 ...
      deltaU.setv(0,4+6*i,U.getv(0,0));
      deltaU.setv(1,4+6*i,U.getv(1,0));
      deltaU.setv(2,4+6*i,U.getv(2,0)+OptParameters.delta);
      //col 6 ...
      deltaU.setv(0,5+6*i,U.getv(0,i));
      deltaU.setv(1,5+6*i,U.getv(1,i));
      deltaU.setv(2,5+6*i,U.getv(2,i)-OptParameters.delta);
    end;
end;


//----------------------------------------------------------------------------
//
// calcGradient()
//
//---------------------------------------------------------------------------
// Calculates the gradient of the cost function from the Jsteps vector
//----------------------------------------------------------------------------
procedure TFormMPC.calcGradient(var Jsteps, Jgradient, Jgradient_prev : TDmatrix);
var
   i : integer;
begin

     for i := 0 to (SimParameters.Nu-1) do begin
         Jgradient_prev.setv(3*i+0,0,Jgradient.getv(3*i+0,0));
         Jgradient.setv(3*i+0,0,Jsteps.getv(6*i+0,0)-Jsteps.getv(6*i+1,0));


         Jgradient_prev.setv(3*i+1,0,Jgradient.getv(3*i+1,0));
         Jgradient.setv(3*i+1,0,Jsteps.getv(6*i+2,0)-Jsteps.getv(6*i+3,0));


         Jgradient_prev.setv(3*i+2,0,Jgradient.getv(3*i+2,0));
         Jgradient.setv(3*i+2,0,Jsteps.getv(6*i+4,0)-Jsteps.getv(6*i+5,0));

     end;
end;

//----------------------------------------------------------------------------
//
// calcSteepestDescentStep()
//
//---------------------------------------------------------------------------
// Calculates the new U after a minimization loop iteration (steepest descent
// method)
//----------------------------------------------------------------------------
procedure TFormMPC.calcSteepestDescentStep(var Jgradient, Uref : TDmatrix);
var   i : integer;
begin
     for i:= 0 to (SimParameters.Nu-1) do begin
       Uref.setv(0,i,Uref.getv(0,i) - OptParameters.alpha*Jgradient.getv(0 + 3*i,0));
       Uref.setv(1,i,Uref.getv(1,i) - OptParameters.alpha*Jgradient.getv(1 + 3*i,0));
       Uref.setv(2,i,Uref.getv(2,i) - OptParameters.alpha*Jgradient.getv(2 + 3*i,0));
     end;
end;


//----------------------------------------------------------------------------
//
// calcConjugateCradientStep()
//
//---------------------------------------------------------------------------
// Calculates the new U after a minimization loop iteration (conjugate gradient
// method)
//----------------------------------------------------------------------------
procedure TFormMPC.calcConjugateCradientStep(var Jsteps, Jgradient, Jgradient_prev, Uref : TDmatrix);
var   i,j : integer;
      beta,t1,t2,t3,a1,a2,a3,b1,b2,b3: float;
      d,x1: array [0..2] of float;

begin
     for i:= 0 to (SimParameters.Nu-1) do begin

           d[0]:=Jgradient.getv(0 + 3*i,0);
           d[1]:=Jgradient.getv(1 + 3*i,0);
           d[2]:=Jgradient.getv(2 + 3*i,0);


           x1[0]:=(Uref.getv(0,i) - OptParameters.alpha*d[0]);
           x1[1]:=(Uref.getv(1,i) - OptParameters.alpha*d[1]);
           x1[2]:=(Uref.getv(2,i) - OptParameters.alpha*d[2]);


           Jgradient_prev.setv(3*i+0,0,Jgradient.getv(3*i+0,0));
           Jgradient.setv(3*i+0,0,Jsteps.getv(6*i+0,0)-Jsteps.getv(6*i+1,0));
           Jgradient_prev.setv(3*i+1,0,Jgradient.getv(3*i+1,0));
           Jgradient.setv(3*i+1,0,Jsteps.getv(6*i+2,0)-Jsteps.getv(6*i+3,0));
           Jgradient_prev.setv(3*i+2,0,Jgradient.getv(3*i+2,0));
           Jgradient.setv(3*i+2,0,Jsteps.getv(6*i+4,0)-Jsteps.getv(6*i+5,0));

           if ((Jgradient.getv(0 + 3*i,0)>=strtofloat(EditJStop.text))or(Jgradient.getv(1 + 3*i,0)>=strtofloat(EditJStop.text))) then begin

             t1:=(Jgradient.getv(3*i+0,0)-Jgradient_prev.getv(3*i+0,0));
             t2:=(Jgradient.getv(3*i+1,0)-Jgradient_prev.getv(3*i+1,0));
             t3:=(Jgradient.getv(3*i+2,0)-Jgradient_prev.getv(3*i+2,0));

             a1:=(Jgradient.getv(0 + 3*i,0)*t1);    //a1
             a2:=(Jgradient.getv(1 + 3*i,0)*t2);    //a2
             a3:=(Jgradient.getv(2 + 3*i,0)*t3);    //a3

             b1:=(Jgradient_prev.getv(0 + 3*i,0)*Jgradient_prev.getv(0 + 3*i,0));    //b1
             b2:=(Jgradient_prev.getv(1 + 3*i,0)*Jgradient_prev.getv(1 + 3*i,0));    //b2
             b3:=(Jgradient_prev.getv(2 + 3*i,0)*Jgradient_prev.getv(2 + 3*i,0));    //b3

             beta:=((a1+a2+a3)/(b1+b2+b3));

             OptData.beta:=beta;
             OptData.a1:=a1;
             OptData.a2:=a2;
             OptData.a3:=a3;
             OptData.b1:=b1;
             OptData.b2:=b2;
             OptData.b3:=b3;

           end;

           Uref.setv(0,i,x1[0] + OptParameters.alpha*(-Jgradient.getv(0+3*i,0) + beta*Jgradient_prev.getv(0 + 3*i,0)));
           Uref.setv(1,i,x1[1] + OptParameters.alpha*(-Jgradient.getv(1+3*i,0) + beta*Jgradient_prev.getv(1 + 3*i,0)));
           Uref.setv(2,i,x1[2] + OptParameters.alpha*(-Jgradient.getv(2+3*i,0) + beta*Jgradient_prev.getv(2 + 3*i,0)));

        //end;
     end;
end;


//----------------------------------------------------------------------------
//
// calcRPROPStep()
//
//---------------------------------------------------------------------------
// Calculates the new U after a minimization loop iteration (RPROP
// method)
//----------------------------------------------------------------------------
procedure TFormMPC.calcRPROPStep(var OptData : TOptData; var Uref : TDmatrix);
var
   i, j : integer;
   currStep, currGrad, prevStep, prevGrad : double;
begin

     for i:= 0 to (SimParameters.Nu-1) do begin
         for j:= 0 to 2 do begin

             prevStep := OptData.Jstep_prev.getv(3*i+j,0);
             currGrad := OptData.Jgradient.getv(3*i+j,0);
             prevGrad := OptData.Jgradient_prev.getv(3*i+j,0);


             if prevGrad*currGrad > 0 then
                currStep := Min(prevStep*OptParameters.etaMinus,OptParameters.stepMax)
             else

             if prevGrad*currGrad <0 then
                currStep := Max(prevStep*OptParameters.etaPlus,OptParameters.stepMin)
             else
                currStep := prevStep;

             if currGrad*prevGrad < 0 then
                Uref.setv(j,i,Uref.getv(j,i)-prevStep)
             else

             if currGrad > 0 then
                Uref.setv(j,i,Uref.getv(j,i)-currStep)
             else
             if currGrad < 0 then
                Uref.setv(j,i,Uref.getv(j,i)+currStep)
             else
                Uref.setv(j,i,Uref.getv(j,i));


             OptData.Jstep_prev.setv(3*i+j,0,currStep);

         end;
     end;
end;

//----------------------------------------------------------------------------
//
// calcRefTraj()
//
//---------------------------------------------------------------------------
// Calculates reference trajectory for controller from given trajectory, desired
// speed, and robot parameters
//----------------------------------------------------------------------------
procedure TFormMPC.calcRefTraj(var traj,trajPred: TTrajectory; V : double);
var
   teta, dl, dtotal: double;
   dSegments, segmentSize : double;
   tetaInc : double;
   deltaD : double;
   i, locTrajIndex : integer;
begin

     //initializations
     i := 0;
     deltaD := V*0.04;
     locTrajIndex := traj.index;
     dTotal := 0;

     //angle of current segment to world
     teta := atan2(traj.pts[locTrajIndex+1].y-traj.pts[locTrajIndex].y,traj.pts[locTrajIndex+1].x-traj.pts[locTrajIndex].x);

     //distance of robot robot along the current segment
     dl := DistPointInLine(traj.pts[locTrajIndex].x,traj.pts[locTrajIndex].y,traj.pts[locTrajIndex+1].x,traj.pts[locTrajIndex+1].y,
           SimRobot.RobotState.x,SimRobot.RobotState.y);
     segmentSize := dist(traj.pts[locTrajIndex+1].x-traj.pts[locTrajIndex].x,traj.pts[locTrajIndex+1].y-traj.pts[locTrajIndex].y);

     //sum of distances from segments (of main trajectory)
     dSegments := segmentSize - dl;

     //initial point of predictive controller trajectory
     trajPred.pts[i].x:= traj.pts[locTrajIndex].x + dl*cos(teta);
     trajPred.pts[i].y:= traj.pts[locTrajIndex].y + dl*sin(teta);
     trajPred.pts[i].teta := SimRobot.RobotState.teta;

     //first tetaInc
     tetaInc := (traj.pts[locTrajIndex+1].teta - SimRobot.RobotState.teta)/(dSegments/deltaD);

     //build new trajectory
     for i:=1 to SimParameters.N2 do begin

         //reached the end of the trajectory
         if(locTrajIndex >= traj.count-1) then begin

             trajPred.pts[i].x := traj.pts[traj.count-1].x;
             trajPred.pts[i].y := traj.pts[traj.count-1].y;
             trajPred.pts[i].teta := traj.pts[traj.count-1].teta;

         end else begin

             //add trajectory points along current segment
             trajPred.pts[i].x := trajPred.pts[i-1].x + deltaD*cos(teta);
             trajPred.pts[i].y := trajPred.pts[i-1].y + deltaD*sin(teta);
             trajPred.pts[i].teta := trajPred.pts[i-1].teta+tetaInc;

             dTotal := dTotal + deltaD;

             //change segment of the main trajectory that's being tracked
             if dtotal >= dSegments then begin

                locTrajIndex:=locTrajIndex+1;
                teta := atan2(traj.pts[locTrajIndex+1].y-traj.pts[locTrajIndex].y,traj.pts[locTrajIndex+1].x-traj.pts[locTrajIndex].x);
                segmentSize := dist(traj.pts[locTrajIndex+1].x-traj.pts[locTrajIndex].x,traj.pts[locTrajIndex+1].y-traj.pts[locTrajIndex].y);

                //prevent division by zero
                if segmentSize = 0 then
                   segmentSize := 0.00000001;

                //add point (already in next segment)
                trajPred.pts[i].x := traj.pts[locTrajIndex].x + (dTotal-dSegments)*cos(teta);
                trajPred.pts[i].y := traj.pts[locTrajIndex].y + (dTotal-dSegments)*sin(teta);
                trajPred.pts[i].teta := traj.pts[locTrajIndex].teta;

                tetaInc := (traj.pts[locTrajIndex+1].teta - traj.pts[locTrajIndex].teta)/(segmentSize/deltaD);
                dSegments := dSegments + segmentSize;

                if i = 1 then
                   traj.index := traj.index+1;
                end;
         end;

     trajPred.count:=i+1;


     end;

   //handle teta references
   for i := 0 to SimParameters.N2 do begin
       if trajPred.pts[i].teta > pi then
              trajPred.pts[i].teta := -2*pi + trajPred.pts[i].teta;
   end;
end;


//-------------------------------------------------------------------------
// ProportionalSat
//
//-------------------------------------------------------------------------
// Keeps vector directions if norms have to be scaled
//-------------------------------------------------------------------------
procedure TFormMPC.proportionalSat(var v1,v2,v3: double; vmax: double);
var maxv,minv: double;
    scale,scalemax,scalemin: double;
begin
  maxv:=Max(v1,Max(v2,v3));
  minv:=Min(v1,Min(v2,v3));

  if maxv>vmax then scalemax:=maxv/vmax else scalemax:=1.0;
  if minv<-vmax then scalemin:=minv/(-vmax) else scalemin:=1.0;

  scale:=Max(scalemin,scalemax);

  v1:=v1/scale;
  v2:=v2/scale;
  v3:=v3/scale;
end;

//-------------------------------------------------------------------------
// scaleForSaturation()
//
//-------------------------------------------------------------------------
// Scales V,Vn,W to prevent saturation
//-------------------------------------------------------------------------
procedure TFormMPC.scaleForSaturation(var U : TDmatrix);
var
   v,vn,w,v1,v2,v3 : double;
   i: integer;
begin

     for i:= 0 to (SimParameters.Nu-1) do begin
           v:=U.getv(0,i);
           vn:=U.getv(1,i);
           w:=U.getv(2,i);

           IK(v,vn,w,v1,v2,v3);
           ProportionalSat(v1,v2,v3,ModelParams.maxWheelSpeed);
           DK(v1,v2,v3,v,vn,w);

           U.setv(0,i,v);
           U.setv(1,i,vn);
           U.setv(2,i,w);
     end;
end;

//----------------------------------------------------------------------
//formation: Generic Formation
//
// Generic cost function applied in any robot
//
//----------------------------------------------------------------------
function GeneralCostFunction(var Robot : TSimRobot; var Ball : TSimBall; var obs: array of TRObstacle; num_obs: integer; RobotState : array of TRobotState; var U : TDMatrix): double;
var
   sum_cost : double;
   deltaU : double;
   temp, temp1: double;
   lambtest: array[1..10] of double;
   lambA,lamb0,lamb1,lamb2,lamb3,lamb4,lamb5,vv,v1,vf,segteta,alfa,distancia,newteta,wnew,vnew,vnnew: double;
   i,j,m,k1,k2,k3,k4,t,mate : integer;
   v_ref,vn_ref,w_ref : double;
   v_real,vn_real,w_real,d,dettemp,aa,bb,cc,dd : double;
   U_limit,sgtemp,sgtemp1: TDMatrix;
begin
     sgtemp.SetSize(2,2);
     sgtemp:=Mzeros(2,2);
     sgtemp1.SetSize(2,2);
     sgtemp1:=Mzeros(2,2);

     //PARAMETER DEFINITION
     // 0 - distance from ball
     // 1 - number of mate robot in formation

     //initialize
     sum_cost := 0;

     //simulate robot progression for prediction instant, calculate J
     for i:= SimParameters.N1 to SimParameters.N2 do begin

         //change control signal for control horizon
         if i <= SimParameters.Nu then begin
              v_ref := U.getv(0,i-1);
              vn_ref := U.getv(1,i-1);
              w_ref := U.getv(2,i-1);
         end else begin
              v_ref := U.getv(0,SimParameters.Nu-1);
              vn_ref := U.getv(1,SimParameters.Nu-1);
              w_ref := U.getv(2,SimParameters.Nu-1);
         end;

         //Simulate behavior for the next 40 ms (simulations timestep is 10 ms)
         for j:= 0 to 3 do begin

             if FormMPC.CheckBoxSimDynamicsR.Checked then begin
                 simRobotComplete(Robot,v_ref,vn_ref,w_ref,v_real,vn_real,w_real);
             end else begin
                 v_real := v_ref;
                 vn_real := vn_ref;
                 w_real := w_ref;
             end;

             //Move simulated robot with v real values
             simRobotMovement(Robot,v_real,vn_real,w_real);

             //Simulates the movement of the mates in the formation (kinematics equations only)
             k1 := FormationSettings.formationMates[0];
             if k1>=0 then simRobotMovement(SimFormationRobots[k1],SimFormationRobots[k1].RobotState.v,SimFormationRobots[k1].RobotState.vn,SimFormationRobots[k1].RobotState.w);

             k2 := FormationSettings.formationMates[1];
             if k2>=0 then simRobotMovement(SimFormationRobots[k2],SimFormationRobots[k2].RobotState.v,SimFormationRobots[k2].RobotState.vn,SimFormationRobots[k2].RobotState.w);

             k3 := FormationSettings.formationMates[2];
             if k3>=0 then simRobotMovement(SimFormationRobots[k3],SimFormationRobots[k3].RobotState.v,SimFormationRobots[k3].RobotState.vn,SimFormationRobots[k3].RobotState.w);

             k4 := FormationSettings.formationMates[3];
             if k4>=0 then simRobotMovement(SimFormationRobots[k4],SimFormationRobots[k4].RobotState.v,SimFormationRobots[k4].RobotState.vn,SimFormationRobots[k4].RobotState.w);

             //Move simulated ball (function also reduces ball speed due to friction)
             simBallMovement(Ball);

         end;

             //Simulates the covariances in the formation
             sigtemp0:=SimCovariance(Robot,Ball,FormationSettings.parameters[0],FormationSettings.k1,FormationSettings.k2);
             sgtemp:=sigtemp0;
             if k1>=0 then begin
                sigtemp1:=SimCovariance2(Robot,SimFormationRobots[k1],Ball,FormationSettings.parameters[0],FormationSettings.k1,FormationSettings.k2);
                aa:=(sgtemp.getv(0,0)+sigtemp1.getv(0,0));    //a
                bb:=(sgtemp.getv(0,1)+sigtemp1.getv(0,1));    //b
                cc:=(sgtemp.getv(1,0)+sigtemp1.getv(1,0));    //c
                dd:=(sgtemp.getv(1,1)+sigtemp1.getv(1,1));    //d
                if ((aa*dd)-(bb*cc))<>0 then begin
                     sgtemp:=SimCovarianceSUM(sgtemp, sigtemp1);
                end;
             end;
             if k2>=0 then begin
                sigtemp2:=SimCovariance2(Robot,SimFormationRobots[k2],Ball,FormationSettings.parameters[0],FormationSettings.k1,FormationSettings.k2);
                aa:=(sgtemp.getv(0,0)+sigtemp2.getv(0,0));    //a
                bb:=(sgtemp.getv(0,1)+sigtemp2.getv(0,1));    //b
                cc:=(sgtemp.getv(1,0)+sigtemp2.getv(1,0));    //c
                dd:=(sgtemp.getv(1,1)+sigtemp2.getv(1,1));    //d
                if ((aa*dd)-(bb*cc))<>0 then begin
                     sgtemp:=SimCovarianceSUM(sgtemp, sigtemp2);
                end;
             end;
             if k3>=0 then begin
                sigtemp3:=SimCovariance2(Robot,SimFormationRobots[k3],Ball,FormationSettings.parameters[0],FormationSettings.k1,FormationSettings.k2);
                aa:=(sgtemp.getv(0,0)+sigtemp3.getv(0,0));    //a
                bb:=(sgtemp.getv(0,1)+sigtemp3.getv(0,1));    //b
                cc:=(sgtemp.getv(1,0)+sigtemp3.getv(1,0));    //c
                dd:=(sgtemp.getv(1,1)+sigtemp3.getv(1,1));    //d
                if ((aa*dd)-(bb*cc))<>0 then begin
                     sgtemp:=SimCovarianceSUM(sgtemp, sigtemp3);
                end;
             end;
             if k4>=0 then begin
                sigtemp4:=SimCovariance2(Robot,SimFormationRobots[k4],Ball,FormationSettings.parameters[0],FormationSettings.k1,FormationSettings.k2);
                aa:=(sgtemp.getv(0,0)+sigtemp4.getv(0,0));    //a
                bb:=(sgtemp.getv(0,1)+sigtemp4.getv(0,1));    //b
                cc:=(sgtemp.getv(1,0)+sigtemp4.getv(1,0));    //c
                dd:=(sgtemp.getv(1,1)+sigtemp4.getv(1,1));    //d
                if ((aa*dd)-(bb*cc))<>0 then begin
                     sgtemp:=SimCovarianceSUM(sgtemp, sigtemp4);
                end;
             end;


         //resulting matrix
         Ball.sigmaS.setv(0,0,sgtemp.getv(0,0));
         Ball.sigmaS.setv(0,1,sgtemp.getv(0,1));
         Ball.sigmaS.setv(1,0,sgtemp.getv(1,0));
         Ball.sigmaS.setv(1,1,sgtemp.getv(1,1));

         sigver.setv(0,0,Ball.sigmaS.getv(0,0));
         sigver.setv(0,1,Ball.sigmaS.getv(0,1));
         sigver.setv(1,0,Ball.sigmaS.getv(1,0));
         sigver.setv(1,1,Ball.sigmaS.getv(1,1));

         Ball.detsigma:=(Ball.sigmaS.getv(0,0)*Ball.sigmaS.getv(1,1))-(Ball.sigmaS.getv(0,1)*Ball.sigmaS.getv(1,0));


         //calculate errors and cost function

         //ball speed direction
         temp1 := sqrt(power(Ball.BallState.vx,2)+power(Ball.BallState.vy,2));        //module of ball speed
         FormationState.BallSpeedDir_y:=round(1000*Ball.BallState.vy)/(1000*temp1);
         FormationState.BallSpeedDir_x:=round(1000*Ball.BallState.vx)/(1000*temp1);

         //robot->ball direction
         temp :=sqrt(power((Robot.RobotState.x - Ball.BallState.x),2)+power((Robot.RobotState.y - Ball.BallState.y),2));
         FormationState.RobotBallPos_x:=(Ball.BallState.x-Robot.RobotState.x)/temp;
         FormationState.RobotBallPos_y:=(Ball.Ballstate.y-Robot.RobotState.y)/temp;

         //distance of robot from ball
         FormationState.RobotBallDist:=sqrt(power((Robot.RobotState.x - Ball.BallState.x),2)+power((Robot.RobotState.y - Ball.BallState.y),2));

         //angle between robot and ball
         FormationState.RobotBallAngle:= ATan2(FormationState.RobotBallPos_y,FormationState.RobotBallPos_x);

         //dot product between ball speed and Robot-Ball position vectors
         if (FormationSettings.pvalue<>0) then begin
            FormationState.RobotBallDotProduct := ((FormationSettings.pvalue)+(FormationState.BallSpeedDir_x*FormationState.RobotBallPos_x + FormationState.BallSpeedDir_y*FormationState.RobotBallPos_y));
         end else begin
            FormationState.RobotBallDotProduct := (FormationState.BallSpeedDir_x*FormationState.RobotBallPos_x + FormationState.BallSpeedDir_y*FormationState.RobotBallPos_y);
         end;

         //distance from the other bot in the formation
         if k1>=0 then begin
                FormationState.RobotFormationDistance[k1]:=sqrt(power((Robot.RobotState.x - SimFormationRobots[k1].RobotState.x),2)+power((Robot.RobotState.y - SimFormationRobots[k1].RobotState.y),2));
                mate:=mate+1;
         end;
         if k2>=0 then begin
                FormationState.RobotFormationDistance[k2]:=sqrt(power((Robot.RobotState.x - SimFormationRobots[k2].RobotState.x),2)+power((Robot.RobotState.y - SimFormationRobots[k2].RobotState.y),2));
                mate:=mate+1;
         end;
         if k3>=0 then begin
                FormationState.RobotFormationDistance[k3]:=sqrt(power((Robot.RobotState.x - SimFormationRobots[k3].RobotState.x),2)+power((Robot.RobotState.y - SimFormationRobots[k3].RobotState.y),2));
                mate:=mate+1;
         end;
         if k4>=0 then begin
                FormationState.RobotFormationDistance[k4]:=sqrt(power((Robot.RobotState.x - SimFormationRobots[k4].RobotState.x),2)+power((Robot.RobotState.y - SimFormationRobots[k4].RobotState.y),2));
                mate:=mate+1;
         end;
 //-->        //distance from the obstacles  - MODIFICAR PARA N OBSTÁCULOS
         for m:=0 to num_obs-1 do begin
             FormationState.RobotObstacleDistance[m]:=sqrt(power((Robot.RobotState.x - Obstacles.Centers[m].x),2)+power((Robot.RobotState.y - Obstacles.Centers[m].y),2))-0.5;
         end;

         lambA:=FormationSettings.weights[6];
         lamb0:=FormationSettings.weights[0];
         lamb3:=FormationSettings.weights[3];
         lamb4:=FormationSettings.weights[4];
         lamb5:=FormationSettings.weights[5];


         if (FormationState.RobotBallDist>4) then begin
              lamb1:=0;
              lamb2:=0;
         end else if (FormationState.RobotBallDist<1.5) then begin
              lamb1:=FormationSettings.weights[1];
              lamb2:=FormationSettings.weights[2];
         end else begin
              lamb1:=-(120*FormationState.RobotBallDist)+480;
              lamb2:=-(80*FormationState.RobotBallDist)+320;
         end;

         for m:=0 to num_obs-1 do begin
             if (FormationState.RobotObstacleDistance[m]<0.6) then begin
               lamb0:=7000;                          //distancia    8000
               lamb1:=10;                            //orientacao
               lamb2:=10;                            //posicionamento
               lamb3:=0.1;                           //esforço de controle
               lamb4:=15000;                         //colegas
               lamb5:=15000;                         //obstáculo  15000
             end else begin
               lamb0:=FormationSettings.weights[0];
               lamb1:=FormationSettings.weights[1];
               lamb2:=FormationSettings.weights[2];
               lamb3:=FormationSettings.weights[3];
               lamb4:=FormationSettings.weights[4];
               lamb5:=FormationSettings.weights[5];
             end;
         end;


         if (mate=0) then begin
            lambA:=0;
         end;

         //SUM COST
         sum_cost := sum_cost + lambA*abs(Ball.detsigma);

         //distance
         sum_cost := sum_cost + lamb0*abs((FormationSettings.parameters[0]-abs(FormationState.RobotBallDist)));    //distance from ball

         //robot orientation
         sum_cost := sum_cost + lamb1*abs(DiffAngle(FormationState.RobotBallAngle,Robot.RobotState.teta));       //angle facing ball

         //robot positioning
         sum_cost := sum_cost + lamb2*abs(FormationState.RobotBallDotProduct);     //follow alongside

         //mate avoidance
         if k1>=0 then sum_cost := sum_cost + lamb4*abs(1/(FormationState.RobotFormationDistance[k1]+1e-6));
         if k2>=0 then sum_cost := sum_cost + lamb4*abs(1/(FormationState.RobotFormationDistance[k2]+1e-6));
         if k3>=0 then sum_cost := sum_cost + lamb4*abs(1/(FormationState.RobotFormationDistance[k3]+1e-6));
         if k4>=0 then sum_cost := sum_cost + lamb4*abs(1/(FormationState.RobotFormationDistance[k4]+1e-6));


         //obstacle avoidance - PARA N OBSTÁCULOS
         for m:=0 to num_obs-1 do begin
             sum_cost := sum_cost + lamb5*abs(1/(FormationState.RobotObstacleDistance[m]+1e-6));
         end;
     end;

     //Output cost
     deltaU:= abs((Robot.RobotState.v-U.getv(0,0))) + abs((Robot.RobotState.vn-U.getv(1,0))) + abs((Robot.RobotState.w-U.getv(2,0)));
     result := sum_cost + lamb3*deltaU;

end;


//----------------------------------------------------------------------
//formation: Generic Formation Function for the Follower
//
// Generic cost function applied in any robot Following a Leader
//
//----------------------------------------------------------------------
function GeneralCostFunctionFollower(var Robot : TSimRobot; var obs: array of TRObstacle; num_obs: integer; RobotState : array of TRobotState; var U : TDMatrix): double;
var
   sum_cost : double;
   deltaU : double;
   temp: double;
   lambtest: array[1..10] of double;
   lamb0,lamb1,lamb2,lamb3,lamb4,lamb5,v1,vf,segteta,alfa,distancia,newteta,wnew,vnew,vnnew: double;
   i,j,m,k1,k2,k3,k4,t : integer;
   v_ref,vn_ref,w_ref : double;
   v_real,vn_real,w_real : double;
   U_limit : TDMatrix;
begin

     //initialize
     sum_cost := 0;

     //simulate robot progression for prediction instant, calculate J
     for i:= SimParameters.N1 to SimParameters.N2 do begin

         //change control signal for control horizon
         if i <= SimParameters.Nu then begin
              v_ref := U.getv(0,i-1);
              vn_ref := U.getv(1,i-1);
              w_ref := U.getv(2,i-1);
         end else begin
              v_ref := U.getv(0,SimParameters.Nu-1);
              vn_ref := U.getv(1,SimParameters.Nu-1);
              w_ref := U.getv(2,SimParameters.Nu-1);
         end;


         //Simulate behavior for the next 40 ms (simulations timestep is 10 ms)
         for j:= 0 to 3 do begin

             v_real := v_ref;
             vn_real := vn_ref;
             w_real := w_ref;

             //Move simulated robot with v real values
             simRobotMovement(Robot,v_real,vn_real,w_real);

             //Simulates the movement of the mates in the formation (kinematics equations only)
             k1 := FormationSettings.formationMates[0];
             if k1>=0 then simRobotMovement(SimFormationRobots[k1],SimFormationRobots[k1].RobotState.v,SimFormationRobots[k1].RobotState.vn,SimFormationRobots[k1].RobotState.w);

             k2 := FormationSettings.formationMates[1];
             if k2>=0 then simRobotMovement(SimFormationRobots[k2],SimFormationRobots[k2].RobotState.v,SimFormationRobots[k2].RobotState.vn,SimFormationRobots[k2].RobotState.w);

             k3 := FormationSettings.formationMates[2];
             if k3>=0 then simRobotMovement(SimFormationRobots[k3],SimFormationRobots[k3].RobotState.v,SimFormationRobots[k3].RobotState.vn,SimFormationRobots[k3].RobotState.w);

             k4 := FormationSettings.formationMates[3];
             if k4>=0 then simRobotMovement(SimFormationRobots[k4],SimFormationRobots[k4].RobotState.v,SimFormationRobots[k4].RobotState.vn,SimFormationRobots[k4].RobotState.w);

         end;

         //calculate errors and cost function

          //robot->Leader direction
         temp :=sqrt(power((Robot.RobotState.x - SimFormationRobots[k1].RobotState.x),2)+power((Robot.RobotState.y - SimFormationRobots[k1].RobotState.y),2));
         FormationState.RobotRobotPos_x:=(SimFormationRobots[k1].RobotState.x-Robot.RobotState.x)/temp;
         FormationState.RobotRobotPos_y:=(SimFormationRobots[k1].RobotState.y-Robot.RobotState.y)/temp;

         //angle between robot and Leader
         FormationState.RobotRobotAngle:= ATan2(FormationState.RobotRobotPos_y,FormationState.RobotRobotPos_x);

         //dot product between Leader Speed and Robot-Leader position vectors
         if (FormationSettings.pvalue<>0) then begin
            FormationState.RobotLeaderDotProduct:=((FormationSettings.pvalue)+(SimFormationRobots[k1].RobotState.vx*FormationState.RobotRobotPos_x+SimFormationRobots[k1].RobotState.vy*FormationState.RobotRobotPos_y));
         end else begin
            FormationState.RobotLeaderDotProduct := (SimFormationRobots[k1].RobotState.vx*FormationState.RobotRobotPos_x+SimFormationRobots[k1].RobotState.vy*FormationState.RobotRobotPos_y);
         end;

         //distance from the other bot in the formation
         if k1>=0 then FormationState.RobotFormationDistance[k1] :=sqrt(power((Robot.RobotState.x - SimFormationRobots[k1].RobotState.x),2)+power((Robot.RobotState.y - SimFormationRobots[k1].RobotState.y),2));
         if k2>=0 then FormationState.RobotFormationDistance[k2] :=sqrt(power((Robot.RobotState.x - SimFormationRobots[k2].RobotState.x),2)+power((Robot.RobotState.y - SimFormationRobots[k2].RobotState.y),2));
         if k3>=0 then FormationState.RobotFormationDistance[k3] :=sqrt(power((Robot.RobotState.x - SimFormationRobots[k3].RobotState.x),2)+power((Robot.RobotState.y - SimFormationRobots[k3].RobotState.y),2));
         if k4>=0 then FormationState.RobotFormationDistance[k4] :=sqrt(power((Robot.RobotState.x - SimFormationRobots[k4].RobotState.x),2)+power((Robot.RobotState.y - SimFormationRobots[k4].RobotState.y),2));

 //-->        //distance from the obstacles  - MODIFICAR PARA N OBSTÁCULOS
         for m:=0 to num_obs-1 do begin
             FormationState.RobotObstacleDistance[m]:=sqrt(power((Robot.RobotState.x - Obstacles.Centers[m].x),2)+power((Robot.RobotState.y - Obstacles.Centers[m].y),2))-0.6;
         end;

         lamb0:=FormationSettings.weights[0];
         lamb1:=FormationSettings.weights[1];
         lamb2:=FormationSettings.weights[2];
         lamb3:=FormationSettings.weights[3];
         lamb4:=FormationSettings.weights[4];
         lamb5:=FormationSettings.weights[5];

//-->         // PARA N OBSTÁCULOS
         for m:=0 to num_obs-1 do begin
             if (FormationState.RobotObstacleDistance[m]<0.6) then begin
               lamb0:=7000;                         //distancia    8000
               lamb1:=10;                           //orientacao
               lamb2:=10;                          //posicionamento
               lamb3:=0.1;                          //esforço de controle
               lamb4:=15000;                         //colegas
               lamb5:=15000;                        //obstáculo  15000
             end else begin
               lamb0:=FormationSettings.weights[0];
               lamb1:=FormationSettings.weights[1];
               lamb2:=FormationSettings.weights[2];
               lamb3:=FormationSettings.weights[3];
               lamb4:=FormationSettings.weights[4];
               lamb5:=FormationSettings.weights[5];
             end;
         end;

         //SUM COST
         //distance
         sum_cost := sum_cost + lamb0*abs((FormationSettings.dvalue)-FormationState.RobotFormationDistance[k1]);    //distance from Leader

         //robot orientation
         sum_cost := sum_cost + lamb1*abs(DiffAngle(FormationState.RobotRobotAngle,Robot.RobotState.teta));       //angle facing Leader

         //robot positioning
         sum_cost := sum_cost + lamb2*abs(FormationState.RobotLeaderDotProduct);     //follow alongside

         //mate avoidance
          if k2>=0 then sum_cost := sum_cost + lamb4*abs(1/(FormationState.RobotFormationDistance[k2]+1e-6));
         if k3>=0 then sum_cost := sum_cost + lamb4*abs(1/(FormationState.RobotFormationDistance[k3]+1e-6));
         if k4>=0 then sum_cost := sum_cost + lamb4*abs(1/(FormationState.RobotFormationDistance[k4]+1e-6));


 //-->        //obstacle avoidance - MODIFICAR PARA N OBSTÁCULOS
         for m:=0 to num_obs-1 do begin
             sum_cost := sum_cost + lamb5*abs(1/(FormationState.RobotObstacleDistance[m]+1e-6));
         end;
     end;

     //Output cost
     deltaU:= abs((Robot.RobotState.v-U.getv(0,0))) + abs((Robot.RobotState.vn-U.getv(1,0))) + abs((Robot.RobotState.w-U.getv(2,0)));
     result := sum_cost + lamb3*deltaU;

end;

//----------------------------------------------------------------------------
//
// TrajCostFunction()
//
//---------------------------------------------------------------------------
// Simulates the evolution of the moving robot across a path with given set
// of input values.
//
// RETURNS: J, the value of the cost function obtained with the current input
//          values.
//----------------------------------------------------------------------------

function TrajCostFunction(var Robot : TSimRobot; RobotState : array of TRobotState; var U : TDMatrix; refTraj: TTrajectory) : double;
var
   sum_costraj : double;
   deltaU : double;
   i,j : integer;
   lamb1,lamb2,lamb3,lamb4: double;
   v_ref,vn_ref,w_ref : double;
   v_real,vn_real,w_real : double;
   U_limit : TDMatrix;
begin

     lamb1:=SimParameters.lambda[0];
     lamb2:=SimParameters.lambda[1];
     //lamb3:=FormationSettings.weights[2];
     lamb4:=SimParameters.lambda[2];


     //initialize
     sum_costraj := 0;

     //simulate robot progression for prediction instant, calculate J
     for i:= SimParameters.N1 to SimParameters.N2 do begin

         //change control signal for control horizon
         if i <= SimParameters.Nu then begin
              v_ref := U.getv(0,i-1);
              vn_ref := U.getv(1,i-1);
              w_ref := U.getv(2,i-1);
         end else begin
              v_ref := U.getv(0,SimParameters.Nu-1);
              vn_ref := U.getv(1,SimParameters.Nu-1);
              w_ref := U.getv(2,SimParameters.Nu-1);
         end;

         //Simulate behavior for the next 40 ms (simulations timestep is 10 ms)
         for j:= 0 to 3 do begin

             //Dynamics simulation (v references -> v real)
             if FormMPC.CheckBoxSimDynamicsR.Checked then begin
                 simRobotComplete(Robot,v_ref,vn_ref,w_ref,v_real,vn_real,w_real);
             end else begin
                 v_real := v_ref;
                 vn_real := vn_ref;
                 w_real := w_ref;

             end;

             //Move simulated robot with v real values
             simRobotMovement(Robot,v_real,vn_real,w_real);
         end;

         //calculate errors
         sum_costraj := sum_costraj + lamb1*(power((refTraj.pts[i].x-Robot.RobotState.x),2) + power((refTraj.pts[i].y-Robot.RobotState.y),2));
         sum_costraj := sum_costraj + lamb2*power(DiffAngle(refTraj.pts[i].teta,Robot.RobotState.teta),2);

     end;

     deltaU:= abs(Robot.RobotState.v-U.getv(0,0)) + abs(Robot.RobotState.vn-U.getv(1,0)) + abs(Robot.RobotState.w-U.getv(2,0));

     //Output cost
     result := sum_costraj + lamb4*(deltaU);

end;   }


initialization
  {$I mpc.lrs}

end.

