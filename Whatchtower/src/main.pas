unit Main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, Forms, Controls, Graphics, Dialogs, ExtCtrls,
  Menus, IniPropStorage, StdCtrls, fuzzy, lNet,lNetComponents, dynmatrix,
  RoCConsts, DOM, XMLRead, IniFiles, Math, Roles;

//------- Types e Consts do WLan---------------//
type
  TPlayerRobotState = packed record
    x, y, teta: single;
    vx, vy, w: single;
    conf: single;
    Active: boolean;
  end;

  TPlayerObsState = packed record
    x, y: single;
    conf: single;
  end;

  TPlayerInfo = packed record
    Magic: LongWord;
    RobotState: TPlayerRobotState;
    ObsState: array [0..MaxRobots-1] of TPlayerObsState;
    num: byte;
    Batery:double;
  end;

  TCoachRobotState = packed record
    x, y, teta: single;
    vx, vy, w: single;
    conf: single;
    role: TRole;
    active: boolean;
  end;

  TCoachObsState = packed record
    x, y: single;
    conf: single;
  end;

  TCoachInfo = packed record
    Magic: LongWord;
    RobotState: array [0..MaxRobots-1] of TCoachRobotState;
    ObsStates:  array [0..MaxRobots-1] of TCoachObsState;
    Play: TPlay;
  end;

const
  Idcoach=6;
  MaxObstacles=70;
  NSim=30;
  MaxV=0.7;
  MaxW=3.14;
  ImgWidth=384;
  ImgHeight=288;
  MaxColors=8;
  CTrackColors=6;
  MaxSegLines=64;
  MaxEdges=96;
  MaxRegions=16;
  MaxOdos=2;
  MaxCenters=16;
  MaxRadarRays=64;

  iYellowGoal=1;
  iBlueGoal=2;
  iPurpleTeam=3;
  iCyanTeam=4;

  colorcolor: array[0..MaxColors-1] of word=($7E00,$7FE0,$001F,$5C1F,$03FF,$4210,$7FFF,$03E0);
  colorcolor24: array[0..MaxColors-1] of Tcolor=(clred,clyellow,clblue,clpurple,clnavy,clgray,clwhite,$00FF00);

  M_E=2.71828182845904523536028747135266;

  PACKET_COACH_MAGIC = $DEADBEEF;
  PACKET_PLAYER_MAGIC = $CAFEBABE;

type
  TGoalColor=(gcYellow,gcBlue);
  TTeamColor=(tcMagenta,tcCyan);

  iColorSet=set of iYellowGoal..iCyanTeam;

const
  CGoalColorStr: array[low(TGoalColor)..High(TGoalColor)] of string = ('Yellow','Blue');
  CTeamColorStr: array[low(TTeamColor)..High(TTeamColor)] of string = ('Magenta','Cyan');
  CGoalColorColor24: array[low(TGoalColor)..High(TGoalColor)] of Tcolor = (clYellow,clBlue);
  CTeamColorColor24: array[low(TTeamColor)..High(TTeamColor)] of Tcolor = ($00FF00FF,$00FFFF00);

type
  TPos=record
    x,y, teta: double;
  end;

type
  TTacticCommand=record
    v,w: double;
    v1,v2: double;
  end;

  Tcenter=record
    ro,teta,d: double;
    xw,yw: double;
    roMin,roMax,tetaMin,tetaMax: double;
    area: integer;
  end;

  TCenters= record
    data: array[0..MaxCenters-1] of Tcenter;
    count: integer;
  end;

  TEdge= record
    lineNum: integer;
    xi,yi: integer;
    ro,teta: double;
    d,xw,yw: double;
    color1,color2: integer;
    quality: integer;
  end;

  TRegion= record
    BestColor: integer;
    BestColorPer: integer;
    x1,y1,x2,y2: integer;
    phi,teta: double;
    xw,yw: double;
  end;

  TRadar= record
    lineNum: integer;
    color: integer;
    size: integer;
    teta,ro: double;
    xw,yw,d: double;
  end;

  TOdo= record
    speedw: array[0..1] of double;
    dwheel: array[0..1] of double;
    count: integer;
  end;

  TDOdos=record
    x:double;
    y:double;
    teta:double;
  end;

  TAbsOdo=record
    pos: array[0..1] of integer;
    count: integer;
  end;

  TView=record
    FrameTime: integer;
    SendTime: integer;
    Centers: array[0..cTrackColors-1] of Tcenters;

    Edges: array[0..MaxEdges-1] of TEdge;
    EdgesCount: integer;

    Regions: array[0..MaxRegions-1] of TRegion;
    RegionsCount: integer;

    Odos,FreeOdos: array[0..MaxOdos-1] of TOdo;
    OdosCount,OdosTotalCount: integer;
    DOdos:TDOdos;

    Radar: array[0..MaxRadarRays-1] of TRadar;
    RadarRaysCount: integer;

  end;

  TRobotState=record
    x,y,teta: double;
    Xk,Pk: TDMatrix;
    cov_x,cov_y,cov_xy,cov_teta: double;
    v,w: double;
    vx,vy: double;
    v1,v2: double;
    timestamp: integer;
    num: integer;
    count: integer;
    Vbatery:double;
  end;

 TObstacle=record
    xw,yw: double;
    x,y,r: double;
    color: integer;
    quality: double;
  end;

  TObstacles=record
    Centers:array[0..MaxObstacles-1] of TObstacle;
    count: integer;
  end;

  TEdgeLine=record
    iEdges: array [0..2] of integer;
    conf: double;
    teta,odist: double;
  end;

Const
  UDPBufSize=1024;

type
  TUDPBuffer= record
    data: array[0..UDPBufSize-1] of byte;
    MessSize, ReadDisp: integer;
  end;

type
  TRobotStateForm = record
    CBRobotActive, CBForceRobotActive: TCheckBox;
    EditRobotState: TEdit;
  end;

  TObsStates = record
    Obs: array[0..MaxRobots-1] of TObstacle;
  end;

type

  { TFMain }

  TFMain = class(TForm)
    CBFuzzy: TCheckBox;
    CBOn: TCheckBox;
    CBRobotActive1: TCheckBox;
    CBRobotActive2: TCheckBox;
    CBRobotActive3: TCheckBox;
    CBRobotActive4: TCheckBox;
    EditRobotAvailableCount: TEdit;
    EditRobotState1: TEdit;
    EditRobotState2: TEdit;
    EditRobotState3: TEdit;
    EditRobotState4: TEdit;
    EditVbatRobot1: TEdit;
    EditVbatRobot2: TEdit;
    EditVbatRobot3: TEdit;
    EditVbatRobot4: TEdit;
    FormStorage: TIniPropStorage;
    GBRobotInfo1: TGroupBox;
    GBRobotInfo2: TGroupBox;
    GBRobotInfo3: TGroupBox;
    GBRobotInfo4: TGroupBox;
    ImageMap: TPaintBox;
    Label2: TLabel;
    RGDecision: TRadioGroup;
    UDPSuper: TLUDPComponent;
    MainMenu: TMainMenu;
    MenuAbout: TMenuItem;
    MenuExit: TMenuItem;
    MenuField: TMenuItem;
    MenuFile: TMenuItem;
    MenuLog: TMenuItem;
    MenuParam: TMenuItem;
    MenuWindow: TMenuItem;
    RGBallSel: TRadioGroup;
    RGRobotSel: TRadioGroup;
    TimerDoTactic: TTimer;
    procedure FormCreate(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure ImageMapMouseDown(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure ImageMapMouseUp(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure MenuAboutClick(Sender: TObject);
    procedure MenuExitClick(Sender: TObject);
    procedure MenuFieldClick(Sender: TObject);
    procedure MenuLogClick(Sender: TObject);
    procedure MenuParamClick(Sender: TObject);

    procedure UDPCoachReceive(aSocket: TLSocket);
    procedure UDPSuperError(const msg: string; aSocket: TLSocket);
    procedure UDPSuperReceive(aSocket: TLSocket);
    procedure ShowAll;
    procedure MainLoop;
    procedure TimerDoTacticTimer(Sender: TObject);
    procedure ShowRobotTimes(var RS: TRobotstate; var RSF: TRobotStateForm; robnum:integer);

  private
    { private declarations }
  public
    { public declarations }
    DataDir: string;
    deb_txt: string;
    SLShowValues: TStrings;
    TempStream : TMemoryStream;
    Good_image: boolean;
    StrTream: TStringStream;
    down_x,down_y: integer;
    draw_x,draw_y: integer;
    NetTime: DWORD;

    procedure ProcessPlayerPacket(packet_str: string);
    procedure SendCoachInfo;
    procedure SetRobotStateForm;
    procedure ResetView;
  end;

var
  FMain: TFMain;

  LastData: string;
  Vsim, Wsim, Sum_err_stress_v,Sum_err_stress_w: double;
  Idx_erro: integer;
  Flag_stress,Flag_stress_ant: boolean;
  ref_v_ant, ref_w_ant : double;
  v_on_stress, w_on_stress: double;
  Buff_erro:array[0..Nsim-1,0..1] of double;
  walk_counter: integer;
  WGLine,GWLine: TEdgeLine;
  TeamColor: TTeamColor;
  LRState: array[0..MaxRobots-1] of TRobotState;
  View, LastView: TView;
  flagForm: boolean=false;
  sigver:TDMatrix;
  myNumber: integer;
  max_linear_acceleration:double;
  max_angular_acceleration:double;
  Obstacles: TObstacles;
  RobotStateForm: array[0..MaxRobots-1] of TRobotStateForm;
  ObsStates: array[0..MaxRobots-1] of TObsStates;
  ControlTimeStamp:LongWord;
  TacticCommands: array[0..MaxRobots-1] of TTacticCommand;
  RobotState: array[0..MaxRobots-1] of TRobotState;
  MouseControlX,MouseControlY,MouseControlTeta: double;
  MouseControlValid: boolean=false;
  Dfieldmag: double;
  DataPath: string = '';

implementation

uses Field, Robots, Utils, Tatic, Param, drawmap;//, Log

{ TFMain }

procedure TFMain.FormCreate(Sender: TObject);
var SessionPropsList: TStringList;
    SessionPropsFileName: string;
begin
  if paramcount>=1 then begin
    DataDir:=paramstr(1);
    if not directoryExists(extractfilepath(application.ExeName)+DataDir) then DataDir:=DataPath+'data';
  end else begin
    DataDir:=DataPath+'data'
  end;
  if not DirectoryExists(ExtractFilePath(Application.ExeName)+DataDir) then
    mkdir(ExtractFilePath(Application.ExeName)+DataDir);

  SessionPropsFileName := ExtractFilePath(Application.ExeName)+DataDir+'/SessionPropsMain.txt';
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
  FormStorage.IniFileName:=ExtractFilePath(Application.ExeName)+DataDir+'/Config.ini';
  FormStorage.Restore;
  SetRobotStateForm;

  loadfuzzysets;


end;

procedure TFMain.FormShow(Sender: TObject);
begin
  // TODO Log crash
  {FLog.FillTreeView(FLog.TreeView);
  FLog.LoadTree(FLog.TreeView);
  FLog.RefreshGrid(FLog.TreeView); }

  FieldImageWidth := ImageMap.Width;
  FieldImageHeight := ImageMap.Height;

  UpdateFieldDims;
  TimerDoTactic.Enabled := true;
end;

procedure TFMain.ShowAll;
var i,tx,ty: integer;
    cl: Tcolor;
begin
    DrawFieldMap(ImageMap.canvas);
    for i:=0 to MaxRobots-1 do begin
      RobotState[i].num:=i;
      if i<>4 then begin
        if (RobotStatus[i].active=true) then begin
           DrawRobot(RobotState[i],cl,ImageMap.canvas);
           DrawRobotInfo(RobotState[i],RobotInfo[i],ImageMap.canvas);
        end else begin
           DrawRobot(RobotState[i],clRed,ImageMap.canvas);
           DrawRobotInfo(RobotState[i],RobotInfo[i],ImageMap.canvas);
        end;
        ShowRobotTimes(RobotState[i],RobotStateForm[i],i);
      end;
    end;
end;

procedure TFMain.ImageMapMouseDown(Sender: TObject; Button: TMouseButton;
  Shift: TShiftState; X, Y: Integer);
begin
  down_x:=x;
  down_y:=y;
end;

procedure TFMain.ImageMapMouseUp(Sender: TObject; Button: TMouseButton;
  Shift: TShiftState; X, Y: Integer);
var Xw,Yw,V: double;
    new_teta: double;
begin
  new_teta:=atan2(-Y+down_y,X-down_x);
  MapToWorld(down_x,down_y,xw,yw);
  V:=sqrt(sqr(-Y+down_y)+sqr(X-down_x));

    with RobotState[RGRobotSel.ItemIndex] do begin
      x:=xw;
      y:=yw;
      teta:=new_teta;
    end;


end;

procedure TFMain.MenuAboutClick(Sender: TObject);
begin
  Showmessage('WatchTower 2.0');
end;

procedure TFMain.MenuExitClick(Sender: TObject);
begin
  Close;
end;

procedure TFMain.MenuFieldClick(Sender: TObject);
begin
  FField.Show;
end;

procedure TFMain.MenuLogClick(Sender: TObject);
begin
   //FLog.Show;
end;

procedure TFMain.MenuParamClick(Sender: TObject);
begin
   FParam.Show;
end;

procedure TFMain.UDPCoachReceive(aSocket: TLSocket);
var PlayerInfo: TPlayerInfo;
    packet_str: string;
    i, RobotNumber: integer;
begin
  try
    aSocket.GetMessage(packet_str);

    // only accpet valid length packets
    if length(packet_str) <> sizeof(PlayerInfo) then exit;

    copymemory(@PlayerInfo, @(packet_str[1]), sizeof(PlayerInfo));

    if PlayerInfo.Magic <> PACKET_PLAYER_MAGIC then exit;
  except
  end;
end;

procedure TFMain.UDPSuperError(const msg: string; aSocket: TLSocket);
begin
  try
    UDPSuper.Disconnect(true);
    UDPSuper.Listen(7272+RGRobotSel.ItemIndex);
  except
  end;
end;

procedure TFMain.UDPSuperReceive(aSocket: TLSocket);
var packet_str: string;
begin
  try
    aSocket.GetMessage(packet_str);
    ProcessPlayerPacket(packet_str);
  except
  end;
end;

procedure TFMain.MainLoop;
var  i,j: integer;
begin
  NetTime:=getTickcount();
  deb_txt:='';

  ShowAll;
  deb_txt:=deb_txt+format(',%3d',[getTickcount()-NetTime]);
end;

procedure TFMain.TimerDoTacticTimer(Sender: TObject);
begin
  UDPSuper.Listen(7373);
  DoTactic;
  SendCoachInfo;
  ShowAll;
  {if (CBOn.Checked=true) then begin
      FLog.LogFrame(GetTickCount,0,1);
  end else begin
      FLog.LogFrame(GetTickCount,0,0);
  end; }
end;

procedure TFMain.ShowRobotTimes(var RS: TRobotstate; var RSF: TRobotStateForm;
  robnum: integer);
begin
  RSF.EditRobotState.Text:=format('%.2f',[(getTickcount-RS.timestamp)/1000]);
  if  (getTickcount-RS.timestamp)/1000 > 10 then begin
    RobotStatus[robnum].Active:=false;
  end;

  if RobotStatus[robnum].Active then begin
   RSF.EditRobotState.Color:=clGreen;
  end else begin
    RSF.EditRobotState.Color:=clRed;
  end;
end;

procedure TFMain.ProcessPlayerPacket(packet_str: string);
var PlayerInfo: TPlayerInfo;
    i: integer;
begin
  if length(packet_str) <> sizeof(PlayerInfo) then exit;

  copymemory(@PlayerInfo, @(packet_str[1]), sizeof(PlayerInfo));

  if PlayerInfo.Magic <> PACKET_PLAYER_MAGIC then exit;

  // update ONE robot state
  with RobotState[PlayerInfo.num] do begin
    case PlayerInfo.num of
    0:EditVbatRobot1.Text:=format('%2f',[Vbatery]);
    1:EditVbatRobot2.Text:=format('%2f',[Vbatery]);
    2:EditVbatRobot3.Text:=format('%2f',[Vbatery]);
    3:EditVbatRobot4.Text:=format('%2f',[Vbatery]);
    end;
    x:=PlayerInfo.RobotState.x;
    y:=PlayerInfo.RobotState.y;
    teta:=PlayerInfo.RobotState.teta;
    vx:=PlayerInfo.RobotState.vx;
    vy:=PlayerInfo.RobotState.vy;
    W:=PlayerInfo.RobotState.w;

    count:=Round(PlayerInfo.RobotState.conf);

    Vbatery:=PlayerInfo.Batery;
    timestamp:=GetTickCount;
  end;

  with RobotStatus[PlayerInfo.num] do begin
    if RobotStateForm[PlayerInfo.num].CBForceRobotActive.Checked then begin
      Active:=RobotStateForm[PlayerInfo.num].CBRobotActive.Checked
    end else
    if (RobotStateForm[PlayerInfo.num].CBRobotActive.Checked) and (PlayerInfo.RobotState.Active)  then begin
      active:=true;
    end else begin
      active:=false;
    end;
  end;

  for i:=0 to MaxRobots-1 do begin
    with ObsStates[PlayerInfo.num] do begin
      Obs[i].xw:=PlayerInfo.ObsState[i].x;
      Obs[i].yw:=PlayerInfo.ObsState[i].y;
      Obs[i].quality:=PlayerInfo.ObsState[i].conf;
    end;
  end;
end;

procedure TFMain.SendCoachInfo;
var CoachInfo: TCoachInfo;
    packet_str: string;
    output, i: integer;
    outputVal,d: double;
begin
  try
    CoachInfo.Magic := PACKET_COACH_MAGIC;

    for i := 0 to MaxRobots-1 do begin
      with RobotState[i] do begin
        CoachInfo.RobotState[i].x := x;
        CoachInfo.RobotState[i].y := y;
        CoachInfo.RobotState[i].teta := teta;
        CoachInfo.RobotState[i].vx := vx;
        CoachInfo.RobotState[i].vy := vy;
        CoachInfo.RobotState[i].w := w;
        CoachInfo.RobotState[i].active := RobotStatus[i].active;
      end;

      {if (CBOn.Checked=true) then begin
          //CoachInfo.RobotState[i].role := RobotInfo[i].role;
          CoachInfo.RobotState[i].role := roleIdle;

          //A SELEÇÃO DAS ROLES DEVE SER FEITA PELO ALGORITMO FUZZY A SER FEITO NA ITALIA!!!
          //if (FCoach.RGDecision.ItemIndex=2) then begin
          if (CBFuzzy.Checked=true) then begin
              d:=sqrt(power((RobotState[i].x - BallState.x),2)+power((RobotState[i].y - BallState.y),2));

              //O ALGORITMO FUZZY DEVERÁ SER POSTO AQUI!!!
              InputVals[BallQuality]:=BallState.quality;
              InputVals[ConfInDistance]:=d;
              outputVal:=ProcessAllSugeno(InputVals);

              if outputVal>0.5 then begin
                 output:=Formation;
              end else begin
                 output:=Search;
              end;

              EditRobotState6.Text:=floattostr(outputVal);
              EditVbatRobot6.Text:=inttostr(output);

              if (output=0) then begin
                  if i<>0 then begin
                     CoachInfo.RobotState[i].role := roleGoSearch;
                     RobotInfo[i].role:=roleGoSearch;
                  end else begin
                     CoachInfo.RobotState[i].role := roleGoSearchFollower;
                     RobotInfo[i].role:=roleGoSearchFollower;
                  end;
              end else begin
                  CoachInfo.RobotState[i].role := roleDoFormation;
                  RobotInfo[i].role:=roleDoFormation;
              end;

          end else begin
            if (CBTraj.Checked=true) then begin
                CoachInfo.RobotState[i].role := roleFollowTrajectory;
                RobotInfo[i].role:=roleFollowTrajectory;
            end else begin
              if BallState.quality<100 then begin
                  if i<>0 then begin
                     CoachInfo.RobotState[i].role := roleGoSearch;
                     RobotInfo[i].role:=roleGoSearch;
                  end else begin
                     CoachInfo.RobotState[i].role := roleGoSearchFollower;
                     RobotInfo[i].role:=roleGoSearchFollower;
                  end;
              end else begin
                 CoachInfo.RobotState[i].role := roleDoFormation;
                 RobotInfo[i].role:=roleDoFormation;
              end;
            end;
          end;
      end else begin}
          CoachInfo.RobotState[i].role := roleIdle;
          RobotInfo[i].role:=roleIdle;
      //end;
    end;

    packet_str := StringOfChar(#0, sizeof(CoachInfo));
    copymemory(@(packet_str[1]), @CoachInfo, sizeof(CoachInfo));

   //Diego: Se os RoCs estiverem na MESMA máquina (localhost) do Watchtower,
   //esse if será executado, com o intuito de abrir uma porta para cada RoC.
      if (RGDecision.ItemIndex=0) then
        for i := 0 to MaxRobots-1 do begin
            UDPSuper.SendMessage(packet_str, '127.0.0.1:'+inttostr(7271 + i))
        end
   //Diego: Se os RoCs estiverem em máquina DIFERENTE do Watchtower,
   //esse if será executado, com o intuito de abrir uma porta para cada RoC.
      else  //Net
          begin
              if(FParam.EditRoc1IP.Text <> '') then
                UDPSuper.SendMessage(packet_str, FParam.EditRoc1IP.Text +':'+inttostr(7272));
              if(FParam.EditRoc2IP.Text <> '') then
                UDPSuper.SendMessage(packet_str, FParam.EditRoc2IP.Text +':'+inttostr(7273));
              if(FParam.EditRoc3IP.Text <> '') then
                UDPSuper.SendMessage(packet_str, FParam.EditRoC3IP.Text +':'+inttostr(7274));
              if(FParam.EditRoc4IP.Text <> '') then
                UDPSuper.SendMessage(packet_str, FParam.EditRoC4IP.Text +':'+inttostr(7275));
          end;

  except
  end;
end;

procedure TFMain.SetRobotStateForm;
begin
  with RobotStateForm[0] do begin
    CBRobotActive:=CBRobotActive1;
    EditRobotState:=EditRobotState1;
  end;
  with RobotStateForm[1] do begin
    CBRobotActive:=CBRobotActive2;
    EditRobotState:=EditRobotState2;
  end;
  with RobotStateForm[2] do begin
    CBRobotActive:=CBRobotActive3;
    EditRobotState:=EditRobotState3;
  end;
  with RobotStateForm[3] do begin
    CBRobotActive:=CBRobotActive4;
    EditRobotState:=EditRobotState4;
  end;
end;

procedure TFMain.ResetView;
var i: integer;
begin
  with View do begin
    FrameTime:=0;
    SendTime:=0;
    EdgesCount:=0;
    RegionsCount:=0;
    OdosCount:=0;

    for i:=0 to CTrackColors-1 do begin
      Centers[i].Count:=0;
    end;
  end;
end;


initialization
   RobotState[0].Vbatery:=0;
   RobotState[1].Vbatery:=0;
   RobotState[2].Vbatery:=0;
   RobotState[3].Vbatery:=0;

{$R *.lfm}

end.

