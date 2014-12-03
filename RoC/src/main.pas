//------------------------------------------------------------------------------
//
// RoC 2.0 LTS
//
//------------------------------------------------------------------------------
//
// Programa de Navegação Genérico para Robôs Móveis
// A princípio utilizado para o TurtleBot 2
// Na segunda fase para> Drone Carcará e Hexapod Minion
//
// FALTA FAZER:
//             - COMUNICAÇÃO COM WATCHTOWER POR RTDB    (Leandro)
//             - COMUNICAÇÃO POR RTDB ENTRE ROBÔS       (Leandro)
//             - SLAM                                   (Leonardo)
//             - USO DO JOYSTICK                        (George)
//             - COMUNICAÇÃO COM ROBÔ REAL POR ROS      (George)
//             - INTEGRAÇÃO COM CÂMERA KINECT           (André)
//
//             - CONTROLADOR NMPFC/NMPC PARA TURTLEBOT  (TCC 1)
//             - ALGORITMO DE LOCALIZAÇÃO ROBÔ REAL     (TCC 2)
//             - DYNAMIC MAPPING SELECTION              (Mestrado 1)
//
//      OBS.: Outras coisas como quebra de formação inteligente, reconhecimento
//            de faces, etc. serão implantadas mais pra frente, depois que os
//            itens acima forem feitos.
//
//------------------------------------------------------------------------------



unit Main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, lNetComponents, Forms, Controls, Graphics,
  Dialogs, Menus, StdCtrls, ExtCtrls, IniPropStorage, RoCConsts, lNet,
  math, dynmatrix, IniFiles, CheckLst, ComCtrls, Grids, Roles;

const

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

  colorcolor24: array[0..MaxColors-1] of Tcolor=(clred,clyellow,clblue,clpurple,clnavy,clgray,clwhite,$00FF00);

type

  TTeamColor=(tcMagenta,tcCyan);

const

  CTeamColorColor24: array[low(TTeamColor)..High(TTeamColor)] of Tcolor = ($00FF00FF,$00FFFF00);

type
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

  type
  TPos=record
    x,y, teta: double;
  end;

type
  TRobotState=record
    x,y,teta: double;
    Xk,Pk: TDMatrix;
    cov_x,cov_y,cov_xy,cov_teta: double;
    v,w: double;
    vx,vy: double;
    v1,v2: double;
    num: integer;
    count: integer;
    Vbatery:double;
  end;


  TTacticCommand=record
    v,w: double;
    v1,v2: double;
  end;

  Tcenter=record
    quality:integer;
    ro,teta,d: double;
    xw,yw: double;
    Vxw,Vyw:double;
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
    RobotDelU, RobotDelTeta:double;
    RobotSpeedV, RobotSpeedW: double;
    Avaible: boolean;
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




  { TFMain }

  TFMain = class(TForm)
    BSetRole: TButton;
    CBSim: TCheckBox;
    CBStartRole: TComboBox;
    CBShow: TCheckBox;
    Edit3: TEdit;
    Edit4: TEdit;
    EditRoleName: TEdit;
    EditTaskName: TEdit;
    FormStorage: TIniPropStorage;
    GBMainControl: TGroupBox;
    GroupBox1: TGroupBox;
    GroupBox3: TGroupBox;
    ImageMap: TPaintBox;
    Label1: TLabel;
    Label2: TLabel;
    Label4: TLabel;
    Label5: TLabel;
    UDPSim: TLUDPComponent;
    MainMenu: TMainMenu;
    MemoActionPars: TMemo;
    MenuFile: TMenuItem;
    MenuAbout: TMenuItem;
    MenuExit: TMenuItem;
    MenuCamera: TMenuItem;
    MenuControl: TMenuItem;
    MenuField: TMenuItem;
    MenuLoc: TMenuItem;
    MenuMap: TMenuItem;
    MenuJoystick: TMenuItem;
    MenuParam: TMenuItem;
    MenuWindows: TMenuItem;
    RGController: TRadioGroup;
    procedure BSetRoleClick(Sender: TObject);
    procedure CBSimClick(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure ImageMapMouseDown(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure ImageMapMouseUp(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure MenuAboutClick(Sender: TObject);
    procedure MenuControlClick(Sender: TObject);
    procedure MenuExitClick(Sender: TObject);
    procedure MenuFieldClick(Sender: TObject);
    procedure MenuJoystickClick(Sender: TObject);
    procedure MenuParamClick(Sender: TObject);
    procedure UDPSimError(const msg: string; aSocket: TLSocket);
    procedure UDPSimReceive(aSocket: TLSocket);


    procedure FormClose(Sender: TObject);



  private
    { private declarations }
    procedure SimMainLoop;
    procedure MainControl;
    procedure ManualControl(var V, W: double);
    procedure ResetView;
    procedure ShowEdges;
    procedure ShowRadar;

 //   procedure SendPlayerInfo;




  public
    { public declarations }
    DataDir: string;
    deb_txt: string;
    ref_v,ref_w: double;
    Ser1_Data, Ser2_Data: string;
    Ser1_Out, Ser2_Out: string;
    SLShowValues: TStrings;
    TempStream : TMemoryStream;
    Good_image: boolean;
    StrTream: TStringStream;
    down_x,down_y: integer;
    draw_x,draw_y: integer;

    procedure ProcessSimMsg(Data: string);
    procedure ControlAccelerationNLTMD(out V:double;out W:double);
    procedure ShowAll;
    procedure UpdateObstacles;
    procedure PropagateObstacles;
    procedure AddObstacle(xw, yw, teta: double; ObstacleColor: integer; var Obstacles: TObstacles);
    procedure UpdateWorldState(var OldRobotState,NewRobotState: TRobotState);
    procedure CheckLines;
    procedure CheckLine(var ELine:TEdgeLine);

    //procedure MainLoop;
    //procedure ProcessCoachPacket(packet_str: string);

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
  ControlTimeStamp:LongWord;
  TacticCommands: array[0..MaxRobots-1] of TTacticCommand;
  RobotState: array[0..MaxRobots-1] of TRobotState;
  MouseControlX,MouseControlY,MouseControlTeta: double;
  MouseControlValid: boolean=false;

implementation

uses Robots, Utils, Field, Tatic, Actions, Tasks, Param, drawmap, MPC, model;//, Camera, Joy, LCLType, omni3

{$R *.lfm}

procedure TFMain.FormCreate(Sender: TObject);
var
  i,j: integer;
  SessionPropsList: TStringList;
  SessionPropsFileName: string;
  k: TRole;
begin
  // parse command line parameters
  DataDir := 'data';

  for i := 1 to paramcount do begin
      DataDir:=paramstr(i);
      if not directoryExists(extractfilepath(application.ExeName)+dataDir) then dataDir:='data';
  end;

  WindowState := wsNormal;
  decimalSeparator:='.';
  walk_counter:=0;

  for i:=0 to Nsim-1 do begin
    for j:=0 to 1 do begin
      Buff_erro[i,j]:=0;
    end;
  end;
  Vsim:=0;
  Wsim:=0;
  Sum_err_stress_v:=0;
  Sum_err_stress_w:=0;
  Idx_erro:=0;

  ref_v_ant:=0;
  ref_w_ant:=0;
  ref_v:=0;
  Flag_stress:=false;
  Flag_stress_ant:=false;
  v_on_stress:=0;
  w_on_stress:=0;

  Ser1_Data:='';
  Ser2_Data:='';
  Ser1_Out:='';
  Ser2_Out:='';
  LastData:='';

  CBStartRole.Items.Clear;
  for k:=Low(TRole) to High(TRole) do begin
    CBStartRole.Items.Add(RoleDefs[k].name);
  end;
  CBStartRole.DropDownCount:=4;
  CBStartRole.ItemIndex:=0 ;

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

  SLShowValues:= TStringList.Create;
  TempStream := TMemoryStream.Create;
  Good_image:=false;
  StrTream:=TStringStream.Create('');


  with View do begin
    for i:=0 to cTrackColors-1 do begin
     Centers[i].Count:=0;
    end;
    EdgesCount:=0;
    RegionsCount:=0;

    OdosCount:=0;
    OdosTotalCount:=0;
  end;
  LastView:=View;

  {for i:=0 to MaxRobots-1 do begin
    with RobotState[i] do begin
      x:=4*-random(100)/100;
      y:=2*(100-random(200))/100;
      teta:=10*random;
      cov_x:=1e-10;
      cov_y:=1e-10;
      cov_xy:=0;
      cov_teta:=1e-9;
      num:=i;
      Xk.SetSize(3,1);
      Pk.SetSize(3,3);
      RobotStateDoublesToMatrix(RobotState[i]);
    end;
  end; }
  with RobotState[myNumber] do begin
    x:=-2;
    y:=0.5;
    teta:=0;
  end;
  ref_v:=0;
  ref_w:=0;
  sigver:=Mzeros(2,2);

end;

procedure TFMain.FormDestroy(Sender: TObject);
begin
  SLShowValues.Free;
  StrTream.Free;
  TempStream.Free;
end;

procedure TFMain.FormShow(Sender: TObject);
begin
  draw_x:= 120;
  draw_y:=2;

  UpdateFieldDims;

  RobotStatus[myNumber].default_role:=TRole(CBStartRole.ItemIndex);
  RobotInfo[myNumber].role:=RobotStatus[myNumber].default_role;

  //SdpoUDP.Listen(7171);
  //SdpoUDPSuper.Listen(7272+myNumber);

  FieldImageWidth := FMain.ImageMap.Width;
  FieldImageHeight := FMain.ImageMap.Height;

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

  if ssCtrl in Shift then begin
    MouseControlX:=xw;
    MouseControlY:=yw;
    MouseControlTeta:=new_teta;
    MouseControlValid:=true;
  end else begin
    if FParam.RGRobotNumber.ItemIndex<>4 then begin
      with RobotState[FParam.RGRobotNumber.ItemIndex] do begin
        x:=xw;
        y:=yw;
        teta:=new_teta;
      end;
    end;
  end;
end;

procedure TFMain.CBSimClick(Sender: TObject);
var i: integer;
begin
  if CBSim.Checked then begin
    UDPSim.Listen(StrToInt(FParam.EditSimTwoListenPort.Text));
    for i := 0 to MaxRobots - 1 do begin
      RobotStatus[i].active := true;
      RobotState[i].count := 1;
    end;
  end else begin
    UDPSim.Disconnect;
  end;
end;

procedure TFMain.BSetRoleClick(Sender: TObject);
begin
  RobotInfo[myNumber].role:=TRole(CBStartRole.ItemIndex);
end;

procedure TFMain.MenuAboutClick(Sender: TObject);
begin
   Showmessage('RoC T7Br 2.0');
end;

procedure TFMain.MenuControlClick(Sender: TObject);
begin
  FormMPC.Show;
end;

procedure TFMain.MenuExitClick(Sender: TObject);
begin
    close;
end;

procedure TFMain.MenuFieldClick(Sender: TObject);
begin
  FField.Show;
end;

procedure TFMain.MenuJoystickClick(Sender: TObject);
begin

end;

procedure TFMain.MenuParamClick(Sender: TObject);
begin
    FParam.Show;
end;

procedure TFMain.FormClose(Sender: TObject);
var
  checks: string;
  i:integer;
begin
  //SdpoUDPSuper.Disconnect;
  //SdpoUDP.Disconnect;
  UDPSim.Disconnect;
end;

procedure TFMain.MainControl;
var
  V,W,k: double;
  w1,w2: double;
  i,j,num:integer;
begin
  V:=0;
  W:=0;

  case RGController.ItemIndex of
    0: begin //freeze
      V:=0;
      W:=0;
    end;
    1: begin //Manual
      ManualControl(v,w);

      ControlAccelerationNLTMD(v,w);

      with MemoActionPars.Lines do begin
        Clear;
        Add(format('Robô - x=%2f y=%2f teta=%2f',[RobotState[myNumber].x,RobotState[myNumber].y,RobotState[myNumber].teta]));
        Add(format('Robô - v=%2f w=%2f',[RobotState[myNumber].v,RobotState[myNumber].w]));
        Add(format('Robô - vx=%2f vy=%2f',[RobotState[myNumber].vx,RobotState[myNumber].vy]));
      end;

    end;
    2: begin //Joy    FALTA FAZER ROTINA PRA USAR JOYSTICK!!!
      //V:=Form_joystick.V;
      //W:=Form_joystick.W;

      ControlAccelerationNLTMD(v,w);
    end;
    3: begin // Play
      DoRobotRules(myNumber);

      EditRoleName.Text:=RoleDefs[RobotInfo[myNumber].role].name;
      EditTaskName.Text:=CTaskString[RobotInfo[myNumber].task];

      v:=TacticCommands[myNumber].v;        //Renova as velocidades a serem enviadas pro turtle
      w:=TacticCommands[myNumber].w;
      ControlAccelerationNLTMD(v,w);

      with MemoActionPars.Lines do begin
        Clear;
        Add(format('Robô - x=%2f y=%2f teta=%2f',[RobotState[myNumber].x,RobotState[myNumber].y,RobotState[myNumber].teta]));
        Add(format('Robô - v=%2f w=%2f',[RobotState[myNumber].v,RobotState[myNumber].w]));
        Add(format('Robô - vx=%2f vy=%2f',[RobotState[myNumber].vx,RobotState[myNumber].vy]));
      end;

    end;
  end;

  if CBSim.Checked=true then begin
    ControlAccelerationNLTMD(v,w);
    UDPSim.SendMessage(IntToStr(myNumber)+' '+
                       floatToStr(v)+' '+
                       floatToStr(w),
                       FParam.EditSimIP.Text+':'+FParam.EditSimPort.Text);
  end;

  RobotState[myNumber].v:=v;
  RobotState[myNumber].w:=w;
end;

//NOTE QUE APENAS TEMOS O ROBÔ SIMULADO!!!
//FALTA A ROTINA DO ROBÔ REAL E MUDA LIGEIRAMENTE POUCO
procedure TFMain.SimMainLoop;
var  i: integer;
     brdist, minBRdist, vbo,vbox,vboy: double;
begin
  deb_txt:='';

  ResetView;
  view.OdosCount:=1;
  view.Odos[0].speedw[0]:=RobotState[mynumber].v1;
  view.Odos[0].speedw[1]:=RobotState[mynumber].v2;

  UpdateObstacles;
  PropagateObstacles;

  VxyToV(RobotState[mynumber].teta, RobotState[mynumber].v, RobotState[mynumber].vx, RobotState[mynumber].vy);

  MainControl;
  sleep(1);
//  SendPlayerInfo;

  ShowAll;
  LastView:=View;

end;

procedure TFMain.UDPSimError(const msg: string; aSocket: TLSocket);
begin
  try
    //UDPSuper.Disconnect;
    //UDPSuper.Listen(7272+FParam.RGRobotNumber.ItemIndex);
  except
  end;
end;

procedure TFMain.ManualControl(var V, W: double);
begin
  v:=strtofloat(Edit3.Text);
  w:=strtofloat(Edit4.Text);
end;

procedure TFMain.UDPSimReceive(aSocket: TLSocket);
var
  msg: string;
begin
    UDPSim.GetMessage(msg);
    if (msg <> '') and (CBSim.Checked=true) then begin
      ProcessSimMsg(msg);
      SimMainLoop;
    end else begin
      exit;
    end;
end;

procedure TFMain.ProcessSimMsg(Data: string);
var
  txt: String;
  msn: TStringList;
  mn, mv, mw, mx, my, mt,q3,q4: string;
  t3,t4: float;
  i: integer;

begin
  msn := TStringList.Create;
  msn.Text := Data;

  SScanf(Data,'%s %s %s %s %s %s %s',[@mn, @mv, @mw, @mx, @my, @q3, @q4]);

  myNumber:=StrToIntDef(mn, 0);
  RobotState[myNumber].v:=strtofloatdef(mv,0);
  RobotState[myNumber].w:=strtofloatdef(mw,0);
  RobotState[myNumber].x:=strtofloatDef(mx, 0);
  RobotState[myNumber].y:=strtofloatDef(my, 0);

  //Transformando quartenion (para uma rotação em z) em teta
  t3:=strtofloatDef(q3, 0);
  t4:=strtofloatDef(q4, 0);
  RobotState[myNumber].teta:=2*arctan2(t3,t4);

  msn.Free;

  IK(RobotState[myNumber].v,RobotState[myNumber].w,RobotState[mynumber].v1,RobotState[mynumber].v2);

  if CBSim.Checked=False then begin
     UDPSim.Disconnect;
  end;

end;

procedure TFMain.CheckLine(var ELine:TEdgeLine);
var x1,y1,x2,y2,x3,y3,s,ac,tetauv,kapa,xr,yr: double;
begin
  with Eline do begin
    conf:=0;
    if (iEdges[0]<>-1) and (iEdges[1]<>-1) and (iEdges[2]<>-1) then begin
      x1:=View.Edges[iEdges[0]].xw;
      y1:=View.Edges[iEdges[0]].yw;
      x2:=View.Edges[iEdges[1]].xw;
      y2:=View.Edges[iEdges[1]].yw;
      x3:=View.Edges[iEdges[2]].xw;
      y3:=View.Edges[iEdges[2]].yw;
      s:= ((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)) * ((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1));
      if s>0 then begin
        ac:=((x2-x1)*(x3-x1)+(y2-y1)*(y3-y1))/sqrt(s);
        if (ac<1) and (ac>-1) then
          tetauv:=arccos(ac)
        else
          tetauv:=pi;
      end else begin
        tetauv:=pi;
      end;
      if tetauv<10*pi/180 then begin  // Parametro
        conf:=1;
        teta:=(atan2(y3-y1,x3-x1)+pi/2);
        kapa:=(-x1*(x1-x3)-y1*(y1-y3))/((x1-x3)*(x1-x3)+(y1-y3)*(y1-y3));
        xr:=kapa*(x1-x3)+x1;
        yr:=kapa*(y1-y3)+y1;
        odist:=sqrt(xr*xr+yr*yr);;
      end;
    end;
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

procedure TFmain.ShowRadar;
var i,x2,y2: integer;
    RS:TRobotState;
    ct,st,dx,dy: double;
begin
  with View do begin
    for i:=0 to RadarRaysCount-1 do begin
      if CBShow.Checked then with ImageMap.canvas,Radar[i] do begin
        RS:=RobotState[myNumber];
        pen.color:=colorcolor24[color];
        ct:=cos(RS.teta);
        st:=sin(RS.teta);
        WorldToMap(RS.x+xw*ct+yw*-st,RS.y+xw*st+yw*ct,x2,y2);

        if i=0 then moveto(x2,y2)
        else lineto(x2,y2);
      end;
    end;
  end;
end;

procedure TFmain.ShowEdges;
var i, x1, y1: integer;
    RS: TRobotState;
    cu, cv, ra: integer;
begin
  cu := 196;
  cv := 144;
  ra := 2;
  with View do begin
    for i:=0 to EdgesCount-1 do begin
      if CBShow.Checked then with ImageMap.canvas,Edges[i] do begin
        RS:=RobotState[myNumber];
        pen.color:=colorcolor24[Edges[i].color1];
        WorldToMap(RS.x+Edges[i].xw*cos(RS.teta)+Edges[i].yw*sin(-(RS.teta)), RS.y+Edges[i].xw*sin(RS.teta)+Edges[i].yw*cos(RS.teta),x1,y1);
        moveto(x1-ra,y1);
        lineto(x1+ra+1,y1);
        pen.color:=colorcolor24[Edges[i].color2];
        moveto(x1,y1-ra);
        lineto(x1,y1+ra+1);
      end;
    end;
  end;
end;

procedure TFMain.ControlAccelerationNLTMD(out V:double;out W:double);
var
  Derivada_V:double;
  Derivada_W:double;
  Vaux, Waux,k_acel:double;
begin

  derivada_V:=(V-RobotState[myNumber].V)/0.04;
  derivada_W:=(W-RobotState[myNumber].W)/0.04;


  if abs(v)>0.01 then begin
    if derivada_V > max_linear_acceleration then begin
      if v>0 then
        k_acel:=1
      else
        k_acel:=1;
      Vaux:=k_acel*max_linear_acceleration*0.04+RobotState[myNumber].V;
      if Vaux<V then V:=Vaux;
    end else if derivada_V <(-max_linear_acceleration) then begin
      if v<0 then
         k_acel:=1
      else
         k_acel:=1;
      Vaux:=k_acel*(-max_linear_acceleration)*0.04+RobotState[myNumber].V;
      if Vaux>V then V:=Vaux;
    end;
  end;

  if abs(w)>0.1 then begin
    if derivada_W >max_angular_acceleration then begin
      k_acel:=1;
      Waux:=k_acel*max_angular_acceleration*0.04+RobotState[myNumber].W;
      if Waux<W then W:=Waux;
    end else if derivada_W <(-max_angular_acceleration) then begin
      k_acel:=1;
      Waux:=k_acel*(-max_angular_acceleration)*0.04+RobotState[myNumber].W;
      if Waux>W then W:=Waux;
    end;
  end;
end;

procedure TFMain.ShowAll;
var i,tx,ty: integer;
    cl: Tcolor;
begin
  if CBShow.Checked then begin
      DrawFieldMap(ImageMap.canvas);
      for i:=0 to MaxRobots-1 do begin
        if i=myNumber then begin
          cl:=clwhite;
          DrawRobotText(RobotState[i],cl,ImageMap.canvas);
        end else begin
          //cl:=CTeamColorColor24[TeamColor];
          //DrawRobot(RobotState[i],cl,ImageMap.canvas);    //Não desenhar por enquanto
        end;
      end;
      DrawTRaj(Traj,ImageMap.canvas);
      ShowEdges;
      ShowRadar;
      DrawObstacles(Obstacles);
    end;
      ShowEdges;
      DrawObstacles(Obstacles);//add by tiago

      tx:=draw_x;
      ty:=draw_y;

end;

procedure TFmain.UpdateObstacles;
var i:integer;
    ydisps: array [0..1] of double;
begin
  with view do begin
    for i:=0 to RegionsCount-1 do begin
      with Regions[i] do begin
        if BestColor in [1,2,3,4,5] then begin
          AddObstacle(Regions[i].xw,Regions[i].yw,Regions[i].teta,Regions[i].BestColor,Obstacles);
        end;
      end;
    end;

    ydisps[0]:=-0.2;
    ydisps[1]:=0.2;
  end;
end;

procedure TFmain.PropagateObstacles;
var i:integer;
begin
  with Obstacles do begin
    i:=0;
    while i<Obstacles.count do begin
      Centers[i].quality:=Centers[i].quality*0.9; //parametro
      if Centers[i].quality<0.2 then begin
        Centers[i].xw:=Centers[count-1].xw;
        Centers[i].yw:=Centers[count-1].yw;
        Centers[i].color:=Centers[count-1].color;
        Centers[i].quality:=0;
        dec(count);
      end;
      inc(i);
    end;
  end;
end;

procedure TFmain.AddObstacle(xw,yw,teta: double; ObstacleColor: integer ; var Obstacles: TObstacles);
var j,idx:integer;
    xt,yt,delta,d: double;
begin
  delta:=0.2;
  d:=Dist(yw,xw);            // parametro
  if (d>1.5) or (d<1e-3) or (abs(teta)>30/180*pi) then exit;// só interessam obstáculos à frente e perto
  idx:=-1;
  for j:=0 to Obstacles.count-1 do begin   //procura um obstáculo existente no qual encaixe
    RotateAndTranslate(xt,yt,
                       xw,yw,
                       RobotState[myNumber].x,RobotState[myNumber].y,RobotState[myNumber].teta);
    if (abs(xt-Obstacles.Centers[j].xw)<delta) and
       (abs(yt-Obstacles.Centers[j].yw)<delta) then begin
      Obstacles.Centers[j].color:=ObstacleColor;
      Obstacles.Centers[j].quality:=1;
      Obstacles.Centers[j].quality:=Obstacles.Centers[j].quality*1.2;
      if Obstacles.Centers[j].quality >1 then Obstacles.Centers[j].quality:=1;
      idx:=j;
      break;
    end;
  end; // senão vai criar um novo
  if idx<>-1 then exit;
  if Obstacles.count<MaxObstacles-1 then begin
    RotateAndTranslate(Obstacles.Centers[Obstacles.count].xw,
                       Obstacles.Centers[Obstacles.count].yw,
                       xw,yw,
                       RobotState[myNumber].x,RobotState[myNumber].y,RobotState[myNumber].teta);
    Obstacles.Centers[Obstacles.count].color:=ObstacleColor;
    Obstacles.Centers[Obstacles.count].quality:=1;
    inc(Obstacles.count);
  end;

end;

procedure TFMain.UpdateWorldState(var OldRobotState,NewRobotState: TRobotState);
var pdist,pteta: double;
    i: integer;
begin
  // update do estado "observado" dos obstáculos
  for i:=0 to Obstacles.count-1 do begin
    with Obstacles.centers[i] do begin
      pdist:=Dist(xw-OldRobotState.x,yw-OldRobotState.y);
      pteta:=ATan2(yw-OldRobotState.y,xw-OldRobotState.x)-OldRobotState.teta;
      xw:=NewRobotState.x+cos(pteta+NewRobotState.teta)*pdist;
      yw:=NewRobotState.y+sin(pteta+NewRobotState.teta)*pdist;
    end;
  end;
end;

procedure TFMain.CheckLines;
var i: integer;
begin
  for i:=low(GWline.iEdges) to high(GWline.iEdges) do begin
    GWLine.iEdges[i]:=-1;
    WGLine.iEdges[i]:=-1;
  end;
  WGLine.conf:=0;
  GWline.conf:=0;

  // Procura s edges de verde para branco e/ou de branco para verde
  for i:=0 to View.EdgesCount-1 do begin
    if (View.Edges[i].color1=7) and
       (View.Edges[i].color2=6) and
       (GWLine.iEdges[min(View.Edges[i].lineNum,high(GWLine.iEdges))]=-1) then begin
      GWLine.iEdges[min(View.Edges[i].lineNum,high(GWLine.iEdges))]:=i;
    end else if (View.Edges[i].color1=6) and
                (View.Edges[i].color2=7) and
                (WGLine.iEdges[min(View.Edges[i].lineNum,high(WGLine.iEdges))]=-1) then begin
      WGLine.iEdges[min(View.Edges[i].lineNum,high(WGLine.iEdges))]:=i;
    end;
  end;

  CheckLine(GWLine);
  CheckLine(WGLine);    // parametro
end;




end.

