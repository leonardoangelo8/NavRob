unit Param;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, Forms, Controls, Graphics, Dialogs, StdCtrls,
  IniPropStorage, ComCtrls, CheckLst, ExtCtrls, RoCConsts, Main, inifiles;

type

  { TFParam }

  TFParam = class(TForm)
    BOk: TButton;
    EditCompassOffset: TEdit;
    EditMaxAngularAcceleration: TEdit;
    EditMaxLinearAcceleration: TEdit;
    EditMaxRobotSpeed: TEdit;
    EditRobotSpace: TEdit;
    EditSimIP: TEdit;
    EditSimTwoListenPort: TEdit;
    EditSimPort: TEdit;
    EditSuperIP: TEdit;
    EditVisionIP: TEdit;
    FormStorage: TIniPropStorage;
    Label1: TLabel;
    Label18: TLabel;
    Label2: TLabel;
    Label25: TLabel;
    Label3: TLabel;
    Label32: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    Label8: TLabel;
    RGRobotNumber: TRadioGroup;
    procedure BOkClick(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure RGRobotNumberClick(Sender: TObject);
    procedure LoadArray;
    procedure SaveArray;


  private
    { private declarations }
  public
    { public declarations }
    procedure ComponentsToParameters;
  end;

var
  FParam: TFParam;

  CompassOffset: double=0;
  RobotSpace: double=0.6;

implementation

uses  Utils,StrUtils, Field;

procedure TFParam.FormCreate(Sender: TObject);
var
  i:integer;
  SessionPropsList: TStringList;
  SessionPropsFileName: string;
begin
  SessionPropsFileName := ExtractFilePath(Application.ExeName)+FMain.DataDir+'/SessionPropsParam.txt';
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
  FormStorage.IniFileName:=FMain.FormStorage.IniFileName;
  FormStorage.Restore;
  ComponentsToParameters;

  LoadArray;

end;

procedure TFParam.RGRobotNumberClick(Sender: TObject);
begin
  myNumber:=RGRobotNumber.ItemIndex;
  {RobotState[myNumber].teta:=pi/2;
  RobotState[myNumber].x:=0;
  RobotState[myNumber].y:=-FieldDims.fieldwidth/2;}
end;

procedure TFParam.ComponentsToParameters;
var
i: integer;
begin

  SpeedMax:=EditToFloatDef(EditMaxRobotSpeed,SpeedMax);
  RobotSpace:=EditToFloatDef(EditRobotSpace,RobotSpace);
  CompassOffset:=EditToFloatDef(EditCompassOffset,CompassOffset);

  myNumber:=RGRobotNumber.ItemIndex;

  //AvgSpeed:=EditToFloatDef(EditAvgRobotSpeed,AvgSpeed);
  max_linear_acceleration:=EditToFloatDef(EditMaxLinearAcceleration, 1.2);
  max_angular_acceleration:=EditToFloatDef(EditMaxAngularAcceleration,1);

end;

procedure TFParam.BOkClick(Sender: TObject);
begin
  ComponentsToParameters;
end;

procedure TFParam.SaveArray;
var sl: TStringList;
begin
  sl:=TStringList.Create;
  try
    sl.SaveToFile(extractfilepath(application.ExeName)+FMain.DataDir+'/Params.ini');
  finally
    sl.Free;
  end;
end;

procedure TFParam.LoadArray;
var sl: TStringList;
begin
  sl:=TStringList.Create;
  try
    sl.LoadFromFile(extractfilepath(application.ExeName)+FMain.DataDir+'/Params.ini');
  except
  end;
  sl.Free;
end;



{$R *.lfm}

end.

