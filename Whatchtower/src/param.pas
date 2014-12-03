unit Param;

{$mode objfpc}{$H+}

interface

uses
  SysUtils, Classes, Graphics, Controls, Forms, Dialogs, LResources,
  ComCtrls, StdCtrls, Main, ExtCtrls, IniPropStorage, RoCConsts;

type

  { TFParam }

  TFParam = class(TForm)
    EditRoc1IP: TEdit;
    EditRoc2IP: TEdit;
    EditRoC3IP: TEdit;
    EditRoc4IP: TEdit;
    EditRoc5IP: TEdit;
    EditRoc6IP: TEdit;
    EditSimTwoIP: TEdit;
    EditSimTwoListenPort: TEdit;
    EditSimTwoPort: TEdit;
    FormStorage: TIniPropStorage;
    Label1: TLabel;
    Label2: TLabel;
    Label3: TLabel;
    Label4: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    Label8: TLabel;
    procedure FormCreate(Sender: TObject);
  private
    { private declarations }
  public
    { public declarations }
  end;

var
  FParam: TFParam;

implementation

uses  Utils,StrUtils, Field;


function FToStr(f: double): string;
begin
  result:=Format('%.6f',[f]);
  result:=AnsiReplaceStr(TrimRight(AnsiReplaceStr(result,'0',' ')),' ','0');
  if result[length(result)]='.' then result:=result+'0';
end;


procedure TFParam.FormCreate(Sender: TObject);
var
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
end;

function EditToFloatDef(edit: TEdit; default: double): double;
begin
  try
    result:=strtofloat(edit.text);
  except
    result:=default;
    edit.text:=Format('%.8g',[default]);
  end;
end;


initialization
  {$I param.lrs}

end.

