unit Field;

{$mode objfpc}{$H+}

interface

uses
  SysUtils, Classes, Graphics, Controls, Forms, Dialogs,
  StdCtrls, LResources, IniPropStorage, RoCConsts;

const
  MaxFieldLines=32;

type
  TFieldDimensions = record
    FieldDepth,FieldWidth: double;
    GoalDepth,GoalWidth: double;
    KeeperAreaDepth,KeeperAreaWidth: double;
    AreaDepth,AreaWidth: double;
    CircleRadius: double;
    CornerDist, CornerLinesDist, CornerLinesWidth: double;
    LinesWidth,FinalLinesWidth: double;
    PoleSectionHeight: double;
    BoundaryDepth, BoundaryWidth: double;
  end;

  TLineType=(ltHorizontal,ltVertical);

  TFieldEdgeLine=record
    xy,xy1,xy2: double;
    LWidth: integer;
    LineType: TLineType;
  end;

  TFieldEdgeLines=record
    Lines: array[0..MaxFieldLines-1] of TFieldEdgeLine;
    count: integer;
  end;


type

  { TFField }

  TFField = class(TForm)
    FormStorage: TIniPropStorage;
    Label26: TLabel;
    EditWidth: TEdit;
    Label1: TLabel;
    EditDepth: TEdit;
    Label2: TLabel;
    EditGoalWidth: TEdit;
    EditGoalDepth: TEdit;
    Label3: TLabel;
    Label4: TLabel;
    EditAreaWidth: TEdit;
    EditAreaDepth: TEdit;
    Label5: TLabel;
    Label6: TLabel;
    EditCircleRadius: TEdit;
    Label7: TLabel;
    EditCornerDist: TEdit;
    Label8: TLabel;
    EditCornerLinesDist: TEdit;
    Label9: TLabel;
    EditLinesWidth: TEdit;
    Label10: TLabel;
    EditCornerLinesWidth: TEdit;
    BRefresh: TButton;
    Label11: TLabel;
    EditPoleSectionHeight: TEdit;
    Label12: TLabel;
    EditFinalLinesWidth: TEdit;
    Label13: TLabel;
    EditBoundaryDepth: TEdit;
    Label14: TLabel;
    EditBoundaryWidth: TEdit;
    Label15: TLabel;
    EditKeeperAreaDepth: TEdit;
    Label16: TLabel;
    EditKeeperAreaWidth: TEdit;
    editfieldmag: TEdit;
    Label17: TLabel;
    procedure FormCreate(Sender: TObject);
    procedure BRefreshClick(Sender: TObject);
  private
    { Private declarations }
  public

  end;

procedure UpdateFieldDims;

var
  FField: TFField;
  FieldDims: TFieldDimensions;
  FieldEdgelines: TFieldEdgeLines;

implementation

uses Main, drawmap;

procedure TFField.FormCreate(Sender: TObject);
var
  SessionPropsList: TStringList;
  SessionPropsFileName: string;
begin
  SessionPropsFileName := ExtractFilePath(Application.ExeName)+FMain.DataDir+'/SessionPropsField.txt';
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
end;

//------------------------------------------------------------------------------------------

procedure TFField.BRefreshClick(Sender: TObject);
begin
  UpdateFieldDims;
end;

procedure addEdgeline(nx,ny1,ny2,nLWidth: double; nLineType: TLineType);
begin
  if FieldEdgelines.count<MaxFieldLines-1 then begin
    with FieldEdgelines.Lines[FieldEdgelines.count] do begin
      xy:=nx;
      xy1:=ny1;
      xy2:=ny2;
      LineType:=nLineType;
    end;
    inc(FieldEdgelines.count);
  end;
end;

procedure UpdateFieldDims;
var
  i: integer;
begin
  try
    with FField,FieldDims do begin
      FieldDepth:=StrToFloat(EditDepth.Text);
      FieldWidth:=StrToFloat(EditWidth.text);
      GoalDepth:=StrToFloat(EditGoalDepth.text);
      GoalWidth:=StrToFloat(EditGoalWidth.text);
      KeeperAreaDepth:=StrToFloat(EditKeeperAreaDepth.text);
      KeeperAreaWidth:=StrToFloat(EditKeeperAreaWidth.text);
      AreaDepth:=StrToFloat(EditAreaDepth.text);
      AreaWidth:=StrToFloat(EditAreaWidth.text);
      CircleRadius:=StrToFloat(EditCircleRadius.text);
      CornerDist:=StrToFloat(EditCornerDist.text);
      CornerLinesDist:=StrToFloat(EditCornerLinesDist.text);
      CornerLinesWidth:=StrToFloat(EditCornerLinesWidth.text);
      LinesWidth:=StrToFloat(EditLinesWidth.text);
      FinalLinesWidth:=StrToFloat(EditFinalLinesWidth.text);
      PoleSectionHeight:=StrToFloat(EditPoleSectionHeight.Text);
      BoundaryDepth:=StrToFloat(EditBoundaryDepth.text);
      BoundaryWidth:=StrToFloat(EditBoundaryWidth.text);
      Dfieldmag:=StrToFloat(editfieldmag.text);

    end;

    // from DecConsts
    GoalWidth := FieldDims.GoalWidth;
    // this is actually Area Radius...
    AreaWidth := FieldDims.KeeperAreaDepth;
    // calculated constants
    OurGoalX := -FieldDims.FieldDepth / 2;
    OurAreaX := -FieldDims.FieldDepth / 2 + FieldDims.AreaWidth;
    TheirGoalX := FieldDims.FieldDepth / 2;
    MaxFieldX := FieldDims.FieldDepth / 2 + (FieldDims.BoundaryDepth-FieldDims.FieldDepth) / 2;
    MaxFieldY := FieldDims.FieldDepth / 2 + (FieldDims.BoundaryWidth-FieldDims.FieldWidth) / 2;

    //UpdatePolePositions;

    FieldEdgelines.count:=0;
    with FieldDims do begin
      addEdgeline(-FieldDepth/2,-FieldWidth/2,FieldWidth/2,FinalLinesWidth,ltVertical); // linha do fundo |_
      addEdgeline(-FieldDepth/2+AreaDepth,-AreaWidth/2,AreaWidth/2,LinesWidth,ltVertical); // linha da área
      addEdgeline(-FieldDepth/2+KeeperAreaDepth,-KeeperAreaWidth/2,KeeperAreaWidth/2,LinesWidth,ltVertical); // linha da área do guarda redes
      addEdgeline(0,-FieldWidth/2,FieldWidth/2,LinesWidth,ltVertical); // linha do meio campo
      addEdgeline(FieldDepth/2-AreaDepth,-AreaWidth/2,AreaWidth/2,LinesWidth,ltVertical); // linha da área
      addEdgeline(FieldDepth/2-KeeperAreaDepth,-KeeperAreaWidth/2,KeeperAreaWidth/2,LinesWidth,ltVertical); // linha da área do guarda redes
      addEdgeline(FieldDepth/2,-FieldWidth/2,FieldWidth/2,FinalLinesWidth,ltVertical); // linha do fundo _|

      addEdgeline(FieldWidth/2,-FieldDepth/2,FieldDepth/2,FinalLinesWidth,ltHorizontal); // linha da equerda
      addEdgeline(-FieldWidth/2,-FieldDepth/2,FieldDepth/2,FinalLinesWidth,ltHorizontal); // linha da direita

      addEdgeline(AreaWidth/2,-FieldDepth/2,-FieldDepth/2+AreaDepth,LinesWidth,ltHorizontal); // linha da área defesa esquerda
      addEdgeline(-AreaWidth/2,-FieldDepth/2,-FieldDepth/2+AreaDepth,LinesWidth,ltHorizontal); // linha da área defesa direita
      addEdgeline(AreaWidth/2,FieldDepth/2-AreaDepth,FieldDepth/2,LinesWidth,ltHorizontal); // linha da área ataque esquerda
      addEdgeline(-AreaWidth/2,FieldDepth/2-AreaDepth,FieldDepth/2,LinesWidth,ltHorizontal); // linha da área ataque direita

      addEdgeline(KeeperAreaWidth/2,-FieldDepth/2,-FieldDepth/2+KeeperAreaDepth,LinesWidth,ltHorizontal); // linha da área defesa esquerda
      addEdgeline(-KeeperAreaWidth/2,-FieldDepth/2,-FieldDepth/2+KeeperAreaDepth,LinesWidth,ltHorizontal); // linha da área defesa direita
      addEdgeline(KeeperAreaWidth/2,FieldDepth/2-KeeperAreaDepth,FieldDepth/2,LinesWidth,ltHorizontal); // linha da área ataque esquerda
      addEdgeline(-KeeperAreaWidth/2,FieldDepth/2-KeeperAreaDepth,FieldDepth/2,LinesWidth,ltHorizontal); // linha da área ataque direita
    end;
  except
  end;
end;

initialization
  {$I Field.lrs}

end.

