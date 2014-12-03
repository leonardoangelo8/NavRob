program watchtower;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  Interfaces, // this includes the LCL widgetset
  Forms, lnetvisual, Main, Fuzzy, drawmap, Robots, Field, RoCConsts, Roles,
  Utils, Tatic, Param
  { you can add units after this };

{$R *.res}

begin
  RequireDerivedFormResource := True;
  Application.Initialize;
  Application.CreateForm(TFMain, FMain);
  Application.CreateForm(TFField, FField);
  Application.CreateForm(TFParam, FParam);
  Application.Run;
end.

