program RoC;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  Interfaces, // this includes the LCL widgetset
  Forms, lnetvisual, Main, Robots, Roles, Tatic, Actions, Astar, Tasks, Utils,
  Field, drawmap, MPC, model, Param
  { you can add units after this };

{$R *.res}

begin
  RequireDerivedFormResource := True;
  Application.Initialize;
  Application.CreateForm(TFMain, FMain);
  Application.CreateForm(TFField, FField);
  Application.CreateForm(TFormMPC, FormMPC);
  Application.CreateForm(TFParam, FParam);
  Application.Run;
end.

