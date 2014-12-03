unit Fuzzy;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Dialogs, Math;

type
  TFFSet = record
    SetName : string;
    ptX, ptY : array[0..2] of real;
  end;

  TFFSugenoRule= record
     Var1, Var1Set : integer;
     Var2, Var2Set : integer;
     parameters: array[0..2] of real; // Only supports Zero order and first order Sugeno fuzzy models
     VarOutSet : integer;
  end;

  TFFRule = record
    Var1, Var1Set : integer; // if Var1 is SetN1
    Var2, Var2Set : integer; // Var2 is SetN2
    VarOut, VarOutSet : integer;
  end;

  TFFVar = record
    Sets    : array[0..2] of TFFSet;
    VarName : string;
  end;

type
  TFFSystem = record
      SysName  : string;
      Vars  : array[0..2] of TFFVar;
      Rules : array[0..8] of TFFRule;
      SugenoRules: array[0..8] of TFFSugenoRule;
  end;

const
  Formation=1;
  Search=0;
  BallQuality=0;
  ConfInDistance=1;
  State=2;

var
  CoachControl: TFFSystem;


procedure loadfuzzysets;
function  FFSetEvaluate(const CrispInputX : real ; const MySet : TFFSet) : real;
function  ProcessSugeno(RuleNum : integer ; InpVals : array of real; out membership: real): double;
function  ProcessAllSugeno(InpVals : array of real): double;

implementation

procedure loadfuzzysets;
var q,c,r: integer;
begin
    CoachControl.SysName:='CoachFuzzy';

    CoachControl.Vars[0].VarName:='BallQuality';

    CoachControl.Vars[0].Sets[0].SetName:='NV';
    CoachControl.Vars[0].Sets[0].ptX[0]:=-1000;
    CoachControl.Vars[0].Sets[0].ptY[0]:=1;
    CoachControl.Vars[0].Sets[0].ptX[1]:=150;
    CoachControl.Vars[0].Sets[0].ptY[1]:=1;
    CoachControl.Vars[0].Sets[0].ptX[2]:=300;
    CoachControl.Vars[0].Sets[0].ptY[2]:=0;

    CoachControl.Vars[0].Sets[1].SetName:='VR';
    CoachControl.Vars[0].Sets[1].ptX[0]:=200;
    CoachControl.Vars[0].Sets[1].ptY[0]:=0;
    CoachControl.Vars[0].Sets[1].ptX[1]:=450;
    CoachControl.Vars[0].Sets[1].ptY[1]:=1;
    CoachControl.Vars[0].Sets[1].ptX[2]:=700;
    CoachControl.Vars[0].Sets[1].ptY[2]:=0;

    CoachControl.Vars[0].Sets[2].SetName:='VM';
    CoachControl.Vars[0].Sets[2].ptX[0]:=600;
    CoachControl.Vars[0].Sets[2].ptY[0]:=0;
    CoachControl.Vars[0].Sets[2].ptX[1]:=750;
    CoachControl.Vars[0].Sets[2].ptY[1]:=1;
    CoachControl.Vars[0].Sets[2].ptX[2]:=1000;
    CoachControl.Vars[0].Sets[2].ptY[2]:=1;

    CoachControl.Vars[1].VarName:='ConfInDistance';
    CoachControl.Vars[1].Sets[0].SetName:='MC';
    CoachControl.Vars[1].Sets[0].ptX[0]:=0;
    CoachControl.Vars[1].Sets[0].ptY[0]:=1;
    CoachControl.Vars[1].Sets[0].ptX[1]:=2.5;
    CoachControl.Vars[1].Sets[0].ptY[1]:=1;
    CoachControl.Vars[1].Sets[0].ptX[2]:=4;
    CoachControl.Vars[1].Sets[0].ptY[2]:=0;

    CoachControl.Vars[1].Sets[1].SetName:='CR';
    CoachControl.Vars[1].Sets[1].ptX[0]:=3;
    CoachControl.Vars[1].Sets[1].ptY[0]:=0;
    CoachControl.Vars[1].Sets[1].ptX[1]:=5;
    CoachControl.Vars[1].Sets[1].ptY[1]:=1;
    CoachControl.Vars[1].Sets[1].ptX[2]:=7;
    CoachControl.Vars[1].Sets[1].ptY[2]:=0;

    CoachControl.Vars[1].Sets[2].SetName:='PC';
    CoachControl.Vars[1].Sets[2].ptX[0]:=6;
    CoachControl.Vars[1].Sets[2].ptY[0]:=0;
    CoachControl.Vars[1].Sets[2].ptX[1]:=7.5;
    CoachControl.Vars[1].Sets[2].ptY[1]:=1;
    CoachControl.Vars[1].Sets[2].ptX[2]:=10;
    CoachControl.Vars[1].Sets[2].ptY[2]:=1;

    r:=0;
    for q:=0 to 2 do begin
     for c:=0 to 2 do begin
        with CoachControl.SugenoRules[r] do begin
          Var1    := BallQuality;
          Var1Set := q;

          Var2    := ConfInDistance;
          Var2Set := c;

          if ((q=0)and(c>=0))or((q>=0)and(c=2)) then begin
              parameters[0]  := 0;
              parameters[1]  := 0;
              parameters[2]  := -0.3;
              VarOutSet:=Search;
          end else if ((q=1)and(c=1)) then begin
              parameters[0]  := 0;
              parameters[1]  := 0;
              parameters[2]  := 0;
              VarOutSet:=Search;
          end else if (((q=1)and(c=0))or((q=2)and(c=0))or((q=2)and(c=1))) then begin
              parameters[0]  := 0;
              parameters[1]  := 0;
              parameters[2]  := 1.6;
              VarOutSet:=Formation;
          end;
        end;
        r:=r+1;
     end;
    end;



end;

function  FFSetEvaluate(const CrispInputX : real ; const MySet : TFFSet) : real;
var
  leftX,leftY,rightX,rightY,m,b : real;
  i : integer;
begin

  if (high(MySet.ptX)-low(MySet.ptX))<0 then begin
    ShowMessage('Invalid Set');
    result:=0;
    exit;
  end;

  if CrispInputX <= Myset.ptX[low(MySet.ptX)] then begin       // menor que o primeiro ponto do FuzzySet
    result := MySet.ptY[low(Myset.ptX)];
    exit;
  end;

  if CrispInputX >= Myset.ptX[high(Myset.ptX)] then begin      // maior que o ultimo ponto do FuzzySet
    result := MySet.ptY[high(Myset.ptX)];
    exit;
  end;

  for  i := 0 to High(Myset.ptX)-1 do begin
    if (MySet.ptX[i] = CrispInputX) and (MySet.ptX[i+1] = CrispInputX) then begin
      leftX  := MySet.ptX[i];
      leftY  := MySet.ptY[i];
      rightX := MySet.ptX[i];
      rightY := MySet.ptY[i+1];
    end;
    if (MySet.ptX[i] < CrispInputX) and (MySet.ptX[i+1] >= CrispInputX) then begin
      leftX  := MySet.ptX[i];
      leftY  := MySet.ptY[i];
      rightX := MySet.ptX[i+1];
      rightY := MySet.ptY[i+1];
    end;
  end;

  if abs(rightX-leftX) < 1e-6 then begin
    result := math.min(rightY, leftY);    // Changed
  end else begin
    m := ((leftY - rightY)/(leftX - rightX));    // declive da recta
    b := leftY - leftX*m;                        // ponto na origem da recta
    result := m*CrispInputX + b;                  // substituir pelo parametro
  end;
end;

function  ProcessSugeno(RuleNum : integer ; InpVals : array of real; out membership: real): double;
var
   eval1, eval2 : real;
   ThisSet: TFFset;
begin

  //VAR1
  with CoachControl.SugenoRules[RuleNum] do begin
      ThisSet:=CoachControl.Vars[Var1].Sets[Var1Set];
      eval1:= FFSetEvaluate(InpVals[0],ThisSet);
  end;

    //VAR2
  with CoachControl.SugenoRules[RuleNum] do begin
      ThisSet:=CoachControl.Vars[Var2].Sets[Var2Set];
      eval2:= FFSetEvaluate(InpVals[1],ThisSet);
  end;

  membership:=eval1+eval2-(eval1*eval2);

  with CoachControl.SugenoRules[RuleNum] do begin
      result:=membership*(InpVals[Var1]*parameters[0]+InpVals[Var2]*parameters[1]+parameters[2]);
  end;
end;

function ProcessAllSugeno(InpVals : array of real): double;
var
   i: integer;
   sum, inf: real;
begin
     sum:=0;
     result:=0;

     for i:=0 to 8 do begin
         result:=result+ProcessSugeno(i,InpVals,inf);
         sum:=sum+inf;
     end;
     result:=result/sum;
end;


end.

