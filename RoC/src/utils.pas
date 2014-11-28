unit Utils;

{$mode objfpc}{$H+}

interface

uses Classes,sysutils,StdCtrls,graphics,Math, Unix, dynmatrix, Main;

type
  TMVEst=record
    Vmean,Vcov: double;
    speed: double;
    n: integer;
  end;

  TLinearReg=record
    Sx,Sy,Sxy,Sx2: double;
    a,b: double;
    N:integer;
  end;

  TCMarkerType=(mtCross,mtCircle,mtSquare,mtXis);


procedure mse(var e:double; m1,m2: TDMatrix; N: integer);
procedure ClearLinearReg(var TL: TLinearReg);
procedure AddXYtoLinearReg(var TL: TLinearReg; x,y: double);
procedure CalcLinearReg(var TL: TLinearReg);

procedure MVEstInit(var MV: TMVEst);
procedure MVEstAddValue(var MV: TMVEst; v: double);

procedure ZeroMemory(Ptr: Pointer; Len: integer);
procedure CopyMemory(DPtr,SPtr: Pointer; Len: integer);

function GetTickCount: LongWord;

function strtofloatDef(s: string; def: double): double;
function EditToFloatDef(edit: TEdit; default: double): double;
procedure ParseString(s,sep: string; sl: TStrings);

function FastATan2(y,x: double): double;

function FMod(x,d: double): double;
function DiffAngle(a1,a2: double): double;
function AngleThreePoints(x1,y1,x2,y2,x3,y3: double): double;
function DistPoint2Line(x1,y1,x2,y2,x3,y3: double): double;
function DistPointInLine(x1,y1,x2,y2,x3,y3: double): double;
function Dist(x,y: double): double;
function ATan2(y,x: double): double;
function Sign(a: double): double;
function Sat(a,limit: double): double;
function Wcontrol(Tref,TRobot,Lambda: double): double;

function IncWrap(var v: integer; size: integer; step:integer=1): integer;
function DecWrap(var v: integer; size: integer; step:integer=1): integer;
procedure SwapInts(var v1, v2: integer);

function Rad(xw: double):double;
function deg(xw: double):double;

procedure TranslateAndRotate(var rx,ry: double; px,py,tx,ty,teta: double);
procedure TranslateAndRotateInv(var rx,ry: double; px,py,tx,ty,teta: double);
procedure RotateAndTranslate(var rx,ry: double; px,py,tx,ty,teta: double);
procedure calcNextBallXY(var rx,ry: double; px,py,vx,vy,tx,ty,teta: double;forwardStep:integer);

function InternalProductCosine(v1x,v1y,v2x,v2y: double): double;
function NormalizeAngle(ang: double): double;

function InFrustum(xc,yc,xp,yp,ang,widthAng: double): boolean;

procedure DrawCovElipse(x,y,cov_x,cov_y, cov_xy: double; n: integer; CNV: TCanvas);
procedure RotateCov( const incov_x,incov_y,incov_xy: double; out cov_x,cov_y, cov_xy: double; teta: double);

function AverageAngle(ang1,ang2: double): double;
function CheckPoint(xgoal,ygoal,xrobot,yrobot: double;  d: double ): integer;
function CheckPointEnd(xgoal,ygoal,xrobot,yrobot: double): integer;

procedure NormalizeVector(var x,y: double);
function MidAngle(a1,a2: double): double;
function FiltAngle(a1, a2, lamb: double): double;

function GetTimeAndDate(tipo:integer) : string;
function innerProduct(x1,y1,x2,y2:double):double;


type
  QSortCmpFunc=function (var a,b): integer;

procedure QSort(base: pointer; num_elem,size_elem: integer; func: QSortCmpFunc);

implementation

uses Robots, Field, drawmap;

var FirstTimeValSec: LongInt;

procedure NormalizeVector(var x,y: double);
var d: double;
begin
  d:=Dist(x,y);
  if abs(d)<1e-6 then begin
    x:=1;
    y:=0;
  end else begin
    x:=x/d;
    y:=y/d;
  end;
end;

{  intersection of the two infinite lines rather than the line segments }
procedure LinesIntersect(const x1,y1,x2,y2: double; { first line}
                         const x3,y3,x4,y4: double; { second line }
                         var code : integer; { =0 OK; =1 lines parallel}
                         var x,y : double); { intersection point }

var
    a1, a2, b1, b2, c1, c2 : double; { Coefficients of line eqns.}
    denom : double;

begin
  a1:= y2-y1;
  b1:= x1-x2;
  c1:= x2*y1 - x1*y2;  { a1*x + b1*y + c1 = 0 is line 1 }

  a2:= y4-y3;
  b2:= x3-x4;
  c2:= x4*y3 - x3*y4;  { a2*x + b2*y + c2 = 0 is line 2 }

  denom:= a1*b2 - a2*b1;
  if denom = 0 then
    begin
      code:=1;
      exit;
    end;

  x:=(b1*c2 - b2*c1)/denom;
  y:=(a2*c1 - a1*c2)/denom;
  code:=0
end;

function strtofloatDef(s: string; def: double): double;
begin
  try
    result:=strtofloat(s);
  except
    result:=def;
  end;
end;

function EditToFloatDef(edit: TEdit; default: double): double;
begin
  if edit.text='*' then begin
    result:=default;
    edit.text:=Format('%.8g',[default]);
    exit;
  end;
  try
    result:=strtofloat(edit.text);
  except
    result:=default;
    edit.text:=Format('%.8g',[default]);
  end;
end;

procedure ParseString(s,sep: string; sl: TStrings);
var p,i,last: integer;
begin
  sl.Clear;
  last:=1;
  for i:=1 to length(s) do begin
    p:=Pos(s[i],sep);
    if p>0 then begin
      if i<>last then
        sl.add(copy(s,last,i-last));
      last:=i+1;
    end;
  end;
  if last<=length(s) then
    sl.add(copy(s,last,length(s)-last+1));
end;

// ---------------------------------------------------------
//     Math functions

function Dist(x,y: double): double;
begin
  result:=sqrt(x*x+y*y);
end;

function CheckPoint(xgoal,ygoal,xrobot,yrobot: double; d: double ): integer;
begin


     if Dist(xgoal-xrobot,ygoal-yrobot) < d
     then result := 1
     else result := 0;

end;

function CheckPointEnd(xgoal,ygoal,xrobot,yrobot: double): integer;
begin

     if Dist(xgoal-xrobot,ygoal-yrobot) < 0.03
     then result := 1
     else result := 0;
end;

function FMod(x,d: double): double;
begin
  result:=Frac(x/d)*d;
end;



function AngleThreePoints(x1,y1,x2,y2,x3,y3: double): double;
var
a,b: double;
begin
  a:= atan2(y3-y1,x3-x1); // distancia
  b:= atan2(y2-y1,x2-x1); // distancia

  result := diffangle(a,b);

end;

function Wcontrol(Tref,TRobot,Lambda: double): double;
begin
  result := sat(lambda*(Tref-TRobot),1.5);
end;

function DistPoint2Line(x1,y1,x2,y2,x3,y3: double): double;
var
teta,alfa,yp1,d1_3: double;
begin

//procedure TranslateAndRotate(var rx,ry: double; px,py,tx,ty,teta: double);
// Translacao do vector (px,py) segundo o vector (tx,ty)
// seguida de Rotacao do angulo teta

  // angulo da recta no mundo
  teta:= atan2(y2-y1,x2-x1);

  // angulo entre a recta e o vector que une o ponto (x1,y1) ao ponto (x3,y3)
  alfa:= atan2(y3-y1,x3-x1)-teta;

  d1_3:=Dist(x3-x1,y3-y1);

  // distancia do ponto (x3,y3) para a linha que passa pelos pontos (x1,y1) e (x2,y2)
  yp1:= d1_3*sin(alfa);

  result := yp1;
end;

function DistPointInLine(x1,y1,x2,y2,x3,y3: double): double;
var
teta,alfa,yp1,d1_3: double;
begin

//procedure TranslateAndRotate(var rx,ry: double; px,py,tx,ty,teta: double);
// Translacao do vector (px,py) segundo o vector (tx,ty)
// seguida de Rotacao do angulo teta

  // angulo da recta no mundo
  teta:= atan2(y2-y1,x2-x1);

  // angulo entre a recta e o vector que une o ponto (x1,y1) ao ponto (x3,y3)
  alfa:= atan2(y3-y1,x3-x1)-teta;

  d1_3:=Dist(x3-x1,y3-y1);

  // distancia do ponto (x3,y3) para a linha que passa pelos pontos (x1,y1) e (x2,y2)
  yp1:= d1_3*cos(alfa);

  result := yp1;
end;

function DiffAngle(a1,a2: double): double;
begin
  result:=a1-a2;
  if result<0 then begin
    result:=-FMod(-result,2*Pi);
    if result<-Pi then result:=result+2*Pi;
  end else begin
    result:=FMod(result,2*Pi);
    if result>Pi then result:=result-2*Pi;
  end;
end;

function FastArcTan(x: double): double;
begin
  result :=0;
end;

function FastATan2(y,x: double): double;
var ax,ay: double;
begin
  ax:=Abs(x);
  ay:=Abs(y);

  if (ax<1e-10) and (ay<1e-10) then begin;
    result:=0.0;
    exit;
  end;
  if ax>ay then begin
    if x<0 then begin
      result:=FastArcTan(y/x)+pi;
      if result>pi then result:=result-2*pi;
    end else begin
      result:=FastArcTan(y/x);
    end;
  end else begin
    if y<0 then begin
      result:=FastArcTan(-x/y)-pi/2
    end else begin
      result:=FastArcTan(-x/y)+pi/2;
    end;
  end;
end;

function ATan2(y,x: double): double;
var ax,ay: double;
begin
  ax:=Abs(x);
  ay:=Abs(y);

  if (ax<1e-10) and (ay<1e-10) then begin;
    result:=0.0;
    exit;
  end;
  if ax>ay then begin
    if x<0 then begin
      result:=ArcTan(y/x)+pi;
      if result>pi then result:=result-2*pi;
    end else begin
      result:=ArcTan(y/x);
    end;
  end else begin
    if y<0 then begin
      result:=ArcTan(-x/y)-pi/2
    end else begin
      result:=ArcTan(-x/y)+pi/2;
    end;
  end;
end;

function Sign(a: double): double;
begin
  if a<0 then result:=-1 else result:=1;
end;

function Sat(a,limit: double): double;
begin
 if a>limit then a:=limit;
 if a<-limit then a:=-limit;
 result:=a;
end;


function IncWrap(var v: integer; size: integer; step:integer=1): integer;
begin
  inc(v,step);
  if v>=size then v:=v-size;
  Result:=v;
end;

function DecWrap(var v: integer; size: integer; step:integer=1): integer;
begin
  dec(v,step);
  if v<0 then v:=v+size;
  Result:=v;
end;

procedure SwapInts(var v1, v2: integer);
var t: integer;
begin
  t:=v1;
  v1:=v2;
  v2:=t;
end;

function Rad(xw: double):double;
begin
  result:=xw*(pi/180);
end;

function deg(xw: double):double;
begin
  result:=xw*(180/pi);
end;

procedure QSortSwapElem(a,b: pbyte; size: integer);
var i: integer;
    tmp: byte;
begin
  if a=b then exit;
  for i:=0 to size-1 do begin
    tmp:=a^;
    a^:=b^;
    b^:=tmp;
    inc(a);
    inc(b);
  end;
end;

procedure QSortSub(base: pointer; num_elem,size_elem: integer; func: QSortCmpFunc; iLo, iHi: integer);
var Lo,Hi: integer;
    MidPtr: pbyte;
begin
  Lo := iLo;
  Hi := iHi;
  MidPtr := pbyte(LongWord(base)+LongWord(((iLo + iHi) div 2)*size_elem));
  repeat
    while func(pbyte(LongWord(base)+LongWord(Lo*size_elem))^,MidPtr^)<0 do Inc(Lo);
    while func(pbyte(LongWord(base)+LongWord(Hi*size_elem))^,MidPtr^)>0 do Dec(Hi);
    if Lo <= Hi then
    begin
      QSortSwapElem(
        pbyte(LongWord(base)+LongWord(Lo*size_elem)),
        pbyte(LongWord(base)+LongWord(Hi*size_elem)),
        size_elem);
      Inc(Lo);
      Dec(Hi);
    end;
  until Lo > Hi;
  if Hi > iLo then QSortSub(base,num_elem,size_elem,func, iLo, Hi);
  if Lo < iHi then QSortSub(base,num_elem,size_elem,func, Lo, iHi);
end;

procedure QSort(base: pointer; num_elem,size_elem: integer; func: QSortCmpFunc);
begin
  if num_elem<2 then exit;
  QSortSub(base,num_elem,size_elem,func,0,num_elem-1);
end;

function InternalProductCosine(v1x,v1y,v2x,v2y: double): double;
var d: double;
begin
  d:=dist(v1x,v1y)*dist(v2x,v2y);
  if abs(d)<1e-6 then result:=-1
  else result:=(v1x*v2x+v1y*v2y)/d;
end;

procedure TranslateAndRotate(var rx,ry: double; px,py,tx,ty,teta: double);
var vx,vy: double;
begin
// Translacao do vector (px,py) segundo o vector (tx,ty)
// seguida de Rotacao do angulo teta

  vx:=px+tx;
  vy:=py+ty;
  rx:=vx*cos(teta)-vy*sin(teta);
  ry:=vx*sin(teta)+vy*cos(teta);
end;


procedure TranslateAndRotateInv(var rx,ry: double; px,py,tx,ty,teta: double);
var vx,vy: double;
begin
// Translacao do vector (px,py) segundo o vector (tx,ty)
// seguida de Rotacao do angulo teta
// com matriz inversa
  vx:=px+tx;
  vy:=py+ty;

  rx:=vx*cos(teta)+vy*sin(teta);
  ry:=vx*-sin(teta)+vy*cos(teta);

end;

procedure RotateAndTranslate(var rx,ry: double; px,py,tx,ty,teta: double);
var xrot,yrot: double;
begin
  //posicao
  xrot:=px*cos(teta)-py*sin(teta);
  yrot:=px*sin(teta)+py*cos(teta);
  rx:=xrot+tx;
  ry:=yrot+ty;

end;

procedure calcNextBallXY(var rx, ry: double; px, py, vx, vy, tx, ty,
  teta: double; forwardStep: integer);
var xrot,yrot: double;
    x_next,y_next:double;
begin
  x_next:=px+vx*forwardStep*0.025;
  y_next:=py+vy*forwardStep*0.025;
  //posicao
  xrot:=x_next*cos(teta)-y_next*sin(teta);
  yrot:=x_next*sin(teta)+y_next*cos(teta);
  rx:=xrot+tx;
  ry:=yrot+ty;
end;

function NormalizeAngle(ang: double): double;
var a: double;
begin
  a:=FMod(ang+Pi,2*Pi);
  if a<0 then result:=a+Pi
  else result:=a-Pi;
end;

function InFrustum(xc,yc,xp,yp,ang,widthAng: double): boolean;
begin
  result:=2*abs(DiffAngle(atan2(yp-yc,xp-xc),ang))<widthAng;
end;

procedure DrawCovElipse(x,y,cov_x,cov_y, cov_xy: double; n: integer; CNV: TCanvas);
var i,x1,y1: integer;
    alfa,te,ce,se,qxx,qyy,A,phi,xx,yy,wx,wy: double;
begin
  with CNV do begin //tcanvas
      alfa:=0.5;
      te:=0.5*atan2(2*cov_xy,cov_x-cov_y);
      ce:=cos(te);
      se:=sin(te);
      qxx:=sqrt(abs(cov_y*se*se+cov_x*ce*ce+2*cov_xy*se*ce));
      qyy:=sqrt(abs(cov_y*ce*ce+cov_x*se*se-2*cov_xy*se*ce));
      A:=sqrt(-2*ln(1-alfa));
      for i:=0 to n do begin
        phi:=i*2*pi/n;
        xx:=A*qxx*cos(phi);
        yy:=A*qyy*sin(phi);
        wx:=ce*xx-se*yy;
        wy:=se*xx+ce*yy;
        WorldToMap(x+wx,y+wy,x1,y1);
        if i=0 then moveto(x1,y1) else lineto(x1,y1);
    end;
  end;
end;

procedure RotateCov( const incov_x,incov_y,incov_xy: double; out cov_x,cov_y, cov_xy: double; teta: double);
var ce,se,t1,t3,t5,t6: double;
begin              // teta=0
  ce:=cos(teta);   // 1
  se:=sin(teta);   // 0
  t1:= ce*ce;      // 1
  t3:= ce*se;      // 0
  t5:= 2.0*t3*incov_xy;  // 0
  t6:= se*se;            // 0
  cov_xy:= t3*incov_x-t6*incov_xy+t1*incov_xy-t3*incov_y;
  cov_x:= t1*incov_x+t5+t6*incov_y;
  cov_y:= t6*incov_x-t5+t1*incov_y;
end;

function RotateCovM( P: TDMatrix): TDMatrix;
begin

end;

procedure ZeroMemory(Ptr: Pointer; Len: integer);
begin
  FillChar(Ptr^,len,0);
end;

function GetTickCount: LongWord;
var tv: TTimeVal;
begin
  fpGetTimeOfDay(@tv,nil);
  if FirstTimeValSec=0 then FirstTimeValSec:=tv.tv_sec;
  result:=(tv.tv_sec-FirstTimeValSec)*1000+(tv.tv_usec div 1000);
end;

procedure CopyMemory(DPtr,SPtr: Pointer; Len: integer);
begin
  Move(SPtr^,DPtr^,len);
end;


procedure MVEstInit(var MV: TMVEst);
begin
  with MV do begin
    Vmean:=0;
    Vcov:=0;
    speed:=0.95;
    n:=0;
  end;
end;

procedure MVEstAddValue(var MV: TMVEst; v: double);
begin
  with MV do begin
    Vmean:=Vmean*speed+v*(1-speed);
    Vcov:=Vcov*speed+sqr(Vmean-v)*(1-speed);
    inc(n);
  end;
end;

procedure mse(var e:double; m1,m2: TDMatrix; N: integer);
var
i: integer;
aux:double;
begin

  e:=0;

  for i:=0 to N-1 do
  begin
    aux:=m1.getv(i,0)-m2.getv(i,0);
    e := e + aux*aux;
  end;

  e:=e/N;

end;

procedure ClearLinearReg(var TL: TLinearReg);
begin
  with TL do begin
    Sx:=0;
    Sy:=0;
    Sxy:=0;
    Sx2:=0;
    N:=0;
    a:=0;
    b:=0
  end;
end;

procedure AddXYtoLinearReg(var TL: TLinearReg; x,y: double);
begin
  with TL do begin
    Sx:=Sx+x;
    Sy:=Sy+y;
    Sxy:=Sxy+x*y;
    Sx2:=Sx2+x*x;
    inc(N);
  end;
end;

procedure CalcLinearReg(var TL: TLinearReg);
var mx,my,d: double;
begin
  with TL do begin
    if N=0 then exit;
    mx:=Sx/N;
    my:=Sy/N;
    d:=N*Sx2-Sx*Sx;
    if abs(d)<1e-8 then exit;
    b:=(N*Sxy-Sx*Sy)/d;
    a:=my-b*mx;
  end;
end;

function AverageAngle(ang1,ang2: double): double;
begin
  result:=NormalizeAngle(ang1+DiffAngle(ang2,ang1)*0.5);
end;

function MidAngle(a1,a2: double): double;
var x,y: double;
begin
  x:=cos(a1)+cos(a2);
  y:=sin(a1)+sin(a2);
  result:=ATan2(y,x);
end;


function FiltAngle(a1, a2, lamb: double): double;
var x,y: double;
begin
  x:=lamb*cos(a1)+(1-lamb)*cos(a2);
  y:=lamb*sin(a1)+(1-lamb)*sin(a2);
  result:=ATan2(y,x);
end;

function GetTimeAndDate(tipo:integer) : string;
var
  d,t: string;
begin
  DateTimeToString(d, 'ddmmyy',Date);
  DateTimeToString(t, 'hhnnss',Time);
  case tipo of
    0: result:=d;
    1: begin
      DateTimeToString(t, 'hhnnsszzz',Time);
      result:=t;
    end;
    2: result:=d+'_'+t;
  end;
end;

function innerProduct(x1, y1, x2, y2: double): double;
var x1n,x2n,y1n,y2n:double;
begin
  x1n:=x1;
  x2n:=x2;
  y1n:=y1;
  y2n:=y2;
  NormalizeVector(x1n,y1n);
  NormalizeVector(x2n,y2n);
  result:=x1n*x2n+y1n*y2n;
end;


end.

