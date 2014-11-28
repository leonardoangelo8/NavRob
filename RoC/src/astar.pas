unit Astar;

{$mode objfpc}{$H+}

interface

uses Main,RoCConsts,SysUtils,Field;

const
  MaxTrajectoryCount = 128;

type
  TAvoid=(avoidRobot,avoidObstacles);
  TAvoidSet=set of TAvoid;

type
  TTrajectoryPoint = record
    x, y: double;
    teta, teta_power: double;
    vRef : double;
  end;

  type
  TTrajectory = record
    count: integer;
    currentPoint : integer;
    distance: double;
    static : boolean; // is trajectory static or dynamicly (A*) generated?
    index : integer;
    predIndex : integer;
    varSpeed : boolean;
    pts: array[0 .. MaxTrajectoryCount - 1] of TTrajectoryPoint;
  end;

  type
  TRoundObstacle=record
    x,y,r: double;
    used: boolean;
  end;

procedure SatFieldLimits(var x,y: double);
function RobotBestPath(num: integer; tg_x, tg_y:double; var traj: TTrajectory; avoid: TAvoidSet): boolean;
//function ObstacleInSegment(st_x,st_y,tg_x,tg_y: double; var obs: array of TRoundObstacle; num_obs: integer): boolean;

var
  avoided_x,avoided_y: array [0..MaxRobots-1] of double;
  avoided: array [0..MaxRobots-1] of boolean;

// -----------------------------------------------------
//     AStar definitions

const
  // TODO - moved to DecConsts as a global variable
  // total space where the robot can actually move around
  AStarSlack = 0.1;
  //manuel
  TotalFieldWidth = (FieldWidth + FieldOutsideSpace * 2 - (RobotSpace + AStarSlack));
  TotalFieldLength = (FieldLength + FieldOutsideSpace * 2 - (RobotSpace + AStarSlack));

 // AStarGridYSize = 128;
  //manuel
  AStarGridYSize = 256;
  // the "2" are for the extra cells needed to draw an all around obstacle
  AStarGridCellSize = (TotalFieldWidth / (AStarGridYSize - 2));
  // make sure we have an even number of cells so that the 0.0 coordinate is in the middle
  AStarGridXSize = (round((TotalFieldLength / AStarGridCellSize) + 2) div 2) * 2;

  AStarVirgin = 0;
  AStarObstacle = 1;
  AStarClosed = 2;
  AStarOpen = 3;
  //manuel
  AStarGoals = 4;

  FixedPointConst = 65536;

  sqrt2=round(1.4142135623730950488016887242097 * FixedPointConst);


type
  TGridCoord = record
    x,y: Smallint;
  end;

  TNeighbour = record
    x,y: Smallint;
    dist: integer;
  end;

const
  nilCoord: TGridCoord = (x:0; y:0);

  EightWayNeighbours: array[0..7] of TNeighbour =
    ((x:1;  y:0; dist:FixedPointConst), (x:1;  y:-1; dist:sqrt2), (x:0; y:-1; dist:FixedPointConst), (x:-1; y:-1; dist:sqrt2),
     (x:-1; y:0; dist:FixedPointConst), (x:-1; y:1; dist:sqrt2),  (x:0; y:1; dist:FixedPointConst),  (x:1;  y:1; dist:sqrt2));

type
  PAStarCell = ^TAStarCell;
  TAStarCell = record
    G,H: integer;
    ParentDir: smallint;
    HeapIdx: smallint;
    MyCoord: TGridCoord;
  end;

  TAStarGridState = array[0..AStarGridXSize-1, 0..AStarGridYSize-1] of byte;
  TAStarGrid = array[0..AStarGridXSize-1, 0..AStarGridYSize-1] of TAStarCell;

  TAStarHeapArray = record
    count: integer;
    data: array[0 .. 2047] of PAStarCell;
  end;


  TAStarProfile = record
    RemovePointFromAStarList_count: integer;
    RemoveBestFromAStarList_count: integer;
    AddToAStarList_count: integer;
    HeapArrayTotal: integer;
    comparesTotal: integer;
    iter: integer;
  end;

  TAStarMap = record
    InitialPoint, TargetPoint: TGridCoord;
    Grid: TAStarGrid;
    GridState: TAStarGridState;
    HeapArray: TAStarHeapArray;
    Profile: TAStarProfile;
    EucliDistK: double;
  end;

procedure AStarClear(var Map: TAStarMap);
procedure AStarInit(var Map: TAStarMap);
procedure AStarStep(var Map: TAStarMap);
procedure AStarGo(var Map: TAStarMap);
procedure RecalcSqrtCache(var Map: TAStarMap);

var
  AStarMap: TAStarMap;
  CalcHCache: array[0..AStarGridXSize-1, 0..AStarGridYSize-1] of integer;

implementation

uses Graphics, Robots, Roles, Tatic, Utils, Param, Dialogs;

// -----------------------------------------------------------------------
//     AStar implementation

procedure AStarGridToWorld(var pnt: TGridCoord; var wx,wy: double);
begin
  wx := (pnt.x - (AStarGridXSize div 2)) * AStarGridCellSize + AStarGridCellSize / 2;
  wy := (pnt.y - (AStarGridYSize div 2)) * AStarGridCellSize + AStarGridCellSize / 2;
end;

procedure AStarWorldToGrid(wx,wy: double; var pnt: TGridCoord);
begin
  pnt.x := round(wx / AStarGridCellSize + AStarGridXSize div 2);
  if pnt.x < 0 then pnt.x := 0;
  if pnt.x >= AStarGridXSize then pnt.x := AStarGridXSize - 1;
  pnt.y := round(wy / AStarGridCellSize + AStarGridYSize div 2);
  if pnt.y < 0 then pnt.y := 0;
  if pnt.y >= AStarGridYSize then pnt.y := AStarGridYSize - 1;
end;

procedure RecalcSqrtCache(var Map: TAStarMap);
var x, y: integer;
begin
  for x := 0 to AStarGridXSize - 1 do begin
    for y := 0 to AStarGridYSize - 1 do begin
      CalcHCache[x,y] := round(sqrt(x * x * 1.0 + y * y) * Map.EucliDistK * FixedPointConst);
      Map.Grid[x,y].MyCoord.x := x;
      Map.Grid[x,y].MyCoord.y := y;
    end;
  end;
end;

procedure AStarClear(var Map: TAStarMap);
var i: integer;
    pt1, pt2:TGridCoord;
begin
  // clear the grid, set all nodes to state "virgin"
  FillChar(Map.GridState,sizeof(TAStarGridState), ord(AStarVirgin));
  //ZeroMemory(@Map.GridState[0,0],sizeof(TAStarGridState));

  // Build the wall around the grid,
  // this way, we don't have to worry about the frontier conditions
  for i:=0 to AStarGridXSize-1 do begin
    Map.GridState[i,0]:= AStarObstacle;
    Map.GridState[i,AStarGridYSize-1]:= AStarObstacle;
  end;

  for i:=0 to AStarGridYSize-1 do begin
    Map.GridState[0,i]:= AStarObstacle;
    Map.GridState[AStarGridXSize-1,i]:= AStarObstacle;
  end;
  // clear the heap
  Map.HeapArray.Count := 0;
end;


function GridCoordIsEqual( G1,G2: TGridCoord): boolean;
begin
  if (g1.x = g2.x) and (g1.y = g2.y) then result := true
  else result := false;
end;

function GridCoordIsNil( G1: TGridCoord): boolean;
begin
  if (g1.x = 0) and (g1.y = 0) then result := true
  else result := false;
end;


// ---------------------------------------------------------------
//     Heap Operations

function CalcHeapCost(var Map: TAStarMap; idx: integer): integer;
begin
  with Map.HeapArray.data[idx]^ do begin
    result := G + H;
  end;
end;

procedure SwapHeapElements(var Map: TAStarMap; idx1, idx2: integer);
var ptr1, ptr2: PAStarCell;
begin
  ptr1 := Map.HeapArray.data[idx1];
  ptr2 := Map.HeapArray.data[idx2];
  ptr1^.HeapIdx := idx2;
  ptr2^.HeapIdx := idx1;
  Map.HeapArray.data[idx1] := ptr2;
  Map.HeapArray.data[idx2] := ptr1;
end;

procedure UpdateHeapPositionByPromotion(var Map: TAStarMap; idx: integer);
var parent_idx: integer;
    node_cost: integer;
begin
  // if we are on the first node, there is no way to promote
  if idx = 0 then exit;
  // calc node cost
  node_cost := CalcHeapCost(Map, idx);
  // repeat until we can promote no longer
  while true do begin
    // if we are on the first node, there is no way to promote
    if idx = 0 then exit;

    parent_idx := (idx - 1) div 2;
    // if the parent is better than we are, there will be no promotion
    if CalcHeapCost(Map, parent_idx) < node_cost then exit;
    // if not, just promote it
    SwapHeapElements(Map, idx, parent_idx);
    idx:= parent_idx;
  end;
end;

// update one node after increasing its cost
procedure UpdateHeapPositionByDemotion(var Map: TAStarMap; idx: integer);
var c1, c2, cost: integer;
    idx_child, new_idx: integer;
begin

  cost := CalcHeapCost(Map, idx);

  while true do begin

    idx_child := idx * 2 + 1;
    // if the node has no childs, there is no way to demote
    if idx_child >= Map.HeapArray.count then exit;

    // calc our cost and the first node cost
    c1 := CalcHeapCost(Map, idx_child);
    // if there is only one child, just compare with this one
    if idx_child + 1 >= Map.HeapArray.count then begin
      // if we are better than this child, then no demotion
      if cost < c1 then exit;
      // if not, then do the demotion
      SwapHeapElements(Map, idx, idx_child);
      exit;
    end;

    // calc the second node cost
    c2 := CalcHeapCost(Map, idx_child + 1);

    // select the best node to demote to
    new_idx := idx;
    if c2 < cost then begin
      if c1 < c2 then begin
        new_idx := idx_child;
      end else begin
        new_idx := idx_child + 1;
      end;
    end else if c1 < cost then begin
      new_idx := idx_child;
    end;

    // if there is no better child, just return
    if new_idx = idx then exit;

    // if we want to demote, then swap the elements
    SwapHeapElements(Map, idx, new_idx);
    idx := new_idx;
  end;
end;


procedure RemoveBestFromAStarList(var Map: TAStarMap; out Pnt: TGridCoord);
begin
  inc(Map.Profile.RemoveBestFromAStarList_count);

  with Map.HeapArray do begin
    // return the first node
    Pnt := data[0]^.MyCoord;
    // move the last node into the first position
    data[count - 1]^.HeapIdx := 0;
    data[0] := data[count - 1];
    // update the array size
    Dec(count);
  end;
  // re-sort that "first" node
  UpdateHeapPositionByDemotion(Map,0);
end;


procedure AddToAStarList( var Map: TAStarMap; Pnt: TGridCoord);
var idx: integer;
begin
  inc(Map.Profile.AddToAStarList_count);

  // update the grid state
  Map.GridState[Pnt.x, Pnt.y]:=AStarOpen;

  // insert at the bottom of the heap
  idx := Map.HeapArray.count;
  Map.HeapArray.data[idx] := @Map.Grid[Pnt.x,Pnt.y];
  Map.Grid[Pnt.x,Pnt.y].HeapIdx := idx;
  Inc(Map.HeapArray.count);

  // update by promotion up to the right place
  UpdateHeapPositionByPromotion(Map, idx);
end;


function CalcH(var Map: TAStarMap; Pi, Pf: TGridCoord): integer;
begin
  Result:= CalcHCache[Abs(Pi.x-Pf.x), Abs(Pi.y-Pf.y)];
end;


procedure AStarStep(var Map: TAStarMap);
var curPnt, NewPnt: TGridCoord;
    ith, NeighboursCount: integer;
    newG: integer;
begin
  inc(Map.Profile.iter);
  inc(Map.Profile.HeapArrayTotal, Map.HeapArray.count);

  RemoveBestFromAStarList(Map,curPnt);
  Map.GridState[curPnt.x,curPnt.Y]:=AStarClosed;

  NeighboursCount := 8;
  for ith:=0 to NeighboursCount-1 do begin
    NewPnt.x := CurPnt.x + EightWayNeighbours[ith].x;
    NewPnt.y := CurPnt.y + EightWayNeighbours[ith].y;

    case Map.GridState[NewPnt.x, NewPnt.y] of
      AStarClosed: continue;

      AStarVirgin: begin
        with Map.Grid[NewPnt.x, NewPnt.y] do begin
          ParentDir := ith;
          G := Map.Grid[CurPnt.x, CurPnt.y].G + EightWayNeighbours[ith].dist;
          H := CalcH(Map, NewPnt, Map.TargetPoint);
        end;
        AddToAStarList(Map,NewPnt);
      end;

      AStarOpen: begin
        newG := Map.Grid[CurPnt.x, CurPnt.y].G + EightWayNeighbours[ith].dist;
        if newG < Map.Grid[NewPnt.x, NewPnt.y].G then begin
          Map.Grid[NewPnt.x, NewPnt.y].G := newG;
          Map.Grid[NewPnt.x, NewPnt.y].ParentDir := ith;
          UpdateHeapPositionByPromotion(Map, Map.Grid[NewPnt.x, NewPnt.y].HeapIdx);
        end;
      end;
    end;
  end;

end;


// check if the given point is inside an obstacle and move it to the closest open space
procedure AStarCheckBoundary(var pnt: TGridCoord; var Map: TAStarMap);
var i, j, x, y: integer;
begin
  // if we are not inside an obstacle, just return
  if Map.GridState[pnt.x,pnt.y] <> AStarObstacle then exit;

  for i := 1 to AStarGridXSize - 1 do begin
    for j := 7 downto 0 do begin
      x := pnt.x + EightWayNeighbours[j].x * i;
      y := pnt.y + EightWayNeighbours[j].y * i;
      if (x >= 0) and (y >= 0) and (x < AStarGridXSize) and (y < AStarGridYSize) then begin
        if Map.GridState[x,y] <> AStarObstacle then begin
          pnt.x := x;
          pnt.y := y;
          exit;
        end;
      end;
    end;
  end;
end;

procedure AStarCheckBoundaries(var Map: TAStarMap);
begin
  AStarCheckBoundary(Map.InitialPoint, Map);
  AStarCheckBoundary(Map.TargetPoint, Map);
end;


procedure AStarInit(var Map: TAStarMap);
begin
  zeromemory(@(Map.Profile),sizeof(Map.Profile));
  AddToAStarList(Map, Map.InitialPoint);
  Map.Grid[Map.InitialPoint.x, Map.InitialPoint.y].H:= CalcH( Map, Map.InitialPoint, Map.TargetPoint);
end;


procedure AStarGo(var Map: TAStarMap);
begin
  AStarCheckBoundaries(Map);
  AStarInit(Map);
  while true do begin
    AStarStep(Map);

    //if Map.iter >= 400 then exit;
    // found the way
    if Map.GridState[Map.TargetPoint.x, Map.TargetPoint.y] = AStarClosed then break;
    // there is no path
    if Map.HeapArray.count = 0 then break;
  end;
end;


procedure NextPnt(var Map: TAStarMap; var pnt: TGridCoord);
var dir: integer;
begin
  dir := Map.Grid[pnt.x, pnt.y].ParentDir;
  if (dir<0) or (dir>7) then exit; //TODO isto nao deve acontecer, só deve haver dir entre 0 e 7
  pnt.x := pnt.x - EightWayNeighbours[dir].x;
  pnt.y := pnt.y - EightWayNeighbours[dir].y;
end;

procedure AStarBuildTrajectory(st_x,st_y,tg_x,tg_y: double; var Map: TAStarMap; var traj: TTrajectory);
var px, py, last_px, last_py: double;
    cur_pnt, st_pnt, tg_pnt: TGridCoord;
    done: boolean;
    pixx1, pixy1, ite_count: integer;
begin
  traj.count := 0;

  AStarWorldToGrid(st_x, st_y, st_pnt);
  AStarWorldToGrid(tg_x, tg_y, tg_pnt);

  cur_pnt := Map.TargetPoint;
  if GridCoordIsEqual(st_pnt, cur_pnt) then
    NextPnt(Map, cur_pnt);

  last_px := st_x;
  last_py := st_y;

  done := false;
  traj.distance := 0;
  ite_count := 0;
  while not done do begin
    if GridCoordIsEqual(tg_pnt, cur_pnt) then begin
      px := tg_x;
      py := tg_y;
      done := true;
    end else begin
      AStarGridToWorld(cur_pnt, px, py);
      if GridCoordIsEqual(Map.InitialPoint, cur_pnt) then begin
        done := true;
      end else begin
        NextPnt(Map, cur_pnt);
      end;
    end;

    // fail-safe to cover path not found cases
    Inc(ite_count);
    if ite_count >= 256 then begin
      break;
    end;

    //if traj.count >= MaxTrajectoryCount then begin

    if traj.count < MaxTrajectoryCount then begin
      with traj.pts[traj.count] do begin
        x := px;
        y := py;
        teta := 0;
        teta_power := 0;
      end;
      Inc(traj.count);
    end;
    traj.distance := traj.distance + Dist(px - last_px, py - last_py);
    last_px := px;
    last_py := py;
  end;
end;

procedure AStarGetBestPath(st_x,st_y,tg_x,tg_y: double; var traj: TTrajectory; var obs: array of TRoundObstacle; num_obs: integer);
var i, x, y: integer;
    ptl, pbr, cur_pnt: TGridCoord;
    wx,wy: double;
begin
  AStarClear(AStarMap);

  for i := 0 to num_obs - 1 do begin
    with obs[i] do begin
      AStarWorldToGrid(x - r, y - r, ptl);
      AStarWorldToGrid(x + r, y + r, pbr);
    end;
    for x := ptl.x to pbr.x do begin
      cur_pnt.x := x;
      for y := ptl.y to pbr.y do begin
        cur_pnt.y := y;
        AStarGridToWorld(cur_pnt, wx, wy);
        if Dist(wx - obs[i].x, wy - obs[i].y) < obs[i].r then begin
          AStarMap.GridState[cur_pnt.x,cur_pnt.y] := AStarObstacle;
        end;
      end;
    end;
  end;

  // use the target as starting point for the AStar. The advantages are:
  // - usually the place where the robot wants to go to is more crowded than the
  //      the place it is at. AStar works better when the target is the point with
  //      more space around it
  // - we need just a few steps of the trajectory to give to the controller. If we
  //      start at the "target" of the AStar we can do this easily without having
  //      to follow it all the way to the starting point
  AStarWorldToGrid(tg_x, tg_y, AStarMap.InitialPoint);
  AStarWorldToGrid(st_x, st_y, AStarMap.TargetPoint);



  AStarGo(AStarMap);
  AStarBuildTrajectory(st_x,st_y,tg_x,tg_y, AStarMap, traj);
end;

// -----------------------------------------------------------------------
//     Obstacle avoidance code

procedure SatFieldLimits(var x,y: double);
begin
  if x > MaxFieldX then x := MaxFieldX;
  if x < -MaxFieldX then x := -MaxFieldX;
  if y > MaxFieldY then y := MaxFieldY;
  if y < -MaxFieldY then y := -MaxFieldY;
end;

function inside_field(x,y: double): boolean;
begin
  result := (abs(x) < MaxFieldX) and (abs(y) < MaxFieldY);
end;

function CalcIntersectionPoint(x1,y1,x2,y2: double; var vx,vy: double ; obs: TRoundObstacle): boolean;
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

// create a "trajectory" with only one segment to tx,ty
procedure BuildSimpleTrajectory(var traj: TTrajectory; sx,sy,tx,ty: double);
begin
  traj.count := 1;
  traj.distance := Dist(tx-sx, ty-sy);
  with traj.pts[0] do begin
    x := tx;
    y := ty;
    teta := 0;
    teta_power := 0;
  end;
end;

// checks for obstacles in a segment
function ObstacleInSegment(st_x,st_y,tg_x,tg_y: double; var obs: array of TRoundObstacle; num_obs: integer): boolean;
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


procedure GetBestPath(st_x,st_y,tg_x,tg_y: double; var traj: TTrajectory; var obs: array of TRoundObstacle; num_obs: integer);
var i: integer;
    pixx1,pixy1,pixx2,pixy2: integer;
begin
  // if there are no obstacles in this segment, just return a simple trajectory straight to it

  if not ObstacleInSegment(st_x,st_y,tg_x,tg_y,obs,num_obs) then begin
    BuildSimpleTrajectory(traj, st_x, st_y, tg_x, tg_y);
    exit;
  end;

  AStarGetBestPath(st_x, st_y, tg_x, tg_y, traj, obs, num_obs);
end;

function RobotBestPath(num: integer; tg_x, tg_y:double; var traj: TTrajectory; avoid: TAvoidSet): boolean;
var obs: array[0..MaxRobots+MaxObstacles] of TRoundObstacle;  //Acke provisório o mais 70
    i,nobs: integer;
    dball,r1: double;
    initial_tgx,initial_tgy,d1,d2,d: double;
    tmpx,tmpy: double;
    pixx1,pixx2,pixy1,pixy2: integer;
    found_path, target_inside: boolean;
begin
  result := true;

  if Dist(RobotState[num].x - tg_x, RobotState[num].y - tg_y) < AStarGridCellSize/2 then begin
    BuildSimpleTrajectory(traj,RobotState[num].x,RobotState[num].y,tg_x,tg_y);
    exit;
  end;

  nobs := 0;

  SatFieldLimits(tg_x, tg_y);

  if avoidRobot in avoid then begin
    for i:=0 to MaxRobots-1 do begin
      if (RobotStatus[i].active) and (i<>num) then begin
        obs[nobs].x:=RobotState[i].x;
        obs[nobs].y:=RobotState[i].y;
        obs[nobs].r:=RobotSpace;
        inc(nobs);
      end;
    end;
  end;

  if avoidObstacles in avoid then begin
    for i:=0 to Obstacles.count-1 do begin
      obs[nobs].x:=Obstacles.Centers[i].x;
      obs[nobs].y:=Obstacles.Centers[i].y;
      obs[nobs].r:=Obstacles.Centers[i].r + RobotSpace/2;
      inc(nobs);
    end;
  end;

    obs[nobs].x := OurGoalX-FieldDims.GoalDepth;
    obs[nobs].y := 0.5;
    obs[nobs].r := 0.6;
    inc(nobs);
    obs[nobs].x := OurGoalX-FieldDims.GoalDepth;
    obs[nobs].y := -0.5;
    obs[nobs].r := 0.6;
    inc(nobs);
    obs[nobs].x := TheirGoalX+FieldDims.GoalDepth;
    obs[nobs].y := 0.5;
    obs[nobs].r := 0.6;
    inc(nobs);
    obs[nobs].x := TheirGoalX+FieldDims.GoalDepth;
    obs[nobs].y := -0.5;
    obs[nobs].r := 0.6;
    inc(nobs);

    obs[nobs].x := TheirGoalX;
    obs[nobs].y := -FieldDims.GoalWidth+1;
    obs[nobs].r := 0.6;
    inc(nobs);
    obs[nobs].x := TheirGoalX;
    obs[nobs].y := FieldDims.GoalWidth-1;
    obs[nobs].r := 0.6;
    inc(nobs);

    obs[nobs].x := OurGoalX;
    obs[nobs].y := -FieldDims.GoalWidth+1;
    obs[nobs].r := 0.6;
    inc(nobs);
    obs[nobs].x := OurGoalX;
    obs[nobs].y := FieldDims.GoalWidth-1;
    obs[nobs].r := 0.6;
    inc(nobs);

  GetBestPath(RobotState[num].x, RobotState[num].y, tg_x, tg_y, traj, obs, nobs);


end;


initialization

AStarMap.EucliDistK := 1.3;
RecalcSqrtCache(AStarMap);

end.

