{ dynmatrix v0.1

  CopyRight (C) 2008 Paulo Costa

  This library is Free software; you can rediStribute it and/or modify it
  under the terms of the GNU Library General Public License as published by
  the Free Software Foundation; version 2 of the License.

  This program is diStributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; withOut even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Library General Public License
  for more details.

  You should have received a Copy of the GNU Library General Public License
  along with This library; if not, Write to the Free Software Foundation,
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

  This license has been modified. See File LICENSE.ADDON for more inFormation.
}

unit dynmatrix;

{$mode objfpc}{$H+}

interface

uses
  SysUtils;

type

  { TDMatrix }
  PDoubleArray = ^TDoubleArray;
  TDoubleArray = array[0..$8000000-1] of Double;

  TDMatrix = object
  protected
    data: string;
    rows, cols: Longword;

    function Init(newrows, newcols: Longword): PdoubleArray;
    procedure GetData(out NumRows, NumCols: Longword; out Mda: PdoubleArray);

  public
    procedure SetSize(newrows, newcols: Longword);
    procedure setv(r, c: Longword; v: double);
    function getv(r, c: Longword): double;
    procedure Usetv(r, c: Longword; v: double);
    function Ugetv(r, c: Longword): double;

    function IsGood: boolean;
    function NumCols: Longword;
    function NumRows: Longword;
  end;

  TDoubleFunc = function(v: double): double;

function Mzeros(numrows, numcols: LongWord): TDMatrix;
function Meye(n: Longword): TDMatrix;
function Mrandom(numrows, numcols: LongWord): TDMatrix;
function Minc(numrows, numcols: LongWord): TDMatrix;

function Mtran(const M: TDMatrix): TDMatrix;
function Minv(M: TDMatrix): TDMatrix;
function Minv_fast(M: TDMatrix): TDMatrix;

function Mmin(A: TDMatrix): double;
function Mmax(A: TDMatrix): double;
function MmaxAbs(A: TDMatrix): double;

function Mfunc(A: TDMatrix; f: TDoubleFunc): TDMatrix;

operator + (const A, B: TDMatrix): TDMatrix;
operator + (const A: TDMatrix; k: double): TDMatrix;
operator + (k: double; const A: TDMatrix): TDMatrix;
operator - (const A: TDMatrix): TDMatrix;
operator - (const A, B: TDMatrix): TDMatrix;
operator - (const A: TDMatrix; k: double): TDMatrix;
operator - (k: double; const A: TDMatrix): TDMatrix;
operator * (const A: TDMatrix; k: double): TDMatrix;
operator * (k: double; const A: TDMatrix): TDMatrix;
operator * (const A, B: TDMatrix): TDMatrix;
//operator ** (const m: TMatrix; const expo: integer) OutMat : TMatrix;


// Matrix data internal format
// |     string            |
// | TDoubleArray ...      |
// |<- rows*cols doubles ->|

//function MReverse(M:Matrix): Matrix;
//function MConv(A:Matrix; B:Matrix): Matrix;

////function Mdotmult(A:Matrix; B:Matrix): Matrix;
//function MelementProduct(A:Matrix; B:Matrix): Matrix;

//function Mcrop(M:Matrix; uprow, leftcol, downrow, rightcol: integer): Matrix;
//function Mstamp(M, S: matrix; drow, dcol: integer): Matrix;

//function MOneCol(M:Matrix; col: integer): Matrix;
//function MOneRow(M:Matrix; row: integer): Matrix;
//function MStampCol(M, S: matrix; col: integer): Matrix;
//function MStampRow(M, S: matrix; row: integer): Matrix;

//function Mrowsum(M:Matrix): Matrix;
//function Mcolsum(M:Matrix): Matrix;
//function MColNorm2(M:Matrix): Matrix;

//procedure MShape(M:Matrix; newrow,newcol: integer);
//procedure MShape2col(M:Matrix);
//procedure MShape2row(M:Matrix);


//procedure MArray2Matrix(A: matrix; const D: array of double);

//function VDotProduct(A: TDMatrix; B: TDMatrix): TDMatrix;
//function VExtProduct(A: TDMatrix; B: TDMatrix): TDMatrix;
//function VNorm1(A: TDMatrix; B: TDMatrix): TDMatrix;
//function VNorm2(A: TDMatrix; B: TDMatrix): TDMatrix;
//function VNormInf(A: TDMatrix; B: TDMatrix): TDMatrix;


implementation

uses math;


{ $RANGECHECKS OFF }


// <- Transpose of M
function MTran(const M: TDMatrix): TDMatrix;
var
  Mda,Tda: PdoubleArray;
  r,c,rows,cols: Longword;
begin
  M.GetData(rows, cols, Mda);
  //result.SetSize(cols, rows);
  //result.GetData(r, c, Tda);
  Tda := result.Init(cols, rows);
  for c:=0 to cols-1 do
    for r:=0 to rows-1 do begin
      Tda^[r + c*rows]:=Mda^[c + r*cols];
    end;
end;

// Zeros matrix
function Mzeros(numrows, numcols: LongWord): TDMatrix;
begin
  result.SetSize(numrows, numcols);
end;

// Identity matrix
function Meye(n: Longword): TDMatrix;
var Pda: PdoubleArray;
    i: Longword;
begin
  Pda := result.Init(n, n);
  for i := 0 to n - 1 do begin
    Pda^[i + i*n] := 1;
  end;
end;

// Returns a Matrix with (numrows, numcols) elements with random values between 0 e 1
function Mrandom(numrows, numcols: LongWord): TDMatrix;
var Pda: PdoubleArray;
    i: Longword;
begin
  Pda := result.Init(numrows, numcols);
  for i := 0 to numrows * numcols - 1 do begin
    Pda^[i] := random;
  end;
end;

function Minc(numrows, numcols: LongWord): TDMatrix;
var Pda: PdoubleArray;
    i: Longword;
begin
  Pda := result.Init(numrows, numcols);
  for i := 0 to numrows * numcols - 1 do begin
    Pda^[i] := i;
  end;
end;


operator+(const A, B: TDMatrix): TDMatrix;
var
  Adp,Bdp,Sdp: Pdouble;
  Arows,Acols,Brows,Bcols,i: LongWord;
begin
  A.GetData(Arows, Acols,PdoubleArray(Adp));
  B.GetData(Brows, Bcols,PdoubleArray(Bdp));

  if (Arows<>Brows) or (Acols<>Bcols) then
    raise  Exception.Create(format('Cannot add matrix (%d,%d) with matrix (%d,%d)',[Arows, ACols, Brows, Bcols]));

  Sdp := pDouble(result.Init(Arows,Acols));

  for i := 0 to Arows * Acols - 1 do begin
    Sdp^ := Adp^ + Bdp^;
    inc(Sdp);
    inc(Adp);
    inc(Bdp);
  end;
  
end;


operator+(const A: TDMatrix; k: double): TDMatrix;
var
  Adp,Sdp: Pdouble;
  Arows,Acols,i: LongWord;
begin
  A.GetData(Arows, Acols,PdoubleArray(Adp));

  Sdp := pDouble(result.Init(Arows,Acols));

  for i := 0 to Arows * Acols - 1 do begin
    Sdp^ := Adp^ + k;
    inc(Sdp);
    inc(Adp);
  end;

end;


operator + (k: double; const A: TDMatrix): TDMatrix;
begin
  result := A + k;
end;


// <- -A  ie R(i,j) := -A(i,j)
operator-(const A: TDMatrix): TDMatrix;
var
  Adp,Sdp: Pdouble;
  Arows,Acols,i: LongWord;
begin
  A.GetData(Arows, Acols,PdoubleArray(Adp));

  Sdp := pDouble(result.Init(Arows,Acols));

  for i := 0 to Arows * Acols - 1 do begin
    Sdp^ := - Adp^;
    inc(Sdp);
    inc(Adp);
  end;

end;

// <- A-B  ie R(i,j) := A(i,j) - B(i,j)
operator-(const A, B: TDMatrix): TDMatrix;
var
  Adp,Bdp,Sdp: Pdouble;
  Arows,Acols,Brows,Bcols,i: LongWord;
begin
  A.GetData(Arows, Acols,PdoubleArray(Adp));
  B.GetData(Brows, Bcols,PdoubleArray(Bdp));

  if (Arows<>Brows) or (Acols<>Bcols) then
    raise  Exception.Create(format('Cannot subtract matrix (%d,%d) with matrix (%d,%d)',[Arows, ACols, Brows, Bcols]));

  Sdp := pDouble(result.Init(Arows,Acols));

  for i := 0 to Arows * Acols - 1 do begin
    Sdp^ := Adp^ - Bdp^;
    inc(Sdp);
    inc(Adp);
    inc(Bdp);
  end;

end;

operator-(const A: TDMatrix; k: double): TDMatrix;
var
  Adp,Sdp: Pdouble;
  Arows,Acols,i: LongWord;
begin
  A.GetData(Arows, Acols,PdoubleArray(Adp));

  Sdp := pDouble(result.Init(Arows,Acols));

  for i := 0 to Arows * Acols - 1 do begin
    Sdp^ := Adp^ - k;
    inc(Sdp);
    inc(Adp);
  end;

end;

operator-(k: double; const A: TDMatrix): TDMatrix;
var
  Adp,Sdp: Pdouble;
  Arows,Acols,i: LongWord;
begin
  A.GetData(Arows, Acols,PdoubleArray(Adp));

  Sdp := pDouble(result.Init(Arows,Acols));

  for i := 0 to Arows * Acols - 1 do begin
    Sdp^ := k - Adp^;
    inc(Sdp);
    inc(Adp);
  end;

end;

// <- A * k (k: double) ie R(i,j) := A(i,j) * k
operator*(const A: TDMatrix; k: double): TDMatrix;
var
  Adp,Sdp: Pdouble;
  Arows,Acols,i: LongWord;
begin
  A.GetData(Arows, Acols,PdoubleArray(Adp));

  Sdp := pDouble(result.Init(Arows,Acols));

  for i := 0 to Arows * Acols - 1 do begin
    Sdp^ := Adp^ * k;
    inc(Sdp);
    inc(Adp);
  end;

end;

// <- k * A (k: double) ie R(i,j) := A(i,j) * k
operator*(k: double; const A: TDMatrix): TDMatrix;
begin
  result := A * k;
end;


// <- A*B
operator*(const A, B: TDMatrix): TDMatrix;
var
  Ada,Bda,Rda: PdoubleArray;
  Arows,Acols,Brows,Bcols,r,c,i: LongWord;
  sum: double;
begin
  A.Getdata(Arows, Acols, Ada);
  B.Getdata(Brows, Bcols, Bda);

  if Acols<>Brows then
    raise Exception.Create(format('Cannot multiply matrix (%d,%d) with matrix (%d,%d)',[Arows, ACols, Brows, Bcols]));

  Rda := result.Init(Arows,Bcols);

  for r := 0 to Arows-1 do begin
    for c := 0 to Bcols-1 do begin
	    sum := 0;
      for i :=0 to Acols-1 do begin
        sum := sum + Ada^[r*Acols + i] * Bda^[c + i*Bcols];
      end;
	    Rda^[c + r*Bcols] := sum;
    end;
  end;
end;



//// <- A.*B  ie C(i,j):=A(i,j)*B(i,j)
//function Mdotmult(A:Matrix; B:Matrix): Matrix;
//var
  //Adp,Bdp,Tdp: Pdouble;
  //Arows,Acols,Brows,Bcols,i: integer;
//begin
  //Mdata(A,Arows,Acols,PdoubleArray(Adp));
  //Mdata(B,Brows,Bcols,PdoubleArray(Bdp));

  //if Arows<>Brows then
    //raise  EMatrixMismatch.Create('Cannot dot product two matrices with diferent number of rows');
  //if Acols<>Bcols then
    //raise  EMatrixMismatch.Create('Cannot dot product two matrices with diferent number of columns');

  //result:=Mnew(Arows,Acols);
  //Mdata(result,Arows,Acols,PdoubleArray(Tdp));

  //for i:=0 to Arows*Acols-1 do begin
  ////   Pdouble(integer(Tdp)+i*DSIZE)^:=
  ////      Pdouble(integer(Adp)+i*DSIZE)^*Pdouble(integer(Bdp)+i*DSIZE)^;
      //Tdp^:=Adp^*Bdp^;
      //inc(Tdp);
      //inc(Adp);
      //inc(Bdp);
    //end;
//end;


{ TDMatrix }

function TDMatrix.Init(newrows, newcols: Longword): PdoubleArray;
begin
  rows := NewRows;
  cols := NewCols;
  data := stringofchar(#0, (rows * cols) * sizeof(double));
  result := pointer(data);
end;

// Inicializes a matrix with numrows lines and numcols columns
procedure TDMatrix.SetSize(NewRows, NewCols: Longword);
begin
  rows := NewRows;
  cols := NewCols;
  data := stringofchar(#0, (rows * cols) * sizeof(double));
end;


// Write v to Element [r,c]
procedure TDMatrix.setv(r, c: Longword; v: double);
var
  Pda: PdoubleArray;
  TotRows, TotCols: Longword;
begin
  uniquestring(data);
  GetData(TotRows, TotCols, Pda);
  if (r >= TotRows) or (c >= TotCols) then
    raise Exception.Create(format('Invalid (row,col) value. Matrix is (%d,%d), element required is (%d,%d)',[TotRows, TotCols, r,c]));
  Pda^[c + r*TotCols] := v;
end;

// Get Element [r,c]
function TDMatrix.getv(r, c: Longword): double;
var
  Pda: PdoubleArray;
  TotRows, TotCols: Longword;
begin
  GetData(TotRows, TotCols, Pda);
  if (r >= TotRows) or (c >= TotCols) then
    raise Exception.Create(format('Invalid (row,col) value. Matrix is (%d,%d), element required is (%d,%d)',[TotRows, TotCols, r,c]));
  result := Pda^[c + r*TotCols];
end;

// Write to v Element [r,c] , ignore operation if r,c is out of bounds
procedure TDMatrix.Usetv(r, c: Longword; v: double);
var
  Pda: PdoubleArray;
  TotRows, TotCols: Longword;
begin
  uniquestring(data);
  GetData(TotRows, TotCols, Pda);
  if (r >= TotRows) or (c >= TotCols) then exit;
  Pda^[c + r*TotCols] := v;
end;

// Get Element [r,c], 0 if r,c out of bounds
function TDMatrix.Ugetv(r, c: Longword): double;
var
  Pda: PdoubleArray;
  TotRows, TotCols: Longword;
begin
  GetData(TotRows, TotCols, Pda);
  if (r >= TotRows) or (c >= TotCols) then begin
    result := 0;
    exit;
  end;
  result := Pda^[c + r*TotCols];
end;


procedure TDMatrix.GetData(out NumRows, NumCols: Longword; out Mda: PdoubleArray);
begin
  Mda:=pointer(data);
  NumRows := rows;
  NumCols := cols;
  
  if Mda=nil then
    raise Exception.Create('Invalid matrix: nil data');

  if not (rows>0) then
    raise  Exception.Create('Invalid number of rows:'+inttostr(rows));

  if not (cols>0) then
    raise  Exception.Create('Invalid number of columns:'+inttostr(cols));

  if longword(length(data)) <> (rows * cols) * sizeof(double) then
    raise  Exception.Create('Invalid matrix: incompatible data size');
end;

// Get total number of columns
function TDMatrix.NumCols: Longword;
begin
  result := cols;
end;


// Get total number of rows
function TDMatrix.NumRows: Longword;
begin
  result := rows;
end;


// Test the matrix goodness:
//  if it's not an empty string
//  if the number of row and cols is not zero
//  if the string size is compatible with expected embeded array
// <- true if it is good

function TDMatrix.IsGood: boolean;
begin
  result:=false;
  if (pointer(data) = nil) or (data = '') then exit;
  if (rows > 0) and (cols > 0) and (longword(length(data)) = ((rows*cols)*sizeof(double))) then
    result:=True
end;


// <- A+B
function MAdd(A: TDMatrix; B: TDMatrix): TDMatrix; inline;
begin
  result := A + B;
end;


// <- M^-1
function Minv(M: TDMatrix): TDMatrix;
var
  Mda, TMP, INV: PdoubleArray;
  ROW, COL: array of Longword;
  MatINV, MatTMP: TDmatrix;
  HOLD , I_pivot , J_pivot: Longword;
  fv, pivot, abs_pivot, rel_eps: double;
  n, i, j, k, r, c, rin, rkn, ck, cj: Longword;
begin
  M.GetData(r, c, Mda);
  if c <> r then
    raise Exception.Create('Cannot invert non-square matrix');

  n := c;
  SetLength(ROW, n);
  SetLength(COL, n);
  MatTMP := MZeros(n, n);
  MatINV := M;

  uniquestring(MatINV.data);
  MatINV.GetData(r, c, INV);
  MatTMP.GetData(r, c, TMP);


  // Set up row and column interchange vectors
  for k := 0  to n-1 do begin
    ROW[k] := k;
    COL[k] := k;
  end;

  // Find largest element
  rel_eps := 0;
  for i := 0 to n-1 do begin
    for j := 0  to n-1 do begin
      fv := abs(INV^[ROW[i]*n + COL[j]]);
      if  fv > rel_eps then begin
        rel_eps := fv ;
      end;
    end;
  end;
  rel_eps := rel_eps * 1e-10;


  // Begin main reduction loop
  for k := 0  to n-1 do begin
    // Find largest element for pivot
    pivot := INV^[ROW[k]*n+COL[k]];
    abs_pivot := abs(pivot);
    I_pivot := k;
    J_pivot := k;
    for i := k to n-1 do begin
      for j := k  to n-1 do begin
        //abs_pivot := abs(pivot);
        fv := INV^[ROW[i]*n+COL[j]];
        if  abs(fv) > abs_pivot then begin
          I_pivot := i;
          J_pivot := j;
          pivot := fv;
          abs_pivot := abs(pivot);
        end;
      end;
    end;
    if abs(pivot) < rel_eps then
      raise Exception.Create(format('Singular matrix: Pivot is %g, max element = %g',[pivot, rel_eps]));

    HOLD := ROW[k];
    ROW[k] := ROW[I_pivot];
    ROW[I_pivot] := HOLD;

    HOLD := COL[k];
    COL[k] := COL[J_pivot];
    COL[J_pivot] := HOLD;

    rkn := ROW[k]*n;
    ck := COL[k];

    // Reduce around pivot
    INV^[rkn + ck] := 1.0 / pivot ;
    for j :=0 to n-1 do begin
      if j <> k  then begin
        cj := COL[j];
        INV^[rkn + cj] := INV^[rkn + cj] * INV^[rkn + ck];
      end;
    end;

    // Inner reduction loop
    for i := 0 to n-1 do begin
      rin := ROW[i]*n;
      if k <> i then begin
        fv := INV^[rin + ck];
        for j := 0 to n-1 do begin
          if  k <> j then begin
            cj := COL[j];
            INV^[rin + cj] := INV^[rin + cj] - fv * INV^[rkn + cj] ;
          end;
        end;
        INV^[rin + ck] := - INV^[rin + ck] * INV^[rkn + ck];
      end;
    end;
  end; // end of main reduction loop

  // Unscramble rows
  for j := 0  to n-1 do begin
    for i := 0 to n-1 do begin
      TMP^[COL[i]] := INV^[ROW[i]*n + j];
    end;
    for i := 0 to n-1 do begin
      INV^[i*n + j] := TMP^[i];
    end;
  end;

  // Unscramble columns
  for i := 0 to n-1 do begin
    for j := 0 to n-1 do begin
      TMP^[ROW[j]] := INV^[i*n + COL[j]];
    end;
    for j := 0 to n-1 do begin
      INV^[i*n+j] := TMP^[j];
    end;
  end;

  result := MatInv;
end;


// <- M^-1
// Faster and less acurate version
function Minv_fast(M: TDMatrix): TDMatrix;
var
  Mda,Tda,Ida: PdoubleArray;
  Mrows,Mcols,Irows,Icols,Trows,Tcols,dim,r,c,t,pivrow,k: Longword;
  pivmax,pivot: double;
  INV,TMP: TDmatrix;
  ex,pdisp,cdisp:Longword;
  dtmp,victim,rk,norm,invnorm: double;
  Mzero : double;
begin
  M.GetData(Mrows, Mcols, Mda);
  if Mcols <> Mrows then
    raise Exception.Create('Cannot invert non-square matrix');

  dim := Mrows;
  INV := Meye(dim);
  TMP := M;

  uniquestring(TMP.data);
  INV.GetData(Irows, Icols, Ida);
  TMP.GetData(Trows, Tcols, Tda);

  MZero := 1e-10;
  for c := 0 to dim - 1 do begin
    // find the greatest pivot in the remaining columns
    pivmax := abs(Tda^[c + c*dim]);
    pivrow := c;
    for k := c + 1 to dim - 1 do begin
	    if abs(Tda^[c + k*dim]) > pivmax then begin
	      pivmax := abs(Tda^[c + k*dim]);
	      pivrow:=k;
      end;
    end;
    pivot:= Tda^[c + pivrow*dim];
    if abs(pivot) < Mzero then
      raise Exception.Create('Singular matrix: Pivot is '+floattostr(pivot));

    if pivrow <> c then begin
      // swap lines
      pdisp:=pivrow*dim;
	    cdisp:=c*dim;
	    for ex:=c to dim-1 do begin
	       dtmp:=Tda^[cdisp+ex];
               Tda^[cdisp+ex]:=Tda^[pdisp+ex];
               Tda^[pdisp+ex]:=dtmp;
      end;
	    for ex:=0 to dim-1 do begin
	       dtmp:=Ida^[cdisp+ex];
               Ida^[cdisp+ex]:=Ida^[pdisp+ex];
               Ida^[pdisp+ex]:=dtmp;
       end;
    end;

    for r:=0 to dim-1 do begin
	    if r<>c then begin
	      victim:=Tda^[c+r*dim];
	      rk:=-victim/pivot;
	      for t:=0 to dim-1 do Ida^[r*dim+t]:= Ida^[r*dim+t] + rk * Ida^[c*dim+t];
	      for t:=c+1 to dim-1 do Tda^[r*dim+t]:= Tda^[r*dim+t] + rk * Tda^[c*dim+t];
      end;
    end;
  end;

  // normalize the pivots
  for r := 0 to dim - 1 do begin
    norm := Tda^[r + r*dim];
    if abs(norm) < Mzero then
       raise Exception.Create('Singular matrix: Pivot has been '+floattostr(norm));
    invnorm := 1.0 / norm;
    for c := 0 to dim - 1 do
	    Ida^[c + r*dim] := Ida^[c + r*dim] * invnorm;
  end;
  result:=INV;
end;


// <- max A(i,j)
function Mmax(A: TDMatrix): double;
var
  Adp: Pdouble;
  Arows, Acols, i: Longword;
begin
  A.GetData(Arows, Acols, PdoubleArray(Adp));
  result := Adp^;
  inc(Adp);
  for i := 1 to Arows * Acols - 1 do begin
    if (result < Adp^) then result := Adp^;
    inc(Adp);
  end;
end;


// <- max |A(i,j)|
function MmaxAbs(A: TDMatrix): double;
var
  Adp: Pdouble;
  Arows, Acols, i: Longword;
begin
  A.GetData(Arows, Acols, PdoubleArray(Adp));
  result := abs(Adp^);
  inc(Adp);
  for i := 1 to Arows * Acols - 1 do begin
    if (result < abs(Adp^)) then result := abs(Adp^);
    inc(Adp);
  end;
end;


// <- min A(i,j)
function Mmin(A: TDMatrix): double;
var
  Adp: Pdouble;
  Arows, Acols, i: Longword;
begin
  A.GetData(Arows, Acols, PdoubleArray(Adp));
  result := Adp^;
  inc(Adp);
  for i := 1 to Arows * Acols - 1 do begin
    if (result > Adp^) then result := Adp^;
    inc(Adp);
  end;
end;


function Mfunc(A: TDMatrix; f: TDoubleFunc): TDMatrix;
var
  pd: Pdouble;
  r, c, i: Longword;
begin
  result := A;
  uniquestring(result.data);

  result.GetData(r, c, PdoubleArray(pd));
  for i := 0 to r * c - 1 do begin
    pd^ := f(pd^);
    inc(pd);
  end;
end;


//// <- Matrix(numrows,numcols) em que e(i,j)=i+j
//function Minc(numrows,numcols: integer): Matrix;
//var
  //dp: pdouble;
  //i,it: integer;
//begin
  //result:=Mnew(numrows,numcols);
  //Mdata(result,i,it,PdoubleArray(dp));
  //for i:=0 to numrows*numcols-1 do begin
    ////Pdouble(integer(dp)+i*DSIZE)^:=i;
    //dp^:=i;
    //inc(dp);
  //end;
//end;



//// <- pivota as coluna em torno da coluna central
//function MReverse(M:Matrix): Matrix;
//var
  //Mda,Tda: PdoubleArray;
  //r,c,rows,cols: integer;
//begin
  //Mdata(M,rows,cols,Mda);
  //result:=Mnew(rows,cols);
  //Mdata(result,r,c,Tda);
  //for r:=0 to rows-1 do begin
    //for c:=0 to cols-1 do begin
      //Tda^[c+r*rows]:=Mda^[(cols-1)-c+r*cols];
    //end;
  //end;
//end;


//// <- conv(A,B)
//function MConv(A:Matrix; B:Matrix): Matrix;
//var ar,br,disp,r,c,i: integer;
  //Mda,Mdb,Mdm: PdoubleArray;
  //Arows,Acols,Brows,Bcols,Mrows,Mcols: integer;
  //pivot,prod: double;
//begin
  //Mdata(A,Arows,Acols,Mda);
  //Mdata(B,Brows,Bcols,Mdb);

  //result:=Mnew(Arows*Brows,Acols+Bcols-1);
  //Mdata(result,Mrows,Mcols,Mdm);

  //for r:=0 to Mrows-1 do begin
    //for c:=0 to Mcols-1 do begin
      //Mdm^[c+r*Mcols]:=0;
    //end;
  //end;

  //for ar:=0 to Arows-1 do begin
    //for br:=0 to Brows-1 do begin
      //for disp:=0 to Acols-1 do begin
        //r:=ar*Brows+br;
        //pivot:= Mda^[disp+ar*Acols];
        //for i:=0 to Bcols-1 do begin
          //prod:= pivot * Mdb^[i+br*Bcols];
          //c:=disp+i;
          //Mdm^[c+r*Mcols]:=Mdm^[c+r*Mcols] + prod;
        //end;
      //end;
    //end;
  //end;
//end;


//// Preenche a matrix A com os elementos do array D
//procedure MArray2Matrix(A: matrix; const D: array of double);
//var
  //da: PdoubleArray;
  //rows,cols,r,c: integer;
//begin
  //Mdata(A,rows,cols,da);

  //if rows*cols<>high(D)+1 then
    //raise  EMatrixMismatch.Create('Const Array numbers of elements different from Matrix');

  //for c:=0 to cols-1 do
    //for r:=0 to rows-1 do begin
      //da^[c+r*cols]:=D[c+r*cols];
    //end;
//end;



//// <- crop(A)
//function Mcrop(M:Matrix; uprow, leftcol, downrow, rightcol: integer): Matrix;
//var
  //Mda,Tda: PdoubleArray;
  //Mrows,Mcols,Trows,Tcols,rowsize,colsize,r,c: integer;
//begin
  //Mdata(M,Mrows,Mcols,Mda);
  //rowsize:=downrow-uprow+1;
  //colsize:=rightcol-leftcol+1;
  //if (rowsize<1) or (colsize<1) then
     //raise  EMatrixInvalid.Create('Invalid number of rows/cols:'+inttostr(rowsize)+'/'+inttostr(colsize));

  //if (downrow>Mrows-1) or (rightcol>Mcols-1) then
     //raise  EMatrixInvalid.Create('Invalid number of rows/cols:'+inttostr(downrow)+'/'+inttostr(rightcol));

  //result:=Mnew(rowsize,colsize);
  //Mdata(result,Trows,Tcols,Tda);

  //for r:=0 to Trows-1 do begin
      //for c:=0 to Tcols-1 do begin
		////S.rc(r,c)=M.rc(r+uprow,c+leftcol);
          //Tda^[c+r*Tcols]:= Mda^[c+leftcol+(r+uprow)*Mcols];
      //end
  //end;
//end;



//// <- onecol(A)
//function MOneCol(M:Matrix; col: integer): Matrix;
//begin
  //result:=Mcrop(M,0,col,Mrows(M)-1,col)
//end;


//// <- onerow(A)
//function MOneRow(M:Matrix; row: integer): Matrix;
//begin
  //result:=Mcrop(M,row,0,row,Mcols(M)-1)
//end;



//// <- stamp(A)
//function Mstamp(M, S: matrix; drow, dcol: integer): Matrix;
//var
  //Mda,Sda,Tda: PdoubleArray;
  //Mrows,Mcols,Srows,Scols,Trows,Tcols,r,c: integer;
//begin
  //Mdata(M,Mrows,Mcols,Mda);
  //Mdata(S,Srows,Scols,Sda);
  //if (drow+Srows>Mrows) or (dcol+Scols>Mcols) then
     //raise  EMatrixInvalid.Create('Matrix does not fit!');

  //result:=Mnew(Mrows,Mcols);
  //Mdata(result,Trows,Tcols,Tda);

  //for c:=0 to Scols-1 do begin
      //for r:=0 to Srows-1 do begin
          //Tda^[c+dcol+(r+drow)*Tcols]:= Sda^[c+r*Scols];
      //end
  //end;
//end;

//// <-  (col ésima coluna de M) = S
//function MStampCol(M, S: matrix; col: integer): Matrix;
//begin
  //result:=Mstamp(M,S,0,col);
//end;

//// <- (row ésima coluna de M) = S
//function MStampRow(M, S: matrix; row: integer): Matrix;
//begin
  //result:=Mstamp(M,S,row,0);
//end;




//// <-
//function Mcolsum(M:Matrix): Matrix;
//var
  //Mda,Tda: PdoubleArray;
  //Mrows,Mcols,Trows,Tcols,r,c: integer;
//begin
  //Mdata(M,Mrows,Mcols,Mda);

  //result:=Mnew(1,Mcols);
  //Mdata(result,Trows,Tcols,Tda);

  //for c:=0 to Mcols-1 do begin
      //for r:=0 to Mrows-1 do begin
          //Tda^[c]:=Tda^[c]+ Mda^[c+r*Mcols];
      //end
  //end;
//end;

//// <-
//function Mrowsum(M:Matrix): Matrix;
//var
  //Mda,Tda: PdoubleArray;
  //Mrows,Mcols,Trows,Tcols,r,c: integer;
//begin
  //Mdata(M,Mrows,Mcols,Mda);

  //result:=Mnew(Mrows,1);
  //Mdata(result,Trows,Tcols,Tda);

  //for r:=0 to Mrows-1 do begin
      //for c:=0 to Mcols-1 do begin
          //Tda^[r]:=Tda^[r]+ Mda^[c+r*Mcols];
      //end
  //end;
//end;


//// <-
//function MColNorm2(M:Matrix): Matrix;
//var
  //Mda,Tda: PdoubleArray;
  //Mrows,Mcols,Trows,Tcols,r,c: integer;
//begin
  //Mdata(M,Mrows,Mcols,Mda);

  //result:=Mnew(1,Mcols);
  //Mdata(result,Trows,Tcols,Tda);

  //for c:=0 to Mcols-1 do begin
      //for r:=0 to Mrows-1 do begin
          //Tda^[c]:=Tda^[c]+ sqr(Mda^[c+r*Mcols]);
      //end
  //end;
//end;


{$R+}

initialization

//Mzero:=1e-10;

end.
