unit Roles;

{$mode objfpc}{$H+}

interface

uses RoCConsts,Field;

type
  TRoleFunc=procedure(num: integer);

type
  TRole=(roleIdle,
         roleTest,
         roleGoSearch,
         roleGoSearchFollower,
         roleDoFormation
         );

type
  TRoleDef = record
    name: string;
    func: TRoleFunc;
    is_keeper_role: boolean;
  end;

var
  deltadist,deltateta:double;

procedure RoleIdleRules(num: integer);
procedure RoleTestRules(num: integer);
procedure RoleDoFormationRules(num: integer);
procedure RoleGoSearchRules(num: integer);
procedure RoleGoSearchFollowerRules(num: integer);

const
  RoleDefs: array[low(TRole) .. High(TRole)] of TRoleDef = (
    ( name:'roleIdle';                func: @RoleIdleRules ),
    ( name:'roleTest';                func: @roleTestRules ),
    ( name:'roleGoSearchFollower';           func: @roleGoSearchFollowerRules ),
    ( name:'roleGoSearch';           func: @roleGoSearchRules ),
    ( name:'roleDoFormation'; func: @RoleDoFormationRules)
  );

implementation

uses Main, Param, Utils, Math, Tatic;

//----------------------------------------------------------------------
//  Role Rules
//----------------------------------------------------------------------

procedure RoleIdleRules(num: integer);
begin

end;

//Tiago Tese

procedure RoleTestRules(num: integer);
begin

end;

procedure RoleGoSearchFollowerRules(num: integer);
begin

end;

procedure RoleGoSearchRules(num: integer);
begin

end;

procedure RoleDoFormationRules(num: integer);
begin

end;


end.

