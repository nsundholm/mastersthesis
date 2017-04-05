function mympccontroller(block)
% A Level 2 M-file S-function that contains a discrete-time implementation
% of a MPC controller.

% By Martin Enqvist, 2008-02-12, 2011-02-17

setup(block);
  
%endfunction


function setup(block)

  % Register number of ports
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 1;
  
  % Setup port properties to be inherited or dynamic
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  G=block.DialogPrm(2).Data;
  [n,m]=size(G);
  
  block.InputPort(1).Dimensions        = n;
  block.InputPort(1).DirectFeedthrough = true;
  block.InputPort(2).Dimensions        = 1;
  block.InputPort(2).DirectFeedthrough = true;
  block.OutputPort(1).Dimensions        = m;
  
  % Register parameters
  block.NumDialogPrms     = 11;

  % Register sample times
  block.SampleTimes = [block.DialogPrm(8).Data 0];
  
  %
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  block.RegBlockMethod('Outputs', @Outputs);

%endfunction


function DoPostPropSetup(block)

  %% Setup Dwork  
  block.NumDworks = 0;
%endfunction


function InitializeConditions(block)

%endfunction


function Outputs(block)
  F=block.DialogPrm(1).Data;
  G=block.DialogPrm(2).Data;
  M=block.DialogPrm(3).Data;
  N=block.DialogPrm(4).Data; 
  Q1=block.DialogPrm(5).Data;
  Q2=block.DialogPrm(6).Data; 
  ubounds=block.DialogPrm(7).Data; 
  Ts=block.DialogPrm(8).Data;
  ufname=block.DialogPrm(9).Data;
  P_req=block.DialogPrm(10).Data; 
  zbounds=block.DialogPrm(11).Data; 
  x=block.InputPort(1).Data;
  t=block.InputPort(2).Data;

  un=feval(ufname,x,F,G,M,N,Q1,Q2,ubounds,Ts,P_req,zbounds,t);
  block.OutputPort(1).Data = un;
  
%endfunction