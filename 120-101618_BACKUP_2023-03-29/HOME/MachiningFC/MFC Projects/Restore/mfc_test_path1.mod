%%%
  VERSION: 1
  LANGUAGE: ENGLISH
%%%

MODULE mfc_test_path1

!This module is generated by RobotWare Machining FC V3.000. DO NOT EDIT.
!Any changes to this module will be lost when the RW Machining FC application is re-run and the program is exported.
LOCAL VAR robtarget T1Approach1 := [[517.95,-195.73,272.24],[0.000355199,0.785866,-0.618396,-0.000533764],[-1,-1,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget T1Approach2 := [[516.73,-200.3,189.2],[0.000428067,0.785863,-0.6184,-0.000596594],[-1,-1,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget T1Approach3 := [[516.71,-176.4,186.28],[0.00042596,0.785862,-0.618401,-0.000584598],[-1,-1,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget T1Process1  := [[516.72,-55.75,180.26],[0.000421343,0.785859,-0.618405,-0.000576803],[-1,-1,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget T1Process2  := [[516.71,30.15,172.12],[0.000449071,0.785877,-0.618383,-0.000602883],[0,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget T1Withdraw1 := [[516.69,110.5,217.2],[0.000509793,0.785877,-0.618383,-0.000651357],[0,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget T1Withdraw2 := [[517.93,107.2,272.24],[0.000429411,0.785869,-0.618393,-0.000595211],[0,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR num n1ForceX := 0;
LOCAL VAR num n1ForceY := 0;
LOCAL VAR num n1ForceZ := -10;


PROC run_test_path1 (bool bRecover, bool bSpindleOn)
	FCDeact;

	IF bRecover = true THEN
		FCCalib  mfcTool1_LD \Recovery;
	ELSE
		FCCalib  mfcTool1_LD;
	ENDIF

	IF bSpindleOn = true THEN
		SpindleOn;
	ENDIF

	MoveL T1Approach1, v50, z1,mfcTool1\wobj :=mfcWobj1;
	MoveL T1Approach2, v50, z1,mfcTool1\wobj :=mfcWobj1;
	MoveL T1Approach3, v50, fine,mfcTool1\wobj :=mfcWobj1;

	FCPress1LStart T1Process1, v50 \Fx:= n1ForceX \Fy:= n1ForceY \Fz:= n1ForceZ,50 \ForceFrameRef:=FC_REFFRAME_TOOL \ForceChange:=50 \DampingTune:=100 \TimeOut:=5, \UseSpdFFW, \PosSupvDist:=9e9, z1, mfcTool1 \wobj:=mfcWobj1;
	FCPressL T1Process2, v50, 10, z1, mfcTool1, \wobj:=mfcWobj1;

	FCPressLEnd T1Withdraw1,v50,\ForceChange :=50,\ZeroContactValue := 5;
	MoveL T1Withdraw2, v50, z1,mfcTool1\wobj :=mfcWobj1;

	IF bSpindleOn = true THEN
		SpindleOff;
	ENDIF

ENDPROC

ENDMODULE