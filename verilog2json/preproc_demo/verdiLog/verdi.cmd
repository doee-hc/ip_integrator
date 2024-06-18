sidCmdLineBehaviorAnalysisOpt -incr -clockSkew 0 -loopUnroll 0 -bboxEmptyModule 0  -cellModel 0 -bboxIgnoreProtected 0 
debImport "-ssv" "-ssy" "-2012" "-f" "rtl.lst" "-top" "arbiter"
verdiSetActWin -dock widgetDock_MTB_SOURCE_TAB_1
verdiWindowResize -win $_Verdi_1 "427" "71" "852" "679"
verdiSetActWin -dock widgetDock_<Inst._Tree>
srcDeselectAll -win $_nTrace1
srcSelect -word -line 25 -pos 3 -win $_nTrace1
srcAction -pos 25 3 5 -win $_nTrace1 -name "\"config.vh\"" -ctrlKey off
verdiSetActWin -dock widgetDock_MTB_SOURCE_TAB_1
srcDeselectAll -win $_nTrace1
srcBackwardHistory -win $_nTrace1
srcHBSelect "arbiter" -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcHBSelect "arbiter.priority_encoder_inst" -win $_nTrace1
srcSetScope "arbiter.priority_encoder_inst" -delim "." -win $_nTrace1
srcHBSelect "arbiter.priority_encoder_inst" -win $_nTrace1
verdiSetActWin -dock widgetDock_<Inst._Tree>
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
verdiSetActWin -dock widgetDock_MTB_SOURCE_TAB_1
srcSelect -win $_nTrace1 -range {27 29 1 1 1 1} -backward
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -win $_nTrace1 -range {27 29 1 1 1 1} -backward
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcHBSelect "arbiter" -win $_nTrace1
srcSetScope "arbiter" -delim "." -win $_nTrace1
srcHBSelect "arbiter" -win $_nTrace1
verdiSetActWin -dock widgetDock_<Inst._Tree>
srcDeselectAll -win $_nTrace1
srcSelect -word -line 26 -pos 3 -win $_nTrace1
verdiSetActWin -dock widgetDock_MTB_SOURCE_TAB_1
srcDeselectAll -win $_nTrace1
srcSelect -word -line 26 -pos 3 -win $_nTrace1
srcAction -pos 26 3 5 -win $_nTrace1 -name "\"priority_encoder.v\"" -ctrlKey off
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcHBSelect "arbiter.priority_encoder_inst" -win $_nTrace1
srcSetScope "arbiter.priority_encoder_inst" -delim "." -win $_nTrace1
srcHBSelect "arbiter.priority_encoder_inst" -win $_nTrace1
verdiSetActWin -dock widgetDock_<Inst._Tree>
srcHBSelect "arbiter" -win $_nTrace1
srcSetScope "arbiter" -delim "." -win $_nTrace1
srcHBSelect "arbiter" -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -word -line 25 -pos 3 -win $_nTrace1
srcAction -pos 25 3 7 -win $_nTrace1 -name "\"config.vh\"" -ctrlKey off
verdiSetActWin -dock widgetDock_MTB_SOURCE_TAB_1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcHBSelect "arbiter" -win $_nTrace1
srcSetScope "arbiter" -delim "." -win $_nTrace1
srcHBSelect "arbiter" -win $_nTrace1
verdiSetActWin -dock widgetDock_<Inst._Tree>
srcDeselectAll -win $_nTrace1
verdiSetActWin -dock widgetDock_MTB_SOURCE_TAB_1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -word -line 26 -pos 3 -win $_nTrace1
srcHBSelect "arbiter" -win $_nTrace1
srcSetScope "arbiter" -delim "." -win $_nTrace1
srcHBSelect "arbiter" -win $_nTrace1
verdiSetActWin -dock widgetDock_<Inst._Tree>
verdiWindowResize -win $_Verdi_1 "369" "330" "852" "679"
srcHBSelect "arbiter.priority_encoder_inst" -win $_nTrace1
srcSetScope "arbiter.priority_encoder_inst" -delim "." -win $_nTrace1
srcHBSelect "arbiter.priority_encoder_inst" -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "WIDTH" -line 42 -pos 1 -win $_nTrace1
verdiSetActWin -dock widgetDock_MTB_SOURCE_TAB_1
srcDeselectAll -win $_nTrace1
srcSelect -signal "WIDTH" -line 43 -pos 1 -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "WIDTH" -line 40 -pos 1 -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcAction -pos 27 3 6 -win $_nTrace1 -name "ARB_LSB_HIGH_PRIORITY" -ctrlKey off
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "LSB_HIGH_PRIORITY" -line 74 -pos 1 -win $_nTrace1
srcHBSelect "arbiter.priority_encoder_masked" -win $_nTrace1
srcSetScope "arbiter.priority_encoder_masked" -delim "." -win $_nTrace1
srcHBSelect "arbiter.priority_encoder_masked" -win $_nTrace1
verdiSetActWin -dock widgetDock_<Inst._Tree>
srcHBSelect "arbiter" -win $_nTrace1
srcHBSelect "arbiter" -win $_nTrace1
srcSetScope "arbiter" -delim "." -win $_nTrace1
srcHBSelect "arbiter" -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -word -line 26 -pos 3 -win $_nTrace1
verdiSetActWin -dock widgetDock_MTB_SOURCE_TAB_1
srcDeselectAll -win $_nTrace1
srcSelect -word -line 42 -pos 8 -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -word -line 42 -pos 8 -win $_nTrace1
srcAction -pos 42 8 5 -win $_nTrace1 -name "ARB_BLOCK_ACK" -ctrlKey off
srcDeselectAll -win $_nTrace1
srcBackwardHistory -win $_nTrace1
srcHBSelect "arbiter" -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcSelect -signal "ARB_BLOCK" -line 58 -pos 1 -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
debExit
