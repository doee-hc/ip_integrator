sidCmdLineBehaviorAnalysisOpt -incr -clockSkew 0 -loopUnroll 0 -bboxEmptyModule 0  -cellModel 0 -bboxIgnoreProtected 0 
debImport "-ssv" "-ssy" "-2012" "-f" "rtl.lst" "-top" "arbiter"
verdiSetActWin -dock widgetDock_MTB_SOURCE_TAB_1
verdiWindowResize -win $_Verdi_1 "146" -13 "1644" "1436"
nsMsgSelect -range {0 3-3}
verdiSetActWin -dock widgetDock_<Message>
nsMsgSelect -range {0 2-2}
nsMsgSelect -range {0 1-1}
nsMsgSelect -range {0 2-2}
nsMsgSelect -range {0 1-1}
nsMsgSelect -range {0 3-3}
nsMsgSelect -range {0 2-2}
nsMsgSelect -range {0 1-1}
srcDeselectAll -win $_nTrace1
srcDeselectAll -win $_nTrace1
verdiSetActWin -dock widgetDock_MTB_SOURCE_TAB_1
nsMsgSelect -range {0 2-2}
verdiSetActWin -dock widgetDock_<Message>
nsMsgSelect -range {0 3-3}
nsMsgSelect -range {0 1-1}
nsMsgSelect -range {0 2-2}
nsMsgSelect -range {0 3-3}
