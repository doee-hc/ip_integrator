.PHONY : default help clean regress_clean sim verdi


default: sim 


#=============== These maybe modified manually for your new project ============
# RTL_PATH is used for filelist
export RTL_PATH=$(CURDIR)
export TB_SRC_DIR=$(CURDIR)

TOP := arbiter

# 查找当前目录下的所有 .v 文件
V_FILES := $(wildcard *.v)

# 查找当前目录下的所有 .lst 文件
LST_FILES := $(wildcard *.lst)

# DUT_SRC := $(V_FILES)
DUT_SRC += -f $(LST_FILES)
TB_SRC 	:= 

TB_CMP_OPTIONS += -full64 +lint=TFIPC-L \
				  -lca +v2k -v2005 -sverilog \
				  -timescale=1ns/10ps \
				  -debug_all -debug_acc+dmptf \
				  -debug_region+cell+encrypt \
				  +notimingcheck +nospecify

TB_CMP_OPTIONS += -j16

# define transfer to rtl when compile
#TB_CMP_DEF_OPTIONS := define+RTL

#===============================================================================

TB_SIM_EXEC += ./simv  +vcs+loopreport+1000

ifneq ($(TOP),)
	TB_CMP_OPTIONS += -top $(TOP)
endif


compile :
	vcs $(TB_CMP_OPTIONS)  \
	$(TB_CMP_DEF_OPTIONS)  \
	$(DUT_SRC) \
	$(TB_SRC) 

run     :
	$(TB_SIM_EXEC) \
	$(TB_SIM_OPTIONS)

sim : compile run

verdi : 
	verdi -ssv -ssy -2012  \
		-lca $(DUT_SRC) $(TB_SRC) $(TB_CMP_DEF_OPTIONS) \
		-top $(TOP) &

clean : 
	rm -rf ./csrc
	rm -rf ./wave
	rm -rf ./log
	rm -rf ./output
	rm -rf ./simv.daidir
	rm -rf ./verdiLog
	rm -rf ./simv
	rm -rf ./novas*
	rm -rf ./ucli.key
	rm -rf ./log/*
	rm -rf ./*.log
	rm -rf ./AN.DB ./simv.cst ./simv.vdb vc_hdrs.h ucli.key
	rm -rf log.*











