# Generated by the VisualDSP++ IDDE

# Note:  Any changes made to this Makefile will be lost the next time the
# matching project file is loaded into the IDDE.  If you wish to preserve
# changes, rename this file and run it externally to the IDDE.

# The syntax of this Makefile is such that GNU Make v3.77 or higher is
# required.

# The current working directory should be the directory in which this
# Makefile resides.

# Supported targets:
#     AIX Startup_Debug
#     AIX Startup_Debug_clean

# Define this variable if you wish to run this Makefile on a host
# other than the host that created it and VisualDSP++ may be installed
# in a different directory.

ADI_DSP=C:\Program Files\Analog Devices\VisualDSP 5.1


# $VDSP is a gmake-friendly version of ADI_DIR

empty:=
space:= $(empty) $(empty)
VDSP_INTERMEDIATE=$(subst \,/,$(ADI_DSP))
VDSP=$(subst $(space),\$(space),$(VDSP_INTERMEDIATE))

RM=cmd /C del /F /Q

#
# Begin "AIX Startup_Debug" configuration
#

ifeq ($(MAKECMDGOALS),AIX Startup_Debug)

AIX\ Startup_Debug : ./Debug/AIX\ Startup.ldr 

Debug/hardware.doj :hardware.cpp hardware.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/AD21161.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/always.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/irq_typedefs.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/MSC.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/MSC2K/msc2k001.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_mem.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_bitmod_bit.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/extract_bit.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/extract.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_bitmod_bitset.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/assert.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/error.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/bit.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/insert.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/deposit.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/atomic.h $(VDSP)/211xx/include/sysreg.h $(VDSP)/211xx/include/def21161.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_readonly_string.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_readonly_bit.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/packed.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/MSC2K/xcp2000_msc0003a@u56.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/MSC2K/xcp2000_msc0003a_lib.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/XCP2000_and_XCP2005_mem.vdsp.h $(VDSP)/211xx/include/signal.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/PORT_A.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/STDIO/STDIO001.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/register/reg_readonly_bitset.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/PORT_B.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/PWM_C.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/blocks.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/PWM16/pwm08_pwm16001.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_simple_bit.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_simple_bitset.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_writeonly_word.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_writeonly_bitset.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/cache.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/combine2_16.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/swap2_16.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/lib_clip.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/lib_min.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/lib_max.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/trunc.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/clock.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/sync.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/PORT_D.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/PORT_E.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ENCOD_A.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ENCOD/ENCOD001.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_simple_word.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_readonly_sword.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/extract_signed.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_writeonly_sword.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/DM.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/XCI.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/slots.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/I2C.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/XCI/xci_mem.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/DISPL_H.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/DISPL/ComUnit.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/irq.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG/ANALG001.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG/xcp2000_aio0003b@u50.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG/xcp2000_aio0003b@u52.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG/xcp2000_aio0003b_lib.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG/ANALG_TYPES.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG/CalibBlock.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/bitset.h $(VDSP)/211xx/include/math.h $(VDSP)/211xx/include/math_21xxx.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/PWM_B.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG_L.h init_pwm.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/nop.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG_N.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/XCSDataLink.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/init.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/terminal.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/pstring.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/bitmask.h 
	@echo ".\hardware.cpp"
	$(VDSP)/cc21k.exe -c .\hardware.cpp -c++ -file-attr ProjectName=AIX\ Startup -g -structs-do-not-overlap -no-multiline -D DEBUG -D XCP2005 -I C:\Users\RDL\Documents\AIX\ 2015\AixControl\lib\XCS2000LIB-100916-Yukai -double-size-32 -warn-protos -proc ADSP-21161 -o .\Debug\hardware.doj -MM

Debug/main_Tj.doj :main_Tj.cpp hardware.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/AD21161.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/always.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/irq_typedefs.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/MSC.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/MSC2K/msc2k001.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_mem.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_bitmod_bit.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/extract_bit.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/extract.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_bitmod_bitset.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/assert.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/error.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/bit.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/insert.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/deposit.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/atomic.h $(VDSP)/211xx/include/sysreg.h $(VDSP)/211xx/include/def21161.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_readonly_string.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_readonly_bit.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/packed.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/MSC2K/xcp2000_msc0003a@u56.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/MSC2K/xcp2000_msc0003a_lib.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/XCP2000_and_XCP2005_mem.vdsp.h $(VDSP)/211xx/include/signal.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/PORT_A.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/STDIO/STDIO001.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/register/reg_readonly_bitset.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/PORT_B.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/PWM_C.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/blocks.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/PWM16/pwm08_pwm16001.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_simple_bit.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_simple_bitset.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_writeonly_word.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_writeonly_bitset.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/cache.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/combine2_16.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/swap2_16.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/lib_clip.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/lib_min.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/lib_max.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/trunc.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/clock.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/sync.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/PORT_D.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/PORT_E.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ENCOD_A.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ENCOD/ENCOD001.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_simple_word.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_readonly_sword.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/extract_signed.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/Register/reg_writeonly_sword.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/DM.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/XCI.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/slots.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/I2C.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/XCI/xci_mem.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/DISPL_H.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/DISPL/ComUnit.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/irq.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG/ANALG001.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG/xcp2000_aio0003b@u50.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG/xcp2000_aio0003b@u52.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG/xcp2000_aio0003b_lib.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG/ANALG_TYPES.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG/CalibBlock.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/bitset.h $(VDSP)/211xx/include/math.h $(VDSP)/211xx/include/math_21xxx.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/PWM_B.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG_L.h init_pwm.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/shared/nop.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/ANALG_N.h ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/XCSDataLink.h 
	@echo ".\main_Tj.cpp"
	$(VDSP)/cc21k.exe -c .\main_Tj.cpp -c++ -file-attr ProjectName=AIX\ Startup -g -structs-do-not-overlap -no-multiline -D DEBUG -D XCP2005 -I C:\Users\RDL\Documents\AIX\ 2015\AixControl\lib\XCS2000LIB-100916-Yukai -double-size-32 -warn-protos -proc ADSP-21161 -o .\Debug\main_Tj.doj -MM

./Debug/AIX\ Startup.dxe :../../AIX\ 2015/AixControl/lib/New_XCS2000LIB&DataLinkFile/Embedded/XCS2000LIB/xcp2005.ldf ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/DebugXCP2005/CDSP_Runtime_Header.doj ./Debug/hardware.doj ./Debug/main_Tj.doj $(VDSP)/211xx/lib/libc161.dlb $(VDSP)/211xx/lib/libio_lite.dlb $(VDSP)/211xx/lib/libdsp160.dlb $(VDSP)/211xx/lib/libcpp.dlb $(VDSP)/211xx/lib/libeh.dlb $(VDSP)/211xx/lib/libcpprt.dlb ../../AIX\ 2015/AixControl/lib/XCS2000LIB-100916-Yukai/DebugXCP2005/XCP2005LIB.dlb 
	@echo "Linking..."
	$(VDSP)/cc21k.exe .\Debug\hardware.doj .\Debug\main_Tj.doj -T ..\..\AIX\ 2015\AixControl\lib\New_XCS2000LIB&DataLinkFile\Embedded\XCS2000LIB\xcp2005.ldf -map .\Debug\AIX\ Startup.map.xml -L .\Debug -L ..\..\AIX\ 2015\AixControl\lib\XCS2000LIB-100916-Yukai\DebugXCP2005 -flags-link -MDDEBUG,-MDXCP2005 -flags-link -i,C:\Users\RDL\Documents\AIX\ 2015\AixControl\lib\New_XCS2000LIB&DataLinkFile\Embedded\XCS2000LIB\DebugXCP2005; -add-debug-libpaths -flags-link -od,.\Debug -o .\Debug\AIX\ Startup.dxe -proc ADSP-21161 -flags-link -MM

./Debug/AIX\ Startup.ldr :./Debug/AIX\ Startup.dxe 
	@echo "Creating loader file..."
	$(VDSP)/elfloader.exe .\Debug\AIX\ Startup.dxe -bPROM -fHEX -HostWidth 8 -l $(VDSP)\211xx\ldr\161_prom.dxe -l $(VDSP)\211xx\ldr\161_prom.dxe -o .\Debug\AIX\ Startup.ldr -p0x0 -proc ADSP-21161 -MM

endif

ifeq ($(MAKECMDGOALS),AIX Startup_Debug_clean)

AIX\ Startup_Debug_clean:
	-$(RM) "Debug\hardware.doj"
	-$(RM) "Debug\main_Tj.doj"
	-$(RM) ".\Debug\AIX Startup.dxe"
	-$(RM) ".\Debug\AIX Startup.ldr"
	-$(RM) ".\Debug\*.ipa"
	-$(RM) ".\Debug\*.opa"
	-$(RM) ".\Debug\*.ti"
	-$(RM) ".\Debug\*.pgi"
	-$(RM) ".\*.rbld"

endif


