#
# OMNeT++/OMNEST Makefile for libveins
#
# This file was generated with the command:
#  opp_makemake -f --deep --make-so -O out --no-deep-includes -I/Users/user/projects/veins/src
#

# Name of target to be created (-o option)
TARGET = libveins$(SHARED_LIB_SUFFIX)

# C++ include paths (with -I)
INCLUDE_PATH = -Isrc

# Additional object and library files to link with
EXTRA_OBJS =

# Additional libraries (-L, -l options)
LIBS =

# Output directory
PROJECT_OUTPUT_DIR = out
PROJECTRELATIVE_PATH =
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc, .msg and .sm files
OBJS = \
    $O/src/veins/base/connectionManager/BaseConnectionManager.o \
    $O/src/veins/base/connectionManager/ChannelAccess.o \
    $O/src/veins/base/connectionManager/ConnectionManager.o \
    $O/src/veins/base/connectionManager/NicEntryDebug.o \
    $O/src/veins/base/connectionManager/NicEntryDirect.o \
    $O/src/veins/base/modules/BaseApplLayer.o \
    $O/src/veins/base/modules/BaseBattery.o \
    $O/src/veins/base/modules/BaseLayer.o \
    $O/src/veins/base/modules/BaseMacLayer.o \
    $O/src/veins/base/modules/BaseMobility.o \
    $O/src/veins/base/modules/BaseModule.o \
    $O/src/veins/base/modules/BaseWorldUtility.o \
    $O/src/veins/base/modules/BatteryAccess.o \
    $O/src/veins/base/phyLayer/BaseDecider.o \
    $O/src/veins/base/phyLayer/BasePhyLayer.o \
    $O/src/veins/base/phyLayer/ChannelInfo.o \
    $O/src/veins/base/phyLayer/ChannelState.o \
    $O/src/veins/base/phyLayer/Decider.o \
    $O/src/veins/base/phyLayer/MappingBase.o \
    $O/src/veins/base/phyLayer/MappingUtils.o \
    $O/src/veins/base/phyLayer/PhyUtils.o \
    $O/src/veins/base/phyLayer/Signal.o \
    $O/src/veins/base/utils/asserts.o \
    $O/src/veins/base/utils/Coord.o \
    $O/src/veins/base/utils/NetwToMacControlInfo.o \
    $O/src/veins/base/utils/SimpleAddress.o \
    $O/src/veins/base/utils/winsupport.o \
    $O/src/veins/modules/analogueModel/BreakpointPathlossModel.o \
    $O/src/veins/modules/analogueModel/JakesFading.o \
    $O/src/veins/modules/analogueModel/LogNormalShadowing.o \
    $O/src/veins/modules/analogueModel/NakagamiFading.o \
    $O/src/veins/modules/analogueModel/PERModel.o \
    $O/src/veins/modules/analogueModel/SimpleObstacleShadowing.o \
    $O/src/veins/modules/analogueModel/SimplePathlossModel.o \
    $O/src/veins/modules/analogueModel/TwoRayInterferenceModel.o \
    $O/src/veins/modules/application/ieee80211p/BaseWaveApplLayer.o \
    $O/src/veins/modules/application/traci/ClusterManager.o \
    $O/src/veins/modules/application/traci/TraCICluster.o \
    $O/src/veins/modules/application/traci/TraCIDemo11p.o \
    $O/src/veins/modules/application/traci/TraCIDemoRSU11p.o \
    $O/src/veins/modules/application/traci/TraCIMyExp11p.o \
    $O/src/veins/modules/application/traci/TraCIStat.o \
    $O/src/veins/modules/application/traci/TraCITestApp.o \
    $O/src/veins/modules/mac/ieee80211p/Mac1609_4.o \
    $O/src/veins/modules/mobility/traci/TraCIBuffer.o \
    $O/src/veins/modules/mobility/traci/TraCIColor.o \
    $O/src/veins/modules/mobility/traci/TraCICommandInterface.o \
    $O/src/veins/modules/mobility/traci/TraCIConnection.o \
    $O/src/veins/modules/mobility/traci/TraCIMobility.o \
    $O/src/veins/modules/mobility/traci/TraCIScenarioManager.o \
    $O/src/veins/modules/mobility/traci/TraCIScenarioManagerInet.o \
    $O/src/veins/modules/mobility/traci/TraCIScenarioManagerLaunchd.o \
    $O/src/veins/modules/mobility/traci/TraCIScreenRecorder.o \
    $O/src/veins/modules/obstacle/Obstacle.o \
    $O/src/veins/modules/obstacle/ObstacleControl.o \
    $O/src/veins/modules/phy/Decider80211p.o \
    $O/src/veins/modules/phy/NistErrorRate.o \
    $O/src/veins/modules/phy/PhyLayer80211p.o \
    $O/src/veins/modules/phy/SNRThresholdDecider.o \
    $O/src/veins/modules/world/annotations/AnnotationDummy.o \
    $O/src/veins/modules/world/annotations/AnnotationManager.o \
    $O/src/veins/base/messages/AirFrame_m.o \
    $O/src/veins/base/messages/BorderMsg_m.o \
    $O/src/veins/base/messages/ChannelSenseRequest_m.o \
    $O/src/veins/base/messages/MacPkt_m.o \
    $O/src/veins/modules/messages/AirFrame11p_m.o \
    $O/src/veins/modules/messages/Mac80211Pkt_m.o \
    $O/src/veins/modules/messages/PhyControlMessage_m.o \
    $O/src/veins/modules/messages/WaveShortMessage_m.o \
    $O/src/veins/modules/messages/WaveShortMessageClusterBeacon_m.o \
    $O/src/veins/modules/messages/WaveShortMessageWithDst_m.o

# Message files
MSGFILES = \
    src/veins/base/messages/AirFrame.msg \
    src/veins/base/messages/BorderMsg.msg \
    src/veins/base/messages/ChannelSenseRequest.msg \
    src/veins/base/messages/MacPkt.msg \
    src/veins/modules/messages/AirFrame11p.msg \
    src/veins/modules/messages/Mac80211Pkt.msg \
    src/veins/modules/messages/PhyControlMessage.msg \
    src/veins/modules/messages/WaveShortMessage.msg \
    src/veins/modules/messages/WaveShortMessageClusterBeacon.msg \
    src/veins/modules/messages/WaveShortMessageWithDst.msg

# SM files
SMFILES =

#------------------------------------------------------------------------------

# Pull in OMNeT++ configuration (Makefile.inc or configuser.vc)

ifneq ("$(OMNETPP_CONFIGFILE)","")
CONFIGFILE = $(OMNETPP_CONFIGFILE)
else
ifneq ("$(OMNETPP_ROOT)","")
CONFIGFILE = $(OMNETPP_ROOT)/Makefile.inc
else
CONFIGFILE = $(shell opp_configfilepath)
endif
endif

ifeq ("$(wildcard $(CONFIGFILE))","")
$(error Config file '$(CONFIGFILE)' does not exist -- add the OMNeT++ bin directory to the path so that opp_configfilepath can be found, or set the OMNETPP_CONFIGFILE variable to point to Makefile.inc)
endif

include $(CONFIGFILE)

# Simulation kernel and user interface libraries
OMNETPP_LIB_SUBDIR = $(OMNETPP_LIB_DIR)/$(TOOLCHAIN_NAME)
OMNETPP_LIBS = -L"$(OMNETPP_LIB_SUBDIR)" -L"$(OMNETPP_LIB_DIR)" -loppenvir$D $(KERNEL_LIBS) $(SYS_LIBS)

COPTS = $(CFLAGS)  $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)
SMCOPTS =

# we want to recompile everything if COPTS changes,
# so we store COPTS into $COPTS_FILE and have object
# files depend on it (except when "make depend" was called)
COPTS_FILE = $O/.last-copts
ifneq ($(MAKECMDGOALS),depend)
ifneq ("$(COPTS)","$(shell cat $(COPTS_FILE) 2>/dev/null || echo '')")
$(shell $(MKPATH) "$O" && echo "$(COPTS)" >$(COPTS_FILE))
endif
endif

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# <<<
#------------------------------------------------------------------------------

# Main target
all: $O/$(TARGET)
	$(Q)$(LN) $O/$(TARGET) .

$O/$(TARGET): $(OBJS)  $(wildcard $(EXTRA_OBJS)) Makefile
	@$(MKPATH) $O
	@echo Creating shared library: $@
	$(Q)$(SHLIB_LD) -o $O/$(TARGET)  $(OBJS) $(EXTRA_OBJS) $(AS_NEEDED_OFF) $(WHOLE_ARCHIVE_ON) $(LIBS) $(WHOLE_ARCHIVE_OFF) $(OMNETPP_LIBS) $(LDFLAGS)
	$(Q)$(SHLIB_POSTPROCESS) $O/$(TARGET)

.PHONY: all clean cleanall depend msgheaders smheaders

.SUFFIXES: .cc

$O/%.o: %.cc $(COPTS_FILE)
	@$(MKPATH) $(dir $@)
	$(qecho) "$<"
	$(Q)$(CXX) -c $(CXXFLAGS) $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(qecho) MSGC: $<
	$(Q)$(MSGC) -s _m.cc $(MSGCOPTS) $?

%_sm.cc %_sm.h: %.sm
	$(qecho) SMC: $<
	$(Q)$(SMC) -c++ -suffix cc $(SMCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)

smheaders: $(SMFILES:.sm=_sm.h)

clean:
	$(qecho) Cleaning...
	$(Q)-rm -rf $O
	$(Q)-rm -f veins veins.exe libveins.so libveins.a libveins.dll libveins.dylib
	$(Q)-rm -f ./*_m.cc ./*_m.h ./*_sm.cc ./*_sm.h
	$(Q)-rm -f doc/*_m.cc doc/*_m.h doc/*_sm.cc doc/*_sm.h
	$(Q)-rm -f examples/*_m.cc examples/*_m.h examples/*_sm.cc examples/*_sm.h
	$(Q)-rm -f examples/TAPASCologne-0.17.0/*_m.cc examples/TAPASCologne-0.17.0/*_m.h examples/TAPASCologne-0.17.0/*_sm.cc examples/TAPASCologne-0.17.0/*_sm.h
	$(Q)-rm -f examples/TAPASCologne-0.17.0/results/*_m.cc examples/TAPASCologne-0.17.0/results/*_m.h examples/TAPASCologne-0.17.0/results/*_sm.cc examples/TAPASCologne-0.17.0/results/*_sm.h
	$(Q)-rm -f examples/veins/*_m.cc examples/veins/*_m.h examples/veins/*_sm.cc examples/veins/*_sm.h
	$(Q)-rm -f examples/veins/results/*_m.cc examples/veins/results/*_m.h examples/veins/results/*_sm.cc examples/veins/results/*_sm.h
	$(Q)-rm -f images/*_m.cc images/*_m.h images/*_sm.cc images/*_sm.h
	$(Q)-rm -f images/status/*_m.cc images/status/*_m.h images/status/*_sm.cc images/status/*_sm.h
	$(Q)-rm -f src/*_m.cc src/*_m.h src/*_sm.cc src/*_sm.h
	$(Q)-rm -f src/scripts/*_m.cc src/scripts/*_m.h src/scripts/*_sm.cc src/scripts/*_sm.h
	$(Q)-rm -f src/veins/*_m.cc src/veins/*_m.h src/veins/*_sm.cc src/veins/*_sm.h
	$(Q)-rm -f src/veins/base/*_m.cc src/veins/base/*_m.h src/veins/base/*_sm.cc src/veins/base/*_sm.h
	$(Q)-rm -f src/veins/base/connectionManager/*_m.cc src/veins/base/connectionManager/*_m.h src/veins/base/connectionManager/*_sm.cc src/veins/base/connectionManager/*_sm.h
	$(Q)-rm -f src/veins/base/messages/*_m.cc src/veins/base/messages/*_m.h src/veins/base/messages/*_sm.cc src/veins/base/messages/*_sm.h
	$(Q)-rm -f src/veins/base/modules/*_m.cc src/veins/base/modules/*_m.h src/veins/base/modules/*_sm.cc src/veins/base/modules/*_sm.h
	$(Q)-rm -f src/veins/base/phyLayer/*_m.cc src/veins/base/phyLayer/*_m.h src/veins/base/phyLayer/*_sm.cc src/veins/base/phyLayer/*_sm.h
	$(Q)-rm -f src/veins/base/utils/*_m.cc src/veins/base/utils/*_m.h src/veins/base/utils/*_sm.cc src/veins/base/utils/*_sm.h
	$(Q)-rm -f src/veins/modules/*_m.cc src/veins/modules/*_m.h src/veins/modules/*_sm.cc src/veins/modules/*_sm.h
	$(Q)-rm -f src/veins/modules/analogueModel/*_m.cc src/veins/modules/analogueModel/*_m.h src/veins/modules/analogueModel/*_sm.cc src/veins/modules/analogueModel/*_sm.h
	$(Q)-rm -f src/veins/modules/application/*_m.cc src/veins/modules/application/*_m.h src/veins/modules/application/*_sm.cc src/veins/modules/application/*_sm.h
	$(Q)-rm -f src/veins/modules/application/ieee80211p/*_m.cc src/veins/modules/application/ieee80211p/*_m.h src/veins/modules/application/ieee80211p/*_sm.cc src/veins/modules/application/ieee80211p/*_sm.h
	$(Q)-rm -f src/veins/modules/application/traci/*_m.cc src/veins/modules/application/traci/*_m.h src/veins/modules/application/traci/*_sm.cc src/veins/modules/application/traci/*_sm.h
	$(Q)-rm -f src/veins/modules/mac/*_m.cc src/veins/modules/mac/*_m.h src/veins/modules/mac/*_sm.cc src/veins/modules/mac/*_sm.h
	$(Q)-rm -f src/veins/modules/mac/ieee80211p/*_m.cc src/veins/modules/mac/ieee80211p/*_m.h src/veins/modules/mac/ieee80211p/*_sm.cc src/veins/modules/mac/ieee80211p/*_sm.h
	$(Q)-rm -f src/veins/modules/messages/*_m.cc src/veins/modules/messages/*_m.h src/veins/modules/messages/*_sm.cc src/veins/modules/messages/*_sm.h
	$(Q)-rm -f src/veins/modules/mobility/*_m.cc src/veins/modules/mobility/*_m.h src/veins/modules/mobility/*_sm.cc src/veins/modules/mobility/*_sm.h
	$(Q)-rm -f src/veins/modules/mobility/traci/*_m.cc src/veins/modules/mobility/traci/*_m.h src/veins/modules/mobility/traci/*_sm.cc src/veins/modules/mobility/traci/*_sm.h
	$(Q)-rm -f src/veins/modules/nic/*_m.cc src/veins/modules/nic/*_m.h src/veins/modules/nic/*_sm.cc src/veins/modules/nic/*_sm.h
	$(Q)-rm -f src/veins/modules/obstacle/*_m.cc src/veins/modules/obstacle/*_m.h src/veins/modules/obstacle/*_sm.cc src/veins/modules/obstacle/*_sm.h
	$(Q)-rm -f src/veins/modules/phy/*_m.cc src/veins/modules/phy/*_m.h src/veins/modules/phy/*_sm.cc src/veins/modules/phy/*_sm.h
	$(Q)-rm -f src/veins/modules/utility/*_m.cc src/veins/modules/utility/*_m.h src/veins/modules/utility/*_sm.cc src/veins/modules/utility/*_sm.h
	$(Q)-rm -f src/veins/modules/world/*_m.cc src/veins/modules/world/*_m.h src/veins/modules/world/*_sm.cc src/veins/modules/world/*_sm.h
	$(Q)-rm -f src/veins/modules/world/annotations/*_m.cc src/veins/modules/world/annotations/*_m.h src/veins/modules/world/annotations/*_sm.cc src/veins/modules/world/annotations/*_sm.h
	$(Q)-rm -f src/veins/nodes/*_m.cc src/veins/nodes/*_m.h src/veins/nodes/*_sm.cc src/veins/nodes/*_sm.h
	$(Q)-rm -f tests/*_m.cc tests/*_m.h tests/*_sm.cc tests/*_sm.h
	$(Q)-rm -f tests/traci/*_m.cc tests/traci/*_m.h tests/traci/*_sm.cc tests/traci/*_sm.h

cleanall: clean
	$(Q)-rm -rf $(PROJECT_OUTPUT_DIR)

depend:
	$(qecho) Creating dependencies...
	$(Q)$(MAKEDEPEND) $(INCLUDE_PATH) -f Makefile -P\$$O/ -- $(MSG_CC_FILES) $(SM_CC_FILES)  ./*.cc doc/*.cc examples/*.cc examples/TAPASCologne-0.17.0/*.cc examples/TAPASCologne-0.17.0/results/*.cc examples/veins/*.cc examples/veins/results/*.cc images/*.cc images/status/*.cc src/*.cc src/scripts/*.cc src/veins/*.cc src/veins/base/*.cc src/veins/base/connectionManager/*.cc src/veins/base/messages/*.cc src/veins/base/modules/*.cc src/veins/base/phyLayer/*.cc src/veins/base/utils/*.cc src/veins/modules/*.cc src/veins/modules/analogueModel/*.cc src/veins/modules/application/*.cc src/veins/modules/application/ieee80211p/*.cc src/veins/modules/application/traci/*.cc src/veins/modules/mac/*.cc src/veins/modules/mac/ieee80211p/*.cc src/veins/modules/messages/*.cc src/veins/modules/mobility/*.cc src/veins/modules/mobility/traci/*.cc src/veins/modules/nic/*.cc src/veins/modules/obstacle/*.cc src/veins/modules/phy/*.cc src/veins/modules/utility/*.cc src/veins/modules/world/*.cc src/veins/modules/world/annotations/*.cc src/veins/nodes/*.cc tests/*.cc tests/traci/*.cc

# DO NOT DELETE THIS LINE -- make depend depends on it.
$O/src/veins/base/connectionManager/BaseConnectionManager.o: src/veins/base/connectionManager/BaseConnectionManager.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/connectionManager/NicEntryDebug.h \
  src/veins/base/connectionManager/NicEntryDirect.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/connectionManager/ChannelAccess.o: src/veins/base/connectionManager/ChannelAccess.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/ChannelAccess.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/messages/WaveShortMessageWithDst_m.h \
  src/veins/modules/messages/WaveShortMessage_m.h
$O/src/veins/base/connectionManager/ConnectionManager.o: src/veins/base/connectionManager/ConnectionManager.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/ConnectionManager.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/connectionManager/NicEntryDebug.o: src/veins/base/connectionManager/NicEntryDebug.cc \
  src/veins/base/connectionManager/ChannelAccess.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/connectionManager/NicEntryDebug.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/messages/WaveShortMessageWithDst_m.h \
  src/veins/modules/messages/WaveShortMessage_m.h
$O/src/veins/base/connectionManager/NicEntryDirect.o: src/veins/base/connectionManager/NicEntryDirect.cc \
  src/veins/base/connectionManager/ChannelAccess.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/connectionManager/NicEntryDirect.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/messages/WaveShortMessageWithDst_m.h \
  src/veins/modules/messages/WaveShortMessage_m.h
$O/src/veins/base/modules/BaseApplLayer.o: src/veins/base/modules/BaseApplLayer.cc \
  src/veins/base/modules/BaseApplLayer.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseLayer.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/PassedMessage.h \
  src/veins/base/utils/SimpleAddress.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/modules/BaseBattery.o: src/veins/base/modules/BaseBattery.cc \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/modules/BaseLayer.o: src/veins/base/modules/BaseLayer.cc \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseLayer.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/PassedMessage.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/modules/BaseMacLayer.o: src/veins/base/modules/BaseMacLayer.cc \
  src/veins/base/connectionManager/ChannelAccess.h \
  src/veins/base/messages/MacPkt_m.h \
  src/veins/base/modules/AddressingInterface.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseLayer.h \
  src/veins/base/modules/BaseMacLayer.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/phyLayer/AnalogueModel.h \
  src/veins/base/phyLayer/ChannelState.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/MacToPhyControlInfo.h \
  src/veins/base/phyLayer/MacToPhyInterface.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/PhyUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MacToNetwControlInfo.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/NetwToMacControlInfo.h \
  src/veins/base/utils/PassedMessage.h \
  src/veins/base/utils/SimpleAddress.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/messages/WaveShortMessageWithDst_m.h \
  src/veins/modules/messages/WaveShortMessage_m.h
$O/src/veins/base/modules/BaseMobility.o: src/veins/base/modules/BaseMobility.cc \
  src/veins/base/messages/BorderMsg_m.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/modules/BaseModule.o: src/veins/base/modules/BaseModule.cc \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/modules/BaseWorldUtility.o: src/veins/base/modules/BaseWorldUtility.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/modules/BatteryAccess.o: src/veins/base/modules/BatteryAccess.cc \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/phyLayer/BaseDecider.o: src/veins/base/phyLayer/BaseDecider.cc \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/messages/ChannelSenseRequest_m.h \
  src/veins/base/phyLayer/BaseDecider.h \
  src/veins/base/phyLayer/ChannelState.h \
  src/veins/base/phyLayer/Decider.h \
  src/veins/base/phyLayer/DeciderToPhyInterface.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/phyLayer/BasePhyLayer.o: src/veins/base/phyLayer/BasePhyLayer.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/ChannelAccess.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/messages/ChannelSenseRequest_m.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/phyLayer/AnalogueModel.h \
  src/veins/base/phyLayer/BasePhyLayer.h \
  src/veins/base/phyLayer/ChannelInfo.h \
  src/veins/base/phyLayer/ChannelState.h \
  src/veins/base/phyLayer/Decider.h \
  src/veins/base/phyLayer/DeciderToPhyInterface.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/MacToPhyControlInfo.h \
  src/veins/base/phyLayer/MacToPhyInterface.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/PhyToMacControlInfo.h \
  src/veins/base/phyLayer/PhyUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/messages/WaveShortMessageWithDst_m.h \
  src/veins/modules/messages/WaveShortMessage_m.h
$O/src/veins/base/phyLayer/ChannelInfo.o: src/veins/base/phyLayer/ChannelInfo.cc \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/phyLayer/ChannelInfo.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/phyLayer/ChannelState.o: src/veins/base/phyLayer/ChannelState.cc \
  src/veins/base/phyLayer/ChannelState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/phyLayer/Decider.o: src/veins/base/phyLayer/Decider.cc \
  src/veins/base/messages/ChannelSenseRequest_m.h \
  src/veins/base/phyLayer/ChannelState.h \
  src/veins/base/phyLayer/Decider.h \
  src/veins/base/phyLayer/DeciderToPhyInterface.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/phyLayer/MappingBase.o: src/veins/base/phyLayer/MappingBase.cc \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/phyLayer/MappingUtils.o: src/veins/base/phyLayer/MappingUtils.cc \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/phyLayer/PhyUtils.o: src/veins/base/phyLayer/PhyUtils.cc \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/phyLayer/AnalogueModel.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/PhyUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/phyLayer/Signal.o: src/veins/base/phyLayer/Signal.cc \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/utils/Coord.o: src/veins/base/utils/Coord.cc \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/utils/NetwToMacControlInfo.o: src/veins/base/utils/NetwToMacControlInfo.cc \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/NetwToMacControlInfo.h \
  src/veins/base/utils/SimpleAddress.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/utils/SimpleAddress.o: src/veins/base/utils/SimpleAddress.cc \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/SimpleAddress.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/base/utils/asserts.o: src/veins/base/utils/asserts.cc \
  src/veins/base/utils/asserts.h
$O/src/veins/base/utils/winsupport.o: src/veins/base/utils/winsupport.cc \
  src/veins/base/utils/winsupport.h
$O/src/veins/modules/analogueModel/BreakpointPathlossModel.o: src/veins/modules/analogueModel/BreakpointPathlossModel.cc \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/phyLayer/AnalogueModel.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/analogueModel/BreakpointPathlossModel.h
$O/src/veins/modules/analogueModel/JakesFading.o: src/veins/modules/analogueModel/JakesFading.cc \
  src/veins/base/connectionManager/ChannelAccess.h \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/phyLayer/AnalogueModel.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/analogueModel/JakesFading.h \
  src/veins/modules/messages/WaveShortMessageWithDst_m.h \
  src/veins/modules/messages/WaveShortMessage_m.h
$O/src/veins/modules/analogueModel/LogNormalShadowing.o: src/veins/modules/analogueModel/LogNormalShadowing.cc \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/phyLayer/AnalogueModel.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/analogueModel/LogNormalShadowing.h
$O/src/veins/modules/analogueModel/NakagamiFading.o: src/veins/modules/analogueModel/NakagamiFading.cc \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/phyLayer/AnalogueModel.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/analogueModel/NakagamiFading.h
$O/src/veins/modules/analogueModel/PERModel.o: src/veins/modules/analogueModel/PERModel.cc \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/phyLayer/AnalogueModel.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/analogueModel/PERModel.h
$O/src/veins/modules/analogueModel/SimpleObstacleShadowing.o: src/veins/modules/analogueModel/SimpleObstacleShadowing.cc \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/phyLayer/AnalogueModel.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/analogueModel/SimpleObstacleShadowing.h \
  src/veins/modules/obstacle/Obstacle.h \
  src/veins/modules/obstacle/ObstacleControl.h \
  src/veins/modules/world/annotations/AnnotationManager.h
$O/src/veins/modules/analogueModel/SimplePathlossModel.o: src/veins/modules/analogueModel/SimplePathlossModel.cc \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/phyLayer/AnalogueModel.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/analogueModel/SimplePathlossModel.h
$O/src/veins/modules/analogueModel/TwoRayInterferenceModel.o: src/veins/modules/analogueModel/TwoRayInterferenceModel.cc \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/phyLayer/AnalogueModel.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/analogueModel/TwoRayInterferenceModel.h
$O/src/veins/modules/application/ieee80211p/BaseWaveApplLayer.o: src/veins/modules/application/ieee80211p/BaseWaveApplLayer.cc \
  src/veins/base/connectionManager/ChannelAccess.h \
  src/veins/base/modules/BaseApplLayer.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseLayer.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/NetwToMacControlInfo.h \
  src/veins/base/utils/PassedMessage.h \
  src/veins/base/utils/SimpleAddress.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/application/ieee80211p/BaseWaveApplLayer.h \
  src/veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  src/veins/modules/messages/WaveShortMessageWithDst_m.h \
  src/veins/modules/messages/WaveShortMessage_m.h \
  src/veins/modules/utility/Consts80211p.h
$O/src/veins/modules/application/traci/ClusterManager.o: src/veins/modules/application/traci/ClusterManager.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/ChannelAccess.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/modules/BaseApplLayer.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseLayer.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/NetwToMacControlInfo.h \
  src/veins/base/utils/PassedMessage.h \
  src/veins/base/utils/SimpleAddress.h \
  src/veins/base/utils/asserts.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/application/ieee80211p/BaseWaveApplLayer.h \
  src/veins/modules/application/traci/ClusterManager.h \
  src/veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  src/veins/modules/messages/WaveShortMessageWithDst_m.h \
  src/veins/modules/messages/WaveShortMessage_m.h \
  src/veins/modules/mobility/traci/TraCIBuffer.h \
  src/veins/modules/mobility/traci/TraCIColor.h \
  src/veins/modules/mobility/traci/TraCICommandInterface.h \
  src/veins/modules/mobility/traci/TraCIConnection.h \
  src/veins/modules/mobility/traci/TraCICoord.h \
  src/veins/modules/mobility/traci/TraCIMobility.h \
  src/veins/modules/mobility/traci/TraCIScenarioManager.h \
  src/veins/modules/mobility/traci/TraCIScenarioManagerLaunchd.h \
  src/veins/modules/obstacle/Obstacle.h \
  src/veins/modules/obstacle/ObstacleControl.h \
  src/veins/modules/utility/Consts80211p.h \
  src/veins/modules/world/annotations/AnnotationManager.h
$O/src/veins/modules/application/traci/TraCICluster.o: src/veins/modules/application/traci/TraCICluster.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/ChannelAccess.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/modules/BaseApplLayer.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseLayer.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/NetwToMacControlInfo.h \
  src/veins/base/utils/PassedMessage.h \
  src/veins/base/utils/SimpleAddress.h \
  src/veins/base/utils/asserts.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/application/ieee80211p/BaseWaveApplLayer.h \
  src/veins/modules/application/traci/NodeClusterRole.h \
  src/veins/modules/application/traci/NodeData.h \
  src/veins/modules/application/traci/NodeStrategy.h \
  src/veins/modules/application/traci/TraCICluster.h \
  src/veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  src/veins/modules/messages/WaveShortMessageClusterBeacon_m.h \
  src/veins/modules/messages/WaveShortMessageWithDst_m.h \
  src/veins/modules/messages/WaveShortMessage_m.h \
  src/veins/modules/mobility/traci/TraCIBuffer.h \
  src/veins/modules/mobility/traci/TraCIColor.h \
  src/veins/modules/mobility/traci/TraCICommandInterface.h \
  src/veins/modules/mobility/traci/TraCIConnection.h \
  src/veins/modules/mobility/traci/TraCICoord.h \
  src/veins/modules/mobility/traci/TraCIMobility.h \
  src/veins/modules/mobility/traci/TraCIScenarioManager.h \
  src/veins/modules/mobility/traci/TraCIScenarioManagerLaunchd.h \
  src/veins/modules/obstacle/Obstacle.h \
  src/veins/modules/obstacle/ObstacleControl.h \
  src/veins/modules/utility/Consts80211p.h \
  src/veins/modules/world/annotations/AnnotationManager.h
$O/src/veins/modules/application/traci/TraCIDemo11p.o: src/veins/modules/application/traci/TraCIDemo11p.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/ChannelAccess.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/modules/BaseApplLayer.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseLayer.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/NetwToMacControlInfo.h \
  src/veins/base/utils/PassedMessage.h \
  src/veins/base/utils/SimpleAddress.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/application/ieee80211p/BaseWaveApplLayer.h \
  src/veins/modules/application/traci/TraCIDemo11p.h \
  src/veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  src/veins/modules/messages/WaveShortMessageWithDst_m.h \
  src/veins/modules/messages/WaveShortMessage_m.h \
  src/veins/modules/mobility/traci/TraCIBuffer.h \
  src/veins/modules/mobility/traci/TraCIColor.h \
  src/veins/modules/mobility/traci/TraCICommandInterface.h \
  src/veins/modules/mobility/traci/TraCIConnection.h \
  src/veins/modules/mobility/traci/TraCICoord.h \
  src/veins/modules/mobility/traci/TraCIMobility.h \
  src/veins/modules/mobility/traci/TraCIScenarioManager.h \
  src/veins/modules/obstacle/Obstacle.h \
  src/veins/modules/obstacle/ObstacleControl.h \
  src/veins/modules/utility/Consts80211p.h \
  src/veins/modules/world/annotations/AnnotationManager.h
$O/src/veins/modules/application/traci/TraCIDemoRSU11p.o: src/veins/modules/application/traci/TraCIDemoRSU11p.cc \
  src/veins/base/connectionManager/ChannelAccess.h \
  src/veins/base/modules/BaseApplLayer.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseLayer.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/NetwToMacControlInfo.h \
  src/veins/base/utils/PassedMessage.h \
  src/veins/base/utils/SimpleAddress.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/application/ieee80211p/BaseWaveApplLayer.h \
  src/veins/modules/application/traci/TraCIDemoRSU11p.h \
  src/veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  src/veins/modules/messages/WaveShortMessageWithDst_m.h \
  src/veins/modules/messages/WaveShortMessage_m.h \
  src/veins/modules/utility/Consts80211p.h \
  src/veins/modules/world/annotations/AnnotationManager.h
$O/src/veins/modules/application/traci/TraCIMyExp11p.o: src/veins/modules/application/traci/TraCIMyExp11p.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/ChannelAccess.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/modules/BaseApplLayer.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseLayer.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/NetwToMacControlInfo.h \
  src/veins/base/utils/PassedMessage.h \
  src/veins/base/utils/SimpleAddress.h \
  src/veins/base/utils/asserts.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/application/ieee80211p/BaseWaveApplLayer.h \
  src/veins/modules/application/traci/TraCIMyExp11p.h \
  src/veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  src/veins/modules/messages/WaveShortMessageWithDst_m.h \
  src/veins/modules/messages/WaveShortMessage_m.h \
  src/veins/modules/mobility/traci/TraCIBuffer.h \
  src/veins/modules/mobility/traci/TraCIColor.h \
  src/veins/modules/mobility/traci/TraCICommandInterface.h \
  src/veins/modules/mobility/traci/TraCIConnection.h \
  src/veins/modules/mobility/traci/TraCICoord.h \
  src/veins/modules/mobility/traci/TraCIMobility.h \
  src/veins/modules/mobility/traci/TraCIScenarioManager.h \
  src/veins/modules/mobility/traci/TraCIScenarioManagerLaunchd.h \
  src/veins/modules/obstacle/Obstacle.h \
  src/veins/modules/obstacle/ObstacleControl.h \
  src/veins/modules/utility/Consts80211p.h \
  src/veins/modules/world/annotations/AnnotationManager.h
$O/src/veins/modules/application/traci/TraCIStat.o: src/veins/modules/application/traci/TraCIStat.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/ChannelAccess.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/modules/BaseApplLayer.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseLayer.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/NetwToMacControlInfo.h \
  src/veins/base/utils/PassedMessage.h \
  src/veins/base/utils/SimpleAddress.h \
  src/veins/base/utils/asserts.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/application/ieee80211p/BaseWaveApplLayer.h \
  src/veins/modules/application/traci/TraCIStat.h \
  src/veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  src/veins/modules/messages/WaveShortMessageWithDst_m.h \
  src/veins/modules/messages/WaveShortMessage_m.h \
  src/veins/modules/mobility/traci/TraCIBuffer.h \
  src/veins/modules/mobility/traci/TraCIColor.h \
  src/veins/modules/mobility/traci/TraCICommandInterface.h \
  src/veins/modules/mobility/traci/TraCIConnection.h \
  src/veins/modules/mobility/traci/TraCICoord.h \
  src/veins/modules/mobility/traci/TraCIMobility.h \
  src/veins/modules/mobility/traci/TraCIScenarioManager.h \
  src/veins/modules/mobility/traci/TraCIScenarioManagerLaunchd.h \
  src/veins/modules/obstacle/Obstacle.h \
  src/veins/modules/obstacle/ObstacleControl.h \
  src/veins/modules/utility/Consts80211p.h \
  src/veins/modules/world/annotations/AnnotationManager.h
$O/src/veins/modules/application/traci/TraCITestApp.o: src/veins/modules/application/traci/TraCITestApp.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/modules/BaseApplLayer.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseLayer.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/PassedMessage.h \
  src/veins/base/utils/SimpleAddress.h \
  src/veins/base/utils/asserts.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/application/traci/TraCITestApp.h \
  src/veins/modules/mobility/traci/TraCIBuffer.h \
  src/veins/modules/mobility/traci/TraCIColor.h \
  src/veins/modules/mobility/traci/TraCICommandInterface.h \
  src/veins/modules/mobility/traci/TraCIConnection.h \
  src/veins/modules/mobility/traci/TraCICoord.h \
  src/veins/modules/mobility/traci/TraCIMobility.h \
  src/veins/modules/mobility/traci/TraCIScenarioManager.h \
  src/veins/modules/obstacle/Obstacle.h \
  src/veins/modules/obstacle/ObstacleControl.h \
  src/veins/modules/world/annotations/AnnotationManager.h
$O/src/veins/modules/mac/ieee80211p/Mac1609_4.o: src/veins/modules/mac/ieee80211p/Mac1609_4.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/ChannelAccess.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/messages/ChannelSenseRequest_m.h \
  src/veins/base/messages/MacPkt_m.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseLayer.h \
  src/veins/base/modules/BaseMacLayer.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/phyLayer/AnalogueModel.h \
  src/veins/base/phyLayer/BaseDecider.h \
  src/veins/base/phyLayer/BasePhyLayer.h \
  src/veins/base/phyLayer/ChannelInfo.h \
  src/veins/base/phyLayer/ChannelState.h \
  src/veins/base/phyLayer/Decider.h \
  src/veins/base/phyLayer/DeciderToPhyInterface.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/MacToPhyControlInfo.h \
  src/veins/base/phyLayer/MacToPhyInterface.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/PhyToMacControlInfo.h \
  src/veins/base/phyLayer/PhyUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/NetwToMacControlInfo.h \
  src/veins/base/utils/PassedMessage.h \
  src/veins/base/utils/SimpleAddress.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/analogueModel/JakesFading.h \
  src/veins/modules/analogueModel/LogNormalShadowing.h \
  src/veins/modules/analogueModel/SimplePathlossModel.h \
  src/veins/modules/mac/ieee80211p/Mac1609_4.h \
  src/veins/modules/mac/ieee80211p/Mac80211pToPhy11pInterface.h \
  src/veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
  src/veins/modules/messages/Mac80211Pkt_m.h \
  src/veins/modules/messages/PhyControlMessage_m.h \
  src/veins/modules/messages/WaveShortMessageWithDst_m.h \
  src/veins/modules/messages/WaveShortMessage_m.h \
  src/veins/modules/phy/Decider80211p.h \
  src/veins/modules/phy/Decider80211pToPhy80211pInterface.h \
  src/veins/modules/phy/DeciderResult80211.h \
  src/veins/modules/phy/PhyLayer80211p.h \
  src/veins/modules/phy/SNRThresholdDecider.h \
  src/veins/modules/utility/Consts80211p.h \
  src/veins/modules/utility/ConstsPhy.h
$O/src/veins/modules/mobility/traci/TraCIBuffer.o: src/veins/modules/mobility/traci/TraCIBuffer.cc \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/mobility/traci/TraCIBuffer.h \
  src/veins/modules/mobility/traci/TraCIConstants.h \
  src/veins/modules/mobility/traci/TraCICoord.h
$O/src/veins/modules/mobility/traci/TraCIColor.o: src/veins/modules/mobility/traci/TraCIColor.cc \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/mobility/traci/TraCIColor.h
$O/src/veins/modules/mobility/traci/TraCICommandInterface.o: src/veins/modules/mobility/traci/TraCICommandInterface.cc \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/mobility/traci/TraCIBuffer.h \
  src/veins/modules/mobility/traci/TraCIColor.h \
  src/veins/modules/mobility/traci/TraCICommandInterface.h \
  src/veins/modules/mobility/traci/TraCIConnection.h \
  src/veins/modules/mobility/traci/TraCIConstants.h \
  src/veins/modules/mobility/traci/TraCICoord.h
$O/src/veins/modules/mobility/traci/TraCIConnection.o: src/veins/modules/mobility/traci/TraCIConnection.cc \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/mobility/traci/TraCIBuffer.h \
  src/veins/modules/mobility/traci/TraCIConnection.h \
  src/veins/modules/mobility/traci/TraCIConstants.h \
  src/veins/modules/mobility/traci/TraCICoord.h
$O/src/veins/modules/mobility/traci/TraCIMobility.o: src/veins/modules/mobility/traci/TraCIMobility.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/mobility/traci/TraCIBuffer.h \
  src/veins/modules/mobility/traci/TraCIColor.h \
  src/veins/modules/mobility/traci/TraCICommandInterface.h \
  src/veins/modules/mobility/traci/TraCIConnection.h \
  src/veins/modules/mobility/traci/TraCICoord.h \
  src/veins/modules/mobility/traci/TraCIMobility.h \
  src/veins/modules/mobility/traci/TraCIScenarioManager.h \
  src/veins/modules/obstacle/Obstacle.h \
  src/veins/modules/obstacle/ObstacleControl.h \
  src/veins/modules/world/annotations/AnnotationManager.h
$O/src/veins/modules/mobility/traci/TraCIScenarioManager.o: src/veins/modules/mobility/traci/TraCIScenarioManager.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/mobility/traci/TraCIBuffer.h \
  src/veins/modules/mobility/traci/TraCIColor.h \
  src/veins/modules/mobility/traci/TraCICommandInterface.h \
  src/veins/modules/mobility/traci/TraCIConnection.h \
  src/veins/modules/mobility/traci/TraCIConstants.h \
  src/veins/modules/mobility/traci/TraCICoord.h \
  src/veins/modules/mobility/traci/TraCIMobility.h \
  src/veins/modules/mobility/traci/TraCIScenarioManager.h \
  src/veins/modules/mobility/traci/TraCIScenarioManagerInet.h \
  src/veins/modules/obstacle/Obstacle.h \
  src/veins/modules/obstacle/ObstacleControl.h \
  src/veins/modules/world/annotations/AnnotationManager.h
$O/src/veins/modules/mobility/traci/TraCIScenarioManagerInet.o: src/veins/modules/mobility/traci/TraCIScenarioManagerInet.cc \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h
$O/src/veins/modules/mobility/traci/TraCIScenarioManagerLaunchd.o: src/veins/modules/mobility/traci/TraCIScenarioManagerLaunchd.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/mobility/traci/TraCIBuffer.h \
  src/veins/modules/mobility/traci/TraCIColor.h \
  src/veins/modules/mobility/traci/TraCICommandInterface.h \
  src/veins/modules/mobility/traci/TraCIConnection.h \
  src/veins/modules/mobility/traci/TraCIConstants.h \
  src/veins/modules/mobility/traci/TraCICoord.h \
  src/veins/modules/mobility/traci/TraCIScenarioManager.h \
  src/veins/modules/mobility/traci/TraCIScenarioManagerLaunchd.h \
  src/veins/modules/obstacle/Obstacle.h \
  src/veins/modules/obstacle/ObstacleControl.h \
  src/veins/modules/world/annotations/AnnotationManager.h
$O/src/veins/modules/mobility/traci/TraCIScreenRecorder.o: src/veins/modules/mobility/traci/TraCIScreenRecorder.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/mobility/traci/TraCIBuffer.h \
  src/veins/modules/mobility/traci/TraCIColor.h \
  src/veins/modules/mobility/traci/TraCICommandInterface.h \
  src/veins/modules/mobility/traci/TraCIConnection.h \
  src/veins/modules/mobility/traci/TraCICoord.h \
  src/veins/modules/mobility/traci/TraCIScenarioManager.h \
  src/veins/modules/mobility/traci/TraCIScreenRecorder.h \
  src/veins/modules/obstacle/Obstacle.h \
  src/veins/modules/obstacle/ObstacleControl.h \
  src/veins/modules/world/annotations/AnnotationManager.h
$O/src/veins/modules/obstacle/Obstacle.o: src/veins/modules/obstacle/Obstacle.cc \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/obstacle/Obstacle.h \
  src/veins/modules/world/annotations/AnnotationManager.h
$O/src/veins/modules/obstacle/ObstacleControl.o: src/veins/modules/obstacle/ObstacleControl.cc \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/obstacle/Obstacle.h \
  src/veins/modules/obstacle/ObstacleControl.h \
  src/veins/modules/world/annotations/AnnotationManager.h
$O/src/veins/modules/phy/Decider80211p.o: src/veins/modules/phy/Decider80211p.cc \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/messages/ChannelSenseRequest_m.h \
  src/veins/base/messages/MacPkt_m.h \
  src/veins/base/phyLayer/AnalogueModel.h \
  src/veins/base/phyLayer/BaseDecider.h \
  src/veins/base/phyLayer/ChannelState.h \
  src/veins/base/phyLayer/Decider.h \
  src/veins/base/phyLayer/DeciderToPhyInterface.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/MacToPhyInterface.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/PhyUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/SimpleAddress.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/mac/ieee80211p/Mac80211pToPhy11pInterface.h \
  src/veins/modules/messages/AirFrame11p_m.h \
  src/veins/modules/messages/Mac80211Pkt_m.h \
  src/veins/modules/phy/Decider80211p.h \
  src/veins/modules/phy/Decider80211pToPhy80211pInterface.h \
  src/veins/modules/phy/DeciderResult80211.h \
  src/veins/modules/phy/NistErrorRate.h \
  src/veins/modules/utility/Consts80211p.h \
  src/veins/modules/utility/ConstsPhy.h
$O/src/veins/modules/phy/NistErrorRate.o: src/veins/modules/phy/NistErrorRate.cc \
  src/veins/modules/phy/NistErrorRate.h \
  src/veins/modules/utility/ConstsPhy.h
$O/src/veins/modules/phy/PhyLayer80211p.o: src/veins/modules/phy/PhyLayer80211p.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/ChannelAccess.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/messages/ChannelSenseRequest_m.h \
  src/veins/base/modules/BaseBattery.h \
  src/veins/base/modules/BaseMobility.h \
  src/veins/base/modules/BaseModule.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/modules/BatteryAccess.h \
  src/veins/base/phyLayer/AnalogueModel.h \
  src/veins/base/phyLayer/BaseDecider.h \
  src/veins/base/phyLayer/BasePhyLayer.h \
  src/veins/base/phyLayer/ChannelInfo.h \
  src/veins/base/phyLayer/ChannelState.h \
  src/veins/base/phyLayer/Decider.h \
  src/veins/base/phyLayer/DeciderToPhyInterface.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/MacToPhyControlInfo.h \
  src/veins/base/phyLayer/MacToPhyInterface.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/PhyUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/HostState.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/Move.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/analogueModel/BreakpointPathlossModel.h \
  src/veins/modules/analogueModel/JakesFading.h \
  src/veins/modules/analogueModel/LogNormalShadowing.h \
  src/veins/modules/analogueModel/NakagamiFading.h \
  src/veins/modules/analogueModel/PERModel.h \
  src/veins/modules/analogueModel/SimpleObstacleShadowing.h \
  src/veins/modules/analogueModel/SimplePathlossModel.h \
  src/veins/modules/analogueModel/TwoRayInterferenceModel.h \
  src/veins/modules/mac/ieee80211p/Mac80211pToPhy11pInterface.h \
  src/veins/modules/messages/AirFrame11p_m.h \
  src/veins/modules/messages/WaveShortMessageWithDst_m.h \
  src/veins/modules/messages/WaveShortMessage_m.h \
  src/veins/modules/obstacle/Obstacle.h \
  src/veins/modules/obstacle/ObstacleControl.h \
  src/veins/modules/phy/Decider80211p.h \
  src/veins/modules/phy/Decider80211pToPhy80211pInterface.h \
  src/veins/modules/phy/PhyLayer80211p.h \
  src/veins/modules/phy/SNRThresholdDecider.h \
  src/veins/modules/utility/Consts80211p.h \
  src/veins/modules/world/annotations/AnnotationManager.h
$O/src/veins/modules/phy/SNRThresholdDecider.o: src/veins/modules/phy/SNRThresholdDecider.cc \
  src/veins/base/messages/AirFrame_m.h \
  src/veins/base/messages/ChannelSenseRequest_m.h \
  src/veins/base/phyLayer/BaseDecider.h \
  src/veins/base/phyLayer/ChannelState.h \
  src/veins/base/phyLayer/Decider.h \
  src/veins/base/phyLayer/DeciderToPhyInterface.h \
  src/veins/base/phyLayer/Interpolation.h \
  src/veins/base/phyLayer/Mapping.h \
  src/veins/base/phyLayer/MappingBase.h \
  src/veins/base/phyLayer/MappingUtils.h \
  src/veins/base/phyLayer/Signal_.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/phy/SNRThresholdDecider.h
$O/src/veins/modules/world/annotations/AnnotationDummy.o: src/veins/modules/world/annotations/AnnotationDummy.cc \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/world/annotations/AnnotationDummy.h
$O/src/veins/modules/world/annotations/AnnotationManager.o: src/veins/modules/world/annotations/AnnotationManager.cc \
  src/veins/base/connectionManager/BaseConnectionManager.h \
  src/veins/base/connectionManager/NicEntry.h \
  src/veins/base/modules/BaseWorldUtility.h \
  src/veins/base/utils/Coord.h \
  src/veins/base/utils/FWMath.h \
  src/veins/base/utils/FindModule.h \
  src/veins/base/utils/MiXiMDefs.h \
  src/veins/base/utils/miximkerneldefs.h \
  src/veins/modules/mobility/traci/TraCIBuffer.h \
  src/veins/modules/mobility/traci/TraCIColor.h \
  src/veins/modules/mobility/traci/TraCICommandInterface.h \
  src/veins/modules/mobility/traci/TraCIConnection.h \
  src/veins/modules/mobility/traci/TraCICoord.h \
  src/veins/modules/mobility/traci/TraCIScenarioManager.h \
  src/veins/modules/obstacle/Obstacle.h \
  src/veins/modules/obstacle/ObstacleControl.h \
  src/veins/modules/world/annotations/AnnotationManager.h
