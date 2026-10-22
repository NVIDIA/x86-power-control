#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: Copyright OpenBMC Authors

"""Exercise production shutdown methods with simulated GPIOs and side effects.

Run: CXX=c++ python3 test/nvl72_shutdown_state_test.py
Reproduce the original failure: append --baseline <unpatched-ref>
This is a focused host-side regression test, not a full daemon or DUT test.
"""

import argparse
import os
import subprocess
import tempfile
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--baseline", help="Git ref without the NVL72 shutdown overrides"
)
parser.add_argument(
    "--source-ref", help="Git ref with the overrides to exercise"
)
args = parser.parse_args()
repo = Path(__file__).resolve().parents[1]
before = args.baseline is not None
temporary = tempfile.TemporaryDirectory(prefix="nvl72-shutdown-test-")
out = Path(temporary.name)


def source(path):
    if before or args.source_ref:
        return subprocess.check_output(
            ["git", "show", (args.baseline or args.source_ref) + ":" + path],
            cwd=repo,
            text=True,
        )
    return (repo / path).read_text()


def extract(text, signature):
    start = text.index(signature)
    a = text.index("{", start)
    depth = 1
    i = a + 1
    while depth:
        depth += (text[i] == "{") - (text[i] == "}")
        i += 1
    return text[start:i]


h = source("src/power_control_base.hpp")
b = source("src/power_control_base.cpp")
v = source("src/vr_power_control.cpp")
n = source("src/platform/nvl72/nvl72_power_control.cpp")
state = (
    extract(h, "enum class PowerState")
    + ";"
    + extract(h, "enum class AuxPowerCycleState")
    + ";"
    + extract(h, "enum class AuxPowerCycleVariant")
    + ";"
)
action = extract(h, "enum class PowerAction") + ";"
code = (
    r"""
#include <string>
#include <string_view>
#include <vector>
#include <stdexcept>
#include <system_error>
#include <iostream>
#include <memory>
#include <functional>
namespace boost::asio { template<class F> void post(std::vector<std::function<void()>>& q,F f){q.emplace_back(f);} }
namespace lg2 { template<class... A> void info(A&&...){} template<class... A> void error(A&&...){} template<class... A> void warning(A&&...){} }
"""
    + state
    + action
    + r"""
enum class Event { pdbMainPowerOkDeAssert, pdbMainPowerOkWatchdogTimerExpired, powerOnRequest };
struct Line { int value=0; bool throws=false, unexpected=false; explicit operator bool() const {return true;} int get_value() {if(unexpected) throw std::runtime_error("unexpected failure"); if(throws) throw std::system_error(std::make_error_code(std::errc::io_error), "GPIO read failed"); return value;} };
struct Signal { Line gpioLine; int polarity=1; };
class PowerControl {
public:
    PowerState powerState=PowerState::off, auxPowerCycleReturnState=PowerState::off, preSysResetReturnState=PowerState::off;
    PowerAction action=PowerAction::NONE;
    int errors=0, offOutputWrites=0, shutdowns=0, cycleDelays=0, pdbMainPowerOkWatchdogTimer=0;
    int cuts=0, forces=0, ended=0;
    AuxPowerCycleState auxPowerCycleState=AuxPowerCycleState::inactive;
    AuxPowerCycleVariant auxPowerCycleVariant=AuxPowerCycleVariant::none;
    std::vector<std::function<void()>> ioContext;
    bool isAuxPowerCycleActive() const { return auxPowerCycleState!=AuxPowerCycleState::inactive; }
    void initiateForcefulShutdown(){++forces;}
    void endAuxPowerCycle(){++ended;auxPowerCycleState=AuxPowerCycleState::inactive;}
    bool assertAuxPowerCycle(){++cuts;auxPowerCycleReturnState=powerState;powerState=PowerState::waitForAuxPowerCycle;return true;}
    void drain(){auto tasks=std::move(ioContext);for(auto& task:tasks)task();}
    ADVANCE_DECL

    Signal hpm, pdb; bool missing=false;
    virtual std::string_view getHostState() const;
    virtual std::string_view getChassisState() const;
    std::string getPowerStateName() const {return "test";}
    void logResourceEvent(const char*,std::vector<std::string>,const char*) {++errors;}
    void cancelTimer(const char*,int&) {}
    void setGPIOsForHostStateOff() {++offOutputWrites;}
    void setPowerState(PowerState state) {powerState=state;}
    Signal* getSignal(const std::string& s) {return missing ? nullptr : (s=="Board0RunPowerPG"? &hpm : &pdb);}
    void transitionToOffState() {++shutdowns;action=PowerAction::NONE;setPowerState(PowerState::off);}
    void transitionToPowerCycleDelay() {++cycleDelays;setPowerState(PowerState::waitForPowerCycleDelay);}
};
class VRPowerControl: public PowerControl {
public:
    std::string_view getHostState() const override;
    std::string_view getChassisState() const override;
    virtual void completeShutdownAndTransitionToOff(bool success);
    void applyShutdownAction();
    void handleWaitForPDBMainPowerOff(Event);
};
class NVL72PowerControl: public VRPowerControl {
public:
    bool isSystemPowerOff();
"""
)
if not before:
    code += "std::string_view getHostState() const override;\nvoid completeShutdownAndTransitionToOff(bool success) override;\n"
code += "};\n"
code = code.replace(
    "ADVANCE_DECL",
    (
        "void advanceAuxPowerCycle(bool shutdownFailed=false);"
        if "void PowerControl::advanceAuxPowerCycle(bool shutdownFailed)" in b
        else "void advanceAuxPowerCycle();"
    ),
)
code += extract(b, "void PowerControl::advanceAuxPowerCycle(") + "\n"
for text, sigs in [
    (
        b,
        [
            "std::string_view PowerControl::getHostState() const",
            "std::string_view PowerControl::getChassisState() const",
        ],
    ),
    (
        v,
        [
            "std::string_view VRPowerControl::getHostState() const",
            "std::string_view VRPowerControl::getChassisState() const",
            "void VRPowerControl::completeShutdownAndTransitionToOff(bool success)",
            "void VRPowerControl::applyShutdownAction()",
            "void VRPowerControl::handleWaitForPDBMainPowerOff(Event event)",
        ],
    ),
    (
        n,
        ["bool NVL72PowerControl::isSystemPowerOff()"]
        + (
            []
            if before
            else [
                "std::string_view NVL72PowerControl::getHostState() const",
                "void NVL72PowerControl::completeShutdownAndTransitionToOff(bool success)",
            ]
        ),
    ),
]:
    for sig in sigs:
        code += extract(text, sig) + "\n"
code += r"""
int main() {
 int checks=0,failures=0;
 auto check=[&](bool ok,const std::string& name){++checks;if(!ok){++failures;std::cout<<"FAIL: "<<name<<"\n";}};
 auto host=[](const NVL72PowerControl& x,std::string_view suffix){return x.getHostState()==std::string("xyz.openbmc_project.State.Host.HostState.")+std::string(suffix);};
 for(auto a:{PowerAction::FORCE_OFF,PowerAction::GRACE_OFF,PowerAction::POWER_CYCLE,PowerAction::GRACEFUL_POWER_CYCLE,PowerAction::HOST_INITIATED_SHUTDOWN,PowerAction::NONE}) {
   NVL72PowerControl x; x.action=a;
   x.powerState=PowerState::waitForHPMPowerGoodDeAssert;
   check(host(x,"TransitioningToOff"),"HPM teardown must report PoweringOff");
   x.powerState=PowerState::waitForPDBMainPowerOff;
   check(host(x,"TransitioningToOff"),"PDB teardown must report PoweringOff");
 }
 for(auto a:{PowerAction::FORCE_OFF,PowerAction::GRACE_OFF,PowerAction::POWER_CYCLE,PowerAction::GRACEFUL_POWER_CYCLE,PowerAction::HOST_INITIATED_SHUTDOWN}) {
   NVL72PowerControl x;x.action=a;x.powerState=PowerState::waitForPDBMainPowerOff;x.pdb.gpioLine.value=1;
   x.handleWaitForPDBMainPowerOff(Event::pdbMainPowerOkWatchdogTimerExpired);
   check(host(x,"TransitioningToOff"),"stuck PDB must not report Off after timeout");
   check(x.errors==1,"stuck PDB must record error");
   check(x.cycleDelays==0,"timeout must not start automatic power-on");
   x.handleWaitForPDBMainPowerOff(Event::powerOnRequest);
   check(x.powerState==PowerState::waitForPDBMainPowerOff,"new On must not bypass incomplete shutdown");
   x.pdb.gpioLine.value=0;x.handleWaitForPDBMainPowerOff(Event::pdbMainPowerOkDeAssert);
   check(host(x,"Off") && x.cycleDelays==0,"late falling edge completes Off without delayed restart");
 }
 for(auto a:{PowerAction::FORCE_OFF,PowerAction::GRACE_OFF,PowerAction::POWER_CYCLE,PowerAction::GRACEFUL_POWER_CYCLE}) {
   NVL72PowerControl x;x.action=a;x.powerState=PowerState::waitForPDBMainPowerOff;
   x.handleWaitForPDBMainPowerOff(Event::pdbMainPowerOkWatchdogTimerExpired);
   check(x.errors==0,"missed edge with both domains off must not report a hardware fault");
   bool cycle=a==PowerAction::POWER_CYCLE || a==PowerAction::GRACEFUL_POWER_CYCLE;
   check(cycle ? x.cycleDelays==1 : host(x,"Off"),"confirmed missed edge preserves successful shutdown/cycle");
 }
 for(int mode=0;mode<3;++mode) {
   NVL72PowerControl x;x.action=PowerAction::FORCE_OFF;x.powerState=PowerState::waitForPDBMainPowerOff;
   if(mode==0)x.hpm.gpioLine.value=1;
   if(mode==1)x.missing=true;
   if(mode==2)x.pdb.gpioLine.throws=true;
   x.completeShutdownAndTransitionToOff(false);
   check(host(x,"TransitioningToOff"),"HPM still on, missing signal, or read exception must not confirm Off");
 }
 if (!BASELINE) {
   NVL72PowerControl x;x.action=PowerAction::POWER_CYCLE;x.powerState=PowerState::waitForPDBMainPowerOff;x.pdb.gpioLine.unexpected=true;
   bool propagated=false;
   try {x.completeShutdownAndTransitionToOff(false);} catch(const std::runtime_error&) {propagated=true;}
   check(propagated,"unexpected exception must propagate");
   check(x.cycleDelays==0 && x.shutdowns==0,"unexpected failure must not complete or restart shutdown");
 }
 NVL72PowerControl x;x.powerState=PowerState::waitForPDBMainPowerOff;x.action=PowerAction::POWER_ON;
 check(host(x,"TransitioningToRunning"),"power-on teardown indication preserved");
 x.powerState=PowerState::on;x.action=PowerAction::NONE;check(host(x,"Running"),"stable On preserved");
 x.powerState=PowerState::off;check(host(x,"Off"),"stable Off preserved");
 x.action=PowerAction::HOST_INITIATED_REBOOT;check(x.getChassisState()=="xyz.openbmc_project.State.Chassis.PowerState.On","warm reboot chassis power preserved");
 x.powerState=PowerState::waitForPDBMainPowerOff;x.action=PowerAction::FORCE_OFF;
 x.handleWaitForPDBMainPowerOff(Event::pdbMainPowerOkDeAssert);check(host(x,"Off"),"normal falling edge completes shutdown");
 for(auto variant:{AuxPowerCycleVariant::auxCycle,AuxPowerCycleVariant::auxCycleForce,AuxPowerCycleVariant::fullCycle}) {
   for(auto phase:{AuxPowerCycleState::graceful,AuxPowerCycleState::forceful}) {
     NVL72PowerControl y;y.action=PowerAction::FORCE_OFF;y.powerState=PowerState::waitForPDBMainPowerOff;y.pdb.gpioLine.value=1;
     y.auxPowerCycleVariant=variant;y.auxPowerCycleState=phase;
     y.completeShutdownAndTransitionToOff(false);
     check(y.cuts==0 && y.forces==0,"aux failure handoff is deferred until state handler returns");
     y.drain();
     if(phase==AuxPowerCycleState::graceful) {
       check(y.forces==1 && y.cuts==0,"active AUX graceful failure escalates exactly once");
       check(y.auxPowerCycleState==AuxPowerCycleState::forceful,"AUX escalation records forceful phase");
     } else if(variant==AuxPowerCycleVariant::auxCycle) {
       check(y.ended==1 && y.cuts==0,"ordinary AUX aborts after unconfirmed forceful shutdown");
       check(host(y,"TransitioningToOff"),"ordinary AUX failure never publishes false Off");
     } else {
       check(y.cuts==1 && y.forces==0,"forced AUX and FullPowerCycle cut standby after shutdown failure");
       check(y.powerState==PowerState::waitForAuxPowerCycle,"forced cycle advances instead of hanging in PDB wait");
       check(host(y,"TransitioningToOff"),"forced AUX preserves unconfirmed shutdown state while cutting standby");
       y.powerState=y.auxPowerCycleReturnState;
       check(host(y,"TransitioningToOff"),"failed standby cut restores pending shutdown instead of Off");
     }
   }
 }
 NVL72PowerControl late;late.action=PowerAction::FORCE_OFF;late.powerState=PowerState::waitForPDBMainPowerOff;late.pdb.gpioLine.value=1;
 late.auxPowerCycleVariant=AuxPowerCycleVariant::fullCycle;late.auxPowerCycleState=AuxPowerCycleState::forceful;
 late.completeShutdownAndTransitionToOff(false);late.pdb.gpioLine.value=0;
 late.handleWaitForPDBMainPowerOff(Event::pdbMainPowerOkDeAssert);late.drain();
 check(late.cuts==0 && late.forces==0,"late completion invalidates deferred shutdown-failure handoff");
 std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;
}
"""
code = code.replace("BASELINE", "true" if before else "false")
variant = "before" if before else "after"
cpp = out / (variant + ".cpp")
cpp.write_text(code)
subprocess.run(
    [
        os.environ.get("CXX", "c++"),
        "-std=c++23",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-fsanitize=undefined",
        "-g",
        str(cpp),
        "-o",
        str(out / variant),
    ],
    check=True,
)
result = subprocess.run([str(out / variant)]).returncode
temporary.cleanup()
raise SystemExit(result)
