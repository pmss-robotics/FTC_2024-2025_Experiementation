package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

// https://docs.ftclib.org/ftclib/command-base/command-system/subsystems
public class SampleSubsystem extends SubsystemBase {

    // declare hardware here

    public SampleSubsystem (HardwareMap hardwareMap) {
        // initialize hardware here alongside other parameters
    }
    @Override
    public void periodic() {
        /*
        Put necessary control flows that need to be updated per command scheduler run
        Similar functionality can be accomplished through default commands.
        I think we should limit periodic() to telemetry and other things alike
         */
    }

    public void doThing() {
        /*
        In each subsystem there will be various 'simpler' methods that can be called
        by commands to achieve some desired more 'complex' behaviour.
         */
    }
}
