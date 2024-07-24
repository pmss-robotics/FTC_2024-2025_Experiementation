package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.SampleMechanism;

public class SampleSubsystem extends SubsystemBase {
    public SampleMechanism sampleMechanism;
    public SampleSubsystem (HardwareMap hardwareMap) {
        sampleMechanism = new SampleMechanism(hardwareMap);
    }
    public Action doAction(){
        return sampleMechanism.doSampleMechanismAction();
    }
}
