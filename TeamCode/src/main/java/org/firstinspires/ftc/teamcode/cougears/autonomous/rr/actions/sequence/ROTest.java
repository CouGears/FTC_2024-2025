package org.firstinspires.ftc.teamcode.cougears.autonomous.rr.actions.sequence;

import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.cougears.autonomous.rr.actions.drive.TAB;
import org.firstinspires.ftc.teamcode.cougears.autonomous.rr.actions.hardware.SampleActions;
import org.firstinspires.ftc.teamcode.cougears.autonomous.rr.hardware.Arm;
import org.firstinspires.ftc.teamcode.cougears.autonomous.rr.hardware.Axis1;
import org.firstinspires.ftc.teamcode.cougears.autonomous.rr.hardware.Axis2;
import org.firstinspires.ftc.teamcode.cougears.autonomous.rr.hardware.Claw;
import org.firstinspires.ftc.teamcode.cougears.autonomous.rr.hardware.Viper;
import org.firstinspires.ftc.teamcode.cougears.autonomous.rr.actions.logic.Wait;
public class ROTest {
    // Starts at RBI
    // Scores all nearby neutral field samples in high bucket
    // Parks in E6
    public static SequentialAction AutonRO1(MecanumDrive drive, Viper viper, Arm arm, Claw claw, Axis1 axis1, Axis2 axis2) {
        TAB tab = new TAB(drive);
        SampleActions sampleActions = new SampleActions(drive, viper, arm, claw, axis1, axis2);
        Wait wait = new Wait(drive);

        return new SequentialAction(
                axis1.axis1To(0.8),
                arm.armToImpatient(300),
                tab.ROItoRSubPreppose1().build(),
                arm.armToImpatient(400),
                wait.waitMillieconds(250),
                tab.RSubPreppose1toRSubPostpose1().build()
        );
    }
}
