package org.firstinspires.ftc.teamcode.cougears.autonomous.rr.actions.sequence;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;

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

    public static SequentialAction AutonRO1(MecanumDrive drive, Viper viper, Arm arm, Claw claw, Axis1 axis1, Axis2 axis2) {
        TAB tab = new TAB(drive);
        SampleActions sampleActions = new SampleActions(drive, viper, arm, claw, axis1, axis2);
        Wait wait = new Wait(drive);

        return new SequentialAction(
                //Place first specimen
                axis1.axis1To(0.8),
                arm.armToImpatient(300),
                tab.ROItoRSubPreppose1().build(),
                arm.armToImpatient(400),
                wait.waitMillieconds(250),
                tab.RSubPreppose1toRSubPostpose1().build(),
                claw.openClaw(),

                //Pick up sample and drop it
                tab.RSubPostpose1toFS10().build(),
                sampleActions.grabSampleAtAngle(1),
                tab.FS10toFSX10().build(),
                arm.armToPatient(600),
                claw.openClaw(),

                //Move to wait spot, wait, then pick up prepped specimen
                tab.FSX10toROWaiting().build(),
                wait.waitSeconds(3),
                tab.ROWaittoFSXX10().build(),
//                tab.FSX10toFSXX10().build(),
                axis1.axis1To(1.0),
                arm.armToPatientCustomSpeed(820, 0.2),
                claw.closeClaw(),
                wait.waitMillieconds(500),
                arm.armToImpatient(300),

                //Move to bar and place second sample
                axis1.axis1To(0.8),
                axis2.axis2ToPreset(4),
                arm.armToImpatient(300),
                tab.FSXX10toRSubPreppose2().build(),
                arm.armToImpatient(400),
                wait.waitMillieconds(250),
                tab.RSubPreppose2toRSubPostpose2().build(),
                claw.openClaw(),

                //Move to pickup (no wait here) and pickup new specimen
                axis1.axis1To(1.0),
                axis2.axis2ToPreset(0),
                tab.RSubPostpose2toFSXX10().build(),
                arm.armToPatientCustomSpeed(820, 0.2),
                claw.closeClaw(),
                wait.waitMillieconds(500),
                arm.armToImpatient(300),

                //Move back to bar and place last specimen
                axis1.axis1To(0.8),
                axis2.axis2ToPreset(4),
                arm.armToImpatient(300),
                tab.FSXX10toRSubPreppose3().build(),
                arm.armToImpatient(400),
                wait.waitMillieconds(250),
                tab.RSubPreppose3toRSubPostpose3().build(),
                claw.openClaw(),
                wait.waitMillieconds(500),
                arm.armToPatient(0) // set arm up for teleop
        );
    }
}
