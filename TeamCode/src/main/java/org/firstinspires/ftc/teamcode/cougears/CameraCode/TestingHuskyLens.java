/*
Copyright (c) 2023 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.cougears.CameraCode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;


import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 *
 * For detailed instructions on how a HuskyLens is used in FTC, please see this tutorial:
 * https://ftc-docs.firstinspires.org/en/latest/devices/huskylens/huskylens.html
 * 
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "HuskyLens Test")
public class TestingHuskyLens extends LinearOpMode {

    private HuskyLens huskyLens;
    private final String aprilTagIDToName[] = {"BLoadingZone", "BMiddle", "BBucket", "RLoadingZone", "RMiddle", "RBucket"};

    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, "HuskyLens");
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (HuskyLens.Block currblock : blocks)
            {
//                if (currblock.id - 1 > -1) // Safety :)
//                    telemetry.addData("Block ID", aprilTagIDToName[currblock.id - 1]);
                telemetry.addData("Block Center X", currblock.x);
                telemetry.addData("Block Center Y", currblock.y);
                telemetry.addData("Block Height", currblock.height);
                telemetry.addData("Block Width", currblock.width);
                telemetry.addData("Block Left", currblock.left);
                telemetry.addData("Block Top", currblock.top);
                // Calculate rotation using width and height
                // Honestly this is too confusing and can be simplified
                double width = currblock.width;
                double height = currblock.height;
                double ratio = Math.min(width, height)/Math.max(width, height);
                double thetaRadians = Math.atan(ratio);
                double thetaDegrees = Math.toDegrees(thetaRadians);
                if(width < height){
                    thetaDegrees = 90 - thetaDegrees;
                }
                thetaDegrees = map(thetaDegrees, 24, 67, 0, 90);
//                if (currblock.top - height < 120) // Bottom Left y cord
//                    thetaDegrees *= -1;
                telemetry.addData("Rotation (degrees)", thetaDegrees);
            }
            telemetry.update();
        }
    }
    public static double map(double value, double originalLow, double originalHigh, double newLow, double newHigh) {
        if(originalLow == originalHigh){
            throw new IllegalArgumentException("Original low cannot equal original high");
        }

        // Calculate the normalized value within the original range.
        double normalizedValue = (value - originalLow) / (originalHigh - originalLow);

        // Map the normalized value to the new range.
        double mappedValue = newLow + normalizedValue * (newHigh - newLow);

        return mappedValue;
    }
}