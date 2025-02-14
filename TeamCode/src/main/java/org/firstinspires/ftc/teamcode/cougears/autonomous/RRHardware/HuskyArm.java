package org.firstinspires.ftc.teamcode.cougears.autonomous.RRHardware;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HuskyArm {
    private HuskyLens husky;
    public HuskyArm(HardwareMap hardwareMap) {
        husky = hardwareMap.get(HuskyLens.class, "husky2");
    }
}
