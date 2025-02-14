package org.firstinspires.ftc.teamcode.cougears.autonomous;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class AdvancedAutonomousMethods {
    public HardwareMap map;
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBL;
    public DcMotor motorBR;
    public DcMotorEx slideLeft;
    public DcMotorEx slideRight;
    public DcMotor armThetaDC;
    public Servo clawAxis1Servo;
    public Servo clawAxis2Servo;
    public Servo clawGrabServo;
    public GoBildaPinpointDriver odo;
    public HuskyLens husky1;
    public HuskyLens husky2;

    public AdvancedAutonomousMethods(HardwareMap hardwareMap) {
        this.map = hardwareMap;
        mapDevices();
        setupDevices();
    }

    private void mapDevices() {
        motorFL = map.get(DcMotor.class, "motorFL");
        motorFR = map.get(DcMotor.class, "motorFR");
        motorBL = map.get(DcMotor.class, "motorBL");
        motorBR = map.get(DcMotor.class, "motorBR");

        // Initialize slide motors
        slideLeft = map.get(DcMotorEx.class, "viperL");
        slideRight = map.get(DcMotorEx.class, "viperR");

        armThetaDC = map.get(DcMotor.class, "arm");

        // Initialize servos
        clawAxis1Servo = map.get(Servo.class, "axis1");
        clawAxis2Servo = map.get(Servo.class, "axis2");
        clawGrabServo = map.get(Servo.class, "claw");

        odo = map.get(GoBildaPinpointDriver.class, "odo");

        husky1 = map.get(HuskyLens.class, "husky1");
        husky2 = map.get(HuskyLens.class, "husky2");
    }

    public void setupDevices() {

    }
}
