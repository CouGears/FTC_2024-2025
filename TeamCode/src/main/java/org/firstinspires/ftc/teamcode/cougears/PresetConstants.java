package org.firstinspires.ftc.teamcode.cougears;


import com.acmerobotics.dashboard.config.Config;

@Config
public class PresetConstants {
    // init, high drop, high hold, mid, low
    public static int[] slidePresets = {0, 5400, 5400, 0, 0};
    // init, high drop, high hold, mid, low
    public static int[] armThetaPresets = {0, 200, 100, 600, 700};
    // init, high drop, high hold, mid, low
    public static double[] axis1Presets = {0.38, 0.5, 0.5, 0.57, 0.63};
    public static int[] axis2Positions = {2, 1, 0, 3, 4};

    // center, half left, full left, half right, full right
    public static double[] axis2Presets = {0.43, 0.595, 0.76, 0.265, 0.1}; // good
    // open, closed
    public static double[] clawPresets = {0.35, 0.26}; // good
}
