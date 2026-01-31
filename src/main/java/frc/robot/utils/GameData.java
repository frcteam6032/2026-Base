package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class GameData {
    public static DriverStation.Alliance Alliance = DriverStation.Alliance.Blue;

    public static int shouldInvertControls() {
        return Alliance == DriverStation.Alliance.Red ? 1 : -1;
    }

    public static boolean getIsRed() {
        return Alliance == DriverStation.Alliance.Red;
    }
}
