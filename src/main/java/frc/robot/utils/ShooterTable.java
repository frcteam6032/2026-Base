package frc.robot.utils;

import java.util.ArrayList;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterTable {
    public static final class ShooterTableEntry {
        public double distanceMeters;
        public double angleDegrees;
        public double beans;
        public double heckyOffsetMeters;

        public ShooterTableEntry(double distanceMeters, double angleDegrees, double beans,
                double heckyOffsetMeters) {
            this.distanceMeters = distanceMeters;
            this.angleDegrees = angleDegrees;
            this.beans = beans;
            this.heckyOffsetMeters = heckyOffsetMeters;
        }

        public ShooterTableEntry average(ShooterTableEntry other) {
            return new ShooterTableEntry((distanceMeters + other.distanceMeters) / 2.0,
                    (angleDegrees + other.angleDegrees) / 2.0,
                    (beans + other.beans) / 2.0,
                    (heckyOffsetMeters + other.heckyOffsetMeters) / 2.0);
        }

        public ShooterTableEntry applyHeckyOffset() {
            double rotationThreshold = 30.0;
            double allianceZeroAngle = allianceIsBlue() ? 0 : 180;
            if (heckinessSupplier != null
                    && Math.abs(heckinessSupplier.get().getDegrees() - allianceZeroAngle) > rotationThreshold) {
                distanceMeters += heckyOffsetMeters;
            }

            return this;
        }
    }



    private static ArrayList<ShooterTableEntry> shooterTable = new ArrayList<>();


    private static Supplier<Rotation2d> heckinessSupplier;


    private static boolean allianceIsBlue() {
        var maybe = DriverStation.getAlliance();
        return maybe != null ? (maybe.equals(DriverStation.Alliance.Blue)) : false;
    }

    private static double feetToMeters(double feet) {
        return Units.inchesToMeters(feet * 12.0);
    }

    private static void fillInTable() {
        /*
         * *
         * // shooter table entries (converted from example values in feet)
         * shooterTable.add(new ShooterTableEntry(feetToMeters(4.4), 30.9 + 0.25, 0.6,
         * 0.0));
         * shooterTable.add(new ShooterTableEntry(feetToMeters(5.0), 28.0 + 0.25, 0.7,
         * 0.0));
         * shooterTable.add(new ShooterTableEntry(feetToMeters(6.0), 23.8 + 0.25, 0.8,
         * 0.0));
         * shooterTable.add(new ShooterTableEntry(feetToMeters(8.0), 17.0 + 0.25, 0.9,
         * 0.0));
         * shooterTable.add(new ShooterTableEntry(feetToMeters(10.0), 13.1 + 0.25, 1.0,
         * 0.0));
         * shooterTable.add(new ShooterTableEntry(feetToMeters(11.5), 10.2 + 0.25, 1.0,
         * 0.0));
         * shooterTable.add(new ShooterTableEntry(feetToMeters(13.0), 7.7 + 0.75, 1.0,
         * 0.0));
         * shooterTable.add(new ShooterTableEntry(feetToMeters(14.5), 6.5 + 0.75, 1.0,
         * 0.0));
         * shooterTable.add(new ShooterTableEntry(feetToMeters(16.0), 5.1 + 0.75, 1.0,
         * 0.0));
         * shooterTable.add(new ShooterTableEntry(feetToMeters(17.5), 4.9 + 0.75, 1.0,
         * 0.0));
         * shooterTable.add(new ShooterTableEntry(feetToMeters(19.0), 4.4 + 0.75, 1.0,
         * 0.0));
         * shooterTable.add(new ShooterTableEntry(feetToMeters(20.5), 3.58 + 0.25, 1.0,
         * 0.0));
         * shooterTable.add(new ShooterTableEntry(feetToMeters(22.0), 3.1 + 0.25, 1.0,
         * 0.0));
         * shooterTable.add(new ShooterTableEntry(feetToMeters(24.0), 2.6 + 0.25, 1.0,
         * 0.0));
         * shooterTable.add(new ShooterTableEntry(feetToMeters(27.0), 0.25 + 0.25, 0.98,
         * 0.0));
         */
    }

    static {
        fillInTable();
    }


    public static ShooterTableEntry calcShooterTableEntry(double distanceMeters) {
        SmartDashboard.putNumber("Index_m", distanceMeters);
        ShooterTableEntry closestLower = shooterTable.get(0);
        ShooterTableEntry closestHigher = shooterTable.get(shooterTable.size() - 1);

        if (distanceMeters <= closestLower.distanceMeters)
            return closestLower;
        if (distanceMeters >= closestHigher.distanceMeters)
            return closestHigher;

        for (ShooterTableEntry entry : shooterTable) {
            if (entry.distanceMeters < distanceMeters
                    && (Math.abs(distanceMeters - closestLower.distanceMeters) > Math
                            .abs(distanceMeters - entry.distanceMeters))) {
                closestLower = entry;
            } else if (entry.distanceMeters > distanceMeters
                    && (Math.abs(closestHigher.distanceMeters - distanceMeters) > Math
                            .abs(entry.distanceMeters - distanceMeters))) {
                closestHigher = entry;
            } else if (Math.abs(entry.distanceMeters - distanceMeters) < 1e-9) {
                return entry;
            }
        }

        double scaleFactor = (distanceMeters - closestLower.distanceMeters)
                / (closestHigher.distanceMeters - closestLower.distanceMeters);

        double calculatedBeans = scaleFactor * (closestHigher.beans - closestLower.beans) + closestLower.beans;
        double calculatedAngle = scaleFactor * (closestHigher.angleDegrees - closestLower.angleDegrees)
                + closestLower.angleDegrees;
        double calculatedHeckyOffset = closestLower.heckyOffsetMeters
                + (closestHigher.heckyOffsetMeters - closestLower.heckyOffsetMeters) * scaleFactor;

        return new ShooterTableEntry(distanceMeters, calculatedAngle, calculatedBeans, calculatedHeckyOffset);
    }

    public static void setHeckinessLevel(Supplier<Rotation2d> heckSource) {
        heckinessSupplier = heckSource;
    }

    // API for adding/modifying entries programmatically
    public static void addShooterEntry(ShooterTableEntry e) {
        shooterTable.add(e);
        shooterTable.sort((a, b) -> Double.compare(a.distanceMeters, b.distanceMeters));
    }
}
