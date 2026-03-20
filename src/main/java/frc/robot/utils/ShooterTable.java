package frc.robot.utils;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.ArrayList;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class ShooterTable {
    public static final class ShooterTableEntry {
        public Distance distance;
        public AngularVelocity wheelSpeed;

        public ShooterTableEntry(Distance distance, AngularVelocity wheelSpeed) {
            this.distance = distance;
            this.wheelSpeed = wheelSpeed;
        }

        public ShooterTableEntry average(ShooterTableEntry other) {
            return new ShooterTableEntry(distance.plus(other.distance).div(2.0),
                    wheelSpeed.plus(other.wheelSpeed).div(2.0));
        }
    }

    private static ArrayList<ShooterTableEntry> shooterTable = new ArrayList<>();
    private static ArrayList<ShooterTableEntry> shuttleTable = new ArrayList<>();

    private static void addEntryShooter(Distance distance, AngularVelocity speed) {
        shooterTable.add(new ShooterTableEntry(distance, speed));
    }

    private static void addEntryShuttle(Distance distance, AngularVelocity speed) {
        shuttleTable.add(new ShooterTableEntry(distance, speed));
    }

    private static void fillInTable() {
        // Shooter
        addEntryShooter(Feet.of(7), RPM.of(2700));
        addEntryShooter(Feet.of(8), RPM.of(2760));
        addEntryShooter(Feet.of(9), RPM.of(2860));
        addEntryShooter(Feet.of(10), RPM.of(2985));
        addEntryShooter(Feet.of(11), RPM.of(3050));
        addEntryShooter(Feet.of(12), RPM.of(3150));
        addEntryShooter(Feet.of(13), RPM.of(3250));
        addEntryShooter(Feet.of(14), RPM.of(3350));

        // Shuttle Table
        addEntryShuttle(Feet.of(10), RPM.of(2000));
        addEntryShuttle(Feet.of(20), RPM.of(3000));
        addEntryShuttle(Feet.of(30), RPM.of(4000));
        addEntryShuttle(Feet.of(40), RPM.of(5000));
        addEntryShuttle(Feet.of(50), RPM.of(6000));
    }

    static {
        fillInTable();
    }

    private static ShooterTableEntry calculateEntry(Distance distance, ArrayList<ShooterTableEntry> table) {
        ShooterTableEntry closestLower = table.get(0);
        ShooterTableEntry closestHigher = table.get(table.size() - 1);

        if (distance.lte(closestLower.distance))
            return closestLower;

        if (distance.gte(closestHigher.distance))
            return closestHigher;

        for (ShooterTableEntry entry : table) {
            if (entry.distance.lt(distance) &&
                    distance.minus(closestLower.distance).abs(Meters) > distance.minus(entry.distance).abs(Meters))
                closestLower = entry;
            else if (entry.distance.gt(distance) &&
                    distance.minus(closestHigher.distance).abs(Meters) > distance.minus(entry.distance).abs(Meters))
                closestHigher = entry;
            else if (entry.distance.minus(distance).abs(Meters) < 1e-9)
                return entry;
        }

        double scale = distance.minus(closestLower.distance).div(closestHigher.distance.minus(closestLower.distance))
                .magnitude();
        var angle = closestHigher.wheelSpeed.minus(closestLower.wheelSpeed).times(scale).plus(closestLower.wheelSpeed);

        return new ShooterTableEntry(distance, angle);
    }

    public static ShooterTableEntry calcShooterTableEntryShooter(Distance distance) {
        return calculateEntry(distance, shooterTable);
    }

    public static ShooterTableEntry calcShooterTableEntryShuttle(Distance distance) {
        return calculateEntry(distance, shuttleTable);
    }
}
