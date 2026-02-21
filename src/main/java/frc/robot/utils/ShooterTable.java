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

    private static void addEntry(Distance distance, AngularVelocity speed) {
        shooterTable.add(new ShooterTableEntry(distance, speed));
    }

    private static void fillInTable() {
        addEntry(Feet.of(4), RPM.of(2950));
        addEntry(Feet.of(5), RPM.of(3000));
        addEntry(Feet.of(6), RPM.of(3110));
        addEntry(Feet.of(7), RPM.of(3250));
        addEntry(Feet.of(8), RPM.of(3360));
        addEntry(Feet.of(9), RPM.of(3450));
        addEntry(Feet.of(10), RPM.of(3610));
    }

    static {
        fillInTable();
    }

    public static ShooterTableEntry calcShooterTableEntry(Distance distance) {
        ShooterTableEntry closestLower = shooterTable.get(0);
        ShooterTableEntry closestHigher = shooterTable.get(shooterTable.size() - 1);

        if (distance.lte(closestLower.distance))
            return closestLower;

        if (distance.gte(closestHigher.distance))
            return closestHigher;

        for (ShooterTableEntry entry : shooterTable) {
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
}
