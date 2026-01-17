package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GameData {
    enum Hub {
        RED_HUB,
        BLUE_HUB,
        ALL,
        NO_DATA
    }

    private static Optional<Alliance> m_clientTeam;
    private Alliance m_autoWinner = null;
    private boolean m_isDataCaptured = false;

    private static final Alliance RED = DriverStation.Alliance.Red;
    private static final Alliance BLUE = DriverStation.Alliance.Blue;

    private final double END_GAME_START_TIME = 30.0; // Seconds remaining
    private final double SHIFT_DURATION = 25.0;

    public GameData(Optional<Alliance> team) {
        m_clientTeam = team;
        init();
    }

    private void init() {
        m_autoWinner = null;
        m_isDataCaptured = false;
    }

    private void update() {
        if (m_isDataCaptured)
            return;

        String data = DriverStation.getGameSpecificMessage();
        if (data != null && !data.isEmpty()) {
            char winnerChar = data.toUpperCase().charAt(0);
            if (winnerChar == 'R') {
                m_autoWinner = Alliance.Red;
                m_isDataCaptured = true;
            } else if (winnerChar == 'B') {
                m_autoWinner = Alliance.Blue;
                m_isDataCaptured = true;
            }
        }
    }

    public Hub getActiveHub() {
        update();

        double matchTime = DriverStation.getMatchTime();

        if (DriverStation.isAutonomous() || matchTime <= END_GAME_START_TIME) {
            return Hub.ALL;
        }

        if (!m_isDataCaptured) {
            return Hub.ALL;
        }

        double teleopTime = 135.0 - matchTime; // Time elapsed in Teleop
        int shift = (int) (teleopTime / SHIFT_DURATION) + 1;

        boolean isAutoWinnerActive;
        // Even shifts (2, 4): Auto winner is active.
        // Odd shifts (1, 3): Auto winner is inactive.
        isAutoWinnerActive = (shift % 2 == 0);

        if (m_autoWinner == Alliance.Red) {
            return isAutoWinnerActive ? Hub.RED_HUB : Hub.BLUE_HUB;
        } else {
            return isAutoWinnerActive ? Hub.RED_HUB : Hub.BLUE_HUB;
        }
    }

    public boolean isMyHubActive() {
        if (m_clientTeam.isEmpty())
            return false;

        Hub status = getActiveHub();
        if (status == Hub.ALL)
            return true;

        boolean isRed = m_clientTeam.get() == Alliance.Red;
        return (isRed && status == Hub.RED_HUB) || (!isRed && status == Hub.BLUE_HUB);
    }

    public int shouldInvertControls() {
        if (!m_clientTeam.isPresent())
            return 1;

        return m_clientTeam.get() == RED ? -1 : 1;
    }

    public boolean getIsRed() {
        if (!m_clientTeam.isPresent())
            return false;

        return m_clientTeam.get() == RED;
    }

}
