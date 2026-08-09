package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Reads live, uncached values straight from the physical driver controller. */
public class LiveDriverInputSource implements DriverInputSource {
    private final CommandXboxController controller;

    public LiveDriverInputSource(CommandXboxController controller) {
        this.controller = controller;
    }

    @Override
    public double getLeftY() {
        return controller.getLeftY();
    }

    @Override
    public double getLeftX() {
        return controller.getLeftX();
    }

    @Override
    public double getRightX() {
        return controller.getRawAxis(4);
    }

    @Override
    public double getRightTrigger() {
        return controller.getRightTriggerAxis();
    }

    @Override
    public boolean getXButton() {
        return controller.getHID().getXButton();
    }

    @Override
    public boolean getYButton() {
        return controller.getHID().getYButton();
    }

    @Override
    public boolean getStartButton() {
        return controller.getHID().getStartButton();
    }

    @Override
    public boolean getBackButton() {
        return controller.getHID().getBackButton();
    }

    @Override
    public boolean getLeftBumper() {
        return controller.getHID().getLeftBumperButton();
    }

    @Override
    public boolean getRightBumper() {
        return controller.getHID().getRightBumperButton();
    }

    @Override
    public boolean getPovUp() {
        return controller.getHID().getPOV() == 0;
    }

    @Override
    public boolean getPovDown() {
        return controller.getHID().getPOV() == 180;
    }
}
