package frc.robot.input;

/**
 * A DriverInputSource that forwards to a swappable delegate. RobotContainer's axis suppliers and
 * triggers are all built against one instance of this so they can be redirected from the live
 * controller to a recorded playback (and back) without rebuilding any bindings.
 */
public class SwitchableDriverInputSource implements DriverInputSource {
    private DriverInputSource delegate;

    public SwitchableDriverInputSource(DriverInputSource initialDelegate) {
        this.delegate = initialDelegate;
    }

    public void setDelegate(DriverInputSource delegate) {
        this.delegate = delegate;
    }

    @Override
    public double getLeftY() {
        return delegate.getLeftY();
    }

    @Override
    public double getLeftX() {
        return delegate.getLeftX();
    }

    @Override
    public double getRightX() {
        return delegate.getRightX();
    }

    @Override
    public double getRightTrigger() {
        return delegate.getRightTrigger();
    }

    @Override
    public boolean getXButton() {
        return delegate.getXButton();
    }

    @Override
    public boolean getYButton() {
        return delegate.getYButton();
    }

    @Override
    public boolean getStartButton() {
        return delegate.getStartButton();
    }

    @Override
    public boolean getBackButton() {
        return delegate.getBackButton();
    }

    @Override
    public boolean getLeftBumper() {
        return delegate.getLeftBumper();
    }

    @Override
    public boolean getRightBumper() {
        return delegate.getRightBumper();
    }

    @Override
    public boolean getPovUp() {
        return delegate.getPovUp();
    }

    @Override
    public boolean getPovDown() {
        return delegate.getPovDown();
    }
}
