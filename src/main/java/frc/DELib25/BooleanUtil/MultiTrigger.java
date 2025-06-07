package frc.DELib25.BooleanUtil;

/**
 * Class to differentiate between tapping and holding a joystick button/trigger
 */
public class MultiTrigger {
    private final double mTimeout;
    private boolean lastPressed = false;
    private final IterativeLatchedBoolean wasTapped = new IterativeLatchedBoolean();
    private final IterativeLatchedBoolean wasHeld = new IterativeLatchedBoolean();
    private boolean lastTapped = false;
    private final TimeDelayedBoolean isHeld = new TimeDelayedBoolean();

    public MultiTrigger(double timeout) {
        mTimeout = timeout;
    }

    public void update(boolean pressed) {
        lastPressed = pressed;
        lastTapped = wasTapped.update(pressed);
        isHeld.update(pressed, mTimeout);
    }

    public boolean wasTapped() {
        return lastTapped;
    }

    public boolean isPressed() {
        return lastPressed;
    }

    public boolean isHeld() {
        return isHeld.update(lastPressed, mTimeout);
    }

    public boolean holdStarted() {
        return wasHeld.update(isHeld());
    }
}
