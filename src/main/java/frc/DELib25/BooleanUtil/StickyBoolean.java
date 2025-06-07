package frc.DELib25.BooleanUtil;


// Read a provided input
// When the input becomes true, output true until manually cleared
// Useful for latching
public class StickyBoolean {

    private boolean mOn = false;

    public StickyBoolean() {
        super();
        mOn = false;
    }

    public boolean update(boolean useVision) {
        mOn |= useVision;
        return mOn;
    }

    public void reset() {
        mOn = false;
    }

    public boolean get() {
        return mOn;
    }
}
