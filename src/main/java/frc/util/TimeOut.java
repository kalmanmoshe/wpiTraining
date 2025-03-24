package frc.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;
import com.ctre.phoenix6.StatusCode;

public class TimeOut {
    /**
     *  Shorthand for {@link #checkErrorAndRetry(Supplier, int, int)}.
     */
    public static boolean checkErrorAndRetry(Supplier<StatusCode> runnable) {
        return checkErrorAndRetry(runnable, 3, 200);
    }
    /**
     * Tries to run a function multiple times before giving up.
     * Adds a timeout between retries to allow for recovery.
     *
     * @param function 
     * @param numTries
     * @param timeout Time in milliseconds to wait between attempts
     * @return true if successfull
     */
    public static boolean checkErrorAndRetry(Supplier<StatusCode> function, int numTries, int timeout) {
        int tries = 0;
        StatusCode code = function.get(); // First attempt

        while (code != StatusCode.OK && tries < numTries) {
            DriverStation.reportWarning("Retrying CTRE Device Config: " + code.getName(), false);
            tries++;

            try {
                Thread.sleep(timeout); // Wait before trying again
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // Reset interrupt flag as best practice
            }

            code = function.get(); // Try again
        }

        if (code != StatusCode.OK) {
            DriverStation.reportError("Failed to execute Phoenix Pro API call after " + numTries + " attempts", false);
            return false;
        }

        return true;
    }
}