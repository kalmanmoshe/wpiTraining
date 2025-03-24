package frc.Motors;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.util.TimeOut;

public class TalonFXMotorCreator {
    public static TalonFX createTalonFX(int motorID) {
        return new TalonFX(motorID);
    }
     public static TalonFX createSlaveTalonint(int motorId,int masterId, boolean opposeMasterDirection){
        TalonFX talon = createTalonFX(motorId);
        TimeOut.checkErrorAndRetry(() -> talon.setControl(new Follower(masterId, opposeMasterDirection)));
        return talon;
    }
}
