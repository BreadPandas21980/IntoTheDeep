package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = .001;
        TwoWheelConstants.strafeTicksToInches = 0.0011;
        TwoWheelConstants.forwardY = -6;
        TwoWheelConstants.strafeX = -7;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "intakeMotor";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "rightSlide";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);



    }
}




