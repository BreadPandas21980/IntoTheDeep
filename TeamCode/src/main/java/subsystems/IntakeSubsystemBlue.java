package subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.config.Config;
import java.util.ArrayList;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class IntakeSubsystemBlue extends SubsystemBase {
    public static boolean autoDisabled = false;

    ElapsedTime time = new ElapsedTime();
    public static long flipUpTime = 0;
    public static boolean flippyUp;
    private final CRServo intakeServo;
    private final Servo dropdownServo;
    private final Servo pitchServo;
    public static double timeytime = 1;

    public static double targetPower = 0;
    public static double IN_POWER = 1;
    public static double AUTO_IDLE_POWER = 0.8;
    public static double FULL_POWER = 1;
    public static double OUT_POWER = -1;
    public static double MINI_OUT_POWER = -0.1;
    public static double IN_POWER_SAMP = .8;
    public static double IN_POWER_SPEC = 0.75;
    public static double DROPDOWN_INTAKE = .7;
    public static double DROPDOWN_EJECT = 0.05;
    public static double ARM_INTAKE = .7;
    public static double ARM_STOW = 0.05;
    public static double ARM_EJECTION = 0.05;
    public static boolean colorSeen = false;


    public static boolean uhohTwo = false;
    ArrayList<Integer> data = new ArrayList<Integer>(); //1 is red, 2 is yellow, 3 is blue, -1 is nothing

    public IntakeSubsystemBlue(CRServo intakeServo, Servo pitchServo, Servo dropdownServo ) {
        this.intakeServo = intakeServo;
        this.dropdownServo = dropdownServo;
        this.pitchServo = pitchServo;
        data.add(-1);

    }


    public void setAutoDisabled(boolean disabled) {
        autoDisabled = disabled;
    }

    public Command intakeArmStow() {
        return new RunCommand(() -> {
            dropdownServo.setPosition(ARM_STOW);
        }, this);
    }
    public Command intakeArmIntake() {
        return new RunCommand(() -> {
            dropdownServo.setPosition(ARM_INTAKE);
        }, this);
    }
    public Command intakeArmEject() {
        return new RunCommand(() -> {
            dropdownServo.setPosition(ARM_EJECTION);
        }, this);
    }
    public Command inIntake() {
        return new InstantCommand(() -> {
            IN_POWER = FULL_POWER; /*
            if(flippyUp == false) {
                IN_POWER = 1;
            } else if(uhohTwo == true) {
                IN_POWER = -1;
            } else {
                IN_POWER = 1;
            }
            */
            if(ColorSubsystemBlue.pooping == true && ColorSubsystemBlue.grrr == true) {
                IN_POWER = 0;
            }
            intakeServo.setPower(IN_POWER);

        }, this);
    }
    public Command outIntake() {
        return new InstantCommand(() -> {
            intakeServo.setPower(OUT_POWER);
        }, this);
    }

    public Command outIntakeMini() {
        return new InstantCommand(() -> {
            intakeServo.setPower(MINI_OUT_POWER);
        }, this);
    }
    public Command idle() {
        return new InstantCommand(() -> {
            intakeServo.setPower(0);
            IN_POWER = 0;
        }, this);
    }

    public void autoIntake() {
        if(!autoDisabled) {

            intakeServo.setPower(FULL_POWER);
        }
    }
    public void autoIdle() {
        if(!autoDisabled) {

            intakeServo.setPower(AUTO_IDLE_POWER);
        }
    }
    public void autoDropdownIntake() {
        if(!autoDisabled) {

            pitchServo.setPosition(DROPDOWN_INTAKE);
        }
    }
    public void autoDropdownEject() {
        if(!autoDisabled) {

            pitchServo.setPosition(DROPDOWN_EJECT);
        }
    }
    public void autoIntakeArmIntake() {
        if(!autoDisabled) {

            dropdownServo.setPosition(ARM_INTAKE);
        }
    }
    public void autoIntakeArmStow() {
        if (!autoDisabled) {

            dropdownServo.setPosition(ARM_STOW);
        }
    }
    public Command runIdle() {
        return new RunCommand(() -> {
            intakeServo.setPower(0);
        }, this);
    }
    public Command dropdownIntake() {
        return new InstantCommand(() -> {
            flippyUp = false;
            pitchServo.setPosition(DROPDOWN_INTAKE);
        }, this);
    }
    public Command dropdownEject() {
        return new InstantCommand(() -> {
            flippyUp = true;
            pitchServo.setPosition(DROPDOWN_EJECT);
        }, this);
    }


    @Override
    public void periodic() {

    }


}