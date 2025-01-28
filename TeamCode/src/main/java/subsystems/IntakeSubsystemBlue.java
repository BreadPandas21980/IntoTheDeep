package subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
@Config
public class IntakeSubsystemBlue extends SubsystemBase {

    ElapsedTime time = new ElapsedTime();
    public static long flipUpTime = 0;
    public static boolean flippyUp;
    private final MotorEx intakeMotor;
    private final Servo dropdownServo;
    public static double timeytime = 1;

    public static double targetPower = 0;
    public static double IN_POWER = 1;
    public static double FULL_POWER = 1;
    public static double OUT_POWER = -1;
    public static double MINI_OUT_POWER = -0.1;
    public static double IN_POWER_SAMP = .8;
    public static double IN_POWER_SPEC = 0.75;
    public static double DROPDOWN_DOWN = .7;
    public static double DROPDOWN_UP = 0.05;
    public static boolean colorSeen = false;


    public static boolean uhohTwo = false;
    ArrayList<Integer> data = new ArrayList<Integer>(); //1 is red, 2 is yellow, 3 is blue, -1 is nothing

    public IntakeSubsystemBlue(MotorEx intakeMotor, Servo dropdownServo ) {
        this.intakeMotor = intakeMotor;
        this.dropdownServo = dropdownServo;
        data.add(-1);

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
            intakeMotor.set(IN_POWER);

        }, this);
    }
    public Command outIntake() {
        return new InstantCommand(() -> {
            intakeMotor.set(OUT_POWER);
        }, this);
    }

    public Command outIntakeMini() {
        return new InstantCommand(() -> {
            intakeMotor.set(MINI_OUT_POWER);
        }, this);
    }
    public Command idle() {
        return new InstantCommand(() -> {
            intakeMotor.set(0);
            IN_POWER = 0;
        }, this);
    }

    public void autoIntake() {
        intakeMotor.set(FULL_POWER);
    }
    public void autoIdle() {
        intakeMotor.set(0);
    }
    public void autoFlipDown1() {
        dropdownServo.setPosition(DROPDOWN_DOWN);
    }
    public void autoFlipUp1() {
        dropdownServo.setPosition(DROPDOWN_UP);
    }
    public Command runIdle() {
        return new RunCommand(() -> {
            intakeMotor.set(0);
        }, this);
    }
    public Command flipDown() {
        return new InstantCommand(() -> {
            flippyUp = false;
            dropdownServo.setPosition(DROPDOWN_DOWN);
        }, this);
    }
    public Command flipUp() {
        return new InstantCommand(() -> {
            flippyUp = true;
            dropdownServo.setPosition(DROPDOWN_UP);
        }, this);
    }

    public Action autoInSamp() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                intakeMotor.set(IN_POWER_SAMP);
                return false;
            }
        };
    }

    public Action autoInSpec() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                intakeMotor.set(IN_POWER_SPEC);
                return false;
            }
        };
    }
    public Action autoOutIntake() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                intakeMotor.set(OUT_POWER);
                return false;
            }
        };
    }
    public Action autoIdleIntake() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                intakeMotor.set(0);
                return false;
            }
        };
    }
    public Action autoFlipDown() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                dropdownServo.setPosition(DROPDOWN_DOWN);
                return false;
            }
        };
    }
    public Action autoFlipUp() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                timer.reset();
                dropdownServo.setPosition(DROPDOWN_UP);
                return false;
            }
        };
    }

    @Override
    public void periodic() {



        if (ColorSubsystemBlue.smthIn &&  flippyUp == true) {
            time.reset();
            uhohTwo = true;
         //   intakeMotor.set(-1);
        } else {

            if(time.seconds() > timeytime) {
                uhohTwo = false;
            }
        }

        if(ColorSubsystemBlue.grrr) {
            flippyUp = true;
            time.reset();
            dropdownServo.setPosition(DROPDOWN_UP);
            intakeMotor.set(MINI_OUT_POWER);
            if(time.seconds() > timeytime) {
                intakeMotor.set(0);
            }

        }


    }


}