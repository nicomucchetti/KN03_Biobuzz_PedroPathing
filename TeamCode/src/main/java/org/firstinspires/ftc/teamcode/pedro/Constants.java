//Time to start making edits!
package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static SwerveConfig driveConfig = new SwerveConfig(
            c -> {
                c.zeroPowerBehavior.set(SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
                c.manualBrakeMode.set(true);
                c.voltageCompensation.set(false);
            }
    );
    public static double turnkP = 2;
    public static double turnkI = 0;
    public static double turnkD = 0.5;
    public static double turnkV = 0.00;
    public static CoaxialPodConfig backRight = new CoaxialPodConfig(
            c -> {
                c.name.set("backRight");
                c.motorName.set("backRightM");
                c.servoName.set("backRightS");
                c.servoEncoderName.set("backRightE");
                c.turnController.set(Controller.pid(turnkP, turnkI, turnkD)
                        .plus(Controller.proportionalFeedforward(turnkV)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
                c.analogMinVoltage.set(0.00);
                c.analogMaxVoltage.set(3.21);
                c.angleOffsetRad.set(3.169-(3.14159/2));
            }
    );

    public static CoaxialPodConfig frontLeft = new CoaxialPodConfig(
            c -> {
                c.name.set("frontLeft");
                c.motorName.set("frontLeftM");
                c.servoName.set("frontLeftS");
                c.servoEncoderName.set("frontLeftE");
                c.turnController.set(Controller.pid(turnkP, turnkI, turnkD)
                        .plus(Controller.proportionalFeedforward(turnkV)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
                c.analogMinVoltage.set(0.000);
                c.analogMaxVoltage.set(3.212);
                c.angleOffsetRad.set(3.077-(3.14159/2));
            }
    );

    public static CoaxialPodConfig frontRight = new CoaxialPodConfig(
            c -> {
                c.name.set("frontRight");
                c.motorName.set("frontRightM");
                c.servoName.set("frontRightS");
                c.servoEncoderName.set("frontRightE");
                c.turnController.set(Controller.pid(turnkP, turnkI, turnkD)
                        .plus(Controller.proportionalFeedforward(turnkV)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
                c.analogMinVoltage.set(0.000);
                c.analogMaxVoltage.set(3.196);
                c.angleOffsetRad.set(3.2635-(3.14159/2));
            }
    );

    public static CoaxialPodConfig backLeft = new CoaxialPodConfig(
            c -> {
                c.name.set("backLeft");
                c.motorName.set("backLeftM");
                c.servoName.set("backLeftS");
                c.servoEncoderName.set("backLeftE");
                c.turnController.set(Controller.pid(turnkP, turnkI, turnkD)
                        .plus(Controller.proportionalFeedforward(turnkV)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
                c.analogMinVoltage.set(0.000);
                c.analogMaxVoltage.set(3.21);
                c.angleOffsetRad.set(3.1514-(3.14159/2));
            }
    );
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }
}