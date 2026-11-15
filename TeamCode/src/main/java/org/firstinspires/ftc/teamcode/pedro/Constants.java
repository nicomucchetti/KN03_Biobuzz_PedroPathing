//Time to start making edits!
package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Vector2D;
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
    // Pod error is in radians. turnkV is a static push toward the target, applied only when
    // the pod is more than 2 deg off (CoaxialPod passes turnDirection or 0 as its input).
    public static double turnkP = 0.4;
    public static double turnkI = 0;
    public static double turnkD = 0.016;
    public static double turnkV = 0.07;
    // Extra static push per pod, added to turnkV (for pods with a bit more friction)
    public static double kVExtraFrontLeft = 0;
    public static double kVExtraFrontRight = 0;
    public static double kVExtraBackLeft = 0;
    public static double kVExtraBackRight = 0;
    // Pod centres from robot centre. Pedro's Swerve uses +x forward, +y RIGHT here
    // (it builds the turn vector from atan2(-x, -y)). Only the ratio matters.
    // Measured pod-centre distances in mm: 204.188 front-back, 189.257 left-right.
    public static double podHalfWheelBase = 204.188 / 2.0;   // front-back half distance
    public static double podHalfTrackWidth = 189.257 / 2.0;  // left-right half distance
    // Pedro's default is 0.05: servo power changes smaller than that are never sent,
    // which swallows most of the fine correction near the target.
    public static double servoCachingThreshold = 0.005;
    public static CoaxialPodConfig backRight = new CoaxialPodConfig(
            c -> {
                c.name.set("backRight");
                c.motorName.set("backRightM");
                c.servoName.set("backRightS");
                c.servoEncoderName.set("backRightE");
                c.turnController.set(Controller.pid(turnkP, turnkI, turnkD)
                        .plus(Controller.proportionalFeedforward(turnkV + kVExtraBackRight)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
                c.encoderReversed.set(true); // pods pointed right at a +45 deg (left) target
                c.servoCachingThreshold.set(servoCachingThreshold);
                c.analogMinVoltage.set(0.068);
                c.analogMaxVoltage.set(3.247);
                c.angleOffsetRad.set(3.0843);
                c.podOffset.set(Vector2D.cartesian(-podHalfWheelBase, podHalfTrackWidth));
            }
    );

    public static CoaxialPodConfig frontLeft = new CoaxialPodConfig(
            c -> {
                c.name.set("frontLeft");
                c.motorName.set("frontLeftM");
                c.servoName.set("frontLeftS");
                c.servoEncoderName.set("frontLeftE");
                c.turnController.set(Controller.pid(turnkP, turnkI, turnkD)
                        .plus(Controller.proportionalFeedforward(turnkV + kVExtraFrontLeft)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
                c.encoderReversed.set(true); // pods pointed right at a +45 deg (left) target
                c.servoCachingThreshold.set(servoCachingThreshold);
                c.analogMinVoltage.set(0.059);
                c.analogMaxVoltage.set(3.241);
                c.angleOffsetRad.set(0.0178);
                c.podOffset.set(Vector2D.cartesian(podHalfWheelBase, -podHalfTrackWidth));
            }
    );

    public static CoaxialPodConfig frontRight = new CoaxialPodConfig(
            c -> {
                c.name.set("frontRight");
                c.motorName.set("frontRightM");
                c.servoName.set("frontRightS");
                c.servoEncoderName.set("frontRightE");
                c.turnController.set(Controller.pid(turnkP, turnkI, turnkD)
                        .plus(Controller.proportionalFeedforward(turnkV + kVExtraFrontRight)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
                c.encoderReversed.set(true); // pods pointed right at a +45 deg (left) target
                c.servoCachingThreshold.set(servoCachingThreshold);
                c.analogMinVoltage.set(0.063);
                c.analogMaxVoltage.set(3.249);
                c.angleOffsetRad.set(3.5084);
                c.podOffset.set(Vector2D.cartesian(podHalfWheelBase, podHalfTrackWidth));
            }
    );

    public static CoaxialPodConfig backLeft = new CoaxialPodConfig(
            c -> {
                c.name.set("backLeft");
                c.motorName.set("backLeftM");
                c.servoName.set("backLeftS");
                c.servoEncoderName.set("backLeftE");
                c.turnController.set(Controller.pid(turnkP, turnkI, turnkD)
                        .plus(Controller.proportionalFeedforward(turnkV + kVExtraBackLeft)));
                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.FORWARD);
                c.encoderReversed.set(true); // pods pointed right at a +45 deg (left) target
                c.servoCachingThreshold.set(servoCachingThreshold);
                c.analogMinVoltage.set(0.063);
                c.analogMaxVoltage.set(3.236);
                c.angleOffsetRad.set(6.1971);
                c.podOffset.set(Vector2D.cartesian(-podHalfWheelBase, -podHalfTrackWidth));
            }
    );
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }
}