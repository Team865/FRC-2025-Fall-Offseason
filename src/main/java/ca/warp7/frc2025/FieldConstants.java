// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package ca.warp7.frc2025;

import ca.warp7.frc2025.subsystems.superstructure.Superstructure;
import ca.warp7.frc2025.subsystems.superstructure.Superstructure.AlgaeLevel;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.*;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
    public static final FieldType fieldType = FieldType.WELDED;

    public static final double fieldLength =
            AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
    public static final double fieldWidth =
            AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();
    public static final double startingLineX =
            Units.inchesToMeters(299.438); // Measured from the inside of starting line
    public static final double algaeDiameter = Units.inchesToMeters(16);

    public static class Processor {
        public static final Pose2d centerFace = new Pose2d(
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(16).get().getX(), 0, Rotation2d.fromDegrees(90));
    }

    public static class Barge {
        public static final double netWidth = Units.inchesToMeters(40.0);
        public static final double netHeight = Units.inchesToMeters(88.0);

        public static final Translation2d farCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
        public static final Translation2d middleCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
        public static final Translation2d closeCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

        // Measured from floor to bottom of cage
        public static final double deepHeight = Units.inchesToMeters(3.125);
        public static final double shallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {
        public static final double stationLength = Units.inchesToMeters(79.750);
        public static final Pose2d rightCenterFace = new Pose2d(
                Units.inchesToMeters(33.526), Units.inchesToMeters(25.824), Rotation2d.fromDegrees(144.011 - 90));
        public static final Pose2d leftCenterFace = new Pose2d(
                rightCenterFace.getX(),
                fieldWidth - rightCenterFace.getY(),
                Rotation2d.fromRadians(-rightCenterFace.getRotation().getRadians()));

        public static final Pose2d rightCenterFaceRed = new Pose2d(
                fieldLength - rightCenterFace.getX(),
                rightCenterFace.getY(),
                Rotation2d.k180deg.minus(rightCenterFace.getRotation()));

        public static final Pose2d leftCenterFaceRed = new Pose2d(
                rightCenterFaceRed.getX(),
                leftCenterFace.getY(),
                Rotation2d.k180deg.minus(leftCenterFace.getRotation()));
    }

    public static class Reef {
        public static final double faceLength = Units.inchesToMeters(36.792600);
        public static final Translation2d center = new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0);
        public static final double faceToZoneLine =
                Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

        public static final Pose2d[] centerFaces =
                new Pose2d[12]; // Starting facing the driver station in clockwise order

        public static final Map<Pose2d, AlgaeLevel> algaeLevels = new HashMap<Pose2d, Superstructure.AlgaeLevel>();

        static {
            // Initialize faces
            var aprilTagLayout = AprilTagLayoutType.OFFICIAL.getLayout();
            centerFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
            centerFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
            centerFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
            centerFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
            centerFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
            centerFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();

            centerFaces[6] = aprilTagLayout.getTagPose(7).get().toPose2d();
            centerFaces[7] = aprilTagLayout.getTagPose(6).get().toPose2d();
            centerFaces[8] = aprilTagLayout.getTagPose(11).get().toPose2d();
            centerFaces[9] = aprilTagLayout.getTagPose(10).get().toPose2d();
            centerFaces[10] = aprilTagLayout.getTagPose(9).get().toPose2d();
            centerFaces[11] = aprilTagLayout.getTagPose(8).get().toPose2d();
        }

        static {
            algaeLevels.put(centerFaces[0], AlgaeLevel.HIGH);
            algaeLevels.put(centerFaces[1], AlgaeLevel.LOW);
            algaeLevels.put(centerFaces[2], AlgaeLevel.HIGH);
            algaeLevels.put(centerFaces[3], AlgaeLevel.LOW);
            algaeLevels.put(centerFaces[4], AlgaeLevel.HIGH);
            algaeLevels.put(centerFaces[5], AlgaeLevel.LOW);
            algaeLevels.put(centerFaces[6], AlgaeLevel.HIGH);
            algaeLevels.put(centerFaces[7], AlgaeLevel.LOW);
            algaeLevels.put(centerFaces[8], AlgaeLevel.HIGH);
            algaeLevels.put(centerFaces[9], AlgaeLevel.LOW);
            algaeLevels.put(centerFaces[10], AlgaeLevel.HIGH);
            algaeLevels.put(centerFaces[11], AlgaeLevel.LOW);
        }
    }

    public static class StagingPositions {
        // Measured from the center of the ice cream
        public static final double separation = Units.inchesToMeters(72.0);
        public static final Pose2d middleIceCream =
                new Pose2d(Units.inchesToMeters(48), fieldWidth / 2.0, new Rotation2d());
        public static final Pose2d leftIceCream =
                new Pose2d(Units.inchesToMeters(48), middleIceCream.getY() + separation, new Rotation2d());
        public static final Pose2d rightIceCream =
                new Pose2d(Units.inchesToMeters(48), middleIceCream.getY() - separation, new Rotation2d());
    }

    public enum ReefLevel {
        L1(Units.inchesToMeters(25.0), 0),
        L2(Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
        L3(Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
        L4(Units.inchesToMeters(72), -90);

        ReefLevel(double height, double pitch) {
            this.height = height;
            this.pitch = pitch; // Degrees
        }

        public static ReefLevel fromLevel(int level) {
            return Arrays.stream(values())
                    .filter(height -> height.ordinal() == level)
                    .findFirst()
                    .orElse(L4);
        }

        public final double height;
        public final double pitch;
    }

    public static final double aprilTagWidth = Units.inchesToMeters(6.50);
    public static final int aprilTagCount = 22;
    public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.NO_BARGE;

    @Getter
    public enum AprilTagLayoutType {
        OFFICIAL("2025-official"),
        NO_BARGE("2025-no-barge"),
        BLUE_REEF("2025-blue-reef"),
        RED_REEF("2025-red-reef"),
        FIELD_BORDER("2025-field-border");

        AprilTagLayoutType(String name) {
            if (Constants.disableHAL) {
                try {
                    layout = new AprilTagFieldLayout(Path.of(
                            "src", "main", "deploy", "apriltags", fieldType.getJsonFolder(), "2025-official.json"));
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            } else {
                try {
                    layout = new AprilTagFieldLayout(Path.of(
                            Filesystem.getDeployDirectory().getPath(),
                            "apriltags",
                            fieldType.getJsonFolder(),
                            name + ".json"));
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }

            try {
                layoutString = new ObjectMapper().writeValueAsString(layout);
            } catch (JsonProcessingException e) {
                throw new RuntimeException("Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
            }
        }

        private final AprilTagFieldLayout layout;
        private final String layoutString;
    }

    public record CoralObjective(int branchId, ReefLevel reefLevel) {}

    public record AlgaeObjective(int id) {}

    @RequiredArgsConstructor
    public enum FieldType {
        ANDYMARK("andymark"),
        WELDED("welded");

        @Getter
        private final String jsonFolder;
    }
}
