package com.marswars.proxy_server;

import static org.junit.jupiter.api.Assertions.*;

import com.marswars.proxy_server.proto.EnvelopeProto;
import com.marswars.proxy_server.proto.MatchInfoProto;
import com.marswars.proxy_server.proto.ModuleStatesProto;
import com.marswars.proxy_server.proto.OdometryProto;
import com.marswars.proxy_server.proto.PieceDetectionsProto;
import com.marswars.proxy_server.proto.TagSolutionProto;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Checks the golden envelopes in src/test/resources/chassis_proxy against messages built here. Each
 * .binpb was encoded by protoc (the C++ protobuf library the coprocessor's client uses) from the .txtpb
 * next to it with
 * {@code protoc -Isrc/main/proto --encode=chassis_proxy.v1.Envelope chassis_proxy/v1/envelope.proto},
 * and Aquila-Vision's client checks the same files.
 *
 * <p>QuickBuffers orders fields differently from the C++ library, which protobuf allows, so the bytes
 * each side writes for the same message differ. Each frame therefore also has a .quickbuf.binpb copy
 * holding what this library writes for it: the tests here pin our encoder to it, and Aquila-Vision's
 * client checks that it decodes it. After a change to the .proto or the QuickBuffers version,
 * regenerate those copies by parsing each .binpb and writing {@code toByteArray()}. Expected messages
 * leave fields at their default (0, false, "") unset, because QuickBuffers' equals() tells an
 * explicitly set default apart from an absent field even though both encode the same.
 */
public class ChassisProxyFixturesTest {

    private static final int VERSION = EnvelopeProto.ProtocolVersion.PROTOCOL_VERSION_CURRENT.getNumber();

    private static byte[] fixture(String name) throws IOException {
        try (InputStream stream = ChassisProxyFixturesTest.class.getResourceAsStream("/chassis_proxy/" + name + ".binpb")) {
            assertNotNull(stream, "missing fixture " + name);
            return stream.readAllBytes();
        }
    }

    private static EnvelopeProto.Envelope envelope() {
        return EnvelopeProto.Envelope.newInstance().setProtocolVersion(VERSION);
    }

    /**
     * Asserts the protoc-encoded fixture decodes to the expected envelope, and that the expected envelope
     * encodes to the fixture's QuickBuffers copy.
     */
    private static EnvelopeProto.Envelope assertMatchesFixture(String name, EnvelopeProto.Envelope expected)
            throws IOException {
        EnvelopeProto.Envelope decoded = EnvelopeProto.Envelope.parseFrom(fixture(name));
        assertEquals(expected, decoded);
        assertArrayEquals(fixture(name + ".quickbuf"), expected.toByteArray());
        return decoded;
    }

    @Test
    public void timesyncRequest() throws IOException {
        EnvelopeProto.Envelope expected = envelope();
        expected.getMutableTimesyncRequest().setReqId(7)
                .getMutableClientSend().setSec(1700000000).setNanosec(123456789);
        EnvelopeProto.Envelope decoded = assertMatchesFixture("timesync_request", expected);

        TimesyncRequest.TimesyncRequestData data = TimesyncRequest.fromProto(decoded.getTimesyncRequest());
        assertEquals(7, data.requestId);
        assertEquals(1700000000, data.clientSendSeconds);
        assertEquals(123456789, data.clientSendNanoseconds);
    }

    @Test
    public void timesyncResponse() throws IOException {
        byte[] bytes = TimesyncResponse.serialize(
                new TimesyncResponse.TimesyncResponseData(7, 512, 250000000, 1700000000, 123456789));
        assertArrayEquals(fixture("timesync_response.quickbuf"), bytes);

        TimesyncResponse.TimesyncResponseData data = TimesyncResponse.deserialize(fixture("timesync_response"));
        assertEquals(7, data.requestId);
        assertEquals(512, data.serverReceiveSeconds);
        assertEquals(250000000, data.serverReceiveNanoseconds);
        assertEquals(1700000000, data.clientSendSeconds);
        assertEquals(123456789, data.clientSendNanoseconds);
    }

    @Test
    public void tagSolution() throws IOException {
        EnvelopeProto.Envelope expected = envelope();
        TagSolutionProto.TagSolution msg = expected.getMutableTagSolution().setCameraSerial("40123456");
        msg.getMutableStamp().setSec(512).setNanosec(500000000);
        msg.getMutableRobotPose().setXM(1.25).setYM(-3.5).setThetaRad(0.75);
        msg.addAllTagIds(3, 7, 22);
        EnvelopeProto.Envelope decoded = assertMatchesFixture("tag_solution", expected);

        TagSolutionPacket.TagSolutionData data = TagSolutionPacket.fromProto(decoded.getTagSolution());
        assertEquals(1.25, data.pose.getX());
        assertEquals(-3.5, data.pose.getY());
        assertEquals(0.75, data.pose.getRotation().getRadians(), 1e-12);
        assertEquals(List.of(3, 7, 22), data.detectedIds);
        assertEquals(new Packet.Timestamp(512, 500000000), data.timestamp);
        assertEquals("40123456", data.cameraSerial);
    }

    @Test
    public void pieceDetections() throws IOException {
        EnvelopeProto.Envelope expected = envelope();
        PieceDetectionsProto.PieceDetections msg = expected.getMutablePieceDetections().setCameraSerial("40123456");
        msg.getMutableStamp().setSec(512).setNanosec(750000000);
        msg.addDetections(PieceDetectionsProto.Detection.newInstance().setClassId(1).setThetaXRad(0.125).setThetaYRad(-0.0625));
        msg.addDetections(PieceDetectionsProto.Detection.newInstance().setClassId(2).setThetaXRad(-0.5).setThetaYRad(0.25));
        EnvelopeProto.Envelope decoded = assertMatchesFixture("piece_detections", expected);

        PieceDetectionPacket.PieceDetectionData data = PieceDetectionPacket.fromProto(decoded.getPieceDetections());
        assertEquals(2, data.detectionCount);
        assertEquals(0, data.detectionIndex);
        assertEquals(1, data.classId);
        assertEquals(0.125, data.thetaX);
        assertEquals(-0.0625, data.thetaY);
        assertEquals(2, data.allDetections.size());
        assertEquals(1, data.allDetections.get(1).detectionIndex);
        assertEquals(2, data.allDetections.get(1).classId);
        assertEquals("40123456", data.cameraSerial);
    }

    @Test
    public void emptyPieceDetections() {
        PieceDetectionsProto.PieceDetections msg = PieceDetectionsProto.PieceDetections.newInstance().setCameraSerial("40123456");
        PieceDetectionPacket.PieceDetectionData data = PieceDetectionPacket.fromProto(msg);
        assertEquals(0, data.detectionCount);
        assertTrue(data.allDetections.isEmpty());
    }

    @Test
    public void odometry() throws IOException {
        EnvelopeProto.Envelope expected = envelope();
        OdometryProto.Odometry msg = expected.getMutableOdometry()
                .setVxMps(1.5).setVyMps(-0.25).setOmegaRadps(0.5)
                .setXVariance(0.01).setYVariance(0.02).setThetaVariance(0.005);
        msg.getMutableStamp().setSec(513);
        msg.getMutablePose().setXM(2.5).setYM(4.0).setThetaRad(-1.5);
        EnvelopeProto.Envelope decoded = assertMatchesFixture("odometry", expected);

        OdomPacket.OdometryData data = OdomPacket.fromProto(decoded.getOdometry());
        assertEquals(2.5, data.pose.getX());
        assertEquals(1.5, data.twist.dx);
        assertArrayEquals(new double[] {0.01, 0.02, 0.005}, data.variances);
    }

    @Test
    public void moduleStates() throws IOException {
        EnvelopeProto.Envelope expected = envelope();
        ModuleStatesProto.ModuleStates msg = expected.getMutableModuleStates();
        msg.getMutableStamp().setSec(513).setNanosec(20000000);
        double[][] modules = {{1.0, 0.0}, {1.5, 0.5}, {-2.0, -0.5}, {0.25, 3.0}};
        for (double[] module : modules) {
            ModuleStatesProto.SwerveModuleState state = ModuleStatesProto.SwerveModuleState.newInstance().setSpeedMps(module[0]);
            if (module[1] != 0.0) {
                state.setAngleRad(module[1]);
            }
            msg.addModules(state);
        }
        EnvelopeProto.Envelope decoded = assertMatchesFixture("module_states", expected);

        StatesPacket.ModuleStatesData data = StatesPacket.fromProto(decoded.getModuleStates());
        assertEquals(4, data.moduleStates.length);
        assertEquals(-2.0, data.moduleStates[2].speedMetersPerSecond);
        assertEquals(-0.5, data.moduleStates[2].angle.getRadians(), 1e-12);
    }

    @Test
    public void matchInfo() throws IOException {
        EnvelopeProto.Envelope expected = envelope();
        expected.getMutableMatchInfo()
                .setMatchNum(42)
                .setMatchType(MatchInfoProto.MatchType.MATCH_TYPE_QUALIFICATION)
                .setStation(MatchInfoProto.AllianceStation.ALLIANCE_STATION_RED_2)
                .setEventName("MOKC Kansas City");
        assertMatchesFixture("match_info", expected);
    }

    @Test
    public void autoSnapshot() throws IOException {
        EnvelopeProto.Envelope expected = envelope();
        expected.getMutableAutoSnapshot().setEventName("auto_start").setStart(true);
        assertMatchesFixture("auto_snapshot", expected);
    }
}
