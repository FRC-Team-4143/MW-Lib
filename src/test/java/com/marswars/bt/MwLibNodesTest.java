package com.marswars.bt;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class MwLibNodesTest {
    @Test
    void specListsMwLibNodesOnly() {
        String xml = MwLibNodes.nodeSpecXml();
        assertTrue(xml.contains("<!-- MW-Lib shared nodes (com.marswars.bt) -->"));
        assertTrue(xml.contains("<Control ID=\"ParallelDeadline\">"));
        assertTrue(xml.contains("<Action ID=\"FollowTrajectory\">"));
        assertTrue(xml.contains("<Condition ID=\"IsAtChoreoSetpoint\">"));
        assertFalse(xml.contains("ID=\"Sequence\""), "BT.CPP built-ins are left out");
        assertFalse(xml.contains("Robot nodes"));
    }
}
