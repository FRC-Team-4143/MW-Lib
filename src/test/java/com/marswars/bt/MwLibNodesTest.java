package com.marswars.bt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class MwLibNodesTest {
    static final String RELATIVE = "src/main/resources/com/marswars/bt/mwlib_nodes.xml";

    @Test
    void bundledNodeSpecMatchesRegistrations() throws Exception {
        String generated = MwLibNodes.nodeSpecXml();
        Path file = Path.of(System.getProperty("mwlib.projectDir", ".")).resolve(RELATIVE);
        if (Boolean.getBoolean("mwlib.updateNodeSpec")) {
            Files.createDirectories(file.getParent());
            Files.writeString(file, generated, StandardCharsets.UTF_8);
        }
        assertTrue(Files.exists(file), "missing " + RELATIVE + "; run ./gradlew test -PupdateNodeSpec");
        assertEquals(
                generated,
                Files.readString(file),
                RELATIVE + " is out of date; run ./gradlew test -PupdateNodeSpec");
    }

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
