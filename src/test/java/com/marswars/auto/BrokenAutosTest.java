package com.marswars.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.BtException;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.PortType;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.net.URISyntaxException;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Autos that must not load. Every file in {@code broken-autos/} is one auto with one defect (plus a
 * control that is fine). Each is loaded the way the robot does it ({@link
 * BehaviorTreeAuto#loadAll}), and the test pins what the student or mentor would read: the auto is
 * still listed in the chooser, is marked not loaded, and its error names the defect.
 *
 * <p>(A file with several trees and no {@code main_tree_to_execute} is deliberately not here: that
 * is a library file such as {@code common.xml}, which is not an auto at all; see {@code
 * BehaviorTreeAutoTest.libraryFilesAreNotAutos}.)
 */
class BrokenAutosTest {
    /** File stem to a fragment of the error it must produce. */
    static final Map<String, String> EXPECTED = new LinkedHashMap<>();

    static {
        EXPECTED.put("BrokenXml", ""); // checked separately: the parser's own message
        EXPECTED.put("UnknownNode", "unknown node ID 'DriveFast'");
        EXPECTED.put("UnknownPort", "unknown port 'spedd'");
        EXPECTED.put("MissingRequiredPort", "missing required port 'trajectory'");
        EXPECTED.put("BadPortValue", "'fast' as Double");
        EXPECTED.put("MissingBlackboardEntry", "reads {sped}, which nothing sets");
        EXPECTED.put("TunableWithoutDefault", "reads {speed}, which nothing sets");
        EXPECTED.put("SubTreeNotDefined", "SubTree ID 'Shoot' is not a defined");
        EXPECTED.put("EmptySequence", "at least one child");
        EXPECTED.put("MainTreeNotDefined", "main_tree_to_execute=\"Autonomous\" is not a defined");
    }

    BehaviorTreeFactory factory;

    @BeforeEach
    void setUp() {
        BehaviorTreeAuto.message_log_ = line -> {};
        factory = new BehaviorTreeFactory(() -> 0.0);
        factory.registerInstantAction(
                "Follow",
                "Follow a path",
                List.of(
                        PortInfo.trajectory("trajectory", "path"),
                        PortInfo.input("speed", PortType.DOUBLE, "1", "speed")),
                n -> {});
    }

    @AfterEach
    void restoreLog() {
        BehaviorTreeAuto.message_log_ = edu.wpi.first.wpilibj.DataLogManager::log;
    }

    private Path dir() throws URISyntaxException {
        var url = getClass().getResource("broken-autos");
        assertNotNull(url, "broken-autos fixtures are on the test classpath");
        return Path.of(url.toURI());
    }

    private Map<String, BehaviorTreeAuto> load() throws URISyntaxException {
        Map<String, BehaviorTreeAuto> byName = new LinkedHashMap<>();
        for (BehaviorTreeAuto auto : BehaviorTreeAuto.loadAll(factory, dir())) {
            auto.reload(); // what AutoManager's boot check does (without the paths)
            byName.put(auto.getName(), auto);
        }
        return byName;
    }

    @Test
    void everyFixtureIsListedAndExactlyTheControlLoads() throws Exception {
        Map<String, BehaviorTreeAuto> autos = load();
        // every file becomes an auto; AutoManager leaves the broken ones out of the chooser
        // and raises an alert naming them
        assertEquals(
                EXPECTED.size() + 1, autos.size(), "every fixture is listed: " + autos.keySet());
        assertTrue(autos.get("Good").isLoaded(), autos.get("Good").getError());
        assertEquals("", autos.get("Good").getError());
        for (String name : EXPECTED.keySet()) {
            assertFalse(autos.get(name).isLoaded(), name + " must not load");
        }
    }

    @Test
    void eachBrokenAutoSaysWhatIsWrong() throws Exception {
        Map<String, BehaviorTreeAuto> autos = load();
        EXPECTED.forEach(
                (name, fragment) -> {
                    String error = autos.get(name).getError();
                    assertFalse(error.isEmpty(), name + " has no error text");
                    assertTrue(
                            error.contains(fragment),
                            name + ": expected '" + fragment + "' in: " + error);
                });
    }

    @Test
    void brokenXmlReportsTheParserError() throws Exception {
        String error = load().get("BrokenXml").getError();
        assertFalse(error.isBlank(), "a message the student can act on");
        assertFalse(error.contains("NullPointerException"), error);
    }

    @Test
    void aBrokenAutoRefusesToBuildATree() throws Exception {
        Map<String, BehaviorTreeAuto> autos = load();
        for (String name : EXPECTED.keySet()) {
            BtException e = assertThrows(BtException.class, autos.get(name)::buildTree, name);
            assertFalse(e.getMessage().isBlank(), name);
        }
        assertNotNull(autos.get("Good").buildTree());
    }
}
