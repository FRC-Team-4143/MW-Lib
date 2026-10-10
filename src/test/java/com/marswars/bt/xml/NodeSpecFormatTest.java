package com.marswars.bt.xml;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.NodeKind;
import com.marswars.bt.core.NodeModel;
import com.marswars.bt.core.PortDirection;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.PortType;
import java.nio.file.Files;
import java.util.List;
import org.junit.jupiter.api.Test;

/** The node-spec ({@code <TreeNodesModel>}) format the BT editor consumes. */
class NodeSpecFormatTest {

    /** Node-spec documents have no {@code <BehaviorTree>}; parse their models only. */
    static List<NodeModel> models(String xml) {
        return BtXmlParser.parse(
                        xml.replace(
                                "<TreeNodesModel>",
                                "<BehaviorTree ID=\"_\"><AlwaysSuccess/></BehaviorTree><TreeNodesModel>"))
                .models();
    }

    @Test
    void readsEditorExample() throws Exception {
        String xml =
                Files.readString(
                        BtXmlParserTest.resource("/com/marswars/bt/node_spec_example.xml"));
        List<NodeModel> models = models(xml);
        assertEquals(9, models.size());

        NodeModel battery = models.get(0);
        assertEquals("IsBatteryOk", battery.id());
        assertEquals(NodeKind.CONDITION, battery.kind());
        assertEquals("SUCCESS while the battery is above min_percent", battery.description());
        PortInfo min = battery.ports().get(0);
        assertEquals("double", min.xmlType());
        assertEquals("20", min.defaultValue());
        assertEquals("Battery level below which this returns FAILURE", min.description());

        NodeModel detect = models.get(3);
        PortInfo pose = detect.ports().get(1);
        assertEquals(PortDirection.OUTPUT, pose.direction());
        assertEquals("geometry_msgs::msg::PoseStamped", pose.xmlType(), "custom type kept");
        assertEquals(PortType.ANY, pose.type());

        assertTrue(models.get(6).ports().isEmpty(), "<Action ID=\"Dock\"/>");
        assertEquals("Say", models.get(8).id());
        assertEquals(1, models.get(8).ports().size(), "<MetaFields> skipped");
    }

    @Test
    void writerUsesDescriptionElementAndRoundTrips() throws Exception {
        String xml =
                Files.readString(
                        BtXmlParserTest.resource("/com/marswars/bt/node_spec_example.xml"));
        List<NodeModel> models = models(xml);
        String written = BtXmlWriter.writeModels(models);
        assertTrue(
                written.contains(
                        "    <Condition ID=\"IsBatteryOk\">\n"
                                + "      <input_port name=\"min_percent\" type=\"double\" default=\"20\">"
                                + "Battery level below which this returns FAILURE</input_port>\n"
                                + "      <description>SUCCESS while the battery is above min_percent"
                                + "</description>\n"
                                + "    </Condition>\n"),
                written);
        assertTrue(written.contains("<Action ID=\"Dock\"/>"));
        assertTrue(
                written.contains(
                        "<output_port name=\"pose\" type=\"geometry_msgs::msg::PoseStamped\">"));
        assertFalse(written.contains("MetadataFields"));
        assertEquals(models, models(written));
    }

    @Test
    void factoryNodeSpecUsesSameShape() {
        BehaviorTreeFactory factory = new BehaviorTreeFactory(() -> 0.0);
        factory.registerSimpleCondition(
                "IsBatteryOk",
                "SUCCESS while the battery is above min_percent",
                List.of(
                        PortInfo.input(
                                "min_percent",
                                PortType.DOUBLE,
                                "20",
                                "Battery level below which this returns FAILURE")),
                n -> true);
        factory.registerInstantAction(
                "Aim",
                "",
                List.of(PortInfo.choiceInput("target", List.of("A", "B"), "A", "Target")),
                n -> {});
        String xml = factory.writeTreeNodesModelXml(false);
        assertTrue(xml.startsWith("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<root BTCPP_format=\"4\">\n  <TreeNodesModel>\n"));
        assertTrue(xml.contains("<description>SUCCESS while the battery is above min_percent</description>"));
        assertTrue(xml.contains("<input_port name=\"target\" type=\"std::string\" default=\"A\">Target</input_port>"));
    }
}
