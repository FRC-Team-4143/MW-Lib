package com.marswars.bt.xml;

import com.marswars.bt.core.NodeKind;
import com.marswars.bt.core.NodeModel;
import com.marswars.bt.core.NodeSpec;
import com.marswars.bt.core.PortDirection;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.PortType;
import com.marswars.bt.core.TreeSpec;
import java.io.IOException;
import java.io.StringReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import javax.xml.XMLConstants;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.parsers.SAXParser;
import javax.xml.parsers.SAXParserFactory;
import org.xml.sax.Attributes;
import org.xml.sax.InputSource;
import org.xml.sax.Locator;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException;
import org.xml.sax.helpers.DefaultHandler;

/**
 * Parses BehaviorTree.CPP v4 XML into a {@link TreeSpec}.
 *
 * <p>Supported: {@code <root BTCPP_format="4" main_tree_to_execute>}, any number of {@code
 * <BehaviorTree ID>}, nodes written as {@code <Id .../>} or {@code <Action|Condition|Control|Decorator
 * ID="Id" .../>}, {@code <SubTree ID>} with port remapping and {@code _autoremap}, {@code
 * <TreeNodesModel>}, and {@code <include path>}. Groot2 metadata ({@code _uid}, {@code _fullpath},
 * {@code _description}) is ignored. BT.CPP scripting (pre/post-condition attributes such as {@code
 * _skipIf}) is rejected, since MW-Lib has no script interpreter.
 */
public final class BtXmlParser {
    /** Pre/post-condition attributes that need the BT.CPP scripting language. */
    public static final Set<String> SCRIPT_ATTRIBUTES =
            Set.of(
                    "_skipIf",
                    "_successIf",
                    "_failureIf",
                    "_while",
                    "_onSuccess",
                    "_onFailure",
                    "_onHalted",
                    "_post");

    /** Editor/Groot2 bookkeeping attributes that carry no behavior. */
    public static final Set<String> IGNORED_ATTRIBUTES = Set.of("_uid", "_fullpath", "_description");

    /** Deprecated BT.CPP v3 names still accepted, mapped to their v4 names. */
    public static final Map<String, String> DEPRECATED_ALIASES =
            Map.of(
                    "SequenceStar", "SequenceWithMemory",
                    "RetryUntilSuccesful", "RetryUntilSuccessful");

    private BtXmlParser() {}

    /** Parses XML text; {@code <include>} paths resolve against the working directory. */
    public static TreeSpec parse(String xml) {
        return parse(xml, null);
    }

    /** Parses a file; {@code <include>} paths resolve against the file's directory. */
    public static TreeSpec parse(Path file) {
        return parseFile(file.toAbsolutePath().normalize(), new HashSet<>());
    }

    /** Parses XML text whose {@code <include>} paths resolve against {@code baseDir}. */
    public static TreeSpec parse(String xml, Path baseDir) {
        return parseText(xml, baseDir, new HashSet<>(), true);
    }

    private static TreeSpec parseFile(Path file, Set<Path> visited) {
        if (!visited.add(file)) {
            throw new BtXmlException("include cycle through " + file, 0);
        }
        String xml;
        try {
            xml = Files.readString(file, StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new BtXmlException("cannot read " + file + ": " + e.getMessage(), 0, e);
        }
        try {
            return parseText(xml, file.getParent(), visited, true);
        } catch (BtXmlException e) {
            throw new BtXmlException(file.getFileName() + ": " + e.getMessage(), 0, e);
        }
    }

    private static TreeSpec parseText(String xml, Path baseDir, Set<Path> visited, boolean isMain) {
        Handler handler = new Handler(baseDir, visited);
        try {
            SAXParserFactory factory = SAXParserFactory.newInstance();
            factory.setNamespaceAware(false);
            factory.setFeature(XMLConstants.FEATURE_SECURE_PROCESSING, true);
            factory.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true);
            factory.setFeature("http://xml.org/sax/features/external-general-entities", false);
            factory.setFeature("http://xml.org/sax/features/external-parameter-entities", false);
            SAXParser parser = factory.newSAXParser();
            parser.parse(new InputSource(new StringReader(xml)), handler);
        } catch (SAXParseException e) {
            throw new BtXmlException(e.getMessage(), e.getLineNumber(), e);
        } catch (SAXException e) {
            if (e.getException() instanceof BtXmlException bx) {
                throw bx;
            }
            throw new BtXmlException(e.getMessage(), 0, e);
        } catch (ParserConfigurationException | IOException e) {
            throw new BtXmlException("XML parser failure: " + e.getMessage(), 0, e);
        }
        return handler.result();
    }

    // ------------------------------------------------------------------ SAX handler

    private static final class NodeFrame {
        final String id;
        final String name;
        final Map<String, String> attributes;
        final String subtree_id;
        final int line;
        final List<NodeSpec> children = new ArrayList<>();

        NodeFrame(String id, String name, Map<String, String> attrs, String subtreeId, int line) {
            this.id = id;
            this.name = name;
            this.attributes = attrs;
            this.subtree_id = subtreeId;
            this.line = line;
        }

        NodeSpec toSpec() {
            return new NodeSpec(id, name, attributes, children, subtree_id, line);
        }
    }

    private static final class ModelFrame {
        final String id;
        final NodeKind kind;
        final List<PortInfo> ports = new ArrayList<>();
        String description = "";

        ModelFrame(String id, NodeKind kind) {
            this.id = id;
            this.kind = kind;
        }
    }

    private enum Section {
        NONE,
        ROOT,
        TREE,
        MODELS
    }

    private static final class Handler extends DefaultHandler {
        private final Path base_dir_;
        private final Set<Path> visited_;
        private Locator locator_;

        private final Map<String, NodeSpec> trees_ = new LinkedHashMap<>();
        private final List<NodeModel> models_ = new ArrayList<>();
        private final List<String> warnings_ = new ArrayList<>();
        private String main_tree_attr_ = null;
        private boolean saw_root_ = false;

        private Section section_ = Section.NONE;
        private String current_tree_id_ = null;
        private int current_tree_line_ = 0;
        private final Deque<NodeFrame> node_stack_ = new ArrayDeque<>();
        private NodeSpec current_tree_root_ = null;

        private ModelFrame model_ = null;
        private String port_name_ = null;
        private PortDirection port_dir_ = null;
        private String port_type_ = null;
        private String port_default_ = null;
        private StringBuilder port_text_ = null;
        private StringBuilder description_text_ = null;
        private int depth_in_unknown_ = 0;

        Handler(Path baseDir, Set<Path> visited) {
            base_dir_ = baseDir;
            visited_ = visited;
        }

        @Override
        public void setDocumentLocator(Locator locator) {
            locator_ = locator;
        }

        private int line() {
            return locator_ == null ? 0 : locator_.getLineNumber();
        }

        private BtXmlException error(String message) {
            return new BtXmlException(message, line());
        }

        @Override
        public void startElement(String uri, String local, String qName, Attributes attrs)
                throws SAXException {
            try {
                start(qName, attrs);
            } catch (BtXmlException e) {
                throw new SAXException(e);
            }
        }

        @Override
        public void endElement(String uri, String local, String qName) throws SAXException {
            try {
                end(qName);
            } catch (BtXmlException e) {
                throw new SAXException(e);
            }
        }

        @Override
        public void characters(char[] ch, int start, int length) {
            if (port_text_ != null) {
                port_text_.append(ch, start, length);
            } else if (description_text_ != null) {
                description_text_.append(ch, start, length);
            }
        }

        private void start(String tag, Attributes attrs) {
            if (depth_in_unknown_ > 0) {
                depth_in_unknown_++;
                return;
            }
            switch (section_) {
                case NONE -> {
                    if (!"root".equals(tag)) {
                        throw error("document element must be <root>, found <" + tag + ">");
                    }
                    saw_root_ = true;
                    String format = attrs.getValue("BTCPP_format");
                    if (format == null) {
                        warnings_.add("<root> has no BTCPP_format attribute; assuming 4");
                    } else if (!"4".equals(format.trim())) {
                        throw error("unsupported BTCPP_format=\"" + format + "\" (only 4)");
                    }
                    main_tree_attr_ = attrs.getValue("main_tree_to_execute");
                    section_ = Section.ROOT;
                }
                case ROOT -> startRootChild(tag, attrs);
                case TREE -> startNode(tag, attrs);
                case MODELS -> startModel(tag, attrs);
            }
        }

        private void startRootChild(String tag, Attributes attrs) {
            switch (tag) {
                case "BehaviorTree" -> {
                    String id = attrs.getValue("ID");
                    if (id == null || id.isBlank()) {
                        throw error("<BehaviorTree> needs an ID attribute");
                    }
                    if (trees_.containsKey(id)) {
                        throw error("duplicate <BehaviorTree ID=\"" + id + "\">");
                    }
                    current_tree_id_ = id;
                    current_tree_line_ = line();
                    current_tree_root_ = null;
                    section_ = Section.TREE;
                }
                case "TreeNodesModel" -> section_ = Section.MODELS;
                case "include" -> include(attrs);
                default -> {
                    warnings_.add("line " + line() + ": ignoring <" + tag + "> under <root>");
                    depth_in_unknown_ = 1;
                }
            }
        }

        private void include(Attributes attrs) {
            if (attrs.getValue("ros_pkg") != null) {
                throw error("<include ros_pkg=...> is not supported");
            }
            String path = attrs.getValue("path");
            if (path == null || path.isBlank()) {
                throw error("<include> needs a path attribute");
            }
            Path base = base_dir_ != null ? base_dir_ : Path.of("").toAbsolutePath();
            Path target = base.resolve(path).toAbsolutePath().normalize();
            TreeSpec included = parseFile(target, visited_);
            for (var entry : included.trees().entrySet()) {
                if (trees_.putIfAbsent(entry.getKey(), entry.getValue()) != null) {
                    throw error("included tree '" + entry.getKey() + "' is already defined");
                }
            }
            models_.addAll(included.models());
            warnings_.addAll(included.warnings());
        }

        private void startNode(String tag, Attributes attrs) {
            if (node_stack_.isEmpty() && current_tree_root_ != null) {
                throw error(
                        "<BehaviorTree ID=\"" + current_tree_id_ + "\"> must have exactly one root node");
            }
            String id = tag;
            String subtree_id = null;
            boolean typed = NodeKind.fromXmlTag(tag).isPresent() && !"SubTree".equals(tag);
            if (typed || "SubTree".equals(tag)) {
                String attr_id = attrs.getValue("ID");
                if (attr_id == null || attr_id.isBlank()) {
                    throw error("<" + tag + "> needs an ID attribute");
                }
                if ("SubTree".equals(tag)) {
                    id = NodeSpec.SUBTREE;
                    subtree_id = attr_id;
                } else {
                    id = attr_id;
                }
            }
            String alias = DEPRECATED_ALIASES.get(id);
            if (alias != null) {
                warnings_.add("line " + line() + ": '" + id + "' is deprecated; using " + alias);
                id = alias;
            }

            String name = null;
            Map<String, String> ports = new LinkedHashMap<>();
            for (int i = 0; i < attrs.getLength(); i++) {
                String key = attrs.getQName(i);
                String value = attrs.getValue(i);
                if ("name".equals(key)) {
                    name = value;
                } else if ("ID".equals(key) && (typed || subtree_id != null)) {
                    continue;
                } else if (SCRIPT_ATTRIBUTES.contains(key)) {
                    throw error(
                            "attribute "
                                    + key
                                    + " on <"
                                    + tag
                                    + "> needs BT.CPP scripting, which MW-Lib does not support");
                } else if (IGNORED_ATTRIBUTES.contains(key)) {
                    continue;
                } else if ("_autoremap".equals(key) && subtree_id != null) {
                    ports.put(key, value);
                } else if (key.startsWith("_")) {
                    warnings_.add("line " + line() + ": ignoring attribute " + key);
                } else {
                    ports.put(key, value);
                }
            }
            node_stack_.push(new NodeFrame(id, name, ports, subtree_id, line()));
        }

        private void startModel(String tag, Attributes attrs) {
            if (model_ == null) {
                NodeKind kind =
                        NodeKind.fromXmlTag(tag)
                                .orElseThrow(
                                        () ->
                                                error(
                                                        "unexpected <"
                                                                + tag
                                                                + "> in <TreeNodesModel>"));
                String id = attrs.getValue("ID");
                if (id == null || id.isBlank()) {
                    throw error("<" + tag + "> in <TreeNodesModel> needs an ID attribute");
                }
                model_ = new ModelFrame(id, kind);
                return;
            }
            switch (tag) {
                case "input_port", "output_port", "inout_port" -> {
                    port_dir_ =
                            switch (tag) {
                                case "input_port" -> PortDirection.INPUT;
                                case "output_port" -> PortDirection.OUTPUT;
                                default -> PortDirection.INOUT;
                            };
                    port_name_ = attrs.getValue("name");
                    if (port_name_ == null) {
                        throw error("<" + tag + "> needs a name attribute");
                    }
                    port_type_ = attrs.getValue("type");
                    port_default_ = attrs.getValue("default");
                    port_text_ = new StringBuilder();
                }
                case "description" -> description_text_ = new StringBuilder();
                // Free-form metadata (<MetaFields><owner>hri</owner></MetaFields>) is skipped.
                // BT.CPP 4.6's <MetadataFields><Metadata description=.../> is still read.
                case "MetadataFields" -> {}
                case "Metadata" -> {
                    String description = attrs.getValue("description");
                    if (description != null) {
                        model_.description = description;
                    }
                }
                default -> depth_in_unknown_ = 1;
            }
        }

        private void end(String tag) {
            if (depth_in_unknown_ > 0) {
                depth_in_unknown_--;
                return;
            }
            switch (section_) {
                case ROOT -> {
                    if ("root".equals(tag)) {
                        section_ = Section.NONE;
                    }
                }
                case TREE -> {
                    if (node_stack_.isEmpty()) {
                        // </BehaviorTree>
                        if (current_tree_root_ == null) {
                            throw new BtXmlException(
                                    "<BehaviorTree ID=\""
                                            + current_tree_id_
                                            + "\"> has no root node",
                                    current_tree_line_);
                        }
                        trees_.put(current_tree_id_, current_tree_root_);
                        section_ = Section.ROOT;
                        return;
                    }
                    NodeSpec spec = node_stack_.pop().toSpec();
                    if (node_stack_.isEmpty()) {
                        current_tree_root_ = spec;
                    } else {
                        node_stack_.peek().children.add(spec);
                    }
                }
                case MODELS -> endModel(tag);
                default -> {}
            }
        }

        private void endModel(String tag) {
            if (model_ == null) {
                section_ = Section.ROOT; // </TreeNodesModel>
                return;
            }
            if (description_text_ != null && "description".equals(tag)) {
                model_.description = description_text_.toString().trim();
                description_text_ = null;
                return;
            }
            if (port_text_ != null && tag.endsWith("_port")) {
                model_.ports.add(
                        new PortInfo(
                                port_name_,
                                port_dir_,
                                portTypeFromXml(port_type_),
                                port_default_,
                                port_text_.toString().trim(),
                                List.of(),
                                null,
                                port_type_));
                port_text_ = null;
                return;
            }
            if (NodeKind.fromXmlTag(tag).isPresent()) {
                models_.add(
                        new NodeModel(model_.id, model_.kind, model_.ports, model_.description, false));
                model_ = null;
            }
        }

        TreeSpec result() {
            if (!saw_root_) {
                throw new BtXmlException("empty document", 0);
            }
            if (trees_.isEmpty()) {
                throw new BtXmlException("no <BehaviorTree> defined", 0);
            }
            String main = main_tree_attr_;
            if (main == null || main.isBlank()) {
                main = trees_.size() == 1 ? trees_.keySet().iterator().next() : "";
            } else if (!trees_.containsKey(main)) {
                throw new BtXmlException(
                        "main_tree_to_execute=\"" + main + "\" is not a defined <BehaviorTree>", 0);
            }
            return new TreeSpec(main, trees_, models_, warnings_);
        }
    }

    /** Maps a {@code <TreeNodesModel>} {@code type} attribute to a {@link PortType}. */
    public static PortType portTypeFromXml(String type) {
        if (type == null) {
            return PortType.ANY;
        }
        return switch (type.trim()) {
            case "double", "float" -> PortType.DOUBLE;
            case "int", "unsigned", "unsigned int", "int64_t", "uint64_t", "long", "size_t" ->
                    PortType.INT;
            case "bool" -> PortType.BOOLEAN;
            case "std::string", "string", "BT::StringView" -> PortType.STRING;
            case "trajectory" -> PortType.TRAJECTORY;
            case "" -> PortType.ANY;
            default -> PortType.ANY; // custom C++/Java types keep their name via PortInfo.typeName
        };
    }
}
