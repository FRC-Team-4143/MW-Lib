package com.marswars.auto;

import com.marswars.bt.core.BtException;
import com.marswars.bt.xml.BtXmlParser;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Stream;

/** Finding and reading behavior-tree auto files in a robot's {@code deploy/autos/trees} folder. */
public final class AutoFiles {
    private AutoFiles() {}

    /**
     * The auto files in {@code dir}: {@link #listXml} without library files. A library file (shared
     * subtrees such as {@code common.xml}) defines several trees and no {@code
     * main_tree_to_execute}, so it has no tree to run. A file that doesn't parse is still listed, so
     * it shows up in the chooser with its error.
     */
    public static List<Path> listAutos(Path dir) {
        return listXml(dir).stream().filter(file -> !isLibraryFile(file)).toList();
    }

    /** True for a file that parses but names no tree to run (see {@link #listAutos}). */
    public static boolean isLibraryFile(Path file) {
        try {
            return BtXmlParser.parse(file).mainTreeId().isEmpty();
        } catch (RuntimeException e) {
            return false;
        }
    }

    /** Sorted {@code *.xml} files directly in {@code dir} (empty if it is missing). */
    public static List<Path> listXml(Path dir) {
        if (!Files.isDirectory(dir)) {
            return List.of();
        }
        try (Stream<Path> files = Files.list(dir)) {
            return files.filter(p -> p.getFileName().toString().endsWith(".xml"))
                    .filter(Files::isRegularFile)
                    .sorted()
                    .toList();
        } catch (IOException e) {
            throw new BtException("cannot list " + dir + ": " + e.getMessage(), e);
        }
    }

    /** The auto's name: the file name without its extension ({@code Citrus.xml} gives {@code Citrus}). */
    public static String autoName(Path file) {
        String n = file.getFileName().toString();
        int dot = n.lastIndexOf('.');
        return dot > 0 ? n.substring(0, dot) : n;
    }

    static String read(Path file) {
        try {
            return Files.readString(file, StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new BtException("cannot read " + file + ": " + e.getMessage(), e);
        }
    }
}
