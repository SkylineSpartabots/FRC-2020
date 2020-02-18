package frc.robot.paths;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

class PathTest {
    @Test
    void generatePaths(TestInfo testInfo) {
        PathGenerator pg = PathGenerator.getInstance();
        pg.generatePaths();
        PathGenerator.PathSet paths = pg.getPathSet();
        assertNotNull(paths);
        System.err.println(paths);
    }
}
