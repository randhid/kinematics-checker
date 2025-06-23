package kinematicsutils

import (
	"context"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/test"
)

func TestURDFConverterConfig(t *testing.T) {
	tests := []struct {
		name        string
		config      *URDFConverterConfig
		expectError bool
	}{
		{
			name: "valid config",
			config: &URDFConverterConfig{
				URDFFile: "test.urdf",
			},
			expectError: false,
		},
		{
			name: "missing urdf file",
			config: &URDFConverterConfig{
				URDFFile: "",
			},
			expectError: true,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			required, optional, err := tt.config.Validate("test")
			if tt.expectError {
				test.That(t, err, test.ShouldNotBeNil)
			} else {
				test.That(t, err, test.ShouldBeNil)
				test.That(t, required, test.ShouldBeNil)
				test.That(t, optional, test.ShouldBeNil)
			}
		})
	}
}

func TestNewURDFConverter(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	urdfFile := filepath.Join(tempDir, "test.urdf")

	// Create a simple URDF file
	urdfContent := `<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>`
	err := os.WriteFile(urdfFile, []byte(urdfContent), 0644)
	test.That(t, err, test.ShouldBeNil)

	config := &URDFConverterConfig{
		URDFFile: urdfFile,
	}

	name := generic.Named("test-urdf-converter")
	deps := resource.Dependencies{}

	service, err := NewURDFConverter(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, service, test.ShouldNotBeNil)

	// Test that it implements the resource interface
	_ = service

	// Test Name method
	test.That(t, service.Name(), test.ShouldResemble, name)

	// Test Close method
	err = service.Close(ctx)
	test.That(t, err, test.ShouldBeNil)
}

func TestURDFSanitization(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	urdfFile := filepath.Join(tempDir, "test.urdf")

	// Create a URDF file with mesh geometries
	urdfContent := `<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://test/meshes/base.dae" scale="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://test/meshes/base_collision.stl"/>
      </geometry>
    </collision>
  </link>
  <link name="link1">
    <visual>
      <geometry>
        <mesh filename="package://test/meshes/link1.obj"/>
      </geometry>
    </visual>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
  </joint>
</robot>`
	err := os.WriteFile(urdfFile, []byte(urdfContent), 0644)
	test.That(t, err, test.ShouldBeNil)

	config := &URDFConverterConfig{
		URDFFile: urdfFile,
	}

	name := generic.Named("test-urdf-converter")
	deps := resource.Dependencies{}

	service, err := NewURDFConverter(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldBeNil)
	defer service.Close(ctx)

	// Test URDF sanitization command
	result, err := service.DoCommand(ctx, map[string]interface{}{
		"command":   URDFCommand,
		"urdf_file": urdfFile,
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, result, test.ShouldNotBeNil)

	sanitizedXML, ok := result["sanitized_xml"].(string)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, sanitizedXML, test.ShouldNotEqual, "")

	// Verify that meshes were replaced with boxes
	test.That(t, strings.Contains(sanitizedXML, `<box size="50 60 70"/>`), test.ShouldBeTrue)
	test.That(t, strings.Contains(sanitizedXML, `<mesh filename=`), test.ShouldBeFalse)

	// Verify that joints remain unchanged (no automatic axis/limit addition)
	test.That(t, strings.Contains(sanitizedXML, `type="revolute"`), test.ShouldBeTrue)
	test.That(t, strings.Contains(sanitizedXML, `<axis xyz="0 0 1"/>`), test.ShouldBeFalse)
	test.That(t, strings.Contains(sanitizedXML, `<limit`), test.ShouldBeFalse)

	// Verify each link has exactly one collision geometry (either existing or default)
	linkCount := strings.Count(urdfContent, `<link name=`)
	boxCount := strings.Count(sanitizedXML, `<box size="50 60 70"/>`)
	test.That(t, boxCount, test.ShouldEqual, linkCount)
}

func TestURDFSanitizationWithComplexURDF(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	urdfFile := filepath.Join(tempDir, "complex.urdf")

	// Create a more complex URDF file with multiple links and joints
	urdfContent := `<?xml version="1.0"?>
<robot name="complex_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://complex/meshes/base.dae"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://complex/meshes/base_collision.stl"/>
      </geometry>
    </collision>
  </link>
  <link name="shoulder_link">
    <visual>
      <geometry>
        <mesh filename="package://complex/meshes/shoulder.obj"/>
      </geometry>
    </visual>
  </link>
  <link name="upper_arm_link">
    <visual>
      <geometry>
        <mesh filename="package://complex/meshes/upper_arm.dae"/>
      </geometry>
    </visual>
  </link>
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>
</robot>`
	err := os.WriteFile(urdfFile, []byte(urdfContent), 0644)
	test.That(t, err, test.ShouldBeNil)

	config := &URDFConverterConfig{
		URDFFile: urdfFile,
	}

	name := generic.Named("test-urdf-converter")
	deps := resource.Dependencies{}

	service, err := NewURDFConverter(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldBeNil)
	defer service.Close(ctx)

	// Test URDF sanitization command
	result, err := service.DoCommand(ctx, map[string]interface{}{
		"command":   URDFCommand,
		"urdf_file": urdfFile,
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, result, test.ShouldNotBeNil)

	sanitizedXML, ok := result["sanitized_xml"].(string)
	test.That(t, ok, test.ShouldBeTrue)

	// Verify no meshes remain
	test.That(t, strings.Contains(sanitizedXML, `<mesh filename=`), test.ShouldBeFalse)

	// Verify joints remain unchanged (no automatic modifications)
	test.That(t, strings.Contains(sanitizedXML, `type="revolute"`), test.ShouldBeTrue)
	test.That(t, strings.Contains(sanitizedXML, `<axis xyz="0 0 1"/>`), test.ShouldBeFalse)
	test.That(t, strings.Contains(sanitizedXML, `<limit`), test.ShouldBeFalse)

	// Verify original structure is preserved
	test.That(t, strings.Contains(sanitizedXML, `name="base_link"`), test.ShouldBeTrue)
	test.That(t, strings.Contains(sanitizedXML, `name="shoulder_link"`), test.ShouldBeTrue)
	test.That(t, strings.Contains(sanitizedXML, `name="upper_arm_link"`), test.ShouldBeTrue)
	test.That(t, strings.Contains(sanitizedXML, `name="shoulder_joint"`), test.ShouldBeTrue)
	test.That(t, strings.Contains(sanitizedXML, `name="elbow_joint"`), test.ShouldBeTrue)

	// Verify each link has exactly one collision geometry (either existing or default)
	linkCount := strings.Count(urdfContent, `<link name=`)
	boxCount := strings.Count(sanitizedXML, `<box size="50 60 70"/>`)
	test.That(t, boxCount, test.ShouldEqual, linkCount)
}

func TestURDFSanitizationErrorHandling(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	config := &URDFConverterConfig{
		URDFFile: "nonexistent.urdf",
	}

	name := generic.Named("test-urdf-converter")
	deps := resource.Dependencies{}

	_, err := NewURDFConverter(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestURDFSanitizationInvalidXML(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	urdfFile := filepath.Join(tempDir, "invalid.urdf")

	// Create an invalid XML file
	invalidContent := `invalid xml content`
	err := os.WriteFile(urdfFile, []byte(invalidContent), 0644)
	test.That(t, err, test.ShouldBeNil)

	config := &URDFConverterConfig{
		URDFFile: urdfFile,
	}

	name := generic.Named("test-urdf-converter")
	deps := resource.Dependencies{}

	// This should still work since we're doing simple string replacement, not XML parsing
	service, err := NewURDFConverter(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldBeNil)
	defer service.Close(ctx)

	// Test that DoCommand returns the sanitized file
	result, err := service.DoCommand(ctx, map[string]interface{}{
		"command": URDFCommand,
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, result, test.ShouldNotBeNil)

	sanitizedXML, ok := result["sanitized_xml"].(string)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, sanitizedXML, test.ShouldNotEqual, "")
}

func TestURDFCommandErrorHandling(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	urdfFile := filepath.Join(tempDir, "test.urdf")

	// Create a simple URDF file
	urdfContent := `<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
  </link>
</robot>`
	err := os.WriteFile(urdfFile, []byte(urdfContent), 0644)
	test.That(t, err, test.ShouldBeNil)

	config := &URDFConverterConfig{
		URDFFile: urdfFile,
	}

	name := generic.Named("test-urdf-converter")
	deps := resource.Dependencies{}

	service, err := NewURDFConverter(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldBeNil)
	defer service.Close(ctx)

	// Test unknown command
	_, err = service.DoCommand(ctx, map[string]interface{}{
		"command": "unknown_command",
	})
	test.That(t, err, test.ShouldBeNil) // Should return nil, nil for unknown commands
}
