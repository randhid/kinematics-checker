package kinematicsutils

import (
	"context"
	"os"
	"path/filepath"
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
				URDFFile: "ur20.urdf",
			},
			expectError: false,
		},
		{
			name: "missing URDF file",
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
	urdfFile := filepath.Join(tempDir, "ur20.urdf")

	// Create a simple URDF file
	urdfContent := `<?xml version="1.0"?>
<robot name="ur20-test">
  <link name="base_link"/>
  <link name="shoulder_link"/>
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <axis xyz="0 0 1"/>
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
	test.That(t, service, test.ShouldNotBeNil)

	// Test that it implements the resource interface
	_, ok := service.(resource.Resource)
	test.That(t, ok, test.ShouldBeTrue)

	// Test Name method
	test.That(t, service.Name(), test.ShouldResemble, name)

	// Test Close method
	err = service.Close(ctx)
	test.That(t, err, test.ShouldBeNil)
}

func TestURDFConverterDoCommand(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	urdfFile := filepath.Join(tempDir, "ur20.urdf")

	// Create a simple URDF file
	urdfContent := `<?xml version="1.0"?>
<robot name="ur20-test">
  <link name="base_link"/>
  <link name="shoulder_link"/>
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <axis xyz="0 0 1"/>
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

	// Test URDF command
	result, err := service.DoCommand(ctx, map[string]interface{}{
		"command": URDFCommand,
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, result, test.ShouldBeNil)

	// Test URDF2SVA command
	result, err = service.DoCommand(ctx, map[string]interface{}{
		"command": URDF2SVACommand,
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, result, test.ShouldBeNil)

	// Test unknown command
	result, err = service.DoCommand(ctx, map[string]interface{}{
		"command": "unknown",
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, result, test.ShouldBeNil)

	// Test command without command key
	result, err = service.DoCommand(ctx, map[string]interface{}{
		"other": "value",
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, result, test.ShouldBeNil)
}

func TestURDFConverterWithInvalidFile(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	config := &URDFConverterConfig{
		URDFFile: "nonexistent.urdf",
	}

	name := generic.Named("test-urdf-converter")
	deps := resource.Dependencies{}

	// This should still work since we're not actually reading the file in the constructor
	service, err := NewURDFConverter(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, service, test.ShouldNotBeNil)
	defer service.Close(ctx)

	// Test that the service can still handle commands
	result, err := service.DoCommand(ctx, map[string]interface{}{
		"command": URDFCommand,
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, result, test.ShouldBeNil)
}

func TestURDFConverterServiceInterface(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	urdfFile := filepath.Join(tempDir, "ur20.urdf")

	// Create a simple URDF file
	urdfContent := `<?xml version="1.0"?>
<robot name="ur20-test">
  <link name="base_link"/>
  <link name="shoulder_link"/>
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <axis xyz="0 0 1"/>
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

	// Test Name method
	test.That(t, service.Name(), test.ShouldResemble, name)

	// Test DoCommand method
	result, err := service.DoCommand(ctx, map[string]interface{}{"test": "command"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, result, test.ShouldBeNil)

	// Test Close method
	err = service.Close(ctx)
	test.That(t, err, test.ShouldBeNil)
}
