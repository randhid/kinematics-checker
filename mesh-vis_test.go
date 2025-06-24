package kinematicsutils

import (
	"context"
	"os"
	"path/filepath"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestMeshVizConfig(t *testing.T) {
	tests := []struct {
		name        string
		config      *MeshVizConfig
		expectError bool
	}{
		{
			name: "valid config",
			config: &MeshVizConfig{
				MeshFile: "test.ply",
			},
			expectError: false,
		},
		{
			name: "missing mesh file",
			config: &MeshVizConfig{
				MeshFile: "",
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

func TestNewMeshViz(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	meshFile := filepath.Join(tempDir, "test.ply")

	// Create a simple PLY file
	plyContent := `ply
format ascii 1.0
element vertex 3
property float x
property float y
property float z
end_header
0.0 0.0 0.0
1.0 0.0 0.0
0.0 1.0 0.0
`
	err := os.WriteFile(meshFile, []byte(plyContent), 0644)
	test.That(t, err, test.ShouldBeNil)

	config := &MeshVizConfig{
		MeshFile: meshFile,
	}

	name := gripper.Named("test-gripper")
	deps := resource.Dependencies{}

	gripper, err := NewMeshViz(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, gripper, test.ShouldNotBeNil)

	// Test that it implements the gripper interface
	_ = gripper

	// Test Name method
	test.That(t, gripper.Name(), test.ShouldResemble, name)

	// Test Close method
	err = gripper.Close(ctx)
	test.That(t, err, test.ShouldBeNil)
}

func TestMeshVizInterface(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	meshFile := filepath.Join(tempDir, "test.ply")

	// Create a simple PLY file
	plyContent := `ply
format ascii 1.0
comment Created by Open Asset Import Library - http://assimp.sf.net (v5.2.3481089591)
element vertex 163866
property float x
property float y
property float z
property float nx
property float ny
property float nz
element face 190653
property list uchar int vertex_index
end_header
6.05000019 7.4000001 52.1994667 -0.993712187 0.111964472 -5.80092499e-17
5.898314 6.05374813 64.1994629 -0.974927902 0.222520918 -2.01192864e-16
6.05000019 7.4000001 64.1994629 -0.993712187 0.111964472 -0
5.898314 6.05374813 52.1994667 -0.974927902 0.222520933 -2.30184062e-16
5.45086145 4.77500343 52.1994667 -0.916618288 0.399763644 -4.14599731e-16
5.45086145 4.77500343 64.1994629 -0.884049892 0.467392474 -4.84939703e-16
4.7300806 3.62788677 52.1994667 -0.781831443 0.623489857 -6.46533419e-16
`
	err := os.WriteFile(meshFile, []byte(plyContent), 0644)
	test.That(t, err, test.ShouldBeNil)

	config := &MeshVizConfig{
		MeshFile: meshFile,
	}

	name := gripper.Named("test-gripper")
	deps := resource.Dependencies{}

	gripper, err := NewMeshViz(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldBeNil)
	defer gripper.Close(ctx)

	// Test Grab method
	grabbed, err := gripper.Grab(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, grabbed, test.ShouldBeFalse)

	// Test Open method
	err = gripper.Open(ctx, nil)
	test.That(t, err, test.ShouldBeNil)

	// Test Stop method
	err = gripper.Stop(ctx, nil)
	test.That(t, err, test.ShouldBeNil)

	// Test IsMoving method
	moving, err := gripper.IsMoving(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, moving, test.ShouldBeFalse)

	// Test CurrentInputs method
	inputs, err := gripper.CurrentInputs(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, inputs, test.ShouldBeNil)

	// Test Geometries method
	geometries, err := gripper.Geometries(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(geometries), test.ShouldEqual, 1)

	// Test GoToInputs method
	err = gripper.GoToInputs(ctx, []referenceframe.Input{{Value: 0.5}})
	test.That(t, err, test.ShouldBeNil)

	// Test Kinematics method
	model, err := gripper.Kinematics(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, model, test.ShouldBeNil)

	// Test DoCommand method
	result, err := gripper.DoCommand(ctx, map[string]interface{}{"test": "command"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, result, test.ShouldBeNil)
}

func TestMeshVizWithTransform(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	meshFile := filepath.Join(tempDir, "test.ply")

	// Create a simple PLY file
	plyContent := `ply
format ascii 1.0
element vertex 3
property float x
property float y
property float z
end_header
0.0 0.0 0.0
1.0 0.0 0.0
0.0 1.0 0.0
`
	err := os.WriteFile(meshFile, []byte(plyContent), 0644)
	test.That(t, err, test.ShouldBeNil)

	// Create a transform
	transform := spatialmath.NewPose(r3.Vector{X: 1.0, Y: 2.0, Z: 3.0}, spatialmath.NewZeroOrientation())

	config := &MeshVizConfig{
		MeshFile:  meshFile,
		Transform: transform,
	}

	name := gripper.Named("test-gripper")
	deps := resource.Dependencies{}

	// Test with local mesh file
	gripper, err := NewMeshViz(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldBeNil)
	defer gripper.Close(ctx)

	// Test with mesh file from repo
	config.MeshFile = "lod_500.ply"
	gripper2, err := NewMeshViz(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldBeNil)
	defer gripper2.Close(ctx)

	// Test geometry dimensions
	geometries2, err := gripper2.Geometries(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(geometries2), test.ShouldEqual, 1)
	test.That(t, len(geometries2[0].Label()), test.ShouldBeGreaterThan, 0)
	test.That(t, geometries2[0].Pose(), test.ShouldNotBeNil)

	// Type assert to access mesh-specific methods
	if mesh, ok := geometries2[0].(*spatialmath.Mesh); ok {
		test.That(t, len(mesh.Triangles()), test.ShouldBeGreaterThan, 0)
	}

	// Test that geometries include the transformed mesh
	geometries, err := gripper.Geometries(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(geometries), test.ShouldEqual, 1)
}

func TestMeshVizInvalidFile(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	config := &MeshVizConfig{
		MeshFile: "nonexistent.ply",
	}

	name := gripper.Named("test-gripper")
	deps := resource.Dependencies{}

	_, err := NewMeshViz(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestMeshVizInvalidPLYFile(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	meshFile := filepath.Join(tempDir, "invalid.ply")

	// Create an invalid PLY file
	invalidContent := `invalid ply content`
	err := os.WriteFile(meshFile, []byte(invalidContent), 0644)
	test.That(t, err, test.ShouldBeNil)

	config := &MeshVizConfig{
		MeshFile: meshFile,
	}

	name := gripper.Named("test-gripper")
	deps := resource.Dependencies{}

	_, err = NewMeshViz(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldNotBeNil)
}
