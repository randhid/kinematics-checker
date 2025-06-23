package kinematicsutils

import (
	"context"
	"os"
	"path/filepath"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestPointCloudVizConfig(t *testing.T) {
	tests := []struct {
		name        string
		config      *PointCloudVizConfig
		expectError bool
	}{
		{
			name: "valid config",
			config: &PointCloudVizConfig{
				PointCloudFile: "test.pcd",
			},
			expectError: false,
		},
		{
			name: "missing point cloud file",
			config: &PointCloudVizConfig{
				PointCloudFile: "",
			},
			expectError: true,
		},
		{
			name: "valid config with transform",
			config: &PointCloudVizConfig{
				PointCloudFile: "test.pcd",
				Transform:      spatialmath.NewPose(r3.Vector{X: 1.0, Y: 2.0, Z: 3.0}, spatialmath.NewZeroOrientation()),
			},
			expectError: false,
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

func TestNewPointCloudViz(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	pointCloudFile := filepath.Join(tempDir, "test.pcd")

	// Create a simple PCD file
	pcdContent := `# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 3
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 3
DATA ascii
0.0 0.0 0.0
1.0 0.0 0.0
0.0 1.0 0.0
`
	err := os.WriteFile(pointCloudFile, []byte(pcdContent), 0644)
	test.That(t, err, test.ShouldBeNil)

	config := &PointCloudVizConfig{
		PointCloudFile: pointCloudFile,
	}

	deps := resource.Dependencies{}

	cam, err := newPointCloudViz(ctx, deps, resource.Config{
		ConvertedAttributes: config,
	}, logger)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cam, test.ShouldNotBeNil)

	// Test that it implements the camera interface
	_, ok := cam.(camera.Camera)
	test.That(t, ok, test.ShouldBeTrue)

	// Test Close method
	err = cam.Close(ctx)
	test.That(t, err, test.ShouldBeNil)
}

func TestPointCloudVizInterface(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	pointCloudFile := filepath.Join(tempDir, "test.pcd")

	// Create a simple PCD file
	pcdContent := `# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 3
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 3
DATA ascii
0.0 0.0 0.0
1.0 0.0 0.0
0.0 1.0 0.0
`
	err := os.WriteFile(pointCloudFile, []byte(pcdContent), 0644)
	test.That(t, err, test.ShouldBeNil)

	config := &PointCloudVizConfig{
		PointCloudFile: pointCloudFile,
	}

	deps := resource.Dependencies{}

	cam, err := newPointCloudViz(ctx, deps, resource.Config{
		ConvertedAttributes: config,
	}, logger)
	test.That(t, err, test.ShouldBeNil)
	defer cam.Close(ctx)

	// Test Image method
	img, metadata, err := cam.Image(ctx, "image/jpeg", nil)
	test.That(t, err, test.ShouldNotBeNil) // Should return errUnimplemented
	test.That(t, img, test.ShouldBeNil)
	test.That(t, metadata, test.ShouldResemble, camera.ImageMetadata{})

	// Test Images method
	images, responseMetadata, err := cam.Images(ctx)
	test.That(t, err, test.ShouldNotBeNil) // Should return errUnimplemented
	test.That(t, images, test.ShouldBeNil)
	test.That(t, responseMetadata, test.ShouldResemble, resource.ResponseMetadata{})

	// Test NextPointCloud method
	pc, err := cam.NextPointCloud(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pc, test.ShouldNotBeNil)

	// Verify the point cloud exists
	test.That(t, pc, test.ShouldNotBeNil)

	// Test Properties method
	props, err := cam.Properties(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, props, test.ShouldResemble, camera.Properties{})

	// Test DoCommand method
	result, err := cam.DoCommand(ctx, map[string]interface{}{"test": "command"})
	test.That(t, err, test.ShouldNotBeNil) // Should return errUnimplemented
	test.That(t, result, test.ShouldBeNil)
}

func TestPointCloudVizWithTransform(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	pointCloudFile := filepath.Join(tempDir, "test.pcd")

	// Create a simple PCD file
	pcdContent := `# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 3
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 3
DATA ascii
0.0 0.0 0.0
1.0 0.0 0.0
0.0 1.0 0.0
`
	err := os.WriteFile(pointCloudFile, []byte(pcdContent), 0644)
	test.That(t, err, test.ShouldBeNil)

	// Create a transform
	transform := spatialmath.NewPose(r3.Vector{X: 1.0, Y: 2.0, Z: 3.0}, spatialmath.NewZeroOrientation())

	config := &PointCloudVizConfig{
		PointCloudFile: pointCloudFile,
		Transform:      transform,
	}

	deps := resource.Dependencies{}

	cam, err := newPointCloudViz(ctx, deps, resource.Config{
		ConvertedAttributes: config,
	}, logger)
	test.That(t, err, test.ShouldBeNil)
	defer cam.Close(ctx)

	// Test NextPointCloud method with transform
	pc, err := cam.NextPointCloud(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pc, test.ShouldNotBeNil)

	// Verify the point cloud exists
	test.That(t, pc, test.ShouldNotBeNil)
}

func TestPointCloudVizInvalidFile(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	config := &PointCloudVizConfig{
		PointCloudFile: "nonexistent.pcd",
	}

	deps := resource.Dependencies{}

	_, err := newPointCloudViz(ctx, deps, resource.Config{
		ConvertedAttributes: config,
	}, logger)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestPointCloudVizInvalidPCDFile(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	pointCloudFile := filepath.Join(tempDir, "invalid.pcd")

	// Create an invalid PCD file
	invalidContent := `invalid pcd content`
	err := os.WriteFile(pointCloudFile, []byte(invalidContent), 0644)
	test.That(t, err, test.ShouldBeNil)

	config := &PointCloudVizConfig{
		PointCloudFile: pointCloudFile,
	}

	deps := resource.Dependencies{}

	_, err = newPointCloudViz(ctx, deps, resource.Config{
		ConvertedAttributes: config,
	}, logger)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestPointCloudVizConstructor(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// Create a temporary directory for test files
	tempDir := t.TempDir()
	pointCloudFile := filepath.Join(tempDir, "test.pcd")

	// Create a simple PCD file
	pcdContent := `# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 3
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 3
DATA ascii
0.0 0.0 0.0
1.0 0.0 0.0
0.0 1.0 0.0
`
	err := os.WriteFile(pointCloudFile, []byte(pcdContent), 0644)
	test.That(t, err, test.ShouldBeNil)

	config := &PointCloudVizConfig{
		PointCloudFile: pointCloudFile,
	}

	deps := resource.Dependencies{}

	rawConfig := resource.Config{
		ConvertedAttributes: config,
	}

	cam, err := newPointCloudViz(ctx, deps, rawConfig, logger)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cam, test.ShouldNotBeNil)
	defer cam.Close(ctx)

	// Test that it implements the camera interface
	_, ok := cam.(camera.Camera)
	test.That(t, ok, test.ShouldBeTrue)

	// Test NextPointCloud method
	pc, err := cam.NextPointCloud(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pc, test.ShouldNotBeNil)

	// Verify the point cloud exists
	test.That(t, pc, test.ShouldNotBeNil)
}
