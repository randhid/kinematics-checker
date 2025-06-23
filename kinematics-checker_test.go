package kinematicsutils

import (
	"context"
	"testing"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/test"
)

func TestKinematicsCheckerConfig(t *testing.T) {
	tests := []struct {
		name        string
		config      *KinConfig
		expectError bool
	}{
		{
			name: "valid config",
			config: &KinConfig{
				KinematicsFile: "ur20.json",
			},
			expectError: false,
		},
		{
			name: "missing kinematics file",
			config: &KinConfig{
				KinematicsFile: "",
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

func TestNewKinematicsChecker(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	config := &KinConfig{
		KinematicsFile: "ur20.json",
	}

	name := arm.Named("test-arm")
	deps := resource.Dependencies{}

	arm, err := NewKinematicsChecker(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, arm, test.ShouldNotBeNil)

	// Test that it implements the arm interface
	_ = arm

	// Test Name method
	test.That(t, arm.Name(), test.ShouldResemble, name)

	// Test Close method
	err = arm.Close(ctx)
	test.That(t, err, test.ShouldBeNil)
}

func TestKinematicsCheckerInterface(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	config := &KinConfig{
		KinematicsFile: "ur20.json",
	}

	name := arm.Named("test-arm")
	deps := resource.Dependencies{}

	arm, err := NewKinematicsChecker(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldBeNil)
	defer arm.Close(ctx)

	// Test Kinematics method
	model, err := arm.Kinematics(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, model, test.ShouldNotBeNil)

	// Test EndPosition method
	pose, err := arm.EndPosition(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pose, test.ShouldNotBeNil)

	// Test Geometries method
	geometries, err := arm.Geometries(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, geometries, test.ShouldNotBeNil)

	// Test JointPositions method
	positions, err := arm.JointPositions(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, positions, test.ShouldNotBeNil)

	// Test CurrentInputs method
	inputs, err := arm.CurrentInputs(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, inputs, test.ShouldNotBeNil)

	// Test IsMoving method
	moving, err := arm.IsMoving(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, moving, test.ShouldBeFalse)

	// Test Stop method
	err = arm.Stop(ctx, nil)
	test.That(t, err, test.ShouldBeNil)

	// Test DoCommand method
	result, err := arm.DoCommand(ctx, map[string]interface{}{"test": "command"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, result, test.ShouldBeNil)
}

func TestKinematicsCheckerMovement(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	config := &KinConfig{
		KinematicsFile: "ur20.json",
	}

	name := arm.Named("test-arm")
	deps := resource.Dependencies{}

	arm, err := NewKinematicsChecker(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldBeNil)
	defer arm.Close(ctx)

	// Test MoveToJointPositions
	positions := []referenceframe.Input{{Value: 0.5}}
	err = arm.MoveToJointPositions(ctx, positions, nil)
	test.That(t, err, test.ShouldBeNil)

	// Verify the positions were set
	currentPositions, err := arm.JointPositions(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(currentPositions), test.ShouldEqual, 1)
	test.That(t, currentPositions[0].Value, test.ShouldEqual, 0.5)

	// Test MoveThroughJointPositions
	positionsList := [][]referenceframe.Input{
		{{Value: 0.0}},
		{{Value: 0.5}},
		{{Value: 1.0}},
	}
	err = arm.MoveThroughJointPositions(ctx, positionsList, nil, nil)
	test.That(t, err, test.ShouldBeNil)

	// Verify the final position was set
	currentPositions, err = arm.JointPositions(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(currentPositions), test.ShouldEqual, 1)
	test.That(t, currentPositions[0].Value, test.ShouldEqual, 1.0)

	// Test GoToInputs
	err = arm.GoToInputs(ctx, []referenceframe.Input{{Value: 0.25}})
	test.That(t, err, test.ShouldBeNil)

	// Verify the position was set
	currentPositions, err = arm.JointPositions(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(currentPositions), test.ShouldEqual, 1)
	test.That(t, currentPositions[0].Value, test.ShouldEqual, 0.25)
}

func TestKinematicsCheckerWithCADFile(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	cadFile := "lod_500.ply"
	config := &KinConfig{
		KinematicsFile: "ur20.json",
		CADFile:        cadFile,
	}

	name := arm.Named("test-arm")
	deps := resource.Dependencies{}

	arm, err := NewKinematicsChecker(ctx, deps, name, config, logger)
	test.That(t, err, test.ShouldBeNil)
	defer arm.Close(ctx)

	// Test that geometries include the CAD mesh
	geometries, err := arm.Geometries(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(geometries), test.ShouldBeGreaterThan, 0)
}
