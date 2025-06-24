package kinematicsutils

import (
	"context"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

type KinConfig struct {
	KinematicsFile string           `json:"kinematics-file"`
	CADFile        string           `json:"ply-cad-file,omitempty"`
	CADTransform   spatialmath.Pose `json:"cad-transform,omitempty"`
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns implicit required (first return) and optional (second return) dependencies based on the config.
// The path is the JSON path in your robot's config (not the `Config` struct) to the
// resource being validated; e.g. "components.0".
func (cfg *KinConfig) Validate(path string) ([]string, []string, error) {
	// Add config validation code here
	if cfg.KinematicsFile == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "kinematics-file")
	}
	return nil, nil, nil
}

type kinChecker struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name resource.Name

	logger logging.Logger
	cfg    *KinConfig

	model  referenceframe.Model
	inputs []referenceframe.Input
	mesh   spatialmath.Geometry
}

func newKinChecker(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (arm.Arm, error) {
	conf, err := resource.NativeConfig[*KinConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewKinematicsChecker(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewKinematicsChecker(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *KinConfig, logger logging.Logger) (arm.Arm, error) {
	model, err := referenceframe.KinematicModelFromFile(conf.KinematicsFile, name.Name)
	if err != nil {
		return nil, err
	}

	mesh, err := spatialmath.NewMeshFromPLYFile(conf.CADFile)
	if err != nil {
		logger.Errorf("no cad ply file parsed with error %v", err)
	}

	if conf.CADTransform != nil {
		mesh.Transform(conf.CADTransform)
	}

	numJoints := len(model.DoF())
	logger.Infof("numJoints: %d", numJoints)
	inputs := make([]referenceframe.Input, numJoints)
	for i := range inputs {
		inputs[i] = referenceframe.Input{Value: 0.0}
	}

	s := &kinChecker{
		name:   name,
		logger: logger,
		cfg:    conf,
		model:  model,
		inputs: inputs,
		mesh:   mesh,
	}
	return s, nil
}

func (s *kinChecker) Name() resource.Name {
	return s.name
}

func (s *kinChecker) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return s.model, nil
}

func (s *kinChecker) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	return referenceframe.ComputeOOBPosition(s.model, s.inputs)
}
func (s *kinChecker) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	gif, err := s.model.Geometries(s.inputs)
	if err != nil {
		return nil, err
	}
	kinematicsGeoms := gif.Geometries()
	// tack on the CAD mesh geometry
	return append(kinematicsGeoms, s.mesh), nil

}

func (s *kinChecker) MoveToPosition(ctx context.Context, pose spatialmath.Pose, extra map[string]interface{}) error {
	pose, err := s.model.Transform(s.inputs)
	if err != nil {
		return err
	}

	plan, err := motionplan.PlanFrameMotion(ctx, s.logger, pose, s.model, s.inputs, nil, nil)
	if err != nil {
		return err
	}
	copy(s.inputs, plan[len(plan)-1])
	return nil
}

func (s *kinChecker) MoveToJointPositions(ctx context.Context, positions []referenceframe.Input, extra map[string]interface{}) error {
	s.inputs = positions
	return s.MoveThroughJointPositions(ctx, [][]referenceframe.Input{positions}, nil, nil)
}

func (s *kinChecker) MoveThroughJointPositions(ctx context.Context, positions [][]referenceframe.Input, options *arm.MoveOptions, extra map[string]interface{}) error {
	// we are not simulating motion - just going to the last position
	s.inputs = positions[len(positions)-1]
	return nil
}

func (s *kinChecker) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
	return s.inputs, nil
}

func (s *kinChecker) Stop(ctx context.Context, extra map[string]interface{}) error {
	return nil
}

func (s *kinChecker) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return s.JointPositions(ctx, nil)
}

func (s *kinChecker) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return s.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}

func (s *kinChecker) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

func (s *kinChecker) IsMoving(ctx context.Context) (bool, error) {
	return false, nil
}
