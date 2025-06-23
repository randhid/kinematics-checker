package kinematicsutils

import (
	"context"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

type MeshVizConfig struct {
	MeshFile  string           `json:"mesh-file"`
	Transform spatialmath.Pose `json:"mesh-transform"`
}

// Validate ensures all parts of the config are valid and important fields exist.
func (cfg *MeshVizConfig) Validate(path string) ([]string, []string, error) {
	if cfg.MeshFile == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "mesh-file")
	}
	return nil, nil, nil
}

type MeshVizGripper struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name   resource.Name
	logger logging.Logger
	cfg    *MeshVizConfig
	mesh   spatialmath.Geometry
}

func newMeshViz(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	conf, err := resource.NativeConfig[*MeshVizConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewMeshViz(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewMeshViz(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *MeshVizConfig, logger logging.Logger) (gripper.Gripper, error) {
	mesh, err := spatialmath.NewMeshFromPLYFile(conf.MeshFile)
	if err != nil {
		return nil, err
	}

	if conf.Transform != nil {
		mesh.Transform(conf.Transform)
	}

	return &MeshVizGripper{
		name:   name,
		logger: logger,
		cfg:    conf,
		mesh:   mesh,
	}, nil
}

// Name implements the resource.Resource interface
func (g *MeshVizGripper) Name() resource.Name {
	return g.name
}

// DoCommand implements the resource.Resource interface
func (g *MeshVizGripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

// Close implements the resource.Resource interface
func (g *MeshVizGripper) Close(ctx context.Context) error {
	return nil
}

// Grab implements the gripper.Gripper interface
func (g *MeshVizGripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	return false, nil
}

// Stop implements the gripper.Gripper interface
func (g *MeshVizGripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	return nil
}

// Open implements the gripper.Gripper interface
func (g *MeshVizGripper) Open(ctx context.Context, extra map[string]interface{}) error {
	return nil
}

// IsMoving implements the gripper.Gripper interface
func (g *MeshVizGripper) IsMoving(ctx context.Context) (bool, error) {
	return false, nil
}

// CurrentInputs implements the gripper.Gripper interface
func (g *MeshVizGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return nil, nil
}

// Geometries implements the gripper.Gripper interface
func (g *MeshVizGripper) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return []spatialmath.Geometry{g.mesh}, nil
}

// GoToInputs implements the gripper.Gripper interface
func (g *MeshVizGripper) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return nil
}

// Kinematics implements the gripper.Gripper interface
func (g *MeshVizGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return nil, nil
}
