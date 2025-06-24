package kinematicsutils

import (
	"context"
	"os"
	"strings"

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
	model  referenceframe.Model
}

func newMeshViz(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	conf, err := resource.NativeConfig[*MeshVizConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewMeshViz(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewMeshViz(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *MeshVizConfig, logger logging.Logger) (gripper.Gripper, error) {

	sanitizedPLYFile, err := sanitizePLYFileToTemp(conf.MeshFile, logger)
	if err != nil {
		return nil, err
	}

	mesh, err := spatialmath.NewMeshFromPLYFile(sanitizedPLYFile)
	if err != nil {
		return nil, err
	}

	if conf.Transform != nil {
		mesh.Transform(conf.Transform)
	}

	model := referenceframe.NewSimpleModel(name.Name)
	if err != nil {
		return nil, err
	}

	return &MeshVizGripper{
		name:   name,
		logger: logger,
		cfg:    conf,
		mesh:   mesh,
		model:  model,
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
	return g.model, nil
}

// sanitizePLYFileToTemp creates a sanitized version of a PLY file and returns the path to the temporary file
func sanitizePLYFileToTemp(plyFile string, logger logging.Logger) (string, error) {
	// Read the original PLY file
	content, err := os.ReadFile(plyFile)
	if err != nil {
		return "", err
	}

	// Convert to string for easier manipulation
	plyContent := string(content)

	// Apply string replacements to fix common PLY header issues
	sanitizedContent := sanitizePLYContent(plyContent, logger)

	// Create temporary file
	tmpFile, err := os.CreateTemp("", "sanitized_ply_*.ply")
	if err != nil {
		return "", err
	}

	// Write sanitized content
	_, err = tmpFile.Write([]byte(sanitizedContent))
	if err != nil {
		tmpFile.Close()
		os.Remove(tmpFile.Name())
		return "", err
	}

	tmpFile.Close()
	return tmpFile.Name(), nil
}

// sanitizePLYContent applies string replacements to fix PLY header format issues
func sanitizePLYContent(content string, logger logging.Logger) string {
	// Track what changes we make
	changes := []string{}

	// Fix 1: Replace "vertex_index" with "vertex_indices" (singular to plural)
	if strings.Contains(content, "property list uchar int vertex_index") {
		content = strings.ReplaceAll(content, "property list uchar int vertex_index", "property list uchar int vertex_indices")
		changes = append(changes, "vertex_index → vertex_indices")
	}

	// Fix 2: Replace "uint" with "int" in face properties
	if strings.Contains(content, "property list uchar uint vertex_indices") {
		content = strings.ReplaceAll(content, "property list uchar uint vertex_indices", "property list uchar int vertex_indices")
		changes = append(changes, "uint → int")
	}

	// Fix 3: Replace "double" with "float" in vertex properties
	if strings.Contains(content, "property double") {
		content = strings.ReplaceAll(content, "property double", "property float")
		changes = append(changes, "double → float")
	}

	// Fix 4: Replace "property list uchar int vertex_index" (if it appears in other contexts)
	if strings.Contains(content, "vertex_index") {
		content = strings.ReplaceAll(content, "vertex_index", "vertex_indices")
		changes = append(changes, "vertex_index → vertex_indices (general)")
	}

	// Log the changes made
	if len(changes) > 0 {
		logger.Debugf("PLY sanitization applied: %v", changes)
	} else {
		logger.Debugf("PLY file already in correct format")
	}

	return content
}
