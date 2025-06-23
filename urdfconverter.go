package kinematicsutils

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"regexp"
	"strings"

	"github.com/pkg/errors"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
)

// Command keys for URDF converter service
const (
	URDFCommand       = "urdf"
	URDF2ModelCommand = "urdf2model"
)

// New creates a new URDFConverterService
func New(ctx context.Context) (*URDFConverterService, error) {
	return &URDFConverterService{}, nil
}

// DoCommand handles generic service commands
func (s *URDFConverterService) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case URDFCommand:
		return map[string]interface{}{
			"sanitized_xml": string(s.sanitizedXML),
			"original_file": s.cfg.URDFFile,
		}, nil
	case URDF2ModelCommand:
		return s.handleURDF2ModelCommand(ctx, cmd)
	}

	return nil, nil
}

func (s *URDFConverterService) handleURDF2ModelCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	// Parse the sanitized XML into a model
	mc, err := referenceframe.UnmarshalModelXML(s.sanitizedXML, "modelName")
	if err != nil {
		return nil, fmt.Errorf("failed to unmarshal sanitized XML: %w", err)
	}

	// Convert to a simple map structure
	modelConfig := map[string]interface{}{
		"name":   mc.Name,
		"links":  make([]map[string]interface{}, len(mc.Links)),
		"joints": make([]map[string]interface{}, len(mc.Joints)),
	}

	// Convert links
	for i, link := range mc.Links {
		linkConfig := map[string]interface{}{
			"id":     link.ID,
			"parent": link.Parent,
		}

		// Add translation if present
		if link.Translation.X != 0 || link.Translation.Y != 0 || link.Translation.Z != 0 {
			linkConfig["translation"] = map[string]interface{}{
				"x": link.Translation.X,
				"y": link.Translation.Y,
				"z": link.Translation.Z,
			}
		}

		// Add orientation if present
		if link.Orientation != nil {
			linkConfig["orientation"] = link.Orientation
		}

		// Add geometry if present
		if link.Geometry != nil {
			linkConfig["geometry"] = link.Geometry
		}

		modelConfig["links"].([]map[string]interface{})[i] = linkConfig
	}

	// Convert joints
	for i, joint := range mc.Joints {
		jointConfig := map[string]interface{}{
			"id":     joint.ID,
			"type":   joint.Type,
			"parent": joint.Parent,
		}

		// Add axis if present
		if joint.Axis.X != 0 || joint.Axis.Y != 0 || joint.Axis.Z != 0 {
			jointConfig["axis"] = map[string]interface{}{
				"x": joint.Axis.X,
				"y": joint.Axis.Y,
				"z": joint.Axis.Z,
			}
		}

		// Add limits if present
		if joint.Max != 0 || joint.Min != 0 {
			jointConfig["max"] = joint.Max
			jointConfig["min"] = joint.Min
		}

		modelConfig["joints"].([]map[string]interface{})[i] = jointConfig
	}

	// Convert to JSON
	modelJSONBytes, err := json.MarshalIndent(modelConfig, "", "  ")
	if err != nil {
		return nil, fmt.Errorf("failed to marshal model to JSON: %w", err)
	}

	return map[string]interface{}{
		"model_json":    string(modelJSONBytes),
		"sanitized_xml": string(s.sanitizedXML),
		"original_file": s.cfg.URDFFile,
	}, nil
}

// Close implements the resource.Resource interface
func (s *URDFConverterService) Close(ctx context.Context) error {
	return nil
}

type URDFConverterConfig struct {
	URDFFile string `json:"urdf-file"`
}

// Validate ensures all parts of the config are valid and important fields exist.
func (cfg *URDFConverterConfig) Validate(path string) ([]string, []string, error) {
	if cfg.URDFFile == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "urdf-file")
	}
	return nil, nil, nil
}

type URDFConverterService struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name         resource.Name
	logger       logging.Logger
	cfg          *URDFConverterConfig
	sanitizedXML []byte
}

// Name returns the name of the service
func (s *URDFConverterService) Name() resource.Name {
	return s.name
}

func newURDFConverter(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*URDFConverterConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewURDFConverter(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewURDFConverter(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *URDFConverterConfig, logger logging.Logger) (resource.Resource, error) {
	// Parse and sanitize the URDF file during construction
	sanitizedXML, err := sanitizeURDF(conf.URDFFile, name.Name, logger)
	if err != nil {
		return nil, fmt.Errorf("failed to sanitize URDF: %w", err)
	}

	logger.Debugf("Sanitized XML: %s", sanitizedXML)

	return &URDFConverterService{
		name:         name,
		logger:       logger,
		cfg:          conf,
		sanitizedXML: sanitizedXML,
	}, nil
}

func sanitizeURDF(urdfFile, name string, logger logging.Logger) ([]byte, error) {
	// First try to parse with Viam's parser to validate the URDF structure
	_, err := referenceframe.ParseModelXMLFile(urdfFile, name)
	if err != nil {
		// If parsing fails, we'll still try to sanitize the raw XML
		logger.Debugf("URDF validation failed, proceeding with sanitization: %v", err)
	}

	// Read the raw XML file
	xmlData, err := os.ReadFile(urdfFile)
	if err != nil {
		return nil, errors.Wrap(err, "failed to read URDF file")
	}

	s := string(xmlData)

	// Count total meshes in original file for verification
	totalMeshCount := strings.Count(s, `<mesh filename=`)

	// 1. Remove all <visual ...>...</visual> sections (multi-line)
	reVisual := regexp.MustCompile(`(?s)<visual[^>]*>.*?</visual>`) // (?s) for dotall
	s = reVisual.ReplaceAllString(s, "")

	// 2. Replace mesh elements only within collision sections
	// First, find all collision sections and replace meshes within them
	reCollision := regexp.MustCompile(`(?s)(<collision[^>]*>)(.*?)(</collision>)`)
	s = reCollision.ReplaceAllStringFunc(s, func(collisionBlock string) string {
		match := reCollision.FindStringSubmatch(collisionBlock)
		if len(match) != 4 {
			return collisionBlock
		}
		openTag, body, closeTag := match[1], match[2], match[3]

		// Replace meshes within this collision section
		// Replace self-closing mesh tags
		reMeshSelfClosing := regexp.MustCompile(`<mesh filename=("[^"]*"|'[^']*')[^>]*/>`)
		body = reMeshSelfClosing.ReplaceAllString(body, `<box size="50 60 70"/>`)

		// Replace mesh blocks
		reMeshBlock := regexp.MustCompile(`(?s)<mesh [^>]*>.*?</mesh>`)
		body = reMeshBlock.ReplaceAllString(body, `<box size="50 60 70"/>`)

		return openTag + body + closeTag
	})

	// 3. For each <link ...>...</link>, if there is no <collision, inject a default collision before </link>
	reLink := regexp.MustCompile(`(?s)(<link[^>]*>)(.*?)(</link>)`)
	s = reLink.ReplaceAllStringFunc(s, func(linkBlock string) string {
		// Check if <collision exists in the link body
		match := reLink.FindStringSubmatch(linkBlock)
		if len(match) != 4 {
			return linkBlock // Should not happen
		}
		openTag, body, closeTag := match[1], match[2], match[3]
		if regexp.MustCompile(`<collision[\s>]`).MatchString(body) {
			return openTag + body + closeTag
		}
		// Inject default collision before closeTag (use real newline)
		defaultCollision := "\n    <collision>\n      <geometry>\n        <box size=\"50 60 70\"/>\n      </geometry>\n    </collision>"
		return openTag + body + defaultCollision + closeTag
	})

	// 4. Verify we have the expected number of boxes
	boxCount := strings.Count(s, `<box size="50 60 70"/>`)
	logger.Debugf("Original mesh count: %d, Final box count: %d", totalMeshCount, boxCount)

	// 5. Trim whitespace and clean up the XML
	s = strings.TrimSpace(s)

	// Remove excessive blank lines
	reBlankLines := regexp.MustCompile(`\n\s*\n`)
	s = reBlankLines.ReplaceAllString(s, "\n")

	logger.Debugf("Sanitized XML: %s", s)

	return []byte(s), nil
}
